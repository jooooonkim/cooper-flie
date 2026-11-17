"""crazyflie + sim drone, same methods. setpoints must keep streaming"""

import math
import threading
import time
from dataclasses import dataclass, replace

import numpy as np

from .paths import CACHE_DIR

G = 9.81


@dataclass
class State:
    t: float = 0.0  # host time
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0  # deg

    @property
    def pos(self):
        return np.array([self.x, self.y, self.z])


# from calibrate_signs.py
DEFAULT_SIGNS = {"pitch": -1, "roll": -1, "yaw": 1}


def open_drone(uri, sim=False, signs=None):
    if sim:
        return SimDrone(signs)
    return Drone(uri)


class Drone:
    def __init__(self, uri):
        self.uri = uri
        self.cf = None
        self.flying = False
        self._scf = None
        self._logs = []
        self._streaming = False
        self._state = State()
        self._lock = threading.Lock()
        self.killed = False
        self._cmd_lock = threading.Lock()  # so nothing gets sent after a kill

    def __enter__(self):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.utils.reset_estimator import reset_estimator

        cflib.crtp.init_drivers()
        print(f"Connecting to {self.uri} ...")
        self._scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR)))
        self._scf.open_link()
        self.cf = self._scf.cf
        reset_estimator(self.cf)
        self._start_logging()
        self._wait_for_state()
        self.cf.supervisor.send_arming_request(True)
        time.sleep(1.0)
        print("Connected, estimator converged, armed.")
        return self

    def __exit__(self, *exc):
        try:
            if self.flying:
                self.land()
        finally:
            for log in self._logs:
                log.stop()
            if self._scf is not None:
                self._scf.close_link()

    def _start_logging(self):
        from cflib.crazyflie.log import LogConfig

        # 26 byte limit per block
        pos = LogConfig(name="pos", period_in_ms=10)
        for var in ("x", "y", "z", "yaw"):
            pos.add_variable(f"stateEstimate.{var}", "float")
        vel = LogConfig(name="vel", period_in_ms=10)
        for var in ("vx", "vy", "vz"):
            vel.add_variable(f"stateEstimate.{var}", "float")

        for log in (pos, vel):
            self.cf.log.add_config(log)
            log.data_received_cb.add_callback(self._on_log)
            log.start()
            self._logs.append(log)

    def _on_log(self, _timestamp, data, _logconf):
        values = {k.split(".")[1]: v for k, v in data.items()}
        with self._lock:
            self._state = replace(self._state, t=time.monotonic(), **values)

    def _wait_for_state(self, timeout=3.0):
        end = time.monotonic() + timeout
        while self.state_age() > 0.1:
            if time.monotonic() > end:
                raise RuntimeError("No state estimate from the drone. Is the Lighthouse deck working?")
            time.sleep(0.05)

    @property
    def state(self):
        with self._lock:
            return replace(self._state)

    def state_age(self):
        return time.monotonic() - self.state.t

    # blocking

    def _wait(self, seconds):
        # sleep, but stop early on kill
        end = time.monotonic() + seconds
        while not self.killed and time.monotonic() < end:
            time.sleep(0.02)

    def takeoff(self, height=0.5, duration=2.0):
        with self._cmd_lock:
            if self.killed:
                return
            self.cf.high_level_commander.takeoff(height, duration)
            self.flying = True
        self._wait(duration + 0.3)

    def go_to(self, x, y, z, yaw=0.0, duration=2.0):
        with self._cmd_lock:
            if self.killed:
                return
            self.stop_streaming()
            self.cf.high_level_commander.go_to(x, y, z, math.radians(yaw), duration)
        self._wait(duration + 0.2)

    def land(self, duration=2.5):
        with self._cmd_lock:
            if self.killed:
                return
            self.stop_streaming()
            self.cf.high_level_commander.land(0.0, duration)
        self._wait(duration + 0.5)
        with self._cmd_lock:
            if self.killed:
                return
            self.cf.high_level_commander.stop()
            self.flying = False
            self.cf.supervisor.send_arming_request(False)

    def kill(self):
        # motors off, ignores every command after
        with self._cmd_lock:
            self.killed = True
            self.cf.commander.send_stop_setpoint()
            self.flying = False

    # streaming, call every loop

    def position_setpoint(self, x, y, z, yaw=0.0):
        with self._cmd_lock:
            if self.killed:
                return
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            self._streaming = True

    def attitude_setpoint(self, roll, pitch, yawrate, z):
        # deg, deg/s, m
        with self._cmd_lock:
            if self.killed:
                return
            self.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, z)
            self._streaming = True

    def stop_streaming(self):
        # back to high level
        if self._streaming:
            self.cf.commander.send_notify_setpoint_stop()
            self._streaming = False


class SimDrone:
    """point mass sim, 200 Hz"""

    RATE = 200.0
    WN_POS = 3.0  # rad/s
    MAX_ACC = 4.0  # m/s^2
    TAU_ATT = 0.06  # s, tilt lag
    DRAG = 0.25  # 1/s

    def __init__(self, signs=None):
        self.flying = False
        self._signs = dict(signs or DEFAULT_SIGNS)
        self._lock = threading.Lock()
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._yaw = 0.0
        self._tilt = np.zeros(2)  # pitch, roll, deg
        self._mode = "idle"
        self._target = np.zeros(3)
        self._target_yaw = 0.0
        self._att_cmd = (0.0, 0.0, 0.0, 0.0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.killed = False

    def __enter__(self):
        print("Simulated drone ready.")
        self._thread.start()
        return self

    def __exit__(self, *exc):
        try:
            if self.flying:
                self.land()
        finally:
            self._stop.set()
            self._thread.join()

    def _run(self):
        dt = 1.0 / self.RATE
        while not self._stop.is_set():
            with self._lock:
                self._step(dt)
            time.sleep(dt)

    def _step(self, dt):
        if self._mode == "idle":
            self._v[:] = 0.0
            return
        if self._mode == "fall":
            self._v[2] -= G * dt
            self._p += self._v * dt
            if self._p[2] <= 0.0:
                self._p[2], self._v[:], self._mode = 0.0, 0.0, "idle"
            return

        if self._mode == "position":
            wn = self.WN_POS
            acc = wn * wn * (self._target - self._p) - 2 * wn * self._v
            norm = np.linalg.norm(acc)
            if norm > self.MAX_ACC:
                acc *= self.MAX_ACC / norm
            self._yaw += (self._target_yaw - self._yaw) * min(1.0, 3.0 * dt)
            self._tilt[:] = 0.0
        else:  # attitude
            roll, pitch, yawrate, z = self._att_cmd
            # undo sign
            phys = np.array([self._signs["pitch"] * pitch, self._signs["roll"] * roll])
            self._tilt += (phys - self._tilt) * min(1.0, dt / self.TAU_ATT)
            a_body = G * np.tan(np.radians(self._tilt))  # (forward, left)
            c, s = math.cos(math.radians(self._yaw)), math.sin(math.radians(self._yaw))
            acc = np.array([
                c * a_body[0] - s * a_body[1] - self.DRAG * self._v[0],
                s * a_body[0] + c * a_body[1] - self.DRAG * self._v[1],
                0.0,
            ])
            wn = 4.0
            acc[2] = wn * wn * (z - self._p[2]) - 2 * wn * self._v[2]
            self._yaw += self._signs["yaw"] * yawrate * dt

        self._v += acc * dt
        self._p += self._v * dt
        if self._p[2] < 0.0:
            self._p[2], self._v[2] = 0.0, max(0.0, self._v[2])

    @property
    def state(self):
        with self._lock:
            p, v = self._p, self._v
            return State(time.monotonic(), p[0], p[1], p[2], v[0], v[1], v[2], self._yaw)

    def state_age(self):
        return 0.0

    def _set_target(self, x, y, z, yaw=0.0):
        with self._lock:
            if self.killed:
                return
            self._mode = "position"
            self._target = np.array([x, y, z], dtype=float)
            self._target_yaw = yaw

    def _wait(self, seconds):
        end = time.monotonic() + seconds
        while not self.killed and time.monotonic() < end:
            time.sleep(0.02)

    def takeoff(self, height=0.5, duration=2.0):
        if self.killed:
            return
        s = self.state
        self._set_target(s.x, s.y, height)
        self.flying = True
        self._wait(duration + 0.3)

    def go_to(self, x, y, z, yaw=0.0, duration=2.0):
        self._set_target(x, y, z, yaw)
        self._wait(duration + 0.2)

    def land(self, duration=2.5):
        s = self.state
        self._set_target(s.x, s.y, 0.0)
        self._wait(duration + 0.5)
        with self._lock:
            if self.killed:
                return
            self._mode = "idle"
            self._p[2] = 0.0
        self.flying = False

    def kill(self):
        with self._lock:
            self.killed = True
            self._mode = "fall"
        self.flying = False

    def position_setpoint(self, x, y, z, yaw=0.0):
        self._set_target(x, y, z, yaw)

    def attitude_setpoint(self, roll, pitch, yawrate, z):
        with self._lock:
            if self.killed:
                return
            self._mode = "attitude"
            self._att_cmd = (roll, pitch, yawrate, z)

    def stop_streaming(self):
        pass
