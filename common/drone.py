"""Crazyflie wrapper with a matching simulated drone.

Both classes expose the same methods, so every script can run with --sim:

    with open_drone(cfg, sim=args.sim) as drone:
        drone.takeoff()
        drone.position_setpoint(x, y, z)          # streamed, ~50 Hz
        drone.attitude_setpoint(roll, pitch, yawrate, z)
    # leaving the block lands the drone if it is still flying

Streaming setpoints must keep arriving: the firmware levels the drone after
about 0.5 s without one and cuts the motors after about 2 s.
"""

import math
import threading
import time
from dataclasses import dataclass, replace

import numpy as np

from .config import CACHE_DIR

G = 9.81


@dataclass
class State:
    t: float = 0.0  # host time.monotonic() of the last update
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


def open_drone(cfg, sim=False, uri=None):
    if sim:
        return SimDrone(cfg)
    return Drone(uri or cfg["drones"][0]["uri"], cfg)


class Drone:
    def __init__(self, uri, cfg):
        self.uri = uri
        self.cfg = cfg
        self.cf = None
        self.flying = False
        self._scf = None
        self._logs = []
        self._streaming = False
        self._state = State()
        self._lock = threading.Lock()

    # -- connection -------------------------------------------------------

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

        # A log block holds at most 26 bytes, so position and velocity are split.
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

    # -- state ------------------------------------------------------------

    @property
    def state(self):
        with self._lock:
            return replace(self._state)

    def state_age(self):
        return time.monotonic() - self.state.t

    # -- high-level commands (blocking) -------------------------------------

    def takeoff(self, height=None, duration=2.0):
        height = height or self.cfg["flight"]["takeoff_height"]
        self.cf.high_level_commander.takeoff(height, duration)
        self.flying = True
        time.sleep(duration + 0.3)

    def go_to(self, x, y, z, yaw=0.0, duration=2.0):
        self.stop_streaming()
        self.cf.high_level_commander.go_to(x, y, z, math.radians(yaw), duration)
        time.sleep(duration + 0.2)

    def land(self, duration=2.5):
        self.stop_streaming()
        self.cf.high_level_commander.land(0.0, duration)
        time.sleep(duration + 0.5)
        self.cf.high_level_commander.stop()
        self.flying = False
        self.cf.supervisor.send_arming_request(False)

    def kill(self):
        """Stop the motors immediately. The drone falls."""
        self.cf.commander.send_stop_setpoint()
        self.flying = False

    # -- streaming setpoints (call every loop) --------------------------------

    def position_setpoint(self, x, y, z, yaw=0.0):
        self.cf.commander.send_position_setpoint(x, y, z, yaw)
        self._streaming = True

    def attitude_setpoint(self, roll, pitch, yawrate, z):
        """roll/pitch in deg, yawrate in deg/s, z = absolute height in m."""
        self.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, z)
        self._streaming = True

    def stop_streaming(self):
        """Hand control back to the high-level commander."""
        if self._streaming:
            self.cf.commander.send_notify_setpoint_stop()
            self._streaming = False


class SimDrone:
    """Point-mass stand-in for a Crazyflie, integrated at 200 Hz in a thread.

    Position commands track like a critically damped second-order system.
    Attitude commands tilt the thrust vector with a short lag, a = g*tan(tilt),
    plus linear air drag, using the sign conventions in config.yaml so the
    same controller code runs unchanged on the real drone.
    """

    RATE = 200.0
    WN_POS = 3.0  # rad/s, position-mode bandwidth
    MAX_ACC = 4.0  # m/s^2 in position mode
    TAU_ATT = 0.06  # s, attitude response lag
    DRAG = 0.25  # 1/s, linear drag

    def __init__(self, cfg):
        self.cfg = cfg
        self.flying = False
        self._signs = cfg["attitude"]
        self._lock = threading.Lock()
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._yaw = 0.0
        self._tilt = np.zeros(2)  # physical (pitch, roll) tilt, deg
        self._mode = "idle"
        self._target = np.zeros(3)
        self._target_yaw = 0.0
        self._att_cmd = (0.0, 0.0, 0.0, 0.0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

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
            # Undo the command sign to get the physical tilt the real drone would have.
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
            self._mode = "position"
            self._target = np.array([x, y, z], dtype=float)
            self._target_yaw = yaw

    def takeoff(self, height=None, duration=2.0):
        height = height or self.cfg["flight"]["takeoff_height"]
        s = self.state
        self._set_target(s.x, s.y, height)
        self.flying = True
        time.sleep(duration + 0.3)

    def go_to(self, x, y, z, yaw=0.0, duration=2.0):
        self._set_target(x, y, z, yaw)
        time.sleep(duration + 0.2)

    def land(self, duration=2.5):
        s = self.state
        self._set_target(s.x, s.y, 0.0)
        time.sleep(duration + 0.5)
        with self._lock:
            self._mode = "idle"
            self._p[2] = 0.0
        self.flying = False

    def kill(self):
        with self._lock:
            self._mode = "fall"
        self.flying = False

    def position_setpoint(self, x, y, z, yaw=0.0):
        self._set_target(x, y, z, yaw)

    def attitude_setpoint(self, roll, pitch, yawrate, z):
        with self._lock:
            self._mode = "attitude"
            self._att_cmd = (roll, pitch, yawrate, z)

    def stop_streaming(self):
        pass
