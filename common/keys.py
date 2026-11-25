import threading

from pynput import keyboard


class Keys:
    # q = land, esc = kill motors. keys are global
    #
    #   with open_drone(...) as drone, Keys(drone) as keys:
    #       while not keys.stop: ...

    def __init__(self, drone):
        self.drone = drone
        self._quit = threading.Event()
        self._listener = keyboard.Listener(on_press=self._on_press)

    def _on_press(self, key):
        if key == keyboard.Key.esc:
            if not self.drone.killed:
                print("EMERGENCY STOP")
                self.drone.kill()  # right away, from the key thread
        elif getattr(key, "char", None) in ("q", "Q"):
            self._quit.set()

    @property
    def quit(self):
        return self._quit.is_set()

    @property
    def kill(self):
        return self.drone.killed

    @property
    def stop(self):
        return self.quit or self.kill

    def __enter__(self):
        self._listener.start()
        return self

    def __exit__(self, *exc):
        self._listener.stop()
