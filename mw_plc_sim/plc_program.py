"""What the line PLC's programme does with the robot PC, and nothing more.

One step() per bus cycle: read the robot's image, write ours. Follows the
handshake contract in mw_profinet_bridge/gsdml/README.md:

  * start is HELD until the robot shows servo_busy (or start_hold_s passes),
    stop and fault reset are held for pulse_s -- never a one-cycle pulse,
    which a bus cycle can swallow;
  * nutrunner: a flip of the robot's request toggle starts a "tightening";
    after fasten_s the result (torque_ok, actual torque) is written FIRST and
    the ack toggle flipped LAST, in the same cycle -- the ack is the PLC's
    promise that the result is final.
"""
import threading
import time

from .layout import RobotStatus, set_bit, set_real


class PlcProgram:
    def __init__(self, layout, fasten_s=2.0, pulse_s=0.2, start_hold_s=5.0, log=print):
        self.layout = layout
        self.fasten_s = fasten_s
        self.pulse_s = pulse_s
        self.start_hold_s = start_hold_s
        self.log = log
        self.torque_ok = True
        self.torque_scale = 1.0

        self._lock = threading.Lock()
        self._out = bytearray(layout.out_bytes)
        self._start_since = None
        self._stop_until = 0.0
        self._reset_until = 0.0
        self._fastening_since = None
        self._fasten_target = 0.0
        self._last_req_toggle = None   # adopted from the robot on the first cycle
        self.robot = None              # latest RobotStatus
        self.robot_changed = threading.Condition(self._lock)
        self.tightenings = []          # (target, actual, ok) per completed request

    # ── operator / HMI ───────────────────────────────────────────────────────

    def press_start(self):
        with self._lock:
            self._start_since = time.monotonic()
        self.log("PLC: start_cycle held until the robot is busy")

    def press_stop(self):
        with self._lock:
            self._stop_until = time.monotonic() + self.pulse_s
        self.log("PLC: stop_cycle")

    def press_fault_reset(self):
        with self._lock:
            self._reset_until = time.monotonic() + self.pulse_s
        self.log("PLC: fault_reset")

    def set_torque_result(self, ok, scale=1.0):
        with self._lock:
            self.torque_ok, self.torque_scale = ok, scale
        self.log(f"PLC: next tightenings report torque {'OK' if ok else 'NOT reached'}")

    # ── one bus cycle ────────────────────────────────────────────────────────

    def step(self, robot_image):
        """robot_image: the PC -> PLC bytes, or None when there is no valid data.
        Returns the PLC -> PC bytes to send this cycle."""
        now = time.monotonic()
        L = self.layout
        with self._lock:
            if robot_image is not None:
                robot = RobotStatus.decode(L, robot_image)
                if robot != self.robot:
                    if self.robot is None or robot.short() != self.robot.short():
                        self.log(f"robot: {robot.short()}")
                    self.robot = robot
                    self.robot_changed.notify_all()

                if self._last_req_toggle is None:
                    # In step with whatever the robot holds from earlier -- and
                    # answer with a matching ack, or it refuses to ask at all.
                    self._last_req_toggle = robot.nutrunner_req_toggle
                    set_bit(self._out, L.nutrunner_ack_toggle, robot.nutrunner_req_toggle)

                if self._start_since is not None and (
                        robot.servo_busy or now - self._start_since > self.start_hold_s):
                    if not robot.servo_busy:
                        self.log("PLC: robot never went busy -- released start_cycle")
                    self._start_since = None

                if self._fastening_since is None and robot.nutrunner_req_toggle != self._last_req_toggle:
                    self._fastening_since = now
                    self._fasten_target = robot.target_torque
                    set_bit(self._out, L.torque_ok, False)
                    self.log(f"PLC: nutrunner request (target {robot.target_torque:.3f}) -- tightening")
                if self._fastening_since is not None and now - self._fastening_since >= self.fasten_s:
                    actual = self._fasten_target * self.torque_scale
                    set_bit(self._out, L.torque_ok, self.torque_ok)          # result first
                    set_real(self._out, L.actual_torque, actual)
                    set_bit(self._out, L.nutrunner_ack_toggle, robot.nutrunner_req_toggle)  # ack last
                    self._last_req_toggle = robot.nutrunner_req_toggle
                    self._fastening_since = None
                    self.tightenings.append((self._fasten_target, actual, self.torque_ok))
                    self.log(f"PLC: tightening done, torque {'OK' if self.torque_ok else 'NOT reached'} "
                             f"({actual:.3f}), ack")

            set_bit(self._out, L.start_cycle, self._start_since is not None)
            set_bit(self._out, L.stop_cycle, now < self._stop_until)
            set_bit(self._out, L.fault_reset, now < self._reset_until)
            return bytes(self._out)

    def wait_for(self, predicate, timeout):
        """Block until predicate(RobotStatus) holds; returns it, or None on timeout."""
        deadline = time.monotonic() + timeout
        with self._lock:
            while True:
                if self.robot is not None and predicate(self.robot):
                    return self.robot
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.robot_changed.wait(left)
