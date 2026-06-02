"""
robot/robot_impl/gripper.py
───────────────────────────
GripperMixin — add to Robot's MRO alongside HardwareMixin, SensorsMixin, etc.

Public API (all channels 1-based to match the rest of the Robot class):

    robot.open_gripper(channel=16, pulse_us=1000)
    robot.close_gripper(channel=16, pulse_us=2000)
    robot.squeeze_until_resistance(channel=16, ...)
    robot.burger_pick(height_steps, stepper=1, ...)

Hardware wiring assumptions (from arduino.ino):
    SERVO_CLAW_CHANNEL      = 15  (PCA9685 channel, 0-based on firmware)
    SERVO_MIN_GRIP_PULSE    = 150 (raw PCA9685 ticks — NOT microseconds)
    SERVO_MAX_SAFE_SQUEEZE  = 350
    PIN_GRIPPER_RESISTANCE  = A3

    The ServoController on the Arduino accepts pulse widths in MICROSECONDS
    via the servo_set TLV message (pulseUs field).  Typical range:
        open  → ~1000 µs
        closed → ~2000 µs
    Tune GRIPPER_OPEN_US / GRIPPER_CLOSED_US to match your physical servo.

Resistance reading:
    The Arduino reads A3 and returns it in io_input_state (analog channel).
    We poll robot.get_analog(pin=3) which reads from the cached IOInputState.
    If that ADC channel is not wired up yet, squeeze_until_resistance falls
    back to a timed squeeze and logs a warning.
"""

from __future__ import annotations

import time
import threading

from bridge_interfaces.msg import ServoEnable, ServoSet, StepEnable, StepMove

# ── Defaults (tune to your servo) ────────────────────────────────────────────
GRIPPER_CHANNEL       = 16       # 1-based public API channel (firmware ch 15)
GRIPPER_OPEN_US       = 900     # pulse width µs → claw open
GRIPPER_CLOSED_US     = 1600     # pulse width µs → claw fully closed
GRIPPER_SQUEEZE_STEP  = 20       # µs per squeeze increment
GRIPPER_SQUEEZE_DELAY = 0.03     # seconds between increments
GRIPPER_RESISTANCE_PIN = 3       # analog pin index (A3 on the Arduino)
GRIPPER_RESISTANCE_THRESHOLD = 500  # ADC counts (0–1023) — tune for your sensor


class GripperMixin:
    """
    Gripper methods for the Robot class.

    Requires self._node, self._srv_en_pub, self._srv_set_pub to be set up
    by Robot.__init__ (they already are — ServoEnable / ServoSet publishers
    are created there).
    """

    # ── Low-level servo helpers ───────────────────────────────────────────────

    def _servo_channel_to_firmware(self, channel: bool) -> bool:
        """Convert 1-based public channel to 0-based firmware channel."""
        return channel - 1

    def enable_servo(self, channel: int = GRIPPER_CHANNEL) -> None:
        """Enable a servo channel (must be called before set)."""
        msg = ServoEnable()
        msg.channel = self._servo_channel_to_firmware(channel)
        msg.enable = bool(True)
        self._srv_en_pub.publish(msg)

    def disable_servo(self, channel: int = GRIPPER_CHANNEL) -> None:
        """Disable a servo channel (servo goes limp — no holding torque)."""
        msg = ServoEnable()
        msg.channel = self._servo_channel_to_firmware(channel)
        msg.enable = bool(False)
        self._srv_en_pub.publish(msg)

    def set_servo_pulse(self, channel: int, pulse_us: int) -> None:
        """Send a raw pulse-width command (µs) to a servo channel."""
        msg = ServoSet()
        msg.channel = self._servo_channel_to_firmware(channel)
        msg.pulse_us = int(pulse_us)
        self._srv_set_pub.publish(msg)

    # ── Public gripper API ────────────────────────────────────────────────────

    def open_gripper(
        self,
        channel: int = GRIPPER_CHANNEL,
        pulse_us: int = GRIPPER_OPEN_US,
    ) -> None:
        """
        Open the gripper claw.

        Args:
            channel:  Servo channel (1-based, default 1 = claw).
            pulse_us: Pulse width in µs for the open position.
        """
        self.enable_servo(channel)
        self.set_servo_pulse(channel, pulse_us)

    def close_gripper(
        self,
        channel: int = GRIPPER_CHANNEL,
        pulse_us: int = GRIPPER_CLOSED_US,
    ) -> None:
        """
        Close the gripper claw to a fixed position.

        Args:
            channel:  Servo channel (1-based, default 1 = claw).
            pulse_us: Pulse width in µs for the closed position.
        """
        self.enable_servo(channel)
        self.set_servo_pulse(channel, pulse_us)

    def squeeze_until_resistance(
        self,
        channel: int = GRIPPER_CHANNEL,
        start_pulse_us: int = GRIPPER_OPEN_US,
        max_pulse_us: int = GRIPPER_CLOSED_US,
        step_us: int = GRIPPER_SQUEEZE_STEP,
        step_delay_s: float = GRIPPER_SQUEEZE_DELAY,
        resistance_pin: int = GRIPPER_RESISTANCE_PIN,
        resistance_threshold: int = GRIPPER_RESISTANCE_THRESHOLD,
    ) -> int:
        """
        Incrementally close the gripper until resistance is detected on the
        analog pin, or until max_pulse_us is reached.

        Returns the pulse_us at which resistance was detected (or max_pulse_us
        if the threshold was never crossed).

        Args:
            channel:              Servo channel (1-based).
            start_pulse_us:       Starting (open) pulse width in µs.
            max_pulse_us:         Maximum (fully closed) pulse width in µs.
            step_us:              µs to increment per squeeze step.
            step_delay_s:         Seconds to wait between steps.
            resistance_pin:       Arduino analog pin index (0-based A-pin number).
            resistance_threshold: ADC value (0–1023) above which grip is detected.

        Returns:
            int: Final pulse_us when resistance was detected or max was reached.
        """
        self.enable_servo(channel)
        pulse = start_pulse_us

        while pulse <= max_pulse_us:
            self.set_servo_pulse(channel, pulse)
            time.sleep(step_delay_s)

            # Try to read the resistance sensor from cached IO state.
            adc_value = self._read_analog_pin(resistance_pin)
            if adc_value is not None:
                if adc_value >= resistance_threshold:
                    return pulse
            else:
                # Sensor not available — log once and fall back to timed close.
                if pulse == start_pulse_us:
                    try:
                        self._node.get_logger().warn(
                            f"[Gripper] Analog pin A{resistance_pin} not available "
                            f"in io_input_state — falling back to timed squeeze."
                        )
                    except Exception:
                        pass

            pulse += step_us

        return max_pulse_us

    def _read_analog_pin(self, pin_index: int) -> int | None:
        """
        Read an analog pin value from the cached IOInputState.

        IOInputState.analog_values is a list of uint16 ADC counts (0–1023).
        Returns None if the state hasn't arrived yet or pin is out of range.
        """
        with self._lock:
            io = getattr(self, '_io_input_state', None)
        if io is None:
            return None
        analog = getattr(io, 'analog_values', None)
        if analog is None or pin_index >= len(analog):
            return None
        return int(analog[pin_index])

    # ── Burger pick sequence ──────────────────────────────────────────────────

    def burger_pick(
        self,
        height_steps: int,
        stepper: int = 1,
        gripper_channel: int = GRIPPER_CHANNEL,
        open_pulse_us: int = GRIPPER_OPEN_US,
        squeeze_max_us: int = GRIPPER_CLOSED_US,
        lift_steps: int | None = None,
        resistance_threshold: int = GRIPPER_RESISTANCE_THRESHOLD,
        pre_squeeze_delay_s: float = 0.3,
        post_squeeze_delay_s: float = 0.3,
    ) -> None:
        """
        Full burger-piece pick sequence:
            1. Open the gripper.
            2. Drop the arm to height_steps (absolute stepper move).
            3. Wait for the arm to settle.
            4. Squeeze until resistance detected (or max pulse reached).
            5. Wait for grip to firm up.
            6. Lift the arm back to 0 (or lift_steps if provided).

        Args:
            height_steps:        Absolute stepper target in steps (drop height).
            stepper:             Stepper motor number (1-based, default 1).
            gripper_channel:     Servo channel for the claw (1-based, default 1).
            open_pulse_us:       Pulse width µs for open position.
            squeeze_max_us:      Maximum pulse width µs for squeeze.
            lift_steps:          Steps to lift to after grabbing (default: 0).
            resistance_threshold: ADC threshold for grip detection.
            pre_squeeze_delay_s: Seconds to wait after arm drop before squeezing.
            post_squeeze_delay_s: Seconds to wait after squeeze before lifting.
        """
        if lift_steps is None:
            lift_steps = 0

        logger = None
        try:
            logger = self._node.get_logger()
        except Exception:
            pass

        def log(msg: str) -> None:
            if logger:
                logger.info(f"[burger_pick] {msg}")

        # Step 1 — open gripper
        log("Opening gripper...")
        self.open_gripper(channel=gripper_channel, pulse_us=open_pulse_us)
        time.sleep(0.2)

        # Step 2 — enable stepper and drop to height
        log(f"Enabling stepper {stepper}, dropping to {height_steps} steps...")
        self._publish_step_enable(stepper, enable=True)
        self._publish_step_move_absolute(stepper, height_steps)

        # Step 3 — wait for arm to settle (poll stepper state or use fixed delay)
        self._wait_for_stepper_idle(stepper, timeout_s=10.0)
        time.sleep(pre_squeeze_delay_s)

        # Step 4 — squeeze until resistance
        log("Squeezing until resistance...")
        final_pulse = self.squeeze_until_resistance(
            channel=gripper_channel,
            start_pulse_us=open_pulse_us,
            max_pulse_us=squeeze_max_us,
            resistance_threshold=resistance_threshold,
        )
        log(f"Grip detected at {final_pulse} µs.")

        # Step 5 — hold grip briefly
        time.sleep(post_squeeze_delay_s)

        # Step 6 — lift arm
        log(f"Lifting to {lift_steps} steps...")
        self._publish_step_move_absolute(stepper, lift_steps)
        self._wait_for_stepper_idle(stepper, timeout_s=10.0)
        log("Pick complete.")

    # ── Stepper helpers (thin wrappers around existing publishers) ────────────

    def _publish_step_enable(self, stepper: int, enable: bool) -> None:
        msg = StepEnable()
        msg.stepper_number = stepper
        msg.enable = bool(enable)
        self._step_en_pub.publish(msg)

    def _publish_step_move_absolute(self, stepper: int, target_steps: int) -> None:
        """
        Publish an absolute-position stepper move.
        StepMoveType.ABSOLUTE = 0  (from hardware_map.py StepMoveType enum).
        """
        msg = StepMove()
        msg.stepper_number = stepper
        msg.move_type = 0   # ABSOLUTE
        msg.target = int(target_steps)
        self._step_mv_pub.publish(msg)

    def _wait_for_stepper_idle(
       self,
       stepper: int,
       timeout_s: float = 10.0,
       poll_interval_s: float = 0.05,
    ) -> bool:
         deadline = time.monotonic() + timeout_s
         stepper_idx = stepper - 1

         while time.monotonic() < deadline:
            with self._lock:
               step_state = self._step_state

            if step_state is not None:
               steppers = getattr(step_state, 'steppers', None)
               if steppers and stepper_idx < len(steppers):
                  if int(steppers[stepper_idx].motion_state) == 0:
                    return True

            time.sleep(poll_interval_s)

         try:
            self._node.get_logger().warn(
              f"[Gripper] Timed out waiting for stepper {stepper} to become idle."
            )
         except Exception:
            pass
         return False
