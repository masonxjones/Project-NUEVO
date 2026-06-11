"""
robot/robot_impl/burger_delivery.py
════════════════════════════════════
BurgerDeliveryMixin — hardcoded burger delivery to Customer A or B.

How it works
────────────
  Robot is holding the full burger stack in the gripper and is on the path
  when deliver_burger() is called.

  For each customer:
    1. Drive forward a fixed distance to align with that customer's drop zone
    2. Swing arm to SWING_PICK_STEPS (-420) — same as assembly, toward table
    3. Lower elevator to ELEVATOR_PICK_STEPS (4600) — down to drop height
    4. Open gripper — release burger
    5. Raise elevator to ELEVATOR_HOME (0)
    6. Swing arm back to SWING_HOME (0)
    7. Return — path planner resumes

  ┌──────────────────────────────────────────────────────────────┐
  │  TUNING GUIDE  (search "← TUNE" to find every knob)         │
  │                                                              │
  │  CUSTOMER_A_DRIVE_MM  — how far forward to drive for A      │
  │  CUSTOMER_B_DRIVE_MM  — how far forward to drive for B      │
  │  DELIVERY_DROP_STEPS  — elevator depth to release burger     │
  │  DELIVERY_SWING_STEPS — arm angle for delivery (left = neg) │
  │  DELIVERY_SPEED_MM_S  — drive speed during delivery approach │
  └──────────────────────────────────────────────────────────────┘

Place this file at:
    ros2_ws/src/robot/robot/robot_impl/burger_delivery.py

Add BurgerDeliveryMixin to your Robot class MRO alongside BurgerAssemblyMixin.

Call from main.py:
    robot.deliver_burger(customer=detected_customer)
where detected_customer is "A", "B", or "unknown".
"""

from __future__ import annotations

import time
import math
# Re-use the same arm constants as assembly so nothing goes out of sync
from robot.robot_impl.burger_assembly import (
    SWING_HOME,
    SWING_PICK_STEPS,
    ELEVATOR_HOME,
    ELEVATOR_PICK_STEPS,
    GRIPPER_OPEN_US,
    GRIPPER_CHANNEL,
    GRIP_US,
    STEPPER_ELEVATOR,
    STEPPER_SWING,
    STEPPER_MOVE_TIMEOUT_S,
    STEPPER_SETTLE_S,
    GRIP_SETTLE_S,
)


# ════════════════════════════════════════════════════════════════════════════
# DELIVERY-SPECIFIC TUNABLE CONSTANTS
# ════════════════════════════════════════════════════════════════════════════

# ── How far to drive forward to reach each customer's drop zone ───────────
CUSTOMER_A_DRIVE_MM     = 1435.0   # ← TUNE: mm to drive forward for Customer A
CUSTOMER_B_DRIVE_MM     = 1535.0   # ← TUNE: mm to drive forward for Customer B
CUSTOMER_UNKNOWN_DRIVE_MM = 1900.0 # ← TUNE: fallback if customer not identified

# ── Arm position for delivery (mirrors assembly) ──────────────────────────
DELIVERY_SWING_STEPS    = SWING_PICK_STEPS   # -420 — same side as assembly table
                                              # ← TUNE if delivery side differs

# ── Elevator depth to release burger ─────────────────────────────────────
DELIVERY_DROP_STEPS     = 4300  # 4600 ← TUNE: lower = deeper drop

# ── Drive speed during delivery approach ──────────────────────────────────
DELIVERY_SPEED_MM_S     = 20.0    # ← TUNE: mm/s


# ════════════════════════════════════════════════════════════════════════════
# MIXIN
# ════════════════════════════════════════════════════════════════════════════

class BurgerDeliveryMixin:
    """
    Hardcoded burger delivery for the Robot class.

    Requires BurgerAssemblyMixin already in the MRO (shares arm primitives).
    Requires GripperMixin in the MRO for open_gripper / close_gripper.
    """

    # ── Internal logging (mirrors assembly style) ─────────────────────────────
    
    def _arm_log(self, msg: str) -> None:
        try:
            self._node.get_logger().info(f"[BurgerArm] {msg}")
        except Exception:
            print(f"[BurgerArm] {msg}")

    def _arm_warn(self, msg: str) -> None:
        try:
            self._node.get_logger().warn(f"[BurgerArm] {msg}")
        except Exception:
            print(f"[BurgerArm] WARN: {msg}")

    # ── Stepper helpers ───────────────────────────────────────────────────────

    def _step_enable(self, stepper: int, enable: bool) -> None:
        msg = StepEnable()
        msg.stepper_number = stepper
        msg.enable = bool(enable)
        self._step_en_pub.publish(msg)

    def _step_move_abs(self, stepper: int, target_steps: int) -> None:
        msg = StepMove()
        msg.stepper_number = stepper
        msg.move_type = 0   # ABSOLUTE
        msg.target = int(target_steps)
        self._step_mv_pub.publish(msg)

    def _step_home_cmd(self, stepper, direction, velocity, backoff) -> None:
        msg = StepHome()
        msg.stepper_number = stepper
        msg.direction = direction
        msg.home_velocity = velocity
        msg.backoff_steps = backoff
        self._step_hm_pub.publish(msg)

    def _wait_stepper(
        self,
        stepper: int,
        timeout_s: float = STEPPER_MOVE_TIMEOUT_S,
        poll_s: float = 0.05,
    ) -> bool:
        """Block until stepper reports motion_state == 0 (idle), or timeout."""
        deadline = time.monotonic() + timeout_s
        idx = stepper - 1
        while time.monotonic() < deadline:
            with self._lock:
                step_state = self._step_state
            if step_state is not None:
                steppers = getattr(step_state, 'steppers', None)
                if steppers and idx < len(steppers):
                    if int(steppers[idx].motion_state) == 0:
                        return True
            time.sleep(poll_s)
        self._arm_warn(f"Timeout waiting for stepper {stepper}.")
        return False

    def _move_and_wait(
        self,
        stepper: int,
        steps: int,
        settle_s: float = STEPPER_SETTLE_S,
    ) -> bool:
        self._step_move_abs(stepper, steps)
        ok = self._wait_stepper(stepper)
        if settle_s > 0:
            time.sleep(settle_s)
        return ok

    # ── Arm primitives ────────────────────────────────────────────────────────

    def arm_elevator_to(self, steps: int) -> bool:
        self._arm_log(f"Elevator → {steps} steps")
        return self._move_and_wait(STEPPER_ELEVATOR, steps)

    def arm_swing_to(self, steps: int) -> bool:
        self._arm_log(f"Swing → {steps} steps")
        return self._move_and_wait(STEPPER_SWING, steps)

    def arm_safe_height(self) -> bool:
        """Raise elevator to home (fully raised, safe to drive)."""
        return self.arm_elevator_to(ELEVATOR_HOME)

    def arm_open_gripper(self) -> None:
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    def arm_grip(self) -> None:
        """Close gripper at the universal grip pulse."""
        self.close_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIP_US)
        time.sleep(GRIP_SETTLE_S)

    def _arm_swing_home(self) -> None:
        """
        MANDATORY: return swing to SWING_HOME (0) before any drive move.
        Clears the arm from the lidar cone.
        """
        self._arm_log("Swing → HOME (0) [mandatory before drive]")
        self.arm_swing_to(SWING_HOME)

    # ── Drive helpers ─────────────────────────────────────────────────────────

    def _drive_forward(self, distance_mm: float) -> None:
        """Drive straight forward by distance_mm (timed velocity control)."""
        self._arm_log(f"Drive forward {distance_mm:.1f} mm")
        try:
            self.set_velocity(DRIVE_SPEED_MM_S, 0.0)
            time.sleep(distance_mm / DRIVE_SPEED_MM_S)
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._arm_warn(f"Drive error: {e}")

    def _drive_backward(self, distance_mm: float) -> None:
        """Drive straight backward by distance_mm."""
        self._arm_log(f"Drive backward {distance_mm:.1f} mm")
        try:
            self.set_velocity(-DRIVE_SPEED_MM_S, 0.0)
            time.sleep(distance_mm / DRIVE_SPEED_MM_S)
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._arm_warn(f"Drive error: {e}")

    def _point_turn(self, degrees: float) -> None:
        """
        Rotate in place by 'degrees'.
        Positive = LEFT (counter-clockwise), negative = RIGHT (clockwise).

        Uses timed angular velocity.  If your robot cannot point-turn, replace
        the body of this method with a small-radius arc instead.

        ← TUNE APPROACH_TURN_SPEED_DPS to change how fast this turn is.
        """
        if abs(degrees) < 0.5:
            return
        self._arm_log(f"Point turn {degrees:+.1f}°")
        rad_s = math.radians(APPROACH_TURN_SPEED_DPS)
        duration = abs(math.radians(degrees)) / rad_s
        direction = 1.0 if degrees > 0 else -1.0
        try:
            self.set_velocity(0.0, direction * rad_s)
            time.sleep(duration)
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._arm_warn(f"Turn error: {e}")

    def _delivery_log(self, msg: str) -> None:
        try:
            self._node.get_logger().info(f"[BurgerDelivery] {msg}")
        except Exception:
            print(f"[BurgerDelivery] {msg}")

    def _delivery_warn(self, msg: str) -> None:
        try:
            self._node.get_logger().warn(f"[BurgerDelivery] {msg}")
        except Exception:
            print(f"[BurgerDelivery] WARN: {msg}")

    # ── Drive helper ──────────────────────────────────────────────────────────

    def _delivery_drive_forward(self, distance_mm: float) -> None:
        """Drive straight forward by distance_mm at DELIVERY_SPEED_MM_S."""
        self._delivery_log(f"Driving forward {distance_mm:.1f} mm to drop zone...")
        try:
            self.set_velocity(DELIVERY_SPEED_MM_S, 0.0)
            import time as _t
            _t.sleep(distance_mm / DELIVERY_SPEED_MM_S)
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._delivery_warn(f"Drive error: {e}")

    # ════════════════════════════════════════════════════════════════════════
    # MAIN DELIVERY METHOD
    # ════════════════════════════════════════════════════════════════════════

    def deliver_burger(self, customer: str = "unknown") -> None:
        """
        Drive to the correct customer drop zone and release the burger.

        Args:
            customer: "A", "B", or "unknown" — set by your teammate's
                      vision/detection code in main.py.

        Sequence
        ────────
          1. Choose drive distance based on customer
          2. Drive forward to drop zone (arm at home — elevator=0, swing=0)
          3. Swing arm to DELIVERY_SWING_STEPS (-420)
          4. Lower elevator to DELIVERY_DROP_STEPS (4600)
          5. Open gripper — burger released
          6. Raise elevator to ELEVATOR_HOME (0)
          7. Swing arm back to SWING_HOME (0)
          8. Return — path planner resumes from current position
        """
        customer = str(customer).strip().upper()

        # ── Step 1: Choose distance ───────────────────────────────────────────
        if customer == "A":
            drive_mm = CUSTOMER_A_DRIVE_MM      # ← TUNE CUSTOMER_A_DRIVE_MM
            self._delivery_log(f"Customer A — driving {drive_mm:.0f} mm to drop zone")
        elif customer == "B":
            drive_mm = CUSTOMER_B_DRIVE_MM      # ← TUNE CUSTOMER_B_DRIVE_MM
            self._delivery_log(f"Customer B — driving {drive_mm:.0f} mm to drop zone")
        else:
            drive_mm = CUSTOMER_UNKNOWN_DRIVE_MM  # ← TUNE CUSTOMER_UNKNOWN_DRIVE_MM
            self._delivery_log(
                f"Customer unknown — using fallback distance {drive_mm:.0f} mm"
            )

        self._delivery_log("═══ Burger delivery START ═══")

        # Ensure steppers are enabled
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        # ── Step 2: Drive forward to drop zone ───────────────────────────────
        # Arm must be at home (elevator=0, swing=0) before driving
        self._delivery_log("── Step 1/5: Drive to customer drop zone ──")
        self._delivery_drive_forward(drive_mm)

        # ── Step 3: Swing arm toward customer (left side, same as assembly) ───
        self._delivery_log(
            f"── Step 2/5: Swing arm to {DELIVERY_SWING_STEPS} steps ──"
        )
        self.arm_swing_to(DELIVERY_SWING_STEPS)   # ← TUNE DELIVERY_SWING_STEPS

        # ── Step 4: Lower elevator to drop height ─────────────────────────────
        self._delivery_log(
            f"── Step 3/5: Lower elevator to {DELIVERY_DROP_STEPS} steps ──"
        )
        self.arm_elevator_to(DELIVERY_DROP_STEPS)  # ← TUNE DELIVERY_DROP_STEPS

        # ── Step 5: Release burger ────────────────────────────────────────────
        self._delivery_log("── Step 4/5: Open gripper — releasing burger ──")
        self.arm_open_gripper()
        time.sleep(0.4)   # let burger settle before lifting away

        # ── Step 6: Raise elevator ────────────────────────────────────────────
        self._delivery_log("── Step 5/5: Raise elevator to home ──")
        self.arm_elevator_to(ELEVATOR_HOME)
        time.sleep(2.0)

        # ── Step 7: Swing arm back to home ────────────────────────────────────
        self._delivery_log("Swing arm back to home (0)...")
        self.arm_swing_to(SWING_HOME)   # MANDATORY before path planner resumes

        self._delivery_log("═══ Burger delivery COMPLETE ✓ ═══")
        self._delivery_log("Path planner may resume.")
