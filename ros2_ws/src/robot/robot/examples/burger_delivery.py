# ros2_ws/src/robot/robot/robot_impl/burger_delivery.py

from __future__ import annotations
import time
import math

# Mirror working message types exactly
from bridge_interfaces.msg import StepEnable, StepHome, StepMove

# Re-use working arm and speed variables directly from assembly
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
    DRIVE_SPEED_MM_S,
    APPROACH_TURN_SPEED_DPS,
)

# ════════════════════════════════════════════════════════════════════════════
# DELIVERY-SPECIFIC TUNABLE CONSTANTS
# ════════════════════════════════════════════════════════════════════════════
CUSTOMER_A_DRIVE_MM       = 1435.0   
CUSTOMER_B_DRIVE_MM       = 1535.0   
CUSTOMER_UNKNOWN_DRIVE_MM = 1900.0 

DELIVERY_SWING_STEPS    = -420  # Matches your assembly side 
DELIVERY_DROP_STEPS     = 4300  # Depth to clear drop zone
DELIVERY_SPEED_MM_S     = 20.0  # Safe approach speed

class BurgerDeliveryMixin:
    """
    Hardcoded burger delivery utilizing identical shared hardware loops 
    as BurgerAssemblyMixin.
    """

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

    # ── Stepper Drivers (Identical to Working Assembly) ───────────────────────
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

    def _wait_stepper(
        self,
        stepper: int,
        timeout_s: float = STEPPER_MOVE_TIMEOUT_S,
        poll_s: float = 0.05,
    ) -> bool:
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

    def arm_elevator_to(self, steps: int) -> bool:
        self._arm_log(f"Elevator → {steps} steps")
        return self._move_and_wait(STEPPER_ELEVATOR, steps)

    def arm_swing_to(self, steps: int) -> bool:
        self._arm_log(f"Swing → {steps} steps")
        return self._move_and_wait(STEPPER_SWING, steps)

    def arm_open_gripper(self) -> None:
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    # ── Non-blocking Drive Control ───────────────────────────────────────────
    def _delivery_drive_forward(self, distance_mm: float) -> None:
        try:
            self.set_velocity(DELIVERY_SPEED_MM_S, 0.0)
            duration = distance_mm / DELIVERY_SPEED_MM_S
            
            # Use small slices to keep the ROS Executor pumping updates
            start_time = time.monotonic()
            while time.monotonic() - start_time < duration:
                time.sleep(0.02)
                
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._arm_warn(f"Drive error: {e}")

    # ════════════════════════════════════════════════════════════════════════
    # MAIN DELIVERY SEQUENCE
    # ════════════════════════════════════════════════════════════════════════
    def deliver_burger(self, customer: str = "unknown") -> None:
        customer = str(customer).strip().upper()

        if customer == "A":
            drive_mm = CUSTOMER_A_DRIVE_MM
        elif customer == "B":
            drive_mm = CUSTOMER_B_DRIVE_MM
        else:
            drive_mm = CUSTOMER_UNKNOWN_DRIVE_MM

        self._arm_log("═══ Burger delivery START ═══")

        # CRITICAL FIX: Explicitly awaken the stepper driver coils!
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)
        time.sleep(0.1)

        # Step 1: Drive to location
        self._delivery_drive_forward(drive_mm)

        # Step 2: Swing arm to target
        self.arm_swing_to(DELIVERY_SWING_STEPS)

        # Step 3: Drop down
        self.arm_elevator_to(DELIVERY_DROP_STEPS)

        # Step 4: Drop burger
        self._arm_log("Opening Gripper")
        self.arm_open_gripper()
        time.sleep(0.5)

        # Step 5: Clean up and raise home
        self.arm_elevator_to(ELEVATOR_HOME)
        self.arm_swing_to(SWING_HOME)

        self._arm_log("═══ Burger delivery COMPLETE ✓ ═══")
