"""
robot/robot_impl/burger_assembly.py
════════════════════════════════════
BurgerAssemblyMixin — hardcoded sequential burger assembly.

Full sequence
─────────────
  Robot is already positioned beside the table when assemble_burger() is called.

  Step 1 — Swing to SWING_PICK_STEPS (-420), lower to ELEVATOR_PICK_STEPS (4600)
            Close gripper to GRIP_US (1500), lift to ELEVATOR_HOME (0)
            → picked TOP BUN

  Step 2 — Drive forward PIECE_SPACING_MM (150) to patty position
            Swing to SWING_PICK_STEPS, lower to ELEVATOR_PLACE_BUN (3000)
            Open gripper (drop bun on patty position)
            Lower to ELEVATOR_PICK_STEPS (4600)
            Close gripper (grip bun + patty together)
            Lift to ELEVATOR_HOME (0)
            → picked BUN + PATTY stack

  Step 3 — Drive forward PIECE_SPACING_MM (150) to bottom bun position
            Swing to SWING_PICK_STEPS, lower to ELEVATOR_PLACE_STACK (3000)
            Open gripper (release bun + patty onto bottom bun)
            Lower to ELEVATOR_PICK_STEPS (4600)
            Close gripper (grip full stack)
            Lift to ELEVATOR_HOME (0)
            → picked FULL STACK (bottom bun + patty + top bun)

  Step 4 — Return swing and elevator to home (0, 0)
            Navigation team resumes path from here.

All positions are absolute stepper steps.
"""

from __future__ import annotations

import time
from typing import Optional

from bridge_interfaces.msg import StepEnable, StepHome, StepMove


# ════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS — edit these to match your robot
# ════════════════════════════════════════════════════════════════════════════

# ── Stepper numbers (1-based) ─────────────────────────────────────────────
STEPPER_ELEVATOR        = 1
STEPPER_SWING           = 2

# ── Positions (absolute steps) ────────────────────────────────────────────
ELEVATOR_HOME           = 0       # fully raised — safe travel height
ELEVATOR_PICK_STEPS     = 4600    # depth to pick up any piece
ELEVATOR_PLACE_BUN      = 3000    # height to set bun down on patty
ELEVATOR_PLACE_STACK    = 3000    # height to set bun+patty down on bottom bun

SWING_HOME              = 0       # parked — clear of lidar
SWING_PICK_STEPS        = -420    # arm position over the table for all picks/places

# ── Grip ─────────────────────────────────────────────────────────────────
GRIP_US                 = 1500    # closing pulse for all pieces
GRIPPER_OPEN_US         = 900     # open pulse
GRIPPER_CHANNEL         = 16      # 1-based (firmware ch 15)

# ── Drive ─────────────────────────────────────────────────────────────────
PIECE_SPACING_MM        = 150.0   # centre-to-centre distance between pieces
DRIVE_SPEED_MM_S        = 80.0    # forward speed between pieces

# ── Timing ────────────────────────────────────────────────────────────────
STEPPER_MOVE_TIMEOUT_S  = 20.0
STEPPER_SETTLE_S        = 0.15
GRIP_SETTLE_S           = 0.25    # pause after closing gripper

# ── Homing ────────────────────────────────────────────────────────────────
ELEVATOR_HOME_VELOCITY  = 200
ELEVATOR_HOME_DIRECTION = 0
ELEVATOR_HOME_BACKOFF   = 50
SWING_HOME_VELOCITY     = 200
SWING_HOME_DIRECTION    = 0
SWING_HOME_BACKOFF      = 30


# ════════════════════════════════════════════════════════════════════════════
# MIXIN
# ════════════════════════════════════════════════════════════════════════════

class BurgerAssemblyMixin:
    """
    Hardcoded sequential burger assembly for the Robot class.

    Requires in Robot.__init__ (already present):
        self._step_en_pub, self._step_mv_pub, self._step_hm_pub
        self._step_state   (cached StepStateAll)
        self._lock, self._node
    Also requires GripperMixin in the MRO for open_gripper / close_gripper.
    """

    # ── Logging ──────────────────────────────────────────────────────────────

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
        """Raise elevator to home (fully raised)."""
        return self.arm_elevator_to(ELEVATOR_HOME)

    def arm_open_gripper(self) -> None:
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    def arm_grip(self) -> None:
        """Close gripper at the universal grip pulse."""
        self.close_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIP_US)
        time.sleep(GRIP_SETTLE_S)

    # ── Drive helper ──────────────────────────────────────────────────────────

    def _drive_forward(self, distance_mm: float) -> None:
        """Drive straight forward by distance_mm using timed velocity control."""
        self._arm_log(f"Driving forward {distance_mm:.0f} mm...")
        try:
            self.set_velocity(DRIVE_SPEED_MM_S, 0.0)
            time.sleep(distance_mm / DRIVE_SPEED_MM_S)
            self.stop()
            time.sleep(0.2)
        except Exception as e:
            self._arm_warn(f"Drive error: {e}")

    # ── Homing ────────────────────────────────────────────────────────────────

    def arm_home(self) -> None:
        """Home both axes to their limit switches. Optional at startup."""
        self._arm_log("Homing elevator...")
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_home_cmd(STEPPER_ELEVATOR, ELEVATOR_HOME_DIRECTION,
                            ELEVATOR_HOME_VELOCITY, ELEVATOR_HOME_BACKOFF)
        self._wait_stepper(STEPPER_ELEVATOR, timeout_s=30.0)
        self._arm_log("Homing swing...")
        self._step_enable(STEPPER_SWING, True)
        self._step_home_cmd(STEPPER_SWING, SWING_HOME_DIRECTION,
                            SWING_HOME_VELOCITY, SWING_HOME_BACKOFF)
        self._wait_stepper(STEPPER_SWING, timeout_s=30.0)
        self._arm_log("Homing complete.")

    # ════════════════════════════════════════════════════════════════════════
    # FULL ASSEMBLY SEQUENCE
    # ════════════════════════════════════════════════════════════════════════

    def assemble_burger(
        self,
        home_on_start: bool = False,
    ) -> None:
        """
        Full hardcoded burger assembly sequence.

        Robot must already be positioned beside the table before calling this.

        Sequence
        ────────
          Step 1: Pick top bun
                  swing=-420, elevator=4600, grip, lift=0

          Step 2: Drive 150mm to patty
                  swing=-420, elevator=3000, open (drop bun)
                  elevator=4600, grip (bun+patty), lift=0

          Step 3: Drive 150mm to bottom bun
                  swing=-420, elevator=3000, open (drop bun+patty)
                  elevator=4600, grip (full stack), lift=0

          Step 4: Return arm to home (swing=0, elevator=0)

        Args:
            home_on_start: Home both axes before starting (if position unknown).
        """
        self._arm_log("═══ Burger assembly START ═══")

        # Enable both steppers
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        if home_on_start:
            self.arm_home()

        # ── STEP 1: Pick top bun ──────────────────────────────────────────────
        self._arm_log("── Step 1/4: Pick TOP BUN ──")

        self.arm_open_gripper()
        self.arm_swing_to(SWING_PICK_STEPS)          # swing to -420
        self.arm_elevator_to(ELEVATOR_PICK_STEPS)    # lower to 4600
        self.arm_grip()                               # close at 1500 µs
        self.arm_elevator_to(ELEVATOR_HOME)           # lift to 0
        self._arm_log("Top bun picked ✓")

        # ── STEP 2: Drive to patty, stack bun on patty, pick both ─────────────
        self._arm_log("── Step 2/4: Drive to patty, drop bun, pick bun+patty ──")

        self._drive_forward(PIECE_SPACING_MM)         # drive 150mm

        self.arm_swing_to(SWING_PICK_STEPS)          # swing to -420
        self.arm_elevator_to(ELEVATOR_PLACE_BUN)     # lower to 3000
        self.arm_open_gripper()                       # drop bun onto patty
        time.sleep(0.3)                               # let bun settle

        self.arm_elevator_to(ELEVATOR_PICK_STEPS)    # lower further to 4600
        self.arm_grip()                               # grip bun + patty
        self.arm_elevator_to(ELEVATOR_HOME)           # lift to 0
        self._arm_log("Bun + patty picked ✓")

        # ── STEP 3: Drive to bottom bun, drop stack, pick full burger ─────────
        self._arm_log("── Step 3/4: Drive to bottom bun, drop stack, pick full burger ──")

        self._drive_forward(PIECE_SPACING_MM)         # drive 150mm

        self.arm_swing_to(SWING_PICK_STEPS)          # swing to -420
        self.arm_elevator_to(ELEVATOR_PLACE_STACK)   # lower to 3000
        self.arm_open_gripper()                       # drop bun+patty onto bottom bun
        time.sleep(0.3)                               # let stack settle

        self.arm_elevator_to(ELEVATOR_PICK_STEPS)    # lower to 4600
        self.arm_grip()                               # grip full stack
        self.arm_elevator_to(ELEVATOR_HOME)           # lift to 0
        self._arm_log("Full stack picked ✓")

        # ── STEP 4: Return arm to home ────────────────────────────────────────
        self._arm_log("── Step 4/4: Return arm to home ──")

        self.arm_swing_to(SWING_HOME)                # swing back to 0
        # elevator is already at 0

        self._arm_log("═══ Burger assembly COMPLETE ✓ ═══")
        self._arm_log("Full burger stack in gripper — ready for navigation.")
