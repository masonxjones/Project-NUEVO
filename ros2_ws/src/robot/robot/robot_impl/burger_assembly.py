"""
robot/robot_impl/burger_assembly.py
════════════════════════════════════
BurgerAssemblyMixin — hardcoded sequential burger assembly.

Drive approach (parallel-park to table on the LEFT)
────────────────────────────────────────────────────
  The robot is travelling forward with the table on its LEFT side.
  When assemble_burger() is called the robot is approximately beside
  piece #1 (top bun).  The approach sequence is:

    1. Rotate LEFT  (+angular)  by APPROACH_TURN_DEG  degrees   ← adjust to aim at table
    2. Drive straight forward   APPROACH_STRAFE_MM  mm          ← adjust to reach table edge
    3. Rotate RIGHT (−angular)  by APPROACH_TURN_DEG  degrees   ← straighten up (parallel)

  This gives a "parallel park" motion without needing lateral drive.
  After assembly the robot reverses the manoeuvre to re-join the path.

  ┌─────────────────────────────────────────────────────────┐
  │  TUNING GUIDE  (search for "← TUNE" to find every knob) │
  │                                                          │
  │  APPROACH_TURN_DEG   — bigger  → turns more toward table │
  │  APPROACH_STRAFE_MM  — bigger  → ends up closer to table │
  │  APPROACH_TURN_SPEED — deg/s for the approach turns      │
  │  DRIVE_SPEED_MM_S    — speed between burger pieces       │
  │  PIECE_SPACING_MM    — mm between top-bun / patty / bun  │
  │  ELEVATOR_PICK_STEPS — steps down to grip a piece        │
  │  ELEVATOR_PLACE_BUN  — steps down to drop bun onto patty │
  │  ELEVATOR_PLACE_STACK— steps down to drop bun+patty      │
  │  GRIP_US             — servo µs to close gripper         │
  └─────────────────────────────────────────────────────────┘

All positions are absolute stepper steps.
"""

from __future__ import annotations

import math
import time

from bridge_interfaces.msg import StepEnable, StepHome, StepMove


# ════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS
# ════════════════════════════════════════════════════════════════════════════

# ── Stepper numbers (1-based) ─────────────────────────────────────────────
STEPPER_ELEVATOR        = 1
STEPPER_SWING           = 2

# ── Elevator positions (steps — MORE steps = LOWER) ──────────────────────
ELEVATOR_HOME           = 0       # fully raised; safe to drive and swing
ELEVATOR_PICK_STEPS     = 4600    # ← TUNE: depth to grip any single piece
ELEVATOR_PLACE_BUN      = 3000    # ← TUNE: depth to drop bun onto patty
ELEVATOR_PLACE_STACK    = 2000    # ← TUNE: depth to drop bun+patty onto bottom bun

# ── Swing positions ───────────────────────────────────────────────────────
SWING_HOME              = 0       # parked — arm clear of lidar, MUST be 0
SWING_PICK_STEPS        = -420    # ← TUNE: arm over the table for all picks/places

# ── Grip ─────────────────────────────────────────────────────────────────
GRIP_US                 = 1500    # ← TUNE: servo µs to close gripper
GRIPPER_OPEN_US         = 900     # servo µs to open gripper
GRIPPER_CHANNEL         = 16      # 1-based servo channel

# ── Piece spacing (distance between burger pieces on the table) ───────────
PIECE_SPACING_MM        = 150.0   # ← TUNE: centre-to-centre between pieces

# ── Drive speeds ──────────────────────────────────────────────────────────
DRIVE_SPEED_MM_S        = 80.0    # ← TUNE: forward speed between pieces

# ── Parallel-park approach (robot travels forward; table is on the LEFT) ──
#
#   The robot does a 3-move "parallel park" to get beside the table:
#     1. Rotate left  by APPROACH_TURN_DEG   (points robot toward table)
#     2. Drive forward APPROACH_STRAFE_MM    (closes gap to table)
#     3. Rotate right by APPROACH_TURN_DEG   (straightens parallel to table)
#
#   To move FARTHER LEFT  → increase APPROACH_STRAFE_MM
#   To turn MORE toward table first → increase APPROACH_TURN_DEG
#   To straighten MORE after → APPROACH_TURN_DEG is used symmetrically,
#       so adjust APPROACH_TURN_DEG alone.
#
APPROACH_TURN_DEG       = 30.0    # ← TUNE: degrees to rotate toward table
APPROACH_STRAFE_MM      = 101.6   # ← TUNE: ~4 inches in mm; distance driven toward table
APPROACH_TURN_SPEED_DPS = 40.0    # ← TUNE: deg/s for approach/depart turns

# ── Timing ────────────────────────────────────────────────────────────────
STEPPER_MOVE_TIMEOUT_S  = 20.0
STEPPER_SETTLE_S        = 0.15
GRIP_SETTLE_S           = 0.25

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

    Requires in Robot.__init__:
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

    # ── Parallel-park approach / depart ──────────────────────────────────────

    def _approach_table(self) -> None:
        """
        Move the robot from its path position to beside the table (left side).

        Motion:
          1. Rotate LEFT  by APPROACH_TURN_DEG   (← TUNE to aim at table)
          2. Drive forward APPROACH_STRAFE_MM    (← TUNE distance toward table)
          3. Rotate RIGHT by APPROACH_TURN_DEG   (← TUNE to re-parallelise)

        After this the robot is parallel to the table, ~APPROACH_STRAFE_MM
        closer to it, ready for the arm to reach pieces.
        """
        self._arm_log("── Approach: parallel-park toward table (LEFT) ──")
        self._point_turn(+APPROACH_TURN_DEG)     # ← TUNE APPROACH_TURN_DEG
        self._drive_forward(APPROACH_STRAFE_MM)  # ← TUNE APPROACH_STRAFE_MM
        self._point_turn(-APPROACH_TURN_DEG)     # mirrors the first turn

    def _depart_table(self) -> None:
        """
        Reverse the parallel-park to re-join the forward path.

        Motion:
          1. Rotate LEFT  by APPROACH_TURN_DEG   (point away from table)
          2. Drive backward APPROACH_STRAFE_MM   (back to original path line)
          3. Rotate RIGHT by APPROACH_TURN_DEG   (face forward again)

        After this the robot is back on the path line, facing forward.
        """
        self._arm_log("── Depart: parallel-park back to path ──")
        self._point_turn(+APPROACH_TURN_DEG)      # point away from table
        self._drive_backward(APPROACH_STRAFE_MM)  # back to path line
        self._point_turn(-APPROACH_TURN_DEG)      # face forward

    # ── Homing ────────────────────────────────────────────────────────────────

    def arm_home(self) -> None:
        """Home both axes to limit switches. Optional at startup."""
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

        The robot stops on the path when this is called.  It then:
          • Parallel-parks left to reach the table
          • Picks top bun, drives to patty, stacks them, drives to bottom bun,
            assembles full stack
          • MANDATORY swing back to 0 before every drive move
          • Parallel-parks back to path line
          • Returns — path planner resumes from current position

        Elevator positions used
        ───────────────────────
          ELEVATOR_PICK_STEPS  (4600) — grip any single piece
          ELEVATOR_PLACE_BUN   (3000) — drop bun onto patty
          ELEVATOR_PLACE_STACK (2000) — drop bun+patty onto bottom bun
          ELEVATOR_HOME        (0)    — raised / safe to drive

        Args:
            home_on_start: Home both axes first if position is unknown.
        """
        self._arm_log("═══ Burger assembly START ═══")

        # Enable both steppers
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        if home_on_start:
            self.arm_home()

        # ── Approach: move robot beside the table ────────────────────────────
        # Swing MUST be at home before any driving
        self._arm_swing_home()
        self._approach_table()

        # ── STEP 1: Pick TOP BUN ─────────────────────────────────────────────
        self._arm_log("── Step 1/4: Pick TOP BUN ──")

        self.arm_swing_to(SWING_PICK_STEPS)           # swing out ONCE at the start
        self.arm_open_gripper()                        # open over piece
        time.sleep(0.5)
        self.arm_elevator_to(ELEVATOR_PICK_STEPS)     # lower to 4600
        self.arm_grip()                                # grip bun
        time.sleep(1)
        self.arm_elevator_to(ELEVATOR_HOME)            # raise to 0 — swing stays out
        self._arm_log("Top bun picked ✓")
        time.sleep(1)

# ── STEP 2: Drive to patty, drop bun, pick bun + patty ───────────────
        self._arm_log("── Step 2/4: Drive to patty → drop bun → pick bun+patty ──")

        self._drive_forward(PIECE_SPACING_MM)         # elevator=0, swing still at -420

        # no arm_swing_to here — arm is already over the table
        self.arm_elevator_to(ELEVATOR_PLACE_BUN)      # lower to 3000
        time.sleep(0.3)
        self.arm_open_gripper()                        # drop bun onto patty
        time.sleep(0.5)
        self.arm_elevator_to(ELEVATOR_PICK_STEPS)     # lower to 4600
        self.arm_grip()                                # grip bun + patty
        time.sleep(0.5)
        self.arm_elevator_to(ELEVATOR_HOME)            # raise to 0 — swing stays out
        self._arm_log("Bun + patty picked ✓")
        time.sleep(1)

# ── STEP 3: Drive to bottom bun, drop stack, pick full burger ─────────
        self._arm_log("── Step 3/4: Drive to bottom bun → drop stack → pick full stack ──")

        self._drive_forward(PIECE_SPACING_MM)         # elevator=0, swing still at -420

        # no arm_swing_to here — arm is already over the table
        self.arm_elevator_to(ELEVATOR_PLACE_STACK)    # lower to 2000
        self.arm_open_gripper()                        # drop bun+patty onto bottom bun
        time.sleep(0.3)
        self.arm_elevator_to(ELEVATOR_PICK_STEPS)     # lower to 4600
        self.arm_grip()                                # grip full stack
        self.arm_elevator_to(ELEVATOR_HOME)            # raise to 0

# ── STEP 4: Swing home ONCE, then depart ─────────────────────────────
        self._arm_log("── Step 4/4: Swing home → depart table → re-join path ──")

        self._arm_swing_home()                         # swing back to 0 — only time it returns
        self._depart_table()

        self._arm_log("═══ Burger assembly COMPLETE ✓ ═══")
        self._arm_log("Full burger stack in gripper — path planner may resume.")