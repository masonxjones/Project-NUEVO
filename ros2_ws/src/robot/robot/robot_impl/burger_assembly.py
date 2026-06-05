"""
robot/robot_impl/burger_assembly.py
════════════════════════════════════
BurgerAssemblyMixin — simplified sequential burger assembly.

Assembly sequence
─────────────────
  Step 1 — Camera confirms piece present → PICK TOP BUN
            (arm sweeps, finds anything, centres, grips, lifts)

  Step 2 — Drive forward DRIVE_TO_PATTY_MM
            Lower arm onto patty — now gripping (top bun + patty)
            Grip both together, lift

  Step 3 — Drive forward DRIVE_TO_STACK_MM
            Lower stack onto bottom bun, release, lift, done

One fixed grip pulse (GRIP_US = 1430) for all pieces.
Camera detection: any visible piece — "yellow_block" OR "red_block" OR
                  burger_type attribute set by classify_burger_piece().

Tunable constants are all at the top of this file.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from bridge_interfaces.msg import StepEnable, StepHome, StepMove


# ════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS
# ════════════════════════════════════════════════════════════════════════════

# ── Stepper numbers (1-based) ─────────────────────────────────────────────
STEPPER_ELEVATOR        = 1
STEPPER_SWING           = 2

# ── Elevator positions (steps — MORE steps = LOWER) ──────────────────────
ELEVATOR_SAFE_STEPS     = 0      # fully raised, safe to drive / swing
ELEVATOR_BUN_PICK       = 4000    # depth to reach the top bun
ELEVATOR_PATTY_PICK     = 850    # depth to reach the patty
                                 # (slightly lower — patty sits on table)
ELEVATOR_STACK_PLACE    = 600    # depth to lower stack onto bottom bun

# ── Swing positions (steps) ───────────────────────────────────────────────
SWING_SAFE_STEPS        = 0      # parked — does not block lidar
SWING_SCAN_START        = 100    # sweep begin
SWING_SCAN_END          = 900    # sweep end
SWING_SCAN_STEP         = 30     # steps per sweep increment
SWING_SCAN_PAUSE        = 0.15   # seconds to wait at each sweep position
SWING_STACK_STEPS       = 500    # swing position directly above the stack

# ── Drive distances (mm) ─────────────────────────────────────────────────
DRIVE_TO_PATTY_MM       = 150.0  # forward distance from bun to patty
DRIVE_TO_STACK_MM       = 150.0  # forward distance from patty to bottom bun
DRIVE_SPEED_MM_S        = 80.0   # speed for inter-pick drives

# ── Grip ─────────────────────────────────────────────────────────────────
GRIP_US                 = 1430   # single pulse for all pieces
GRIPPER_OPEN_US         = 900
GRIPPER_CHANNEL         = 16     # 1-based (firmware ch 15)

# ── Piece radii (mm) — kept for logging / future use ─────────────────────
PATTY_RADIUS_MM         = 50.0
BUN_RADIUS_MM           = 55.0

# ── Camera ────────────────────────────────────────────────────────────────
CAMERA_FRAME_WIDTH      = 640
PIXELS_TO_STEPS_GAIN    = 0.5    # tune: steps per pixel of X offset
ALIGNMENT_TOLERANCE_PX  = 20
ALIGNMENT_MAX_ITERS     = 10

# ── Timing ────────────────────────────────────────────────────────────────
STEPPER_MOVE_TIMEOUT_S  = 15.0
STEPPER_SETTLE_S        = 0.15

# ── Homing ────────────────────────────────────────────────────────────────
ELEVATOR_HOME_VELOCITY  = 200
ELEVATOR_HOME_DIRECTION = 0
ELEVATOR_HOME_BACKOFF   = 50
SWING_HOME_VELOCITY     = 200
SWING_HOME_DIRECTION    = 0
SWING_HOME_BACKOFF      = 30

# ── HSV tuned values (from detect_burger_pieces.py testing) ──────────────
BUN_YELLOW_LOW          = (18,  180, 150)
BUN_YELLOW_HIGH         = (38,  255, 255)


# ════════════════════════════════════════════════════════════════════════════
# MIXIN
# ════════════════════════════════════════════════════════════════════════════

class BurgerAssemblyMixin:
    """
    Sequential burger assembly mixin for the Robot class.

    Requires in Robot.__init__ (already present):
        self._step_en_pub, self._step_mv_pub, self._step_hm_pub
        self._step_state        (cached StepStateAll)
        self._vision_detections (cached list[dict] from /vision/detections)
        self._lock, self._node
    Also requires GripperMixin in the MRO.
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

    def _move_and_wait(self, stepper: int, steps: int,
                       settle_s: float = STEPPER_SETTLE_S) -> bool:
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
        return self.arm_elevator_to(ELEVATOR_SAFE_STEPS)

    def arm_open_gripper(self) -> None:
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    def arm_grip(self) -> None:
        """Close gripper at the universal grip pulse."""
        self.close_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIP_US)
        time.sleep(0.15)

    # ── Homing ────────────────────────────────────────────────────────────────

    def arm_home(self) -> None:
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

    # ── Camera helpers ────────────────────────────────────────────────────────

    def _get_any_piece_detection(self) -> Optional[dict]:
        """
        Return the highest-confidence detection that looks like a burger piece.
        Matches: yellow_block, red_block, or any detection with a burger_type
        attribute set by classify_burger_piece().
        """
        with self._lock:
            detections = list(self._vision_detections)

        best      = None
        best_conf = 0.0

        for d in detections:
            matched = d.get("class_name") in ("yellow_block", "red_block")
            if not matched:
                attrs = d.get("attributes", {})
                if attrs.get("burger_type") in ("patty", "bun"):
                    matched = True
            if matched:
                conf = float(d.get("confidence", 0.0))
                if conf > best_conf:
                    best_conf = conf
                    best = d

        return best

    def _detection_x_offset(self, detection: dict) -> float:
        bbox_centre_x = detection["x"] + detection["width"] / 2.0
        return bbox_centre_x - (CAMERA_FRAME_WIDTH / 2.0)

    # ── Sweep and align (any piece) ───────────────────────────────────────────

    def arm_sweep_and_align(self, label: str = "piece") -> Optional[int]:
        """
        Sweep arm from SWING_SCAN_START to SWING_SCAN_END until any burger
        piece is detected, then centre it in frame.

        Returns aligned swing step position, or None if nothing found.
        """
        self._arm_log(f"Sweeping for {label}...")
        current_swing = SWING_SCAN_START
        found = False

        while current_swing <= SWING_SCAN_END and current_swing <= 450:
            self.arm_swing_to(current_swing)
            time.sleep(SWING_SCAN_PAUSE)
            if self._get_any_piece_detection() is not None:
                self._arm_log(f"{label} detected at swing={current_swing}.")
                found = True
                break
            current_swing += SWING_SCAN_STEP

        if not found:
            self._arm_warn(f"No {label} found during sweep.")
            return None

        # Centre in frame
        for i in range(ALIGNMENT_MAX_ITERS):
            time.sleep(SWING_SCAN_PAUSE)
            detection = self._get_any_piece_detection()
            if detection is None:
                self._arm_warn("Lost detection during alignment.")
                break
            x_offset = self._detection_x_offset(detection)
            self._arm_log(f"  Align iter {i+1}: x_offset={x_offset:.1f} px")
            if abs(x_offset) <= ALIGNMENT_TOLERANCE_PX:
                self._arm_log(f"Aligned at swing={current_swing}.")
                break
            correction   = int(x_offset * PIXELS_TO_STEPS_GAIN)
            current_swing = max(SWING_SCAN_START,
                                min(SWING_SCAN_END, current_swing + correction))
            self.arm_swing_to(current_swing)

        return current_swing

    # ── Drive helper ──────────────────────────────────────────────────────────

    def _drive_forward(self, distance_mm: float) -> None:
        """Drive the robot straight forward by distance_mm."""
        self._arm_log(f"Driving forward {distance_mm:.0f} mm...")
        try:
            self.set_velocity(DRIVE_SPEED_MM_S, 0.0)
            # Calculate time needed: t = d / v
            drive_time = distance_mm / DRIVE_SPEED_MM_S
            time.sleep(drive_time)
            self.stop()
            time.sleep(0.2)   # settle before next move
        except Exception as e:
            self._arm_warn(f"Drive failed: {e}")

    # ════════════════════════════════════════════════════════════════════════
    # FULL ASSEMBLY SEQUENCE
    # ════════════════════════════════════════════════════════════════════════

    def assemble_burger(
        self,
        home_on_start: bool = False,
        park_on_finish: bool = True,
    ) -> None:
        """
        Full sequential burger assembly:

            Step 1 — Sweep camera, find TOP BUN, pick it up
            Step 2 — Drive forward DRIVE_TO_PATTY_MM to patty position
                     Lower arm (still holding bun) onto patty
                     Grip both together (bun + patty), lift
            Step 3 — Drive forward DRIVE_TO_STACK_MM to bottom bun
                     Lower stack onto bottom bun, release, lift, done

        Args:
            home_on_start: Home both axes before starting.
            park_on_finish: Swing arm to safe position after completion.
        """
        self._arm_log("═══ Burger assembly START ═══")

        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        if home_on_start:
            self.arm_home()

        # ── STEP 1: Pick top bun ──────────────────────────────────────────────
        self._arm_log("── Step 1/3: Pick TOP BUN ──")

        self.arm_safe_height()
        aligned = self.arm_sweep_and_align("top bun")
        if aligned is None:
            self._arm_warn("Top bun not found — assembly aborted.")
            return

        self.arm_open_gripper()
        self._arm_log(f"Lowering to bun at {ELEVATOR_BUN_PICK} steps...")
        self.arm_elevator_to(ELEVATOR_BUN_PICK)
        self.arm_grip()
        time.sleep(0.2)
        self.arm_safe_height()
        self._arm_log("Top bun picked ✓")

        # ── STEP 2: Drive to patty, pick up patty + bun together ──────────────
        self._arm_log("── Step 2/3: Drive to patty, pick up patty + bun ──")

        self._drive_forward(DRIVE_TO_PATTY_MM)

        # Lower the arm (still holding bun) down onto the patty
        self._arm_log(f"Lowering onto patty at {ELEVATOR_PATTY_PICK} steps...")
        self.arm_elevator_to(ELEVATOR_PATTY_PICK)

        # Re-grip to secure bun + patty together
        self._arm_log("Gripping patty + bun together...")
        self.arm_grip()
        time.sleep(0.3)

        self.arm_safe_height()
        self._arm_log("Patty + bun picked ✓")

        # ── STEP 3: Drive to bottom bun, place full stack ─────────────────────
        self._arm_log("── Step 3/3: Drive to bottom bun, place full stack ──")

        self._drive_forward(DRIVE_TO_STACK_MM)

        # Swing to position above the bottom bun
        self.arm_swing_to(SWING_STACK_STEPS)

        # Lower stack onto bottom bun
        self._arm_log(f"Lowering stack at {ELEVATOR_STACK_PLACE} steps...")
        self.arm_elevator_to(ELEVATOR_STACK_PLACE)

        # Release
        self.arm_open_gripper()
        time.sleep(0.3)

        # Lift clear
        self.arm_safe_height()

        self._arm_log("═══ Burger assembly COMPLETE ✓ ═══")

        if park_on_finish:
            self.arm_swing_to(SWING_SAFE_STEPS)
            self._arm_log("Arm parked.")