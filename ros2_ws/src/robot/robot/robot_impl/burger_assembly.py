"""
robot/robot_impl/burger_assembly.py
════════════════════════════════════
BurgerAssemblyMixin — camera-guided burger assembly with stepper swing alignment.

Physical setup
──────────────
  STEPPER_ELEVATOR (1) — two ganged motors, z-axis lift.
                         Steps INCREASE going DOWN toward the table.
  STEPPER_SWING    (2) — single motor, 0–180° arm rotation over robot top half.
                         Step 0   = left limit switch
                         Step MAX = right limit switch
                         Lidar sits at the midpoint of the arc (~90°)

  Servo channel 16 (firmware ch 15) = gripper claw.

  Camera is mounted ON the arm — it points at whatever the arm faces.
  Bounding-box X offset from frame centre drives swing correction.

Piece detection
───────────────
  Yellow colour → bun   (uses detect_yellow_block from vision node)
  Red colour    → patty (uses detect_red_block, defined below)

  The vision node publishes to /vision/detections.  We subscribe and look for
  detections with class_name == "yellow_block" or "red_block".

Grip pulses (empirically tuned)
────────────────────────────────
  BUN_GRIP_US   = 1430   (softer — buns are squishier)
  PATTY_GRIP_US = 1500   (firmer — patty is denser)

Assembly order
──────────────
  Step 1 — Pick PATTY,      place on BOTTOM BUN position (stack starts)
  Step 2 — Pick TOP BUN,    place on top of patty
  Step 3 — Pick full STACK  (gripper open, lower onto stack, close, lift away)

Navigation hook
───────────────
  assemble_burger() accepts an optional navigate_to_piece(piece_type: str) 
  callback your team fills in. piece_type will be "patty", "top_bun", or "stack".
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from bridge_interfaces.msg import StepEnable, StepHome, StepMove


# ════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS — adjust these to match your robot
# ════════════════════════════════════════════════════════════════════════════

# ── Stepper numbers (1-based) ─────────────────────────────────────────────
STEPPER_ELEVATOR        = 1
STEPPER_SWING           = 2

# ── Elevator positions (steps, increase = lower) ──────────────────────────
ELEVATOR_SAFE_STEPS     = 0      # fully raised — safe to swing
ELEVATOR_PATTY_PICK     = 850    # depth to reach the patty
ELEVATOR_BUN_PICK       = 800    # depth to reach a bun
ELEVATOR_PLACE_STEP1    = 600    # height of empty table (bottom bun resting here)
ELEVATOR_PLACE_STEP2    = 500    # height after patty is on bottom bun
ELEVATOR_PLACE_STEP3    = 400    # height after top bun is on patty
ELEVATOR_STACK_GRAB     = 580    # depth to lower gripper around full stack

# ── Swing positions (steps) ───────────────────────────────────────────────
SWING_HOME_STEPS        = 0      # left limit switch
SWING_SAFE_STEPS        = 0      # parked position (out of lidar FOV)
SWING_SCAN_START        = 100    # where to begin the detection sweep
SWING_SCAN_END          = 900    # where to end the detection sweep
SWING_SCAN_STEP         = 30     # steps per sweep increment
SWING_SCAN_PAUSE        = 0.15   # seconds to pause at each sweep position

# ── Camera / alignment ────────────────────────────────────────────────────
CAMERA_FRAME_WIDTH      = 640    # pixels — must match vision node camera_width
PIXELS_TO_STEPS_GAIN    = 0.5    # tune: steps per pixel of X offset
ALIGNMENT_TOLERANCE_PX  = 20     # pixels — centred enough to pick
ALIGNMENT_MAX_ITERS     = 10     # max correction iterations before giving up

# ── Grip pulses (µs) ─────────────────────────────────────────────────────
GRIPPER_OPEN_US         = 900    # claw fully open
BUN_GRIP_US             = 1430   # grip for buns (softer)
PATTY_GRIP_US           = 1500   # grip for patty (firmer)
STACK_GRIP_US           = 1430   # grip for full stack (treat like bun)
GRIPPER_CHANNEL         = 16     # servo channel (1-based, firmware ch 15)

# ── Piece radii (mm, hard-coded geometry) ────────────────────────────────
PATTY_RADIUS_MM         = 50.0   # radius of patty
BUN_RADIUS_MM           = 55.0   # radius of bun (slightly larger than patty)
STACK_RADIUS_MM         = 55.0   # treat assembled stack as bun-sized

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


# ════════════════════════════════════════════════════════════════════════════
# MIXIN
# ════════════════════════════════════════════════════════════════════════════

class BurgerAssemblyMixin:
    """
    Camera-guided burger assembly mixin for the Robot class.

    Requires in Robot.__init__ (all already present):
        self._step_en_pub, self._step_mv_pub, self._step_hm_pub
        self._step_state   (cached StepStateAll)
        self._vision_detections  (cached list[dict] from /vision/detections)
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

    # ── Low-level stepper helpers ─────────────────────────────────────────────

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
        """Block until stepper reports idle (motion_state == 0) or timeout."""
        deadline = time.monotonic() + timeout_s
        idx = stepper - 1   # 0-based

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
        """Raise elevator to safe travel height before any swing move."""
        return self.arm_elevator_to(ELEVATOR_SAFE_STEPS)

    def arm_open_gripper(self) -> None:
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    def arm_grip(self, pulse_us: int) -> None:
        """Close gripper to a specific pulse width."""
        self.close_gripper(channel=GRIPPER_CHANNEL, pulse_us=pulse_us)
        time.sleep(0.15)

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

    # ── Camera detection helpers ──────────────────────────────────────────────

    def _get_detection(self, class_name: str):
        # Return highest-confidence detection matching class_name OR
        # matching burger_type attribute (for burger pieces detected by
        # classify_burger_piece rather than YOLO directly).
        
        with self._lock:
            detections = list(self._vision_detections)
 
        target_burger_type = None
        if class_name == "red_block":
            target_burger_type = "patty"
        elif class_name == "yellow_block":
            target_burger_type = "bun"
 
        best      = None
        best_conf = 0.0
 
        for d in detections:
            matched = False
 
            # Direct class_name match (e.g. YOLO detects "red_block" natively)
            if d.get("class_name") == class_name:
                matched = True
 
            # Attribute match (classify_burger_piece tagged this detection)
            if target_burger_type is not None:
                attrs = d.get("attributes", {})
                if attrs.get("burger_type") == target_burger_type:
                    matched = True
 
            if matched:
                conf = float(d.get("confidence", 0.0))
                if conf > best_conf:
                    best_conf = conf
                    best = d
 
        return best

    def _detection_x_offset(self, detection: dict) -> float:
        """
        Return the bounding-box centre X offset from the frame centre (pixels).
        Positive = piece is to the right, negative = to the left.
        """
        bbox_centre_x = detection["x"] + detection["width"] / 2.0
        return bbox_centre_x - (CAMERA_FRAME_WIDTH / 2.0)

    # ── Sweep and align ───────────────────────────────────────────────────────

    def arm_sweep_and_align(self, class_name: str) -> Optional[int]:
        """
        Sweep the arm from SWING_SCAN_START to SWING_SCAN_END until a
        detection of class_name appears, then centre the piece in frame
        using proportional swing corrections.

        Returns the final swing step position when aligned, or None if the
        piece was never found.

        Args:
            class_name: "yellow_block" for buns, "red_block" for patty.
        """
        self._arm_log(f"Sweeping for '{class_name}'...")

        # ── Phase 1: sweep until piece appears ───────────────────────────────
        current_swing = SWING_SCAN_START
        found = False

        while current_swing <= SWING_SCAN_END:
            self.arm_swing_to(current_swing)
            time.sleep(SWING_SCAN_PAUSE)   # let camera settle

            if self._get_detection(class_name) is not None:
                self._arm_log(
                    f"'{class_name}' detected at swing={current_swing} steps."
                )
                found = True
                break

            current_swing += SWING_SCAN_STEP

        if not found:
            self._arm_warn(
                f"'{class_name}' not found during sweep "
                f"({SWING_SCAN_START}→{SWING_SCAN_END} steps)."
            )
            return None

        # ── Phase 2: centre the piece in frame ───────────────────────────────
        self._arm_log("Centering piece in frame...")

        for iteration in range(ALIGNMENT_MAX_ITERS):
            time.sleep(SWING_SCAN_PAUSE)
            detection = self._get_detection(class_name)

            if detection is None:
                self._arm_warn("Lost detection during alignment — stopping.")
                break

            x_offset = self._detection_x_offset(detection)
            self._arm_log(
                f"  Alignment iter {iteration + 1}: "
                f"x_offset={x_offset:.1f} px"
            )

            if abs(x_offset) <= ALIGNMENT_TOLERANCE_PX:
                self._arm_log(
                    f"Aligned! x_offset={x_offset:.1f} px ≤ "
                    f"{ALIGNMENT_TOLERANCE_PX} px tolerance."
                )
                break

            # Proportional correction: positive offset → swing more steps
            correction = int(x_offset * PIXELS_TO_STEPS_GAIN)
            current_swing = current_swing + correction
            current_swing = max(SWING_SCAN_START,
                                min(SWING_SCAN_END, current_swing))
            self.arm_swing_to(current_swing)

        return current_swing

    # ── Pick with camera alignment ────────────────────────────────────────────

    def arm_pick_piece(
        self,
        class_name: str,
        elevator_steps: int,
        grip_pulse_us: int,
        piece_radius_mm: float,
    ) -> bool:
        """
        Camera-guided pick sequence:
            1. Raise to safe height
            2. Sweep arm until piece detected, then centre it
            3. Open gripper
            4. Lower elevator to piece
            5. Close gripper at the tuned grip pulse
            6. Raise to safe height

        Args:
            class_name:      "yellow_block" or "red_block"
            elevator_steps:  How far to lower the elevator to reach the piece
            grip_pulse_us:   Servo pulse width µs for this piece type
            piece_radius_mm: Physical radius of the piece (for logging/debug)

        Returns True if pick succeeded, False if piece was not found.
        """
        self._arm_log(
            f"Picking '{class_name}' "
            f"(radius={piece_radius_mm:.0f}mm, grip={grip_pulse_us}µs)"
        )

        # 1. Safe height
        self.arm_safe_height()

        # 2. Sweep and align
        aligned_swing = self.arm_sweep_and_align(class_name)
        if aligned_swing is None:
            self._arm_warn(f"Could not find '{class_name}' — aborting pick.")
            return False

        # 3. Open gripper before descending
        self.arm_open_gripper()

        # 4. Lower to piece
        self._arm_log(f"Lowering to {elevator_steps} steps...")
        self.arm_elevator_to(elevator_steps)

        # 5. Close gripper at piece-specific pulse
        self._arm_log(f"Gripping at {grip_pulse_us} µs...")
        self.arm_grip(grip_pulse_us)
        time.sleep(0.2)

        # 6. Lift piece
        self.arm_safe_height()

        self._arm_log(f"'{class_name}' picked ✓")
        return True

    # ── Place ─────────────────────────────────────────────────────────────────

    def arm_place_piece(
        self,
        swing_steps: int,
        elevator_steps: int,
        label: str = "",
    ) -> None:
        """
        Place the currently held piece:
            1. Safe height (already there after pick)
            2. Swing to place position
            3. Lower to stack height
            4. Open gripper
            5. Raise to safe height
        """
        self._arm_log(
            f"Placing{' ' + label if label else ''} at "
            f"swing={swing_steps}, elevator={elevator_steps}"
        )
        self.arm_safe_height()
        self.arm_swing_to(swing_steps)
        self.arm_elevator_to(elevator_steps)
        self.arm_open_gripper()
        time.sleep(0.25)   # let piece settle before lifting
        self.arm_safe_height()
        self._arm_log("Place complete ✓")

    # ── Full burger assembly ──────────────────────────────────────────────────

    def assemble_burger(
        self,
        stack_swing_steps: int,
        navigate_to_piece: Optional[Callable[[str], None]] = None,
        home_on_start: bool = False,
        park_on_finish: bool = True,
    ) -> None:
        """
        Full burger assembly sequence.

        Assembly order
        ──────────────
          Step 1 — Locate PATTY with camera → pick → place on stack position
          Step 2 — Locate TOP BUN with camera → pick → place on top of patty
          Step 3 — Lower open gripper onto full STACK → grip → lift and carry

        Args:
            stack_swing_steps:
                Swing stepper position directly above the assembly stack.
                The bottom bun is assumed to already be at this position on
                the table before assembly begins.

            navigate_to_piece:
                Optional callback called before each pick so your team's
                navigation code can drive the robot to the right area.
                Receives a string: "patty", "top_bun", or "stack".

                Example:
                    def go_to(piece_type):
                        if piece_type == "patty":
                            robot.navigate_to(patty_x, patty_y)

            home_on_start:
                Home both axes before starting (set True if position unknown).

            park_on_finish:
                Swing arm to SWING_SAFE_STEPS after carrying the stack,
                so it doesn't block the lidar during onward navigation.
        """
        self._arm_log("═══ Burger assembly START ═══")

        # Enable both steppers
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        if home_on_start:
            self.arm_home()

        def navigate(piece_type: str) -> None:
            if navigate_to_piece is not None:
                self._arm_log(f"Navigating to {piece_type}...")
                try:
                    navigate_to_piece(piece_type)
                except Exception as e:
                    self._arm_warn(f"Navigation raised: {e} — continuing.")

        # ── STEP 1: Pick patty, place on bottom bun ───────────────────────────
        self._arm_log("── Step 1/3: Pick PATTY ──")
        navigate("patty")
        ok = self.arm_pick_piece(
            class_name      = "red_block",
            elevator_steps  = ELEVATOR_PATTY_PICK,
            grip_pulse_us   = PATTY_GRIP_US,
            piece_radius_mm = PATTY_RADIUS_MM,
        )
        if not ok:
            self._arm_warn("PATTY pick failed — assembly aborted.")
            return

        self._arm_log("── Step 1/3: Place PATTY on bottom bun ──")
        # Bottom bun is already on the table at stack_swing_steps
        self.arm_place_piece(
            swing_steps    = stack_swing_steps,
            elevator_steps = ELEVATOR_PLACE_STEP1,
            label          = "patty on bottom bun",
        )

        # ── STEP 2: Pick top bun, place on patty ──────────────────────────────
        self._arm_log("── Step 2/3: Pick TOP BUN ──")
        navigate("top_bun")
        ok = self.arm_pick_piece(
            class_name      = "yellow_block",
            elevator_steps  = ELEVATOR_BUN_PICK,
            grip_pulse_us   = BUN_GRIP_US,
            piece_radius_mm = BUN_RADIUS_MM,
        )
        if not ok:
            self._arm_warn("TOP BUN pick failed — assembly aborted.")
            return

        self._arm_log("── Step 2/3: Place TOP BUN on patty ──")
        self.arm_place_piece(
            swing_steps    = stack_swing_steps,
            elevator_steps = ELEVATOR_PLACE_STEP2,
            label          = "top bun on patty",
        )

        # ── STEP 3: Grab full stack and carry it ──────────────────────────────
        self._arm_log("── Step 3/3: Grab FULL STACK ──")
        navigate("stack")

        # Swing over stack, open gripper, lower around stack, grip, lift
        self.arm_safe_height()
        self.arm_swing_to(stack_swing_steps)
        self.arm_open_gripper()

        self._arm_log(f"Lowering onto stack at {ELEVATOR_STACK_GRAB} steps...")
        self.arm_elevator_to(ELEVATOR_STACK_GRAB)

        self._arm_log(f"Gripping stack at {STACK_GRIP_US} µs...")
        self.arm_grip(STACK_GRIP_US)
        time.sleep(0.3)

        self._arm_log("Lifting full stack...")
        self.arm_safe_height()

        self._arm_log("═══ Stack in hand — burger assembly COMPLETE ═══")

        # Park arm after carrying stack
        if park_on_finish:
            self._arm_log(f"Parking arm at swing={SWING_SAFE_STEPS}...")
            self.arm_swing_to(SWING_SAFE_STEPS)
            self._arm_log("Arm parked.")