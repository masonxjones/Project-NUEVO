"""
robot/robot_impl/burger_assembly.py
════════════════════════════════════
BurgerAssemblyMixin — add to Robot's MRO alongside the other mixins.

Physical setup
──────────────
  STEPPER_ELEVATOR  (default 1) — two ganged motors driving the z-axis arm lift.
                                   Steps increase going DOWN toward the table.
  STEPPER_SWING     (default 2) — single motor rotating the arm 0–180° over the
                                   robot's top half.
                                   Step 0   = one limit switch (e.g. left side)
                                   Step MAX = other limit switch (right side)
                                   Step MID ≈ facing forward (over lidar)

  Servo channel 1 (firmware ch 0) = gripper claw on the arm tip.

  Limit switches are wired to the firmware's IO input system and readable via
  robot.get_limit(n).  The exact limit numbers must be configured in
  ARM_SWING_LIMIT_LEFT and ARM_SWING_LIMIT_RIGHT below.

Public API
──────────
All positions are in raw stepper steps (absolute).

  robot.arm_home()
      Drive swing to left limit switch, zero that axis.
      Drive elevator to top limit switch, zero that axis.
      Optional — call at startup if homing is needed.

  robot.arm_swing_to(steps)
      Move swing stepper to absolute position.

  robot.arm_elevator_to(steps)
      Move elevator stepper to absolute position.

  robot.arm_open_gripper()  /  robot.arm_close_gripper()
      Convenience wrappers around GripperMixin.

  robot.arm_pick(elevator_steps, swing_steps)
      Swing → drop → squeeze → lift full pick sequence.

  robot.arm_place(elevator_steps, swing_steps, open_pulse_us)
      Swing → drop → open gripper → lift full place sequence.

  robot.assemble_burger(pieces)
      Full burger stacking sequence.  pieces is a list of BurgerPiece
      namedtuples describing each piece's pick and place positions.

Usage example
─────────────
  from robot.robot_impl.burger_assembly import BurgerAssemblyMixin, BurgerPiece

  pieces = [
      BurgerPiece("bottom_bun", pick_swing=200,  pick_elevator=800,
                               place_swing=900,  place_elevator=600),
      BurgerPiece("patty",      pick_swing=1200, pick_elevator=850,
                               place_swing=900,  place_elevator=500),
      BurgerPiece("top_bun",   pick_swing=2200, pick_elevator=800,
                               place_swing=900,  place_elevator=400),
  ]
  robot.assemble_burger(pieces)

Navigation hook
───────────────
  assemble_burger() accepts an optional `navigate_to_piece` callback:

      def go_to_piece(piece: BurgerPiece) -> None:
          # Your team's navigation code here
          robot.drive_to(piece.pick_x_mm, piece.pick_y_mm)

      robot.assemble_burger(pieces, navigate_to_piece=go_to_piece)

  If not provided, the robot stays in place and just moves the arm.
"""

from __future__ import annotations

import time
from typing import Callable, NamedTuple, Optional

from bridge_interfaces.msg import StepEnable, StepHome, StepMove


# ── Physical constants — tune to your robot ──────────────────────────────────

STEPPER_ELEVATOR        = 1      # stepper number (1-based) for z-axis lift
STEPPER_SWING           = 2      # stepper number (1-based) for arm rotation

ELEVATOR_HOME_STEPS     = 0      # step count when arm is fully raised (safe travel position)
ELEVATOR_SAFE_STEPS     = 0      # safe height to travel at (same as home unless tower has offset)
SWING_HOME_STEPS        = 0      # step count at left limit switch (home)
SWING_SAFE_STEPS        = 0      # step count for arm parked safely out of drive path

ELEVATOR_HOME_VELOCITY  = 200    # steps/s for homing move (slow)
ELEVATOR_HOME_DIRECTION = 0      # 0 = negative direction (upward)
ELEVATOR_HOME_BACKOFF   = 50     # steps to back off after hitting limit

SWING_HOME_VELOCITY     = 200
SWING_HOME_DIRECTION    = 0      # 0 = toward left limit switch
SWING_HOME_BACKOFF      = 30

ARM_SWING_LIMIT_LEFT    = 1      # robot.get_limit() index for left swing limit switch
ARM_SWING_LIMIT_RIGHT   = 2      # robot.get_limit() index for right swing limit switch

GRIPPER_OPEN_US         = 1000   # pulse µs — claw open
GRIPPER_CLOSED_US       = 2000   # pulse µs — claw closed
GRIPPER_CHANNEL         = 16     # servo channel (1-based, firmware ch 15)

STEPPER_MOVE_TIMEOUT_S  = 15.0   # max seconds to wait for any single stepper move
STEPPER_SETTLE_S        = 0.15   # short settle pause after each move


# ── BurgerPiece descriptor ────────────────────────────────────────────────────

class BurgerPiece(NamedTuple):
    """
    Describes one burger piece's pick and place positions.

    All step values are absolute stepper positions.
    pick_x_mm / pick_y_mm are optional — used only when a navigate_to_piece
    callback is provided to assemble_burger().
    """
    name:           str
    pick_swing:     int    # swing stepper position to reach above the piece
    pick_elevator:  int    # elevator stepper position to descend to the piece
    place_swing:    int    # swing stepper position above the stack
    place_elevator: int    # elevator stepper position to set piece on stack
    pick_x_mm:      float = 0.0   # robot XY to drive to before picking (optional)
    pick_y_mm:      float = 0.0


# ── Mixin ─────────────────────────────────────────────────────────────────────

class BurgerAssemblyMixin:
    """
    High-level burger assembly methods for the Robot class.

    Requires (already present in Robot.__init__):
        self._step_en_pub   — StepEnable publisher
        self._step_mv_pub   — StepMove publisher
        self._step_hm_pub   — StepHome publisher
        self._step_state    — cached StepStateAll (updated by subscription)
        self._lock          — threading.Lock
        self._node          — rclpy Node (for logging)

    Also requires GripperMixin to be in the MRO (for open/close gripper calls).
    """

    # ── Logging helper ────────────────────────────────────────────────────────

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

    # ── Low-level stepper wrappers ────────────────────────────────────────────

    def _step_enable(self, stepper: int, enable: bool) -> None:
        msg = StepEnable()
        msg.stepper_number = stepper
        msg.enable = 1 if enable else 0
        self._step_en_pub.publish(msg)

    def _step_move_abs(self, stepper: int, target_steps: int) -> None:
        """Publish an absolute-position stepper move (move_type=0 = ABSOLUTE)."""
        msg = StepMove()
        msg.stepper_number = stepper
        msg.move_type = 0   # StepMoveType.ABSOLUTE
        msg.target = int(target_steps)
        self._step_mv_pub.publish(msg)

    def _step_home_cmd(
        self,
        stepper: int,
        direction: int,
        velocity: int,
        backoff: int,
    ) -> None:
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
        """
        Block until the given stepper reports IDLE in StepStateAll,
        or until timeout_s elapses.  Returns True if idle reached.
        """
        from robot.hardware_map import StepperMotionState
        deadline = time.monotonic() + timeout_s
        idx = stepper - 1   # 0-based index

        while time.monotonic() < deadline:
            with self._lock:
                state = self._step_state
            if state is not None:
                motion_states = getattr(state, 'motion_states', None)
                if motion_states and idx < len(motion_states):
                    if int(motion_states[idx]) == int(StepperMotionState.IDLE):
                        return True
            time.sleep(poll_s)

        self._arm_warn(f"Timeout waiting for stepper {stepper} to idle.")
        return False

    def _move_and_wait(
        self,
        stepper: int,
        target_steps: int,
        settle_s: float = STEPPER_SETTLE_S,
    ) -> bool:
        """Move stepper to target_steps and block until idle."""
        self._step_move_abs(stepper, target_steps)
        ok = self._wait_stepper(stepper)
        if settle_s > 0:
            time.sleep(settle_s)
        return ok

    # ── Arm-level moves ───────────────────────────────────────────────────────

    def arm_elevator_to(self, steps: int) -> bool:
        """Move the elevator to an absolute step position. Blocks until idle."""
        self._arm_log(f"Elevator → {steps} steps")
        return self._move_and_wait(STEPPER_ELEVATOR, steps)

    def arm_swing_to(self, steps: int) -> bool:
        """Swing the arm to an absolute step position. Blocks until idle."""
        self._arm_log(f"Swing → {steps} steps")
        return self._move_and_wait(STEPPER_SWING, steps)

    def arm_safe_height(self) -> bool:
        """Raise elevator to safe travel height before any swing move."""
        return self.arm_elevator_to(ELEVATOR_SAFE_STEPS)

    def arm_open_gripper(self) -> None:
        """Open the gripper claw."""
        self.open_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_OPEN_US)
        time.sleep(0.15)

    def arm_close_gripper(self) -> None:
        """Close the gripper claw to fixed closed position."""
        self.close_gripper(channel=GRIPPER_CHANNEL, pulse_us=GRIPPER_CLOSED_US)
        time.sleep(0.15)

    # ── Homing ────────────────────────────────────────────────────────────────

    def arm_home(self) -> None:
        """
        Home both axes by driving to their limit switches.

        Call at startup if the arm position is unknown.
        Safe to skip if the arm is already at a known position.
        """
        self._arm_log("Homing elevator...")
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_home_cmd(
            STEPPER_ELEVATOR,
            direction=ELEVATOR_HOME_DIRECTION,
            velocity=ELEVATOR_HOME_VELOCITY,
            backoff=ELEVATOR_HOME_BACKOFF,
        )
        self._wait_stepper(STEPPER_ELEVATOR, timeout_s=30.0)
        self._arm_log("Elevator homed.")

        self._arm_log("Homing swing...")
        self._step_enable(STEPPER_SWING, True)
        self._step_home_cmd(
            STEPPER_SWING,
            direction=SWING_HOME_DIRECTION,
            velocity=SWING_HOME_VELOCITY,
            backoff=SWING_HOME_BACKOFF,
        )
        self._wait_stepper(STEPPER_SWING, timeout_s=30.0)
        self._arm_log("Swing homed. Arm at home position.")

    # ── Pick and place primitives ─────────────────────────────────────────────

    def arm_pick(
        self,
        elevator_steps: int,
        swing_steps: int,
        resistance_threshold: int = 500,
    ) -> int:
        """
        Full pick sequence for one piece:
            1. Raise elevator to safe height
            2. Swing to pick position
            3. Open gripper
            4. Lower elevator to piece
            5. Squeeze until resistance (or max)
            6. Raise elevator back to safe height

        Returns the final gripper pulse_us when grip was detected.
        """
        self._arm_log(f"Pick: swing={swing_steps} elevator={elevator_steps}")

        # 1. Safe height before swinging
        self.arm_safe_height()

        # 2. Swing to pick position
        self.arm_swing_to(swing_steps)

        # 3. Open gripper
        self.arm_open_gripper()

        # 4. Lower to piece
        self.arm_elevator_to(elevator_steps)

        # 5. Squeeze until resistance
        self._arm_log("Squeezing...")
        final_pulse = self.squeeze_until_resistance(
            channel=GRIPPER_CHANNEL,
            start_pulse_us=GRIPPER_OPEN_US,
            max_pulse_us=GRIPPER_CLOSED_US,
            resistance_threshold=resistance_threshold,
        )
        self._arm_log(f"Grip at {final_pulse} µs.")

        # 6. Raise back to safe height (piece in gripper)
        self.arm_safe_height()

        return final_pulse

    def arm_place(
        self,
        elevator_steps: int,
        swing_steps: int,
    ) -> None:
        """
        Full place sequence for one piece:
            1. Raise elevator to safe height (should already be there after pick)
            2. Swing to place position
            3. Lower elevator to stack height
            4. Open gripper to release piece
            5. Raise elevator back to safe height
        """
        self._arm_log(f"Place: swing={swing_steps} elevator={elevator_steps}")

        # 1. Confirm safe height
        self.arm_safe_height()

        # 2. Swing to stack position
        self.arm_swing_to(swing_steps)

        # 3. Lower to stack
        self.arm_elevator_to(elevator_steps)

        # 4. Release piece
        self.arm_open_gripper()
        time.sleep(0.2)   # brief pause so piece settles before lifting

        # 5. Raise clear of stack
        self.arm_safe_height()

        self._arm_log("Place complete.")

    # ── Full burger assembly sequence ─────────────────────────────────────────

    def assemble_burger(
        self,
        pieces: list[BurgerPiece],
        navigate_to_piece: Optional[Callable[[BurgerPiece], None]] = None,
        resistance_threshold: int = 500,
        home_on_start: bool = False,
        park_on_finish: bool = True,
    ) -> None:
        """
        Assemble a full burger by picking and stacking each piece in order.

        Args:
            pieces:
                Ordered list of BurgerPiece descriptors.  First piece = bottom
                bun, last piece = top bun.

            navigate_to_piece:
                Optional callback called before each pick.  Receives the
                BurgerPiece about to be picked.  Use this to hook in your
                team's navigation code:

                    def go_to(piece):
                        robot.navigate_to(piece.pick_x_mm, piece.pick_y_mm)

                If None, the robot stays in place (arm-only operation).

            resistance_threshold:
                ADC threshold for grip detection (0–1023).

            home_on_start:
                If True, home both axes before starting.  Set True if arm
                position is unknown at script start.

            park_on_finish:
                If True, swing arm to SWING_SAFE_STEPS after all pieces are
                stacked so it doesn't block the lidar during navigation.
        """
        self._arm_log(f"Starting burger assembly: {len(pieces)} pieces.")

        # Enable both steppers
        self._step_enable(STEPPER_ELEVATOR, True)
        self._step_enable(STEPPER_SWING, True)

        # Optional homing
        if home_on_start:
            self.arm_home()

        for i, piece in enumerate(pieces):
            self._arm_log(
                f"── Piece {i + 1}/{len(pieces)}: {piece.name} ──"
            )

            # Navigate to pick position if a callback was provided
            if navigate_to_piece is not None:
                self._arm_log(
                    f"Navigating to pick position "
                    f"({piece.pick_x_mm:.0f}, {piece.pick_y_mm:.0f}) mm..."
                )
                try:
                    navigate_to_piece(piece)
                except Exception as e:
                    self._arm_warn(f"Navigation callback raised: {e} — continuing anyway.")

            # Pick
            self.arm_pick(
                elevator_steps=piece.pick_elevator,
                swing_steps=piece.pick_swing,
                resistance_threshold=resistance_threshold,
            )

            # Place
            self.arm_place(
                elevator_steps=piece.place_elevator,
                swing_steps=piece.place_swing,
            )

            self._arm_log(f"{piece.name} stacked ✓")

        self._arm_log("All pieces stacked — burger assembly complete!")

        # Park arm out of lidar FOV for post-assembly navigation
        if park_on_finish:
            self._arm_log(f"Parking arm at swing={SWING_SAFE_STEPS}...")
            self.arm_safe_height()
            self.arm_swing_to(SWING_SAFE_STEPS)
            self._arm_log("Arm parked.")