"""
ros2_ws/src/robot/robot/examples/test_burger_assembly.py
─────────────────────────────────────────────────────────
Step-through test for the hardcoded burger assembly sequence.

Run inside Docker:
    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash
    python3 -u src/robot/robot/examples/test_burger_assembly.py

Position the robot beside the table before running.
Press Enter at each prompt to advance — Ctrl+C to abort at any time.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from robot.robot import Robot
from robot.robot_impl.burger_assembly import (
    SWING_PICK_STEPS,
    SWING_HOME,
    ELEVATOR_HOME,
    ELEVATOR_PICK_STEPS,
    ELEVATOR_PLACE_BUN,
    ELEVATOR_PLACE_STACK,
    GRIP_US,
    GRIPPER_OPEN_US,
    PIECE_SPACING_MM,
)

HOME_ON_START = False


def main() -> None:
    rclpy.init()
    node = Node("burger_assembly_test")
    robot = Robot(node)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("\n═══ Burger Assembly Step-Through Test ═══")
    print(f"Swing pick position:   {SWING_PICK_STEPS} steps")
    print(f"Elevator pick depth:   {ELEVATOR_PICK_STEPS} steps")
    print(f"Elevator place bun:    {ELEVATOR_PLACE_BUN} steps")
    print(f"Elevator place stack:  {ELEVATOR_PLACE_STACK} steps")
    print(f"Grip pulse:            {GRIP_US} µs")
    print(f"Piece spacing:         {PIECE_SPACING_MM} mm")
    print()

    try:
        # ── Pre-checks ────────────────────────────────────────────────────────
        input("[ PRE-CHECK ]  Press Enter to verify gripper open/close...")
        robot.arm_open_gripper()
        time.sleep(0.4)
        robot.arm_grip()
        time.sleep(0.4)
        robot.arm_open_gripper()
        print("  ✓ Gripper OK.\n")

        if HOME_ON_START:
            input("[ HOMING ]  Press Enter to home both axes...")
            robot.arm_home()
            print("  ✓ Homing complete.\n")

        # ── Step 1: Pick top bun ──────────────────────────────────────────────
        input("[ STEP 1 ]  Press Enter — swing to pick position and pick TOP BUN...")
        robot.arm_open_gripper()
        robot.arm_swing_to(SWING_PICK_STEPS)
        print(f"  Arm swung to {SWING_PICK_STEPS} steps.")
        robot.arm_elevator_to(ELEVATOR_PICK_STEPS)
        print(f"  Arm lowered to {ELEVATOR_PICK_STEPS} steps.")
        robot.arm_grip()
        robot.arm_elevator_to(ELEVATOR_HOME)
        print("  ✓ Top bun picked — elevator at home.\n")

        # ── Step 2: Drive to patty ────────────────────────────────────────────
        input(f"[ STEP 2a ]  Press Enter — drive {PIECE_SPACING_MM:.0f}mm to patty...")
        robot._drive_forward(PIECE_SPACING_MM)
        print("  ✓ At patty position.\n")

        input("[ STEP 2b ]  Press Enter — lower and drop bun onto patty...")
        robot.arm_swing_to(SWING_PICK_STEPS)
        robot.arm_elevator_to(ELEVATOR_PLACE_BUN)
        robot.arm_open_gripper()
        time.sleep(0.3)
        print("  ✓ Bun dropped on patty.\n")

        input("[ STEP 2c ]  Press Enter — lower to pick bun + patty together...")
        robot.arm_elevator_to(ELEVATOR_PICK_STEPS)
        robot.arm_grip()
        robot.arm_elevator_to(ELEVATOR_HOME)
        print("  ✓ Bun + patty picked.\n")

        # ── Step 3: Drive to bottom bun ───────────────────────────────────────
        input(f"[ STEP 3a ]  Press Enter — drive {PIECE_SPACING_MM:.0f}mm to bottom bun...")
        robot._drive_forward(PIECE_SPACING_MM)
        print("  ✓ At bottom bun position.\n")

        input("[ STEP 3b ]  Press Enter — lower and drop bun+patty onto bottom bun...")
        robot.arm_swing_to(SWING_PICK_STEPS)
        robot.arm_elevator_to(ELEVATOR_PLACE_STACK)
        robot.arm_open_gripper()
        time.sleep(0.3)
        print("  ✓ Bun+patty dropped on bottom bun.\n")

        input("[ STEP 3c ]  Press Enter — lower to pick full stack...")
        robot.arm_elevator_to(ELEVATOR_PICK_STEPS)
        robot.arm_grip()
        robot.arm_elevator_to(ELEVATOR_HOME)
        print("  ✓ Full stack picked.\n")

        # ── Step 4: Return arm to home ────────────────────────────────────────
        input("[ STEP 4 ]  Press Enter — return arm to home position...")
        robot.arm_swing_to(SWING_HOME)
        print(f"  ✓ Arm returned to home (swing={SWING_HOME}, elevator={ELEVATOR_HOME}).\n")

        print("═══ All steps complete! ═══")
        print("If all motions looked correct, run the full sequence:\n")
        input("[ FULL RUN ]  Press Enter to run the COMPLETE sequence from scratch...")

        # Reset arm first
        robot.arm_open_gripper()
        robot.arm_elevator_to(ELEVATOR_HOME)
        robot.arm_swing_to(SWING_HOME)

        robot.assemble_burger(home_on_start=False)
        print("\n  ✓ Full burger assembly complete!\n")

    except KeyboardInterrupt:
        print("\nAborted — returning arm to home...")
        try:
            robot.stop()
            robot.arm_elevator_to(ELEVATOR_HOME)
            robot.arm_swing_to(SWING_HOME)
            robot.arm_open_gripper()
        except Exception:
            pass

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
