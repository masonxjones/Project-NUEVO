"""
ros2_ws/src/robot/robot/examples/test_burger_assembly.py
─────────────────────────────────────────────────────────
Sequential burger assembly test.

Run inside Docker:
    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash
    python3 -u src/robot/robot/examples/test_burger_assembly.py

TUNE BEFORE RUNNING (all in burger_assembly.py):
    ELEVATOR_BUN_PICK     — steps to reach the top bun
    ELEVATOR_PATTY_PICK   — steps to reach the patty
    ELEVATOR_STACK_PLACE  — steps to lower stack onto bottom bun
    DRIVE_TO_PATTY_MM     — distance from bun to patty
    DRIVE_TO_STACK_MM     — distance from patty to bottom bun
    SWING_STACK_STEPS     — swing position above the bottom bun
    PIXELS_TO_STEPS_GAIN  — tune until arm centres correctly on camera
"""

import threading
import time

import rclpy
from rclpy.node import Node

from robot.robot import Robot
from robot.robot_impl.burger_assembly import (
    ELEVATOR_BUN_PICK,
    ELEVATOR_PATTY_PICK,
    ELEVATOR_STACK_PLACE,
    DRIVE_TO_PATTY_MM,
    DRIVE_TO_STACK_MM,
    GRIP_US,
    SWING_STACK_STEPS,
    SWING_SCAN_START,
    SWING_SCAN_END,
)

HOME_ON_START = False


def main() -> None:
    rclpy.init()
    node = Node("burger_assembly_test")
    robot = Robot(node)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("\n═══ Sequential Burger Assembly Test ═══")
    print(f"Grip pulse:          {GRIP_US} µs (all pieces)")
    print(f"Elevator — bun:      {ELEVATOR_BUN_PICK} steps")
    print(f"Elevator — patty:    {ELEVATOR_PATTY_PICK} steps")
    print(f"Elevator — place:    {ELEVATOR_STACK_PLACE} steps")
    print(f"Drive bun→patty:     {DRIVE_TO_PATTY_MM} mm")
    print(f"Drive patty→stack:   {DRIVE_TO_STACK_MM} mm")
    print(f"Swing scan range:    {SWING_SCAN_START}→{SWING_SCAN_END} steps")
    print(f"Swing stack pos:     {SWING_STACK_STEPS} steps")
    print()

    try:
        # ── Pre-checks ────────────────────────────────────────────────────────
        input("[ PRE-CHECK ]  Press Enter to verify gripper open/close...")
        robot.open_gripper()
        time.sleep(0.4)
        robot.close_gripper()
        time.sleep(0.4)
        robot.open_gripper()
        print("  ✓ Gripper OK.\n")

        input("[ PRE-CHECK ]  Press Enter to raise elevator to safe height...")
        robot.arm_safe_height()
        print("  ✓ Safe height OK.\n")

        if HOME_ON_START:
            input("[ HOMING ]  Press Enter to home both axes...")
            robot.arm_home()
            print("  ✓ Homing complete.\n")

        # ── Camera check ──────────────────────────────────────────────────────
        input("[ CAMERA ]  Press Enter to sweep for any piece (verify camera)...")
        robot.arm_safe_height()
        swing = robot.arm_sweep_and_align("any piece")
        if swing is not None:
            print(f"  ✓ Piece found and centred at swing={swing} steps.\n")
        else:
            print("  ✗ No piece found — check camera / vision node.\n")
        robot.arm_safe_height()

        # ── Step-by-step test ─────────────────────────────────────────────────
        input("[ STEP 1 ]  Press Enter to pick the TOP BUN...")
        robot.arm_safe_height()
        aligned = robot.arm_sweep_and_align("top bun")
        if aligned:
            robot.arm_open_gripper()
            robot.arm_elevator_to(ELEVATOR_BUN_PICK)
            robot.arm_grip()
            robot.arm_safe_height()
            print("  ✓ Top bun picked.\n")
        else:
            print("  ✗ Bun not found — check placement and camera.\n")

        input("[ STEP 2 ]  Press Enter to drive to patty and pick patty + bun...")
        robot._drive_forward(DRIVE_TO_PATTY_MM)
        robot.arm_elevator_to(ELEVATOR_PATTY_PICK)
        robot.arm_grip()
        robot.arm_safe_height()
        print("  ✓ Patty + bun picked.\n")

        input("[ STEP 3 ]  Press Enter to drive to bottom bun and place stack...")
        robot._drive_forward(DRIVE_TO_STACK_MM)
        robot.arm_swing_to(SWING_STACK_STEPS)
        robot.arm_elevator_to(ELEVATOR_STACK_PLACE)
        robot.arm_open_gripper()
        robot.arm_safe_height()
        print("  ✓ Stack placed on bottom bun.\n")

        # ── Full run ──────────────────────────────────────────────────────────
        input("[ FULL RUN ]  Press Enter to run the COMPLETE sequence from scratch...")
        robot.assemble_burger(
            home_on_start  = False,
            park_on_finish = True,
        )
        print("\n  ✓ Burger assembly complete!\n")

    except KeyboardInterrupt:
        print("\nAborted — raising arm and stopping robot...")
        try:
            robot.stop()
            robot.arm_safe_height()
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