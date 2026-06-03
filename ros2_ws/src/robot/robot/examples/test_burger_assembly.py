"""
ros2_ws/src/robot/robot/examples/test_burger_assembly.py
─────────────────────────────────────────────────────────
Burger assembly test — camera-guided pick and stack.

Run inside Docker:
    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash
    python3 -u src/robot/robot/examples/test_burger_assembly.py

TUNE BEFORE RUNNING (in burger_assembly.py):
    ELEVATOR_PATTY_PICK   — steps to reach the patty
    ELEVATOR_BUN_PICK     — steps to reach a bun
    ELEVATOR_PLACE_STEP1  — table height (bottom bun already here)
    ELEVATOR_PLACE_STEP2  — height after patty is placed
    ELEVATOR_STACK_GRAB   — depth to lower around full stack
    SWING_SCAN_START/END  — sweep range in steps
    PIXELS_TO_STEPS_GAIN  — tune until arm centres correctly

    STACK_SWING_STEPS below — swing position directly above the assembly stack.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from robot.robot import Robot
from robot.robot_impl.burger_assembly import (
    BurgerAssemblyMixin,
    SWING_SCAN_START,
    SWING_SCAN_END,
    ELEVATOR_PATTY_PICK,
    ELEVATOR_BUN_PICK,
    ELEVATOR_PLACE_STEP1,
    ELEVATOR_PLACE_STEP2,
    ELEVATOR_STACK_GRAB,
    PATTY_GRIP_US,
    BUN_GRIP_US,
    STACK_GRIP_US,
    PATTY_RADIUS_MM,
    BUN_RADIUS_MM,
)

# ── TUNE THIS: swing step position directly above the assembly stack ──────────
STACK_SWING_STEPS = 500    # adjust to where the bottom bun sits on the table

# ── Optional: home the arm at startup ────────────────────────────────────────
HOME_ON_START = False


# ── Navigation stub — your team fills this in ─────────────────────────────────
def navigate_to_piece(piece_type: str) -> None:
    """
    Called before each pick. Replace with your team's navigation.
    piece_type is one of: "patty", "top_bun", "stack"
    """
    # TODO: robot.navigate_to(x, y) based on piece_type
    pass


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = Node("burger_assembly_test")
    robot = Robot(node)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("\n═══ Burger Assembly Test ═══")
    print(f"Stack position:   swing={STACK_SWING_STEPS} steps")
    print(f"Sweep range:      {SWING_SCAN_START} → {SWING_SCAN_END} steps")
    print(f"Patty grip:       {PATTY_GRIP_US} µs  (radius {PATTY_RADIUS_MM:.0f} mm)")
    print(f"Bun grip:         {BUN_GRIP_US} µs  (radius {BUN_RADIUS_MM:.0f} mm)")
    print(f"Stack grip:       {STACK_GRIP_US} µs")
    print(f"Elevator depths:  patty={ELEVATOR_PATTY_PICK}  bun={ELEVATOR_BUN_PICK}")
    print(f"Place heights:    step1={ELEVATOR_PLACE_STEP1}  step2={ELEVATOR_PLACE_STEP2}")
    print(f"Stack grab depth: {ELEVATOR_STACK_GRAB}")
    print()

    try:
        # ── Pre-checks ────────────────────────────────────────────────────────
        input("[ PRE-CHECK ]  Press Enter to verify gripper open/close...")
        robot.arm_open_gripper()
        time.sleep(0.4)
        robot.arm_grip(BUN_GRIP_US)
        time.sleep(0.4)
        robot.arm_open_gripper()
        print("  ✓ Gripper OK.\n")

        input("[ PRE-CHECK ]  Press Enter to raise elevator to safe height...")
        robot.arm_safe_height()
        print("  ✓ Elevator at safe height.\n")

        if HOME_ON_START:
            input("[ HOMING ]  Press Enter to home both axes...")
            robot.arm_home()
            print("  ✓ Homing complete.\n")

        # ── Camera detection check ────────────────────────────────────────────
        input("[ CAMERA CHECK ]  Press Enter to sweep for PATTY (red)...")
        robot.arm_safe_height()
        swing = robot.arm_sweep_and_align("red_block")
        if swing is not None:
            print(f"  ✓ Patty found and centred at swing={swing} steps.\n")
        else:
            print("  ✗ Patty NOT found. Check camera / vision node before continuing.\n")

        input("[ CAMERA CHECK ]  Press Enter to sweep for BUN (yellow)...")
        robot.arm_safe_height()
        swing = robot.arm_sweep_and_align("yellow_block")
        if swing is not None:
            print(f"  ✓ Bun found and centred at swing={swing} steps.\n")
        else:
            print("  ✗ Bun NOT found. Check camera / vision node before continuing.\n")

        # ── Full assembly ─────────────────────────────────────────────────────
        input("[ ASSEMBLY ]  Press Enter to run the FULL burger assembly sequence...")
        robot.assemble_burger(
            stack_swing_steps  = STACK_SWING_STEPS,
            navigate_to_piece  = navigate_to_piece,
            home_on_start      = False,
            park_on_finish     = True,
        )
        print("\n  ✓ Burger assembly complete — stack is in the gripper!\n")

    except KeyboardInterrupt:
        print("\nAborted — raising arm to safe height...")
        try:
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