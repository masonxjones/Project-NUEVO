"""
ros2_ws/src/robot/robot/examples/test_of_gripper.py
────────────────────────────────────────────────────
Gripper test script — uses the ROS bridge (NOT raw serial).

Run from inside the Docker container after sourcing both setup files:

    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash
    python3 src/robot/robot/examples/test_of_gripper.py

What it tests (press Enter to advance each step):
    1. Open gripper
    2. Close gripper to fixed position
    3. Squeeze until resistance (incremental)
    4. Full burger pick sequence at a test height
"""

import rclpy
from rclpy.node import Node

from robot.robot import Robot
from robot.robot_impl.gripper import (
    GRIPPER_CHANNEL,
    GRIPPER_OPEN_US,
    GRIPPER_CLOSED_US,
    GRIPPER_RESISTANCE_THRESHOLD,
)

# ── Tune these for your hardware ──────────────────────────────────────────────
TEST_STEPPER        = 1       # which stepper is the arm
TEST_DROP_STEPS     = 500     # steps to drop for the burger pick test
TEST_LIFT_STEPS     = 0       # steps to return to after grab


def main() -> None:
    rclpy.init()
    node = Node("gripper_test")
    robot = Robot(node)

    # Spin ROS in background so subscriptions stay alive
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Give the bridge a moment to deliver the first state messages
    import time
    time.sleep(1.0)

    print("\n=== Gripper Test Script ===")
    print(f"Channel: {GRIPPER_CHANNEL}  |  Open: {GRIPPER_OPEN_US} µs  |  Closed: {GRIPPER_CLOSED_US} µs")
    print("Press Enter at each prompt to continue, Ctrl+C to abort.\n")

    try:
        # ── Test 1: Open ──────────────────────────────────────────────────────
        input("[ 1 / 4 ]  Press Enter to OPEN the gripper...")
        print("  → Opening gripper...")
        robot.open_gripper()
        time.sleep(0.5)
        print("  ✓ Done.\n")

        # ── Test 2: Close ─────────────────────────────────────────────────────
        input("[ 2 / 4 ]  Press Enter to CLOSE the gripper to fixed position...")
        print("  → Closing gripper...")
        robot.close_gripper()
        time.sleep(0.5)
        print("  ✓ Done.\n")

        # ── Test 3: Squeeze until resistance ──────────────────────────────────
        input("[ 3 / 4 ]  Press Enter to squeeze until RESISTANCE is detected...")
        print("  → Opening first, then squeezing incrementally...")
        robot.open_gripper()
        time.sleep(0.3)
        final_pulse = robot.squeeze_until_resistance(
            resistance_threshold=GRIPPER_RESISTANCE_THRESHOLD,
        )
        print(f"  ✓ Resistance detected (or max reached) at {final_pulse} µs.\n")

        # ── Test 4: Full burger pick ───────────────────────────────────────────
        input(
            f"[ 4 / 4 ]  Press Enter to run a FULL BURGER PICK "
            f"(drop to {TEST_DROP_STEPS} steps, grab, lift back to {TEST_LIFT_STEPS})..."
        )
        print("  → Running burger_pick sequence...")
        robot.burger_pick(
            height_steps=TEST_DROP_STEPS,
            stepper=TEST_STEPPER,
            lift_steps=TEST_LIFT_STEPS,
        )
        print("  ✓ Burger pick complete.\n")

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        print("Opening gripper before exit...")
        robot.open_gripper()
        import time as _t
        _t.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()