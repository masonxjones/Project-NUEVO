from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.path_planner import PurePursuitPlanner

# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 11
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Pure Pursuit parameters
# ---------------------------------------------------------------------------

LOOKAHEAD_DIST = 120.0       # mm
MAX_LINEAR_VEL = 120.0       # mm/s
MAX_ANGULAR_VEL = 2.0        # rad/s
GOAL_TOLERANCE = 20.0        # mm
PRODUCTION_THRESHOLD = 0.25  # Vision matching threshold

# ---------------------------------------------------------------------------
# Field Coordinates
# ---------------------------------------------------------------------------

CHECKPOINT_X = 2440.0
CHECKPOINT_Y = 3660.0
CHECKPOINT_RADIUS = 60.0     # mm tolerance to trigger the arrival pause

DELIVERY_LOCATIONS = {
    "A": (2440.0, 1525.0),    # Target coordinates for Customer A
    "B": (2440.0, 1225.0),    # Target coordinates for Customer B
    "unknown": (2440.0, 1325.0)  # Fallback coordinates
}

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID)
    robot.enable_vision()

def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

def get_traffic_light(robot: Robot):
    """Return 'green', 'red', or None based on highest-confidence detection."""
    for detection in robot.get_detections("traffic light"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            color = detection.get("attributes", {}).get("color", {}).get("value")
            if color in ("red", "green"):
                return color
    return None

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # State machine starts by driving straight to the customer setup area
    state = "DRIVING_TO_CHECKPOINT"
    print("[SYSTEM] Initializing. Driving straight to Customer Checkpoint...")

    # Establish initial path directly to the checkpoint coordinates
    remaining_path = [(0.0, 0.0), (CHECKPOINT_X, CHECKPOINT_Y)]
    planner = PurePursuitPlanner(
        lookahead_dist=LOOKAHEAD_DIST,
        max_angular=MAX_ANGULAR_VEL,
        goal_tolerance=GOAL_TOLERANCE,
    )

    checkpoint_stop_start = 0.0
    detected_customer = "unknown"
    highest_match_score = -1.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if robot.get_button(Button.BTN_2):
            print("BTN_2 pressed — emergency stop.")
            robot.stop()
            robot.shutdown()
            break

        # ==================================================================
        # STATE 1: DRIVE STRAIGHT TO THE CHECKPOINT
        # ==================================================================
        if state == "DRIVING_TO_CHECKPOINT":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            # Check if we are physically close enough to the checkpoint to stop
            dist_to_checkpoint = math.sqrt((current_x - CHECKPOINT_X)**2 + (current_y - CHECKPOINT_Y)**2)
            if dist_to_checkpoint <= CHECKPOINT_RADIUS:
                print(f"[NAV] Arrived at customer location ({current_x:.1f}, {current_y:.1f}). Starting 5s scan...")
                robot.stop()
                checkpoint_stop_start = time.monotonic()
                state = "CUSTOMER_CHECKPOINT"
                continue

            # Standard Pure Pursuit tracking mechanics
            remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, LOOKAHEAD_DIST)
            cp_x, cp_y = planner._lookahead_point(current_x, current_y, remaining_path)
            v, omega = planner.compute_velocity((current_x, current_y, current_theta_rad), remaining_path, MAX_LINEAR_VEL)
            robot.set_velocity(v, math.degrees(omega))

        # ==================================================================
        # STATE 2: SCAN FOR CUSTOMER (At the actual checkpoint)
        # ==================================================================
        elif state == "CUSTOMER_CHECKPOINT":
            robot.stop() 
            robot.set_led(LED.ORANGE, 255)
            robot.set_led(LED.GREEN, 0)

            for person in robot.get_detections("person"):
                attributes = person.get("attributes", {})
                if "customer_id" in attributes:
                    customer_attr = attributes.get("customer_id", {})
                    customer_id = customer_attr.get("value")
                    confidence = float(customer_attr.get("score", 0.0))

                    if customer_id in ("A", "B") and confidence >= PRODUCTION_THRESHOLD:
                        if confidence > highest_match_score:
                            highest_match_score = confidence
                            detected_customer = customer_id

            if time.monotonic() - checkpoint_stop_start >= 5.0:
                print(f"[DECISION] Scan complete. Target: Customer {detected_customer.upper()} "
                      f"(Confidence: {max(0.0, highest_match_score):.2f})")
                print("[FSM] Standby for GREEN light to resume delivery run...")
                state = "WAIT_FOR_LAUNCH_LIGHT"

        # ==================================================================
        # STATE 3: STANDBY FOR GREEN LIGHT
        # ==================================================================
        elif state == "WAIT_FOR_LAUNCH_LIGHT":
            robot.stop()
            robot.set_led(LED.ORANGE, 255)
            robot.set_led(LED.GREEN, 0)

            if get_traffic_light(robot) == "green":
                print("[VISION] Green light detected! Re-routing straight to drop-off.")
                
                target_x, target_y = DELIVERY_LOCATIONS[detected_customer]
                # Path maps a straight trajectory starting from the current checkpoint out to delivery coordinates
                remaining_path = [(CHECKPOINT_X, CHECKPOINT_Y), (target_x, target_y)]
                
                planner = PurePursuitPlanner(
                    lookahead_dist=LOOKAHEAD_DIST,
                    max_angular=MAX_ANGULAR_VEL,
                    goal_tolerance=GOAL_TOLERANCE,
                )
                state = "MOVING_TO_DELIVERY"

        # ==================================================================
        # STATE 4: DRIVE DIRECTLY TO CHOSEN CUSTOMER DROP-OFF
        # ==================================================================
        elif state == "MOVING_TO_DELIVERY":
            robot.set_led(LED.ORANGE, 0)
            robot.set_led(LED.GREEN, 255)

            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, LOOKAHEAD_DIST)
            cp_x, cp_y = planner._lookahead_point(current_x, current_y, remaining_path)
            v, omega = planner.compute_velocity((current_x, current_y, current_theta_rad), remaining_path, MAX_LINEAR_VEL)
            robot.set_velocity(v, math.degrees(omega))

            if planner.CurrentTargetReached(cp_x, cp_y, current_x, current_y):
                print("[ARRIVED] Delivery complete! Stopping robot.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                break

        # FSM tick rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

if __name__ == "__main__":
    main()