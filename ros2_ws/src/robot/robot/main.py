from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 26
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 76.2
WHEEL_BASE = 238.76
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


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
   
    # Keeps vision tracking subscriber active
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    print("[FSM] Starting full Autonomous Navigation System.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
   
    # Global tracking variables for the navigation system
    path_points = []
    remaining_path = []
    planner1 = None
    LOOKAHEAD_DIST = 100.0  # Lookahead distance in mm

    while True:
        if robot.get_button(Button.BTN_2):
            print("BTN_2 pressed. Shutting down.")
            robot.stop()
            robot.shutdown()
            break

        # Check for any high-confidence stop signs in view
        stop_sign_detected = False
        for detection in robot.get_detections("stop sign"):
            if float(detection.get("confidence", 0.0)) >= 0.50:
                stop_sign_detected = True
                break

        # Check live camera stream detections for traffic lights
        traffic_light_color = None
        for detection in robot.get_detections("traffic light"):
            if float(detection.get("confidence", 0.0)) >= 0.50:
                attributes = detection.get("attributes", {})
                color_attribute = attributes.get("color", {})
                color = color_attribute.get("value")
                if color in ("red", "green"):
                    traffic_light_color = color
                    break

        # -- STATE 1: INITIALIZE THE MAP PATH --
        if state == "INIT":
            start_robot(robot)
           
            # Example: 61cm x 61cm square trajectory mapping (Task 3 requirement)
            path_control_points = [
                (0.0, 0.0),
                (0.0, 610.0),
                (610.0, 610.0),
                (610.0, 0.0),
                (0.0, 0.0)
            ]
           
            # Subdivide and densify the map coordinate list
            path_points = densify_polyline(path_control_points, spacing=20.0)
            remaining_path = path_points.copy()
           
            # Initialize the tracking geometry class parameters
            planner1 = PurePursuitPlanner(
                lookahead_dist=LOOKAHEAD_DIST,
                max_angular=1.5,
                goal_tolerance=20.0
            )
           
            print("[FSM] Path compiled and planner calibrated. Swapping to standby.")
            state = "WAIT_FOR_GREEN"

        # -- STATE 2: STANDBY AT A COMPLETE STOP --
        elif state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)

            # Only enter driving routine if path is cleared and light is green
            if traffic_light_color == "green" and not stop_sign_detected:
                print("[VISION] Path cleared with green light! Beginning pure pursuit tracking.")
                state = "DRIVING"

        # -- STATE 3: PURE PURSUIT PATH NAVIGATION LOOP --
        elif state == "DRIVING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            # Interrupt movement instantly if a stop condition arises
            if traffic_light_color == "red":
                print("[VISION] Red light encountered! Halting path tracking.")
                state = "WAIT_FOR_GREEN"
                continue
            elif stop_sign_detected:
                print("[VISION] Stop sign encountered! Halting path tracking.")
                state = "WAIT_FOR_GREEN"
                continue

            # --- PROCESS PURE PURSUIT CALCULATIONS ---
            # Step 1: Read actual position and direction angle from map nodes
            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            # Step 2: Clear passed target waypoints out of the active array buffer
            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y, advance_radius_mm=LOOKAHEAD_DIST
            )

            # Step 3: Check if we have run out of waypoints (Completed the square track)
            if not remaining_path or len(remaining_path) == 0:
                print("[NAV] Destination reached! No remaining waypoints left.")
                robot.stop()
                state = "WAIT_FOR_GREEN"
                # Reset path tracking so it can be triggered to run again if needed
                remaining_path = path_points.copy()
                continue

            # Step 4: Resolve optimal directional velocity adjustments
            # PurePursuitPlanner maps lookahead values internally using remaining_path
            linear_velocity_cmd, angular_velocity_cmd_rad_s = planner1.compute_velocity(
                pose=(current_x, current_y, current_theta_rad),
                waypoints=remaining_path,
                max_linear=80.0
            )

            # Step 5: Command internal motor controller dynamically
            robot.set_velocity(
                linear_velocity_cmd,
                math.degrees(angular_velocity_cmd_rad_s)
            )

        # Loop timing constraints
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()