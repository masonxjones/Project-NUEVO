from __future__ import annotations
import time
import math
import numpy as np


from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
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


LOOKAHEAD_DIST = 100.0       # mm — tune as needed (Task 4: try 50, 100, 150)
MAX_LINEAR_VEL = 80.0        # mm/s
MAX_ANGULAR_VEL = 1.5        # rad/s
GOAL_TOLERANCE = 20.0        # mm


# ---------------------------------------------------------------------------
# Waypoints  (Task 3: densified so corners are sharper)
# ---------------------------------------------------------------------------


RAW_WAYPOINTS = [
    (0.0,    0.0),
    (0.0,    3660.0),
    (610.0,  3660.0),
    (610.0,  610.0),
    (1525.0, 610.0),
    (1525.0, 3660.0),
    (2440.0, 3660.0),
    (2440.0, 0.0),
]




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


    # ------------------------------------------------------------------
    # Build densified path (Task 3 fix: adds intermediate points so the
    # planner doesn't cut corners)
    # ------------------------------------------------------------------
    path_control_points = list(RAW_WAYPOINTS)
    path1 = densify_polyline(path_control_points, spacing=20.0)


    planner1 = None
    remaining_path = []


    state = "WAIT_FOR_GREEN"
    print("[FSM] Waiting for GREEN traffic light before starting path...")


    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()


    while True:
        # ------------------------------------------------------------------
        # Hardware emergency stop (BTN_2)
        # ------------------------------------------------------------------
        if robot.get_button(Button.BTN_2):
            print("BTN_2 pressed — emergency stop.")
            robot.stop()
            robot.shutdown()
            break


        traffic_light_color = get_traffic_light(robot)


        # ==================================================================
        # STATE: WAIT_FOR_GREEN
        #   Hold position; LEDs off; watch for green light
        # ==================================================================
        if state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)


            if traffic_light_color == "green":
                print("[VISION] Green detected — initialising Pure Pursuit planner.")


                planner1 = PurePursuitPlanner(
                    lookahead_dist=LOOKAHEAD_DIST,
                    max_angular=MAX_ANGULAR_VEL,
                    goal_tolerance=GOAL_TOLERANCE,
                )
                remaining_path = path1.copy()


                print("[FSM] → MOVING")
                state = "MOVING"


        # ==================================================================
        # STATE: MOVING
        #   Follow waypoints via Pure Pursuit; pause on red light
        # ==================================================================
        elif state == "MOVING":
            # ---- Pause on red ----
            if traffic_light_color == "red":
                print("[VISION] Red light — pausing.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                state = "PAUSED"


            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)


                # Step 1: current pose
                current_x, current_y, current_theta_deg = robot.get_pose()


                # Step 2: heading in radians
                current_theta_rad = math.radians(current_theta_deg)


                # Step 3: advance (trim already-passed waypoints)
                remaining_path = robot._advance_remaining_path(
                    remaining_path,
                    current_x,
                    current_y,
                    advance_radius_mm=LOOKAHEAD_DIST,
                )


                # Step 4: lookahead point
                current_pursuit_x, current_pursuit_y = planner1._lookahead_point(
                    current_x,
                    current_y,
                    waypoints=remaining_path,
                )


                # Step 5: compute v and ω
                linear_velocity_cmd, angular_velocity_cmd_rad_s = planner1.compute_velocity(
                    pose=(current_x, current_y, current_theta_rad),
                    waypoints=remaining_path,
                    max_linear=MAX_LINEAR_VEL,
                )


                # Step 6: send velocity command
                # robot.set_velocity expects (linear mm/s, angular deg/s)
                robot.set_velocity(
                    linear_velocity_cmd,
                    math.degrees(angular_velocity_cmd_rad_s),
                )


                # Step 7: check goal reached
                if planner1.CurrentTargetReached(
                    current_pursuit_x, current_pursuit_y,
                    current_x, current_y,
                ):
                    print("[MOVING] Final waypoint reached — stopping.")
                    robot.stop()
                    robot.set_led(LED.GREEN, 0)
                    robot.set_led(LED.ORANGE, 0)
                    print("[FSM] → WAIT_FOR_GREEN")
                    state = "WAIT_FOR_GREEN"


                # Step 8: debug print
                # print(f"Pose: ({current_x:.1f}, {current_y:.1f}, {current_theta_deg:.1f}°) "
                #       f"| Target: ({current_pursuit_x:.1f}, {current_pursuit_y:.1f})")


        # ==================================================================
        # STATE: PAUSED  (red light mid-route)
        #   Hold until green resumes; planner state is preserved
        # ==================================================================
        elif state == "PAUSED":
            robot.stop()
            robot.set_led(LED.ORANGE, 255)


            if traffic_light_color == "green":
                print("[VISION] Green again — resuming path.")
                robot.set_led(LED.ORANGE, 0)
                state = "MOVING"


        # ------------------------------------------------------------------
        # FSM tick rate
        # ------------------------------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

