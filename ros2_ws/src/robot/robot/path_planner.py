from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlannerWithAvoidance

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
# Planner parameters — tune these for your arena
# ---------------------------------------------------------------------------

LOOKAHEAD_DIST    = 100.0   # mm
MAX_LINEAR_VEL    = 80.0    # mm/s
MAX_ANGULAR_VEL   = 1.5     # rad/s
GOAL_TOLERANCE    = 20.0    # mm

OBSTACLES_RANGE   = 400.0   # mm — lidar points beyond this are ignored
SAFE_DIST         = 150.0   # mm — minimum clearance before lane-switch triggers
OFFSET            = 120.0   # mm — lateral shift when switching lanes
LANE_WIDTH        = 500.0   # mm — corridor width around x_L centerline
VIEW_ANGLE        = np.pi / 2  # rad — lidar FOV half-angle (90° each side = 180° total)
AVOIDANCE_DELAY   = 200     # FSM ticks to hold avoidance active after cone clears

# ---------------------------------------------------------------------------
# Waypoints
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
    robot.enable_lidar()          # subscribe to lidar topic


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def get_traffic_light(robot: Robot):
    for detection in robot.get_detections("traffic light"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            color = detection.get("attributes", {}).get("color", {}).get("value")
            if color in ("red", "green"):
                return color
    return None


def get_lidar_obstacles(robot: Robot) -> np.ndarray:
    """
    Pull the latest lidar scan and return an (N, 2) array of obstacle
    points in robot frame (mm). Returns an empty array if no scan yet.

    robot.get_lidar_scan() is expected to return a list/array of
    (x_mm, y_mm) points already in robot frame, matching what
    PurePursuitPlannerWithAvoidance.compute_velocity() expects.
    """
    scan = robot.get_lidar_scan()           # returns list of (x, y) or np array
    if scan is None or len(scan) == 0:
        return np.zeros((0, 2), dtype=float)
    pts = np.asarray(scan, dtype=float)
    if pts.ndim == 1:
        pts = pts.reshape(-1, 2)
    return pts


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # ------------------------------------------------------------------
    # Build densified path — needed so the planner's _advance_remaining_path
    # pops points smoothly rather than taking huge jumps at sparse corners
    # ------------------------------------------------------------------
    dense_path = densify_polyline(list(RAW_WAYPOINTS), spacing=20.0)

    planner = PurePursuitPlannerWithAvoidance(
        lookahead_distance  = LOOKAHEAD_DIST,
        max_linear_speed    = MAX_LINEAR_VEL,
        max_angular_speed   = MAX_ANGULAR_VEL,
        goal_tolerance      = GOAL_TOLERANCE,
        obstacles_range     = OBSTACLES_RANGE,
        view_angle          = VIEW_ANGLE,
        safe_dist           = SAFE_DIST,
        avoidance_delay     = AVOIDANCE_DELAY,
        offset              = OFFSET,
        lane_width          = LANE_WIDTH,
        obstacle_avoidance  = True,
    )

    state = "WAIT_FOR_GREEN"
    print("[FSM] Waiting for GREEN traffic light...")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # Emergency stop
        if robot.get_button(Button.BTN_2):
            print("BTN_2 — emergency stop.")
            robot.stop()
            robot.shutdown()
            break

        traffic_light_color = get_traffic_light(robot)

        # ==================================================================
        # STATE: WAIT_FOR_GREEN
        # ==================================================================
        if state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)

            if traffic_light_color == "green":
                print("[VISION] Green — loading path and starting.")

                # set_path() initialises remaining_path and raw_path inside
                # the planner; call it fresh each time we (re-)start
                planner.set_path(dense_path)

                print("[FSM] → MOVING")
                state = "MOVING"

        # ==================================================================
        # STATE: MOVING  — pure pursuit + lidar obstacle avoidance
        # ==================================================================
        elif state == "MOVING":

            if traffic_light_color == "red":
                print("[VISION] Red — pausing.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                state = "PAUSED"

            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)

                # Current pose
                current_x, current_y, current_theta_deg = robot.get_pose()
                current_theta_rad = math.radians(current_theta_deg)
                pose = (current_x, current_y, current_theta_rad)

                # Lidar scan in robot frame — (N, 2) array
                obstacles_r = get_lidar_obstacles(robot)

                # Check goal reached against the planner's internal remaining path
                if planner.TargetReached(planner.remaining_path, current_x, current_y):
                    print("[MOVING] Final waypoint reached — done.")
                    robot.stop()
                    robot.set_led(LED.GREEN, 0)
                    print("[FSM] → WAIT_FOR_GREEN")
                    state = "WAIT_FOR_GREEN"

                else:
                    # compute_velocity handles advance, lookahead, AND
                    # lane-switch avoidance internally
                    v, w_rad_s = planner.compute_velocity(pose, obstacles_r)

                    robot.set_velocity(v, math.degrees(w_rad_s))

                    # Debug (uncomment to enable)
                    # print(f"Pose: ({current_x:.0f}, {current_y:.0f}, "
                    #       f"{current_theta_deg:.1f}°) | v={v:.1f} w={math.degrees(w_rad_s):.1f}°/s "
                    #       f"| lane={planner.current_lane} | obs={len(obstacles_r)}")

        # ==================================================================
        # STATE: PAUSED — red light mid-route; planner state is preserved
        # ==================================================================
        elif state == "PAUSED":
            robot.stop()
            robot.set_led(LED.ORANGE, 255)

            if traffic_light_color == "green":
                print("[VISION] Green again — resuming.")
                robot.set_led(LED.ORANGE, 0)
                state = "MOVING"

        # Tick rate
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()