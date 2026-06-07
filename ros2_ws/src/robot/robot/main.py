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

TAG_ID            = 11
POSITION_UNIT     = Unit.MM
WHEEL_DIAMETER    = 76.2
WHEEL_BASE        = 241.3
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Pure Pursuit parameters (plain legs)
# ---------------------------------------------------------------------------

LOOKAHEAD_DIST  = 100.0   # mm
MAX_LINEAR_VEL  = 140.0   # mm/s
MAX_ANGULAR_VEL = 1.5     # rad/s
GOAL_TOLERANCE  = 20.0    # mm

# ---------------------------------------------------------------------------
# PurePursuitPlannerWithAvoidance parameters (corridor leg)
# ---------------------------------------------------------------------------

AVOID_LOOKAHEAD     = 100.0
AVOID_MAX_LINEAR    = 140.0
AVOID_MAX_ANGULAR   = 1.5
AVOID_GOAL_TOL      = 20.0
AVOID_OBS_RANGE     = 450.0
AVOID_VIEW_ANGLE    = math.radians(70.0)
AVOID_SAFE_DIST     = 250.0
AVOID_DELAY         = 150
AVOID_ALPHA_LD      = 0.7
AVOID_OFFSET        = 270.0
AVOID_LANE_WIDTH    = 500.0
AVOID_X_L           = 1325.0   # must match the corridor x-coordinate so lane
                                # filtering and obstacle side-detection are
                                # relative to the actual path, not x=0

# ---------------------------------------------------------------------------
# Arm / assembly constants
# ---------------------------------------------------------------------------

TRAFFIC_LIGHT_SWING_STEPS = -150
ELEVATOR_HOME = 0
SWING_HOME    = 0

# ---------------------------------------------------------------------------
# Assembly + delivery spatial triggers
# ---------------------------------------------------------------------------

ASSEMBLY_TRIGGER_Y_MM  = 842.9
ASSEMBLY_TRIGGER_X_MAX = 50.0
DELIVERY_TRIGGER_Y_MM  = 1200.0
DELIVERY_TRIGGER_X_MIN = 2000.0

# ---------------------------------------------------------------------------
# Vision / delivery config
# ---------------------------------------------------------------------------

PRODUCTION_THRESHOLD = 0.25

DELIVERY_LOCATIONS = {
    "A":       (2440.0, 1525.0),
    "B":       (2440.0, 1225.0),
    "unknown": (2440.0, 1325.0),
}

# ---------------------------------------------------------------------------
# Course waypoints — split at the corridor entry point
# ---------------------------------------------------------------------------

# LEG1A ends at corridor entry — avoidance planner takes over from there
LEG1A_WAYPOINTS = [
    (0.0,    0.0),
    (0.0,    3560.0),
    (410.0,  3560.0),
    (410.0,  610.0),
    (1325.0, 590.0),
    (1325.0, 610.0),   # ← hand-off point: switch to avoidance planner here
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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
    robot.enable_lidar()


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def get_traffic_light(robot: Robot):
    """Return 'green', 'red', or None based on highest-confidence detection."""
    for detection in robot.get_detections("traffic light"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            color = detection.get("attributes", {}).get("color", {}).get("value")
            if color in ("red", "green"):
                return color
    return None


def detect_stop_sign(robot: Robot) -> bool:
    for detection in robot.get_detections("stop sign"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            return True
    return False


def make_planner() -> PurePursuitPlanner:
    """Create a fresh plain PurePursuitPlanner."""
    return PurePursuitPlanner(
        lookahead_dist=LOOKAHEAD_DIST,
        max_angular=MAX_ANGULAR_VEL,
        goal_tolerance=GOAL_TOLERANCE,
    )


def drive_step(robot: Robot, planner: PurePursuitPlanner, remaining_path: list) -> tuple[list, bool]:
    """
    One Pure Pursuit tick — confirmed working low-level API.
    Returns (updated_remaining_path, goal_reached).
    """
    current_x, current_y, current_theta_deg = robot.get_pose()
    current_theta_rad = math.radians(current_theta_deg)

    remaining_path = robot._advance_remaining_path(
        remaining_path, current_x, current_y,
        advance_radius_mm=LOOKAHEAD_DIST,
    )

    if len(remaining_path) <= 1:
        robot.stop()
        return remaining_path, True

    pursuit_x, pursuit_y = planner._lookahead_point(
        current_x, current_y, waypoints=remaining_path
    )

    linear_vel, angular_vel_rad = planner.compute_velocity(
        pose=(current_x, current_y, current_theta_rad),
        waypoints=remaining_path,
        max_linear=MAX_LINEAR_VEL,
    )

    robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))

    goal_reached = planner.CurrentTargetReached(
        pursuit_x, pursuit_y, current_x, current_y
    )

    return remaining_path, goal_reached


def init_avoidance_planner(robot: Robot) -> None:
    """
    Set up PurePursuitPlannerWithAvoidance for the cone corridor.

    Two critical fixes vs the naive setup:
    1. x_L=AVOID_X_L (1325) — tells the planner the path runs along x=1325.
       Without this, obstacle side-detection and lane-width filtering are
       relative to x=0, so the planner sees every cone as being on the wrong
       side and steers into the wall.
    2. Force current_lane='Center' after construction — the planner hardcodes
       'Left' in __init__, which immediately shifts all waypoints by -offset
       (~1055mm) before any obstacle is detected, sending the robot sideways.
    """
    robot._nav_follow_pp_path(
        lookahead_distance=AVOID_LOOKAHEAD,
        max_linear_speed=AVOID_MAX_LINEAR,
        max_angular_speed=AVOID_MAX_ANGULAR,
        goal_tolerance=AVOID_GOAL_TOL,
        obstacles_range=AVOID_OBS_RANGE,
        view_angle=AVOID_VIEW_ANGLE,
        safe_dist=AVOID_SAFE_DIST,
        avoidance_delay=AVOID_DELAY,
        alpha_Ld=AVOID_ALPHA_LD,
        offset=AVOID_OFFSET,
        lane_width=AVOID_LANE_WIDTH,
        obstacle_avoidance=True,
        x_L=AVOID_X_L,
    )

    # Override the hardcoded 'Left' default so set_path loads the center lane
    robot.planner.current_lane = 'Center'

    # Start 200mm north of the entry point so no waypoint is immediately
    # behind the robot when the avoidance planner takes over
    leg1b = [
        (1325.0,  810.0),
        (1325.0, 3460.0),
        (2000.0, 3460.0),
    ]
    path = densify_polyline(leg1b, spacing=20.0)
    robot._set_obstacle_avoidance_path(path)


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    state  = "IDLE"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"[FSM] period: {period:.3f}s  — waiting for GREEN light")

    next_tick = time.monotonic()

    # Plain planner + path for non-avoidance legs
    planner:        PurePursuitPlanner = None
    remaining_path: list               = []

    # Assembly / delivery flags — set True to skip burger assembly during testing
    assembly_done  = True
    delivery_done  = True

    # Checkpoint flag — prevents re-triggering as robot passes scan zone
    checkpoint_done = False

    # State-specific trackers
    checkpoint_stop_start = 0.0
    stop_sign_start_time  = 0.0
    stop_sign_detected    = False
    stop_sign_completed   = False
    detected_customer     = "unknown"
    highest_match_score   = -1.0

    while True:

        # ── Global red-light pause (only while actively driving) ────────────
        if state in ("MOVING", "MOVING_AVOID", "DELIVERY_MOVING", "FINAL_MOVING"):
            if not stop_sign_detected:
                if get_traffic_light(robot) == "red":
                    print("[TRAFFIC LIGHT] Red — holding.")
                    robot.stop()
                    show_idle_leds(robot)
                    time.sleep(period)
                    continue

        # ── Emergency stop ──────────────────────────────────────────────────
        if robot.get_button(Button.BTN_2):
            print("BTN_2 — emergency stop.")
            robot.stop()
            robot.shutdown()
            break

        # ====================================================================
        # IDLE  — wait for green, arm swing toward light, then launch
        # ====================================================================
        if state == "IDLE":
            robot.stop()
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()

            try:
                robot.arm_elevator_to(ELEVATOR_HOME)
                robot.arm_swing_to(TRAFFIC_LIGHT_SWING_STEPS)
            except Exception as e:
                print(f"[WARN] Arm swing failed: {e}")

            if get_traffic_light(robot) == "green":
                print("[VISION] Green detected — arming planner.")

                try:
                    robot.arm_swing_to(SWING_HOME)
                except Exception as e:
                    print(f"[WARN] Arm return home failed: {e}")

                planner        = make_planner()
                remaining_path = densify_polyline(LEG1A_WAYPOINTS, spacing=20.0)

                print("[FSM] IDLE → MOVING (LEG1A: plain Pure Pursuit to corridor entry)")
                state = "MOVING"

        # ====================================================================
        # MOVING  — LEG1A: start → corridor entry (1325, 610), plain PP
        # ====================================================================
        elif state == "MOVING":
            show_moving_leds(robot)

            current_x, current_y, _ = robot.get_pose()

            # ── Assembly trigger ─────────────────────────────────────────────
            if (
                not assembly_done
                and current_y >= ASSEMBLY_TRIGGER_Y_MM
                and abs(current_x) <= ASSEMBLY_TRIGGER_X_MAX
            ):
                print(f"[FSM] Assembly trigger at ({current_x:.0f}, {current_y:.0f}) → ASSEMBLY")
                robot.stop()
                state = "ASSEMBLY"

            else:
                remaining_path, goal_reached = drive_step(robot, planner, remaining_path)

                if goal_reached:
                    print("[NAV] Reached corridor entry (1325, 610). Switching to avoidance planner.")
                    robot.stop()
                    init_avoidance_planner(robot)
                    print("[LIDAR] Obstacle avoidance INITIATED")
                    print("[FSM] MOVING → MOVING_AVOID (LEG1B: corridor with cones)")
                    state = "MOVING_AVOID"

        # ====================================================================
        # MOVING_AVOID  — LEG1B: corridor (1325,610) → (2000,3460)
        #                 PurePursuitPlannerWithAvoidance via TA's loop API
        # ====================================================================
        elif state == "MOVING_AVOID":
            show_moving_leds(robot)

            nav_result = robot._nav_follow_pp_path_loop()

            if nav_result == "IDLE":
                print("[LIDAR] Obstacle avoidance DISABLED — corridor complete.")
                print("[NAV] Arrived at scanning station (2000, 3460).")
                robot.stop()
                checkpoint_stop_start = time.monotonic()
                state = "CUSTOMER_CHECKPOINT"

        # ====================================================================
        # ASSEMBLY  — burger pick and stack sequence
        # ====================================================================
        elif state == "ASSEMBLY":
            print("[FSM] Starting burger assembly...")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)

            try:
                robot.assemble_burger(home_on_start=False)
                try:
                    robot.arm_safe_height()
                except Exception:
                    pass
                assembly_done = True
                print("[FSM] Assembly complete — resuming path.")
            except Exception as e:
                print(f"[FSM] Assembly error: {e} — resuming anyway.")
                assembly_done = True
                try:
                    robot.arm_safe_height()
                    robot.arm_open_gripper()
                except Exception:
                    pass

            show_moving_leds(robot)
            state = "MOVING"
            print("[FSM] → MOVING (resumed)")

        # ====================================================================
        # DELIVERY  — burger delivery to customer
        # ====================================================================
        elif state == "DELIVERY":
            print(f"[FSM] Delivering to Customer {detected_customer}...")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)

            try:
                robot.deliver_burger(customer=detected_customer)
                delivery_done = True
                print("[FSM] Delivery complete — resuming path.")
            except Exception as e:
                print(f"[FSM] Delivery error: {e} — resuming anyway.")
                delivery_done = True
                try:
                    robot.arm_safe_height()
                    robot.arm_open_gripper()
                    robot.arm_swing_to(0)
                except Exception:
                    pass

            show_moving_leds(robot)
            state = "MOVING"
            print("[FSM] → MOVING (resumed)")

        # ====================================================================
        # CUSTOMER_CHECKPOINT  — 3-second stationary face scan
        # ====================================================================
        elif state == "CUSTOMER_CHECKPOINT":
            robot.stop()
            show_idle_leds(robot)

            for person in robot.get_detections("person"):
                attributes = person.get("attributes", {})
                if "customer_id" in attributes:
                    customer_attr = attributes["customer_id"]
                    customer_id   = customer_attr.get("value")
                    confidence    = float(customer_attr.get("score", 0.0))

                    if customer_id in ("A", "B") and confidence >= PRODUCTION_THRESHOLD:
                        if confidence > highest_match_score:
                            highest_match_score = confidence
                            detected_customer   = customer_id

            if time.monotonic() - checkpoint_stop_start >= 3.0:
                print(f"[DECISION] Target: Customer {detected_customer.upper()}")

                target_x, target_y = DELIVERY_LOCATIONS[detected_customer]
                print(f"[NAV] Routing via (2240, 3460) → ({target_x}, {target_y})")

                current_x, current_y, _ = robot.get_pose()
                planner        = make_planner()
                remaining_path = densify_polyline(
                    [(current_x, current_y), (2240.0, 3460.0), (target_x, target_y)],
                    spacing=20.0,
                )

                checkpoint_done = True
                state = "DELIVERY_MOVING"

        # ====================================================================
        # DELIVERY_MOVING  — scanning station → delivery drop-off
        # ====================================================================
        elif state == "DELIVERY_MOVING":
            show_moving_leds(robot)

            if len(remaining_path) == 0:
                print("[WARN] DELIVERY_MOVING: empty path — forcing FINAL_MOVING")
                robot.stop()
                current_x, current_y, _ = robot.get_pose()
                planner        = make_planner()
                remaining_path = densify_polyline(
                    [(current_x, current_y), (2240.0, 0.0)], spacing=20.0
                )
                stop_sign_detected = False
                state = "FINAL_MOVING"
            else:
                remaining_path, goal_reached = drive_step(robot, planner, remaining_path)

                if goal_reached:
                    print("[NAV] Delivery drop-off complete. Routing home (2240, 0).")
                    robot.stop()

                    current_x, current_y, _ = robot.get_pose()
                    planner        = make_planner()
                    remaining_path = densify_polyline(
                        [(current_x, current_y), (2240.0, 0.0)], spacing=20.0
                    )

                    stop_sign_detected = False
                    state = "FINAL_MOVING"

        # ====================================================================
        # FINAL_MOVING  — drop-off → home (2240, 0), with stop-sign check
        # ====================================================================
        elif state == "FINAL_MOVING":

            if not stop_sign_completed:

                if not stop_sign_detected:
                    show_moving_leds(robot)

                    if detect_stop_sign(robot):
                        print("[VISION] Stop sign — pausing 3s.")
                        robot.stop()
                        stop_sign_detected   = True
                        stop_sign_start_time = time.monotonic()
                    else:
                        remaining_path, goal_reached = drive_step(robot, planner, remaining_path)
                        if goal_reached:
                            print("[NAV] Reached home without stop sign.")
                            state = "FINISHED"

                else:
                    robot.stop()
                    show_idle_leds(robot)

                    if time.monotonic() - stop_sign_start_time >= 3.0:
                        print("[VISION] 3s pause done — resuming.")
                        stop_sign_completed = True
                        stop_sign_detected  = False

            else:
                show_moving_leds(robot)
                remaining_path, goal_reached = drive_step(robot, planner, remaining_path)
                if goal_reached:
                    print("[NAV] Arrived home (2240, 0). Task complete!")
                    robot.stop()
                    state = "FINISHED"

        # ====================================================================
        # FINISHED
        # ====================================================================
        elif state == "FINISHED":
            robot.stop()
            show_idle_leds(robot)

        # ── FSM tick rate ───────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


if __name__ == "__main__":
    pass