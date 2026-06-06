# # # ""
# # # from __future__ import annotations
# # # import time
# # # import math
# # # import numpy as np

# # # from robot.robot import FirmwareState, Robot, Unit
# # # from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
# # # from robot.path_planner import PurePursuitPlanner

# # # # ---------------------------------------------------------------------------
# # # # Robot build configuration
# # # # ---------------------------------------------------------------------------

# # # TAG_ID = 11
# # # POSITION_UNIT = Unit.MM
# # # WHEEL_DIAMETER = 74.0
# # # WHEEL_BASE = 333.0
# # # INITIAL_THETA_DEG = 90.0

# # # LEFT_WHEEL_MOTOR = Motor.DC_M1
# # # LEFT_WHEEL_DIR_INVERTED = False
# # # RIGHT_WHEEL_MOTOR = Motor.DC_M2
# # # RIGHT_WHEEL_DIR_INVERTED = True

# # # # ---------------------------------------------------------------------------
# # # # Pure Pursuit parameters
# # # # ---------------------------------------------------------------------------

# # # LOOKAHEAD_DIST = 120.0       # mm
# # # MAX_LINEAR_VEL = 120.0       # mm/s
# # # MAX_ANGULAR_VEL = 2.0        # rad/s
# # # GOAL_TOLERANCE = 20.0        # mm
# # # PRODUCTION_THRESHOLD = 0.25  # Vision matching threshold

# # # # ---------------------------------------------------------------------------
# # # # Field Coordinates
# # # # ---------------------------------------------------------------------------

# # # CHECKPOINT_X = 2440.0
# # # CHECKPOINT_Y = 3660.0
# # # CHECKPOINT_RADIUS = 60.0     # mm tolerance to trigger the arrival pause

# # # DELIVERY_LOCATIONS = {
# # #     "A": (2440.0, 1525.0),    # Target coordinates for Customer A
# # #     "B": (2440.0, 1225.0),    # Target coordinates for Customer B
# # #     "unknown": (2440.0, 1325.0)  # Fallback coordinates
# # # }

# # # def configure_robot(robot: Robot) -> None:
# # #     robot.set_unit(POSITION_UNIT)
# # #     robot.set_odometry_parameters(
# # #         wheel_diameter=WHEEL_DIAMETER,
# # #         wheel_base=WHEEL_BASE,
# # #         initial_theta_deg=INITIAL_THETA_DEG,
# # #         left_motor_id=LEFT_WHEEL_MOTOR,
# # #         left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
# # #         right_motor_id=RIGHT_WHEEL_MOTOR,
# # #         right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
# # #     )
# # #     robot.set_tracked_tag_id(TAG_ID)
# # #     robot.enable_vision()

# # # def start_robot(robot: Robot) -> None:
# # #     robot.set_state(FirmwareState.RUNNING)
# # #     robot.reset_odometry()
# # #     robot.wait_for_pose_update(timeout=0.2)

# # # def get_traffic_light(robot: Robot):
# # #     """Return 'green', 'red', or None based on highest-confidence detection."""
# # #     for detection in robot.get_detections("traffic light"):
# # #         if float(detection.get("confidence", 0.0)) >= 0.50:
# # #             color = detection.get("attributes", {}).get("color", {}).get("value")
# # #             if color in ("red", "green"):
# # #                 return color
# # #     return None

# # # def run(robot: Robot) -> None:
# # #     configure_robot(robot)
# # #     start_robot(robot)

# # #     # State machine starts by driving straight to the customer setup area
# # #     state = "DRIVING_TO_CHECKPOINT"
# # #     print("[SYSTEM] Initializing. Driving straight to Customer Checkpoint...")

# # #     # Establish initial path directly to the checkpoint coordinates
# # #     remaining_path = [(0.0, 0.0), (CHECKPOINT_X, CHECKPOINT_Y)]
# # #     planner = PurePursuitPlanner(
# # #         lookahead_dist=LOOKAHEAD_DIST,
# # #         max_angular=MAX_ANGULAR_VEL,
# # #         goal_tolerance=GOAL_TOLERANCE,
# # #     )

# # #     checkpoint_stop_start = 0.0
# # #     detected_customer = "unknown"
# # #     highest_match_score = -1.0

# # #     period = 1.0 / float(DEFAULT_FSM_HZ)
# # #     next_tick = time.monotonic()

# # #     while True:
# # #         if robot.get_button(Button.BTN_2):
# # #             print("BTN_2 pressed — emergency stop.")
# # #             robot.stop()
# # #             robot.shutdown()
# # #             break

# # #         # ==================================================================
# # #         # STATE 1: DRIVE STRAIGHT TO THE CHECKPOINT
# # #         # ==================================================================
# # #         if state == "DRIVING_TO_CHECKPOINT":
# # #             robot.set_led(LED.GREEN, 255)
# # #             robot.set_led(LED.ORANGE, 0)

# # #             current_x, current_y, current_theta_deg = robot.get_pose()
# # #             current_theta_rad = math.radians(current_theta_deg)

# # #             # Check if we are physically close enough to the checkpoint to stop
# # #             dist_to_checkpoint = math.sqrt((current_x - CHECKPOINT_X)**2 + (current_y - CHECKPOINT_Y)**2)
# # #             if dist_to_checkpoint <= CHECKPOINT_RADIUS:
# # #                 print(f"[NAV] Arrived at customer location ({current_x:.1f}, {current_y:.1f}). Starting 5s scan...")
# # #                 robot.stop()
# # #                 checkpoint_stop_start = time.monotonic()
# # #                 state = "CUSTOMER_CHECKPOINT"
# # #                 continue

# # #             # Standard Pure Pursuit tracking mechanics
# # #             remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, LOOKAHEAD_DIST)
# # #             cp_x, cp_y = planner._lookahead_point(current_x, current_y, remaining_path)
# # #             v, omega = planner.compute_velocity((current_x, current_y, current_theta_rad), remaining_path, MAX_LINEAR_VEL)
# # #             robot.set_velocity(v, math.degrees(omega))

# # #         # ==================================================================
# # #         # STATE 2: SCAN FOR CUSTOMER (At the actual checkpoint)
# # #         # ==================================================================
# # #         elif state == "CUSTOMER_CHECKPOINT":
# # #             robot.stop() 
# # #             robot.set_led(LED.ORANGE, 255)
# # #             robot.set_led(LED.GREEN, 0)

# # #             for person in robot.get_detections("person"):
# # #                 attributes = person.get("attributes", {})
# # #                 if "customer_id" in attributes:
# # #                     customer_attr = attributes.get("customer_id", {})
# # #                     customer_id = customer_attr.get("value")
# # #                     confidence = float(customer_attr.get("score", 0.0))

# # #                     if customer_id in ("A", "B") and confidence >= PRODUCTION_THRESHOLD:
# # #                         if confidence > highest_match_score:
# # #                             highest_match_score = confidence
# # #                             detected_customer = customer_id

# # #             if time.monotonic() - checkpoint_stop_start >= 5.0:
# # #                 print(f"[DECISION] Scan complete. Target: Customer {detected_customer.upper()} "
# # #                       f"(Confidence: {max(0.0, highest_match_score):.2f})")
# # #                 print("[FSM] Standby for GREEN light to resume delivery run...")
# # #                 state = "WAIT_FOR_LAUNCH_LIGHT"

# # #         # ==================================================================
# # #         # STATE 3: STANDBY FOR GREEN LIGHT
# # #         # ==================================================================
# # #         elif state == "WAIT_FOR_LAUNCH_LIGHT":
# # #             robot.stop()
# # #             robot.set_led(LED.ORANGE, 255)
# # #             robot.set_led(LED.GREEN, 0)

# # #             if get_traffic_light(robot) == "green":
# # #                 print("[VISION] Green light detected! Re-routing straight to drop-off.")
                
# # #                 target_x, target_y = DELIVERY_LOCATIONS[detected_customer]
# # #                 # Path maps a straight trajectory starting from the current checkpoint out to delivery coordinates
# # #                 remaining_path = [(CHECKPOINT_X, CHECKPOINT_Y), (target_x, target_y)]
                
# # #                 planner = PurePursuitPlanner(
# # #                     lookahead_dist=LOOKAHEAD_DIST,
# # #                     max_angular=MAX_ANGULAR_VEL,
# # #                     goal_tolerance=GOAL_TOLERANCE,
# # #                 )
# # #                 state = "MOVING_TO_DELIVERY"

# # #         # ==================================================================
# # #         # STATE 4: DRIVE DIRECTLY TO CHOSEN CUSTOMER DROP-OFF
# # #         # ==================================================================
# # #         elif state == "MOVING_TO_DELIVERY":
# # #             robot.set_led(LED.ORANGE, 0)
# # #             robot.set_led(LED.GREEN, 255)

# # #             current_x, current_y, current_theta_deg = robot.get_pose()
# # #             current_theta_rad = math.radians(current_theta_deg)

# # #             remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, LOOKAHEAD_DIST)
# # #             cp_x, cp_y = planner._lookahead_point(current_x, current_y, remaining_path)
# # #             v, omega = planner.compute_velocity((current_x, current_y, current_theta_rad), remaining_path, MAX_LINEAR_VEL)
# # #             robot.set_velocity(v, math.degrees(omega))

# # #             if planner.CurrentTargetReached(cp_x, cp_y, current_x, current_y):
# # #                 print("[ARRIVED] Delivery complete! Stopping robot.")
# # #                 robot.stop()
# # #                 robot.set_led(LED.GREEN, 0)
# # #                 break

# # #         # FSM tick rate control
# # #         next_tick += period
# # #         sleep_s = next_tick - time.monotonic()
# # #         if sleep_s > 0:
# # #             time.sleep(sleep_s)
# # #         else:
# # #             next_tick = time.monotonic()

# # # if __name__ == "__main__":
# # #     main()

# from __future__ import annotations
# import time
# import math
# import numpy as np


# from robot.robot import FirmwareState, Robot, Unit
# from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
# from robot.util import densify_polyline


# # ---------------------------------------------------------------------------
# # Robot build configuration
# # ---------------------------------------------------------------------------


# TAG_ID = 11
# POSITION_UNIT = Unit.MM
# WHEEL_DIAMETER = 76.2
# WHEEL_BASE = 241.3
# INITIAL_THETA_DEG = 90.0


# LEFT_WHEEL_MOTOR = Motor.DC_M1
# LEFT_WHEEL_DIR_INVERTED = False
# RIGHT_WHEEL_MOTOR = Motor.DC_M2
# RIGHT_WHEEL_DIR_INVERTED = True


# # ---------------------------------------------------------------------------
# # Wheel trim — compensates for right wheel being slower than left
# # Increase RIGHT_WHEEL_TRIM above 1.0 to speed up the right wheel
# # Start at 1.05 and tune in 0.01 increments until robot drives straight
# # ---------------------------------------------------------------------------

# RIGHT_WHEEL_TRIM = 1.05


# # ---------------------------------------------------------------------------
# # Pure Pursuit parameters
# # ---------------------------------------------------------------------------


# LOOKAHEAD_DIST = 50.0
# MAX_LINEAR_VEL = 100
# MAX_ANGULAR_VEL = 0.5
# GOAL_TOLERANCE = 20.0


# # ---------------------------------------------------------------------------
# # Waypoints
# # ---------------------------------------------------------------------------


# RAW_WAYPOINTS = [
#     (0.0,    0.0),
#     (0.0,    3560.0),
#     (410,  3560.0),
#     (410.0,  810.0),
#     (1325.0, 810.0),
#     (1325.0, 3460.0),
#     (2240.0, 3460.0),
#     (2240.0, 0.0),
# ]


# # ---------------------------------------------------------------------------
# # Helpers
# # ---------------------------------------------------------------------------

# def configure_robot(robot: Robot) -> None:
#     robot.set_unit(POSITION_UNIT)
#     robot.set_odometry_parameters(
#         wheel_diameter=WHEEL_DIAMETER,
#         wheel_base=WHEEL_BASE,
#         initial_theta_deg=INITIAL_THETA_DEG,
#         left_motor_id=LEFT_WHEEL_MOTOR,
#         left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
#         right_motor_id=RIGHT_WHEEL_MOTOR,
#         right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
#     )
#     robot.set_tracked_tag_id(TAG_ID)
#     robot.enable_vision()


# def start_robot(robot: Robot) -> None:
#     robot.set_state(FirmwareState.RUNNING)
#     robot.reset_odometry()
#     robot.wait_for_pose_update(timeout=0.2)


# def get_traffic_light(robot: Robot):
#     """Return 'green', 'red', or None based on highest-confidence detection."""
#     for detection in robot.get_detections("traffic light"):
#         if float(detection.get("confidence", 0.0)) >= 0.50:
#             color = detection.get("attributes", {}).get("color", {}).get("value")
#             if color in ("red", "green"):
#                 return color
#     return None


# def set_velocity_trimmed(robot: Robot, linear: float, angular_rad: float) -> None:
#     """
#     Send velocity command with right wheel trim applied to correct
#     for mechanical speed mismatch between left and right wheels.
#     """
#     v_left          = linear - (angular_rad * WHEEL_BASE / 2.0)
#     v_right         = linear + (angular_rad * WHEEL_BASE / 2.0)
#     v_right_trimmed = v_right * RIGHT_WHEEL_TRIM
#     linear_corrected  = (v_left + v_right_trimmed) / 2.0
#     angular_corrected = (v_right_trimmed - v_left) / WHEEL_BASE
#     robot.set_velocity(linear_corrected, math.degrees(angular_corrected))


# def run(robot: Robot) -> None:
#     configure_robot(robot)
#     start_robot(robot)

#     path_control_points = list(RAW_WAYPOINTS)
#     path1 = densify_polyline(path_control_points, spacing=20.0)

#     planner1 = None
#     remaining_path = []

#     state = "WAIT_FOR_GREEN"
#     print("[FSM] Waiting for GREEN traffic light before starting path...")

#     period = 1.0 / float(DEFAULT_FSM_HZ)
#     next_tick = time.monotonic()

#     while True:
#         if robot.get_button(Button.BTN_2):
#             print("BTN_2 pressed — emergency stop.")
#             robot.stop()
#             robot.shutdown()
#             break

#         traffic_light_color = get_traffic_light(robot)

#         # ==================================================================
#         # STATE: WAIT_FOR_GREEN
#         # ==================================================================
#         if state == "WAIT_FOR_GREEN":
#             robot.stop()
#             robot.set_led(LED.GREEN, 0)
#             robot.set_led(LED.ORANGE, 0)

#             if traffic_light_color == "green":
#                 print("[VISION] Green detected — initialising Pure Pursuit planner.")
#                 planner1 = PurePursuitPlanner(
#                     lookahead_dist=LOOKAHEAD_DIST,
#                     max_angular=MAX_ANGULAR_VEL,
#                     goal_tolerance=GOAL_TOLERANCE,
#                 )
#                 remaining_path = path1.copy()
#                 print("[FSM] → MOVING")
#                 state = "MOVING"

#         # ==================================================================
#         # STATE: MOVING
#         # ==================================================================
#         elif state == "MOVING":
#             if traffic_light_color == "red":
#                 print("[VISION] Red light — pausing.")
#                 robot.stop()
#                 robot.set_led(LED.GREEN, 0)
#                 robot.set_led(LED.ORANGE, 255)
#                 state = "PAUSED"

#             else:
#                 robot.set_led(LED.GREEN, 255)
#                 robot.set_led(LED.ORANGE, 0)

#                 current_x, current_y, current_theta_deg = robot.get_pose()
#                 current_theta_rad = math.radians(current_theta_deg)

#                 remaining_path = robot._advance_remaining_path(
#                     remaining_path,
#                     current_x,
#                     current_y,
#                     advance_radius_mm=LOOKAHEAD_DIST,
#                 )

#                 current_pursuit_x, current_pursuit_y = planner1._lookahead_point(
#                     current_x,
#                     current_y,
#                     waypoints=remaining_path,
#                 )

#                 linear_velocity_cmd, angular_velocity_cmd_rad_s = planner1.compute_velocity(
#                     pose=(current_x, current_y, current_theta_rad),
#                     waypoints=remaining_path,
#                     max_linear=MAX_LINEAR_VEL,
#                 )

#                 # Trimmed velocity replaces robot.set_velocity directly
#                 set_velocity_trimmed(
#                     robot,
#                     linear_velocity_cmd,
#                     angular_velocity_cmd_rad_s,
#                 )

#                 if planner1.CurrentTargetReached(
#                     current_pursuit_x, current_pursuit_y,
#                     current_x, current_y,
#                 ):
#                     print("[MOVING] Final waypoint reached — stopping.")
#                     robot.stop()
#                     robot.set_led(LED.GREEN, 0)
#                     robot.set_led(LED.ORANGE, 0)
#                     print("[FSM] → WAIT_FOR_GREEN")
#                     state = "WAIT_FOR_GREEN"

#         # ==================================================================
#         # STATE: PAUSED
#         # ==================================================================
#         elif state == "PAUSED":
#             robot.stop()
#             robot.set_led(LED.ORANGE, 255)

#             if traffic_light_color == "green":
#                 print("[VISION] Green again — resuming path.")
#                 robot.set_led(LED.ORANGE, 0)
#                 state = "MOVING"

#         # ------------------------------------------------------------------
#         # FSM tick rate
#         # ------------------------------------------------------------------
#         next_tick += period
#         sleep_s = next_tick - time.monotonic()
#         if sleep_s > 0:
#             time.sleep(sleep_s)
#         else:
#             next_tick = time.monotonic()

from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.robot_impl.burger_assembly import BurgerAssemblyMixin
from robot.path_planner import PurePursuitPlanner
from robot.robot_impl.burger_assembly import (
    ELEVATOR_SAFE_STEPS,
    ELEVATOR_HOME,
    SWING_PICK_STEPS,
    SWING_HOME,
    PIECE_SPACING_MM,
)



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
# Pure Pursuit + Obstacle Avoidance parameters
# ---------------------------------------------------------------------------

LOOKAHEAD_DIST  = 200.0          # increased — robot looks further ahead, smoother curves
MAX_LINEAR_VEL  = 50.0          # keep the same
MAX_ANGULAR_VEL = 0.8            # reduced — limits how sharply it can turn
GOAL_TOLERANCE  = 20.0

OBS_RANGE_MM    = 400.0          # increased — detects obstacles earlier giving more time to react
VIEW_ANGLE_RAD  = math.radians(60.0)  # narrowed — only looks forward, ignores side clutter
SAFE_DIST_MM    = 300.0          # increased — starts avoiding earlier = gentler curve
AVOIDANCE_DELAY = 100            # reduced — recovers back to original path faster
ALPHA_LD        = 0.85           # increased toward 1.0 — less lookahead reduction during avoidance
D_OFFSET_MM     = 250.0          # reduced — smaller lateral shift = gentler swerve
X_L_MM          = 205.0
LANE_WIDTH_MM   = 450.0


# ---------------------------------------------------------------------------
# Waypoints
# ---------------------------------------------------------------------------

RAW_WAYPOINTS = [
    (0.0,    0.0),
    (0.0,    3560.0),
    (410.0,  3560.0),
    (410.0,  810.0),
    (1325.0, 810.0),
    (1325.0, 3460.0),
    (2240.0, 3460.0),
    (2240.0, 0.0),
]

# ── Assembly trigger ──────────────────────────────────────────────────────────
# Robot transitions from MOVING → ASSEMBLY when it crosses this Y position
# on the first straight (heading north along x=0).
# Set to ~500 mm — just past where the table sits beside the path.
ASSEMBLY_TRIGGER_Y_MM   = 500.0
ASSEMBLY_TRIGGER_X_MAX  = 50.0   # only trigger while still on the x=0 leg
                                  # (prevents re-triggering on later laps)
 
 
# ── Helpers ───────────────────────────────────────────────────────────────────
 
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
 
 
def get_traffic_light(robot: Robot):
    for detection in robot.get_detections("traffic light"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            color = detection.get("attributes", {}).get("color", {}).get("value")
            if color in ("red", "green"):
                return color
    return None
 
 
# ── Main FSM ──────────────────────────────────────────────────────────────────
 
def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
 
    path1 = densify_polyline(list(RAW_WAYPOINTS), spacing=20.0)
 
    planner1       = None
    remaining_path = []
    assembly_done  = False   # only assemble once per run
 
    state     = "WAIT_FOR_GREEN"
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
 
    print("[FSM] Waiting for GREEN traffic light...")
 
    while True:
 
        # ── Emergency stop ────────────────────────────────────────────────────
        if robot.get_button(Button.BTN_2):
            print("BTN_2 — emergency stop.")
            robot.stop()
            robot.shutdown()
            break
 
        traffic_light_color = get_traffic_light(robot)
 
        # ══════════════════════════════════════════════════════════════════════
        # WAIT_FOR_GREEN
        # ══════════════════════════════════════════════════════════════════════
        if state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)
 
            if traffic_light_color == "green":
                print("[VISION] Green — starting path.")
                planner1 = PurePursuitPlanner(
                    lookahead_dist=LOOKAHEAD_DIST,
                    max_angular=MAX_ANGULAR_VEL,
                    goal_tolerance=GOAL_TOLERANCE,
                )
                remaining_path = path1.copy()
                state = "MOVING"
                print("[FSM] → MOVING")
 
        # ══════════════════════════════════════════════════════════════════════
        # MOVING
        # ══════════════════════════════════════════════════════════════════════
        elif state == "MOVING":
 
            # ── Red light pause ───────────────────────────────────────────────
            if traffic_light_color == "red":
                print("[VISION] Red — pausing.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                state = "PAUSED"
 
            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)
 
                current_x, current_y, current_theta_deg = robot.get_pose()
                current_theta_rad = math.radians(current_theta_deg)
 
                # ── Assembly trigger ──────────────────────────────────────────
                # Fire once, on the first northbound leg (x ≈ 0), when y
                # crosses ASSEMBLY_TRIGGER_Y_MM.
                if (
                    not assembly_done
                    and current_y >= ASSEMBLY_TRIGGER_Y_MM
                    and abs(current_x) <= ASSEMBLY_TRIGGER_X_MAX
                ):
                    print(
                        f"[FSM] Assembly trigger at "
                        f"({current_x:.0f}, {current_y:.0f}) mm → ASSEMBLY"
                    )
                    robot.stop()
                    state = "ASSEMBLY"
 
                else:
                    # ── Pure Pursuit step ─────────────────────────────────────
                    remaining_path = robot._advance_remaining_path(
                        remaining_path, current_x, current_y,
                        advance_radius_mm=LOOKAHEAD_DIST,
                    )
                    current_pursuit_x, current_pursuit_y = (
                        planner1._lookahead_point(
                            current_x, current_y, waypoints=remaining_path
                        )
                    )
                    linear_vel, angular_vel = planner1.compute_velocity(
                        pose=(current_x, current_y, current_theta_rad),
                        waypoints=remaining_path,
                        max_linear=MAX_LINEAR_VEL,
                    )
                    robot.set_velocity(linear_vel, math.degrees(angular_vel))
 
                    if planner1.CurrentTargetReached(
                        current_pursuit_x, current_pursuit_y,
                        current_x, current_y,
                    ):
                        print("[MOVING] Final waypoint reached.")
                        robot.stop()
                        robot.set_led(LED.GREEN, 0)
                        robot.set_led(LED.ORANGE, 0)
                        state = "DONE"
                        print("[FSM] → DONE")
 
        # ══════════════════════════════════════════════════════════════════════
        # ASSEMBLY  — burger pick and stack sequence
        # ══════════════════════════════════════════════════════════════════════
                
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
               print(f"[FSM] Assembly error: {e} — resuming path anyway.")
               assembly_done = True
               try:
                  robot.arm_safe_height()
                  robot.arm_open_gripper()
               except Exception:
                    pass
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            state = "MOVING"
            print("[FSM] → MOVING (resumed)")
              
 
        # ══════════════════════════════════════════════════════════════════════
        # PAUSED  — red light mid-route
        # ══════════════════════════════════════════════════════════════════════
        elif state == "PAUSED":
            robot.stop()
            robot.set_led(LED.ORANGE, 255)
 
            if traffic_light_color == "green":
                print("[VISION] Green again — resuming.")
                robot.set_led(LED.ORANGE, 0)
                state = "MOVING"
                print("[FSM] → MOVING")
 
        # ══════════════════════════════════════════════════════════════════════
        # DONE
        # ══════════════════════════════════════════════════════════════════════
        elif state == "DONE":
            robot.stop()
            break
 
        # ── Tick rate control ─────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()