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
WHEEL_BASE        = 221.3
INITIAL_THETA_DEG = 90.0


LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


# ---------------------------------------------------------------------------
# Pure Pursuit parameters
# ---------------------------------------------------------------------------


LOOKAHEAD_DIST  = 100.0   # mm
MAX_LINEAR_VEL  = 140.0   # mm/s
MAX_ANGULAR_VEL = 1.5     # rad/s
GOAL_TOLERANCE  = 20.0    # mm


# ---------------------------------------------------------------------------
# Arm / assembly constants
# ---------------------------------------------------------------------------


TRAFFIC_LIGHT_SWING_STEPS = -100
ELEVATOR_HOME = 0
SWING_HOME    = 0


# ---------------------------------------------------------------------------
# Assembly spatial trigger
# ---------------------------------------------------------------------------


ASSEMBLY_TRIGGER_Y_MM  = 970.9
ASSEMBLY_TRIGGER_X_MAX = 50.0


# ---------------------------------------------------------------------------
# Customer checkpoint trigger
# ---------------------------------------------------------------------------


CHECKPOINT_TRIGGER_X_MIN = 1275.0
CHECKPOINT_TRIGGER_X_MAX = 1375.0
CHECKPOINT_TRIGGER_Y_MIN = 3410.0
CHECKPOINT_TRIGGER_Y_MAX = 3510.0


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
# Course waypoints
# ---------------------------------------------------------------------------


ALL_WAYPOINTS = [
    (0.0,    0.0),
    (0,    3590.0),
    (400.0,  3590.0),
    (400.0,  780.0),
    (1315.0, 780.0),
    (1320.0, 3590.0),
    (2260.0, 3590.0),
    (2270.0, 0.0),
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
    # --- Lidar DISABLED ---
    # robot.enable_lidar()
    # robot.set_lidar_mount(...)
    # robot.set_lidar_filter(...)
    # robot.start_lidar_world_publisher()
    print("[sensor] lidar DISABLED — pure pursuit only")




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
    for detection in robot.get_detections("traffic light"):
        if float(detection.get("confidence", 0.0)) >= 0.30:
            color = detection.get("attributes", {}).get("color", {}).get("value")
            if color in ("red", "green"):
                return color
    return None




def detect_stop_sign(robot: Robot) -> bool:
    for detection in robot.get_detections("stop sign"):
        if float(detection.get("confidence", 0.0)) >= 0.30:
            return True
    return False




def make_planner() -> PurePursuitPlanner:
    return PurePursuitPlanner(
        lookahead_dist=LOOKAHEAD_DIST,
        max_angular=MAX_ANGULAR_VEL,
        goal_tolerance=GOAL_TOLERANCE,
    )




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


    planner:        PurePursuitPlanner = None
    remaining_path: list               = []


    assembly_done       = False
    checkpoint_done     = False
    delivery_done       = False
    idle_arm_positioned = False


    checkpoint_stop_start = 0.0
    stop_sign_start_time  = 0.0
    stop_sign_detected    = False
    stop_sign_completed   = False
    detected_customer     = "unknown"
    highest_match_score   = -1.0


    while True:


        # ── Global red-light pause ──────────────────────────────────────────
        if state in ("MOVING", "DELIVERY_MOVING", "FINAL_MOVING"):
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
        # IDLE
        # ====================================================================
        if state == "IDLE":
            robot.stop()
            show_idle_leds(robot)


            if not idle_arm_positioned:
                try:
                    robot.arm_elevator_to(ELEVATOR_HOME)
                    idle_arm_positioned = True
                except Exception as e:
                    print(f"[WARN] Arm swing failed: {e}")


            if get_traffic_light(robot) == "green":
                idle_arm_positioned = False
                print("[VISION] Green detected — arming planner.")


                planner        = make_planner()
                remaining_path = densify_polyline(ALL_WAYPOINTS, spacing=20.0)
                print("[FSM] IDLE → MOVING")
                state = "MOVING"


        # ====================================================================
        # MOVING — pure pursuit through all waypoints
        # ====================================================================
        elif state == "MOVING":
            show_moving_leds(robot)


            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)


            # Assembly trigger
            if (
                not assembly_done
                and current_y >= ASSEMBLY_TRIGGER_Y_MM
                and abs(current_x) <= ASSEMBLY_TRIGGER_X_MAX
            ):
                print(f"[FSM] Assembly trigger at ({current_x:.0f}, {current_y:.0f}) → ASSEMBLY")
                robot.stop()
                state = "ASSEMBLY"


            # Customer checkpoint trigger — robot is at (1325, 3460) facing north
            elif (
                not checkpoint_done
                and CHECKPOINT_TRIGGER_X_MIN <= current_x <= CHECKPOINT_TRIGGER_X_MAX
                and CHECKPOINT_TRIGGER_Y_MIN <= current_y <= CHECKPOINT_TRIGGER_Y_MAX
            ):
                print(f"[FSM] Checkpoint trigger at ({current_x:.0f}, {current_y:.0f}) → CUSTOMER_CHECKPOINT")
                robot.stop()
                checkpoint_stop_start = time.monotonic()
                state = "CUSTOMER_CHECKPOINT"


            else:
                remaining_path = robot._advance_remaining_path(
                    remaining_path, current_x, current_y,
                    advance_radius_mm=LOOKAHEAD_DIST,
                )


                if len(remaining_path) <= 1:
                    print("[NAV] Path complete.")
                    robot.stop()
                    state = "FINISHED"
                else:
                    linear_vel, angular_vel_rad = planner.compute_velocity(
                        pose=(current_x, current_y, current_theta_rad),
                        waypoints=remaining_path,
                        max_linear=MAX_LINEAR_VEL,
                    )
                    robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))


        # ====================================================================
        # ASSEMBLY
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


            print(f"[DEBUG] remaining_path length before trim: {len(remaining_path)}")
            print(f"[DEBUG] first waypoint: {remaining_path[0] if remaining_path else 'EMPTY'}")


            current_x, current_y, _ = robot.get_pose()
            remaining_path = [(wx, wy) for (wx, wy) in remaining_path if wy >= current_y - 50.0]
            print(f"[DEBUG] remaining_path after Y-filter: {len(remaining_path)} points, "
                  f"first={remaining_path[0] if remaining_path else 'EMPTY'}")


            show_moving_leds(robot)
            state = "MOVING"
            print("[FSM] ASSEMBLY → MOVING")


        # ====================================================================
        # CUSTOMER_CHECKPOINT — 3-second face scan at (1325, 3460)
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
                print(f"[NAV] Routing to ({target_x}, {target_y})")


                current_x, current_y, _ = robot.get_pose()
                planner        = make_planner()
                remaining_path = densify_polyline(
                    [(current_x, current_y), (target_x, target_y)],
                    spacing=20.0,
                )
                checkpoint_done = True
                state = "DELIVERY_MOVING"


        # ====================================================================
        # DELIVERY_MOVING — checkpoint → delivery drop-off
        # ====================================================================
        elif state == "DELIVERY_MOVING":
            show_moving_leds(robot)


            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)


            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y,
                advance_radius_mm=LOOKAHEAD_DIST,
            )


            if len(remaining_path) <= 1:
                print("[NAV] Reached delivery drop-off — delivering burger.")
                robot.stop()
                state = "DELIVERY"
            else:
                linear_vel, angular_vel_rad = planner.compute_velocity(
                    pose=(current_x, current_y, current_theta_rad),
                    waypoints=remaining_path,
                    max_linear=MAX_LINEAR_VEL,
                )
                robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))


        # ====================================================================
        # DELIVERY
        # ====================================================================
        elif state == "DELIVERY":
            print(f"[FSM] Delivering to Customer {detected_customer}...")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)


            try:
                robot.deliver_burger(customer=detected_customer)
                delivery_done = True
                print("[FSM] Delivery complete.")
            except Exception as e:
                print(f"[FSM] Delivery error: {e} — resuming anyway.")
                delivery_done = True
                try:
                    robot.arm_safe_height()
                    robot.arm_open_gripper()
                    robot.arm_swing_to(0)
                except Exception:
                    pass


            current_x, current_y, _ = robot.get_pose()
            planner        = make_planner()
            remaining_path = densify_polyline(
                [(current_x, current_y), (2240.0, 0.0)], spacing=20.0
            )
            stop_sign_detected = False
            show_moving_leds(robot)
            state = "FINAL_MOVING"
            print("[FSM] DELIVERY → FINAL_MOVING")


        # ====================================================================
        # FINAL_MOVING — drop-off → home (2240, 0), with stop-sign check
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
                        current_x, current_y, current_theta_deg = robot.get_pose()
                        current_theta_rad = math.radians(current_theta_deg)


                        remaining_path = robot._advance_remaining_path(
                            remaining_path, current_x, current_y,
                            advance_radius_mm=LOOKAHEAD_DIST,
                        )


                        if len(remaining_path) <= 1:
                            print("[NAV] Reached home without stop sign.")
                            robot.stop()
                            state = "FINISHED"
                        else:
                            linear_vel, angular_vel_rad = planner.compute_velocity(
                                pose=(current_x, current_y, current_theta_rad),
                                waypoints=remaining_path,
                                max_linear=MAX_LINEAR_VEL,
                            )
                            robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))


                else:
                    robot.stop()
                    show_idle_leds(robot)
                    if time.monotonic() - stop_sign_start_time >= 3.0:
                        print("[VISION] 3s pause done — resuming.")
                        stop_sign_completed = True
                        stop_sign_detected  = False


            else:
                show_moving_leds(robot)


                current_x, current_y, current_theta_deg = robot.get_pose()
                current_theta_rad = math.radians(current_theta_deg)


                remaining_path = robot._advance_remaining_path(
                    remaining_path, current_x, current_y,
                    advance_radius_mm=LOOKAHEAD_DIST,
                )


                if len(remaining_path) <= 1:
                    print("[NAV] Arrived home (2240, 0). Task complete!")
                    robot.stop()
                    state = "FINISHED"
                else:
                    linear_vel, angular_vel_rad = planner.compute_velocity(
                        pose=(current_x, current_y, current_theta_rad),
                        waypoints=remaining_path,
                        max_linear=MAX_LINEAR_VEL,
                    )
                    robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))


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



