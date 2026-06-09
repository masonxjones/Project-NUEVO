from __future__ import annotations

import math
import time

import numpy as np

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.path_planner import PurePursuitPlanner
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID            = 26
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
# LAPF corridor parameters  (replaces PurePursuitPlannerWithAvoidance)
# ---------------------------------------------------------------------------
# Corridor leg end-point — the goal LAPF drives toward while avoiding cones.
LAPF_GOAL_MM          = (1800.0, 3460.0)

LAPF_VELOCITY_MM_S    = 140.0   # overall drive speed
LAPF_TOLERANCE_MM     = 50.0    # goal-reached radius
LAPF_MAX_ANGULAR      = 1.5     # rad/s cap

# Leash: ~400 mm for front-wheel-drive, ~50 mm for rear-wheel-drive
LAPF_LEASH_MM         = 400.0
LAPF_REPULSION_MM     = 300.0   # range at which obstacles push
LAPF_TARGET_SPEED     = 200.0   # mm/s for the virtual target
LAPF_REPULSION_GAIN   = 550.0
LAPF_ATTRACTION_GAIN  = 1.0
LAPF_FORCE_EMA        = 0.35
LAPF_INFLATION_MM     = 130.0   # matched to working lapf_to_goal.py example
LAPF_HALF_ANGLE_DEG   = 25.0

# Disable avoidance once the rover has cleared the corridor
LAPF_DISABLE_PAST_X   = 1325.0
LAPF_DISABLE_PAST_Y   = 3460.0

# ---------------------------------------------------------------------------
# Arm / assembly constants
# ---------------------------------------------------------------------------

TRAFFIC_LIGHT_SWING_STEPS = -100
ELEVATOR_HOME = 0
SWING_HOME    = 0
ELEVATOR_LOW  = 4800

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
# Course waypoints
# ---------------------------------------------------------------------------

LEG1A_WAYPOINTS = [
    (0.0,    0.0),
    (0.0,    3560.0),
    (410.0,  3560.0),
    (410.0,  610.0),
    (1325.0, 590.0),
    (1325.0, 610.0),
]

# Entry waypoint handed off from MOVING → MOVING_AVOID.
# LAPF takes over from here; we just need to be close to the corridor mouth.
LEG1B_ENTRY = (1325.0, 810.0)


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
    robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
    robot.enable_vision()

    # --- Lidar setup (mirrors lapf_to_goal.py exactly) ---------------------
    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    robot.start_lidar_world_publisher()
    print("[sensor] lidar enabled — subscribing to /scan")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def get_traffic_light(robot: Robot):
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
    return PurePursuitPlanner(
        lookahead_dist=LOOKAHEAD_DIST,
        max_angular=MAX_ANGULAR_VEL,
        goal_tolerance=GOAL_TOLERANCE,
    )


def start_lapf_corridor(robot: Robot):
    """Launch the non-blocking LAPF goal for the cone corridor."""
    print(
        f"[LAPF] starting corridor goal={LAPF_GOAL_MM} "
        f"leash={LAPF_LEASH_MM:.0f} mm repulsion={LAPF_REPULSION_MM:.0f} mm"
    )
    return robot.lapf_to_goal(
        LAPF_GOAL_MM[0],
        LAPF_GOAL_MM[1],
        velocity=LAPF_VELOCITY_MM_S,
        tolerance=LAPF_TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_MM,
        repulsion_range_mm=LAPF_REPULSION_MM,
        target_speed_mm_s=LAPF_TARGET_SPEED,
        max_angular_rad_s=LAPF_MAX_ANGULAR,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_FORCE_EMA,
        inflation_margin_mm=LAPF_INFLATION_MM,
        leash_half_angle_deg=LAPF_HALF_ANGLE_DEG,
        blocking=False,
    )


def print_lapf_status(robot: Robot) -> None:
    x, y, theta = robot.get_pose()
    vt = robot.get_virtual_target()
    tracks = robot.get_obstacle_tracks()
    vt_str = f"vt=({vt[0]:.0f},{vt[1]:.0f})" if vt else "vt=(none)"
    if tracks:
        nearest = min(
            max(0.0, math.hypot(float(t["x"]) - x, float(t["y"]) - y) - float(t["radius"]))
            for t in tracks
        )
        track_str = f"tracks={len(tracks)} nearest={nearest:.0f} mm"
    else:
        track_str = "tracks=0"
    print(f"  [LAPF] pos=({x:.0f},{y:.0f}) θ={theta:.1f}°  {vt_str}  {track_str}")


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    robot.set_pos_fusion_alpha(0.10)

    state  = "IDLE"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"[FSM] period={period:.3f}s — waiting for GREEN light")

    next_tick = time.monotonic()

    planner:        PurePursuitPlanner = None
    remaining_path: list               = []

    # LAPF corridor handle & status-print throttle
    lapf_handle             = None
    last_lapf_status_at     = 0.0
    LAPF_STATUS_INTERVAL_S  = 0.5

    assembly_done  = True
    delivery_done  = False
    checkpoint_done = False

    checkpoint_stop_start = 0.0
    stop_sign_start_time  = 0.0
    stop_sign_detected    = False
    stop_sign_completed   = False
    detected_customer     = "unknown"
    highest_match_score   = -1.0

    while True:
        now = time.monotonic()

        # ── Global red-light pause ──────────────────────────────────────────
        if state in ("MOVING", "MOVING_AVOID", "DELIVERY_MOVING", "FINAL_MOVING"):
            if not stop_sign_detected:
                if get_traffic_light(robot) == "red":
                    print("[TRAFFIC LIGHT] Red — holding.")
                    if lapf_handle is not None:
                        lapf_handle.cancel()        # pause LAPF during red
                        lapf_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    time.sleep(period)
                    continue

        # ── Emergency stop ──────────────────────────────────────────────────
        if robot.get_button(Button.BTN_2):
            print("BTN_2 — emergency stop.")
            if lapf_handle is not None:
                lapf_handle.cancel()
            robot.stop()
            robot.shutdown()
            break

        # ====================================================================
        # IDLE
        # ====================================================================
        if state == "IDLE":
            robot.stop()
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()

            try:
                robot.arm_elevator_to(ELEVATOR_LOW)
                robot.arm_swing_to(TRAFFIC_LIGHT_SWING_STEPS)
            except Exception as e:
                print(f"[WARN] Arm swing failed: {e}")

            if get_traffic_light(robot) == "green":
                print("[VISION] Green detected — arming planner.")
                try:
                    robot.arm_swing_to(SWING_HOME)
                    robot.arm_elevator_to(ELEVATOR_HOME)
                except Exception as e:
                    print(f"[WARN] Arm return home failed: {e}")

                planner        = make_planner()
                remaining_path = densify_polyline(LEG1A_WAYPOINTS, spacing=20.0)
                time.sleep(2)
                print("[FSM] IDLE → MOVING")
                state = "MOVING"

        # ====================================================================
        # MOVING  — plain Pure Pursuit, no obstacles
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

            else:
                remaining_path = robot._advance_remaining_path(
                    remaining_path, current_x, current_y,
                    advance_radius_mm=LOOKAHEAD_DIST,
                )

                if len(remaining_path) <= 1:
                    # End of LEG1A — hand off to LAPF corridor
                    print("[NAV] Corridor entry reached — starting LAPF avoidance.")
                    robot.stop()
                    lapf_handle         = start_lapf_corridor(robot)
                    last_lapf_status_at = now
                    print("[FSM] MOVING → MOVING_AVOID")
                    state = "MOVING_AVOID"
                else:
                    linear_vel, angular_vel_rad = planner.compute_velocity(
                        pose=(current_x, current_y, current_theta_rad),
                        waypoints=remaining_path,
                        max_linear=MAX_LINEAR_VEL,
                    )
                    robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))

        # ====================================================================
        # MOVING_AVOID  — LAPF handles cone corridor
        # ====================================================================
        elif state == "MOVING_AVOID":
            show_moving_leds(robot)

            # Throttled status print
            if now - last_lapf_status_at >= LAPF_STATUS_INTERVAL_S:
                print_lapf_status(robot)
                last_lapf_status_at = now

            if lapf_handle is not None and lapf_handle.is_finished():
                print("[LAPF] Corridor goal complete.")
                lapf_handle = None
                robot.stop()
                checkpoint_stop_start = now
                print("[NAV] Arrived at scanning station — waiting for customer ID.")
                state = "CUSTOMER_CHECKPOINT"

            # Safety: if LAPF handle was lost (e.g. after red-light cancel),
            # restart it.
            elif lapf_handle is None:
                print("[LAPF] Handle missing — restarting corridor goal.")
                lapf_handle         = start_lapf_corridor(robot)
                last_lapf_status_at = now

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
        # DELIVERY
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

            current_x, current_y, _ = robot.get_pose()
            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y,
                advance_radius_mm=LOOKAHEAD_DIST,
            )

            show_moving_leds(robot)
            state = "MOVING"
            print("[FSM] DELIVERY → MOVING")

        # ====================================================================
        # CUSTOMER_CHECKPOINT  — 3-second face scan
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

            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            remaining_path = robot._advance_remaining_path(
                remaining_path, current_x, current_y,
                advance_radius_mm=LOOKAHEAD_DIST,
            )

            if len(remaining_path) <= 1:
                print("[NAV] Delivery drop-off complete. Routing home (2240, 0).")
                robot.stop()
                current_x, current_y, _ = robot.get_pose()
                planner        = make_planner()
                remaining_path = densify_polyline(
                    [(current_x, current_y), (2240.0, 0.0)], spacing=20.0
                )
                stop_sign_detected = False
                state = "FINAL_MOVING"
            else:
                linear_vel, angular_vel_rad = planner.compute_velocity(
                    pose=(current_x, current_y, current_theta_rad),
                    waypoints=remaining_path,
                    max_linear=MAX_LINEAR_VEL,
                )
                robot.set_velocity(linear_vel, math.degrees(angular_vel_rad))

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
