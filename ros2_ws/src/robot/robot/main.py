from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.robot_impl.burger_assembly import BurgerAssemblyMixin
from robot.path_planner import PurePursuitPlanner
from robot.examples.burger_delivery.py import BurgerDeliveryMixin


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

# ── Swing scan 
TRAFFIC_LIGHT_SWING_STEPS = 100  # ← TUNE: steps to swing arm toward traffic light

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
ASSEMBLY_TRIGGER_Y_MM = 842.9   # 500.0 + 342.9 (13.5 inches)
ASSEMBLY_TRIGGER_X_MAX  = 50.0   # only trigger while still on the x=0 leg
                                  # (prevents re-triggering on later laps)
DELIVERY_TRIGGER_Y_MM  = 1200.0   # ← TUNE: Y position to trigger delivery
DELIVERY_TRIGGER_X_MIN = 2000.0     # only fire on the x=2240 leg
 
 
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
    delivery_done = False
 
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
      # ══════════════════════════════════════════════════════════════════════
        # WAIT_FOR_GREEN
        # ══════════════════════════════════════════════════════════════════════
        if state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 0)

            # Swing arm to face traffic light so camera can see it
            try:
                robot.arm_elevator_to(ELEVATOR_HOME)           # make sure elevator is raised
                robot.arm_swing_to(TRAFFIC_LIGHT_SWING_STEPS)  # ← TUNE: aim at light
            except Exception as e:
                print(f"[WARN] Arm swing for traffic light failed: {e}")

            if traffic_light_color == "green":
                print("[VISION] Green — starting path.")
                # Return arm to safe home before driving
                try:
                    robot.arm_swing_to(SWING_HOME)
                except Exception as e:
                    print(f"[WARN] Arm return to home failed: {e}")

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
                  
                if (
                    assembly_done
                    and not delivery_done
                    and current_y <= DELIVERY_TRIGGER_Y_MM
                    and abs(current_x) >= DELIVERY_TRIGGER_X_MAX
                ):
                    print(
                        f"[FSM] Delivery trigger at "
                        f"({current_x:.0f}, {current_y:.0f}) mm → DELIVERY"
                    )
                    robot.stop()
                    state = "DELIVERY"
 
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
        # DELIVERY  — drive to customer and release burger
        # ══════════════════════════════════════════════════════════════════════
        elif state == "DELIVERY":
            print(f"[FSM] Starting burger delivery to Customer {detected_customer}...")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)
 
            try:
                robot.deliver_burger(customer=detected_customer)
                delivery_done = True
                print("[FSM] Delivery complete — resuming path.")
            except Exception as e:
                print(f"[FSM] Delivery error: {e} — resuming path anyway.")
                delivery_done = True
                try:
                    robot.arm_safe_height()
                    robot.arm_open_gripper()
                    robot.arm_swing_to(0)
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
