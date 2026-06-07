from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline

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
LOOKAHEAD_DIST  = 200.0          
MAX_LINEAR_VEL  = 100.0          
MAX_ANGULAR_VEL = 0.8            
GOAL_TOLERANCE  = 20.0

OBS_RANGE_MM    = 400.0          
VIEW_ANGLE_RAD  = math.radians(60.0)  
SAFE_DIST_MM    = 300.0          
AVOIDANCE_DELAY = 100            
ALPHA_LD        = 0.85           
D_OFFSET_MM     = 250.0          # How far sideways it swerves to avoid objects
X_L_MM          = 205.0
LANE_WIDTH_MM   = 450.0

# ---------------------------------------------------------------------------
# Waypoints (A simple 2-meter straight line)
# ---------------------------------------------------------------------------
RAW_WAYPOINTS = [
    (0.0,    0.0),
    (0.0, 2000.0),  # Drive 2000 mm straight ahead
]

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
    # Vision is disabled for this test, ONLY LiDAR is enabled
    robot.enable_lidar()

def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

def setup_planner(robot: Robot, path: list) -> None:
    """Configures the internal library planner with LiDAR permanently ON."""
    robot._nav_follow_pp_path(
        lookahead_distance=LOOKAHEAD_DIST,
        max_linear_speed=MAX_LINEAR_VEL,
        max_angular_speed=MAX_ANGULAR_VEL,
        goal_tolerance=GOAL_TOLERANCE,
        obstacles_range=OBS_RANGE_MM,
        view_angle=VIEW_ANGLE_RAD,
        safe_dist=SAFE_DIST_MM,
        avoidance_delay=AVOIDANCE_DELAY,
        alpha_Ld=ALPHA_LD,
        offset=D_OFFSET_MM,
        lane_width=LANE_WIDTH_MM,
        obstacle_avoidance=True,  # ALWAYS ON for this test
        x_L=X_L_MM,
    )
    robot.planner.current_lane = 'Center'
    robot.planner.set_path(path)

# ── Main FSM ──────────────────────────────────────────────────────────────────
def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # Densify the 2-meter straight line
    path = densify_polyline(list(RAW_WAYPOINTS), spacing=20.0)

    state         = "IDLE"
    period        = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick     = time.monotonic()

    print("[TEST] Setup complete. Robot is waiting.")
    print("[TEST] Press BTN_1 to start driving 2000mm.")

    while True:
        now = time.monotonic()

        # ── Emergency stop ────────────────────────────────────────────────────
        if robot.get_button(Button.BTN_2):
            print("BTN_2 — emergency stop.")
            robot.stop()
            robot.shutdown()
            break

        # ══════════════════════════════════════════════════════════════════════
        # IDLE
        # ══════════════════════════════════════════════════════════════════════
        if state == "IDLE":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)

            # Wait for physical button press to start the test
            if robot.get_button(Button.BTN_1):
                print("[TEST] BTN_1 Pressed! Starting 2-meter drive.")
                setup_planner(robot, path.copy())
                state = "MOVING"

        # ══════════════════════════════════════════════════════════════════════
        # MOVING
        # ══════════════════════════════════════════════════════════════════════
        elif state == "MOVING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)

            # Let the Pure Pursuit + LiDAR math run!
            result = robot._nav_follow_pp_path_loop()

            if result == "IDLE":
                print("[TEST] Reached 2000mm target! Test successful.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 0)
                state = "DONE"

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