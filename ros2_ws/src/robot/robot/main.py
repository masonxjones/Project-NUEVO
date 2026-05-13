from __future__ import annotations
import time

# Corrected hardware constants based on your hardware_map.py
from robot.hardware_map import (
    DEFAULT_FSM_HZ, LED, POSITION_UNIT,
    WHEEL_DIAMETER, WHEEL_BASE, INITIAL_THETA_DEG,
    LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR,
    LEFT_DIR_INVERTED, RIGHT_DIR_INVERTED  # Fixed these names
)
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
DRIVE_SPEED_MM_S = 100.0 

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    
    # Task 4 Requirement: Set odometry parameters 
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        starting_theta_deg=INITIAL_THETA_DEG,
        left_motor_channel=LEFT_WHEEL_MOTOR,
        right_motor_channel=RIGHT_WHEEL_MOTOR,
        left_motor_inverted=LEFT_DIR_INVERTED,
        right_motor_inverted=RIGHT_DIR_INVERTED
    )
    
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color


# ---------------------------------------------------------------------------
# run() - entry point
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    lights_off_at = 0.0
    last_shown_color = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # -- INIT -----------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] WATCHING - Green drives, Red/None stops")
            state = "WATCHING"

        # -- WATCHING -------------------------------------------------------
        elif state == "WATCHING":
            now = time.monotonic()
            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color == "green":
                # Drive forward for Task 4
                robot.set_velocity(DRIVE_SPEED_MM_S, 0.0) 
                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                robot.set_led(LED.RED, 0)
                lights_off_at = now + LIGHT_HOLD_SEC
                
                if last_shown_color != "green":
                    print("[VISION] Green light: Driving forward")
                last_shown_color = "green"

            elif traffic_light_color == "red":
                # Stop for Task 4
                robot.stop() 
                robot.set_led(LED.RED, LED_BRIGHTNESS)
                robot.set_led(LED.GREEN, 0)
                lights_off_at = now + LIGHT_HOLD_SEC
                
                if last_shown_color != "red":
                    print("[VISION] Red light: Stopping")
                last_shown_color = "red"

            elif lights_off_at > 0.0 and now >= lights_off_at:
                # Stop if detection is lost
                robot.stop()
                dim_all_leds(robot)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[VISION] No detection: Stopping & LEDs off")
                last_shown_color = None

        # -- Tick-rate control ---------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
