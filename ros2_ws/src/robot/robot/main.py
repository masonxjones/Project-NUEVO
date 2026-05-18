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

TAG_ID = 11 # set aruco tag ID 11 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
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
    
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    state = "WAIT_FOR_GREEN"
    print("[FSM] Monitoring active. Green drives, Red stops.")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if robot.get_button(Button.BTN_2):
            print("BTN_2 pressed. Shutting down.")
            robot.stop()
            robot.shutdown()
            break

        traffic_light_color = None
        for detection in robot.get_detections("traffic light"):
            if float(detection.get("confidence", 0.0)) >= 0.50:
                attributes = detection.get("attributes", {})
                color_attribute = attributes.get("color", {})
                color = color_attribute.get("value")
                if color in ("red", "green"):
                    traffic_light_color = color
                    break

        if state == "WAIT_FOR_GREEN":
            robot.stop()
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)

            if traffic_light_color == "green":
                print("[VISION] Green light detected! Driving forward.")
                state = "DRIVING"

        elif state == "DRIVING":
            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            robot.set_velocity(100.0, 0.0)

            if traffic_light_color == "red":
                print("[VISION] Red light detected! Stopping.")
                state = "WAIT_FOR_GREEN"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()