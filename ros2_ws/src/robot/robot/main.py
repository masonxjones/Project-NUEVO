from __future__ import annotations
import time
import math
import numpy as np




from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
# from robot.robot_impl.burger_assembly import BurgerAssemblyMixin
# from robot.examples.burger_delivery import BurgerDeliveryMixin




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




# ── Arm Positional Constants
TRAFFIC_LIGHT_SWING_STEPS = 100  # Steps to swing arm toward traffic light
ELEVATOR_HOME = 0                # Assuming 0 is the default bottom height
SWING_HOME = 0                   # Assuming 0 is straight forward




# ---------------------------------------------------------------------------
# Pure Pursuit + Obstacle Avoidance parameters
# ---------------------------------------------------------------------------
LOOKAHEAD_DIST  = 200.0          
MAX_LINEAR_VEL  = 140.0          
MAX_ANGULAR_VEL = 1           
GOAL_TOLERANCE  = 20.0


OBS_RANGE_MM    = 400.0          
VIEW_ANGLE_RAD  = math.radians(60.0)  
SAFE_DIST_MM    = 300.0          
AVOIDANCE_DELAY = 100            
ALPHA_LD        = 0.85          
D_OFFSET_MM     = 250.0          
X_L_MM          = 205.0
LANE_WIDTH_MM   = 450.0




# ---------------------------------------------------------------------------
# Customer Drop-off & Checkpoint Configurations
# ---------------------------------------------------------------------------
CHECKPOINT_X = 2000.0        # Updated to match your exact map waypoint log
CHECKPOINT_Y = 3660.0        # Updated to match your exact map waypoint log
PRODUCTION_THRESHOLD = 0.25  




DELIVERY_LOCATIONS = {
    "A": (2440.0, 1525.0),    
    "B": (2440.0, 1225.0),    
    "unknown": (2440.0, 1325.0)  
}

FINAL_WAYPOINT = (2440.0, 0.0)

# ---------------------------------------------------------------------------
# Waypoints (Initial Route to Checkpoint)
# ---------------------------------------------------------------------------
RAW_WAYPOINTS = [
    (0.0,    0.0),
    (0.0,    3560.0),
    (410,  3560.0),
    (410.0,  410.0),
    (1325.0, 410.0),
    (1325.0, 610.0),
    (1325.0, 3460.0),
    (2240.0, 3460.0),
    (2240.0, 0.0),
]


# ── Assembly trigger ──────────────────────────────────────────────────────────
ASSEMBLY_TRIGGER_Y_MM   = 875    
ASSEMBLY_TRIGGER_X_MAX  = 50.0    




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




def detect_stop_sign(robot: Robot) -> bool:
    for detection in robot.get_detections("stop sign"):
        if float(detection.get("confidence", 0.0)) >= 0.50:
            return True
    return False




def setup_planner(robot: Robot, path: list) -> None:
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
        obstacle_avoidance=False,
        x_L=X_L_MM,
    )
    robot.planner.current_lane = 'Center'
    robot.planner.set_path(path)




# ── Main FSM ──────────────────────────────────────────────────────────────────
def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)




    path = densify_polyline(list(RAW_WAYPOINTS), spacing=20.0)




    # State tracking variables
    assembly_done = False
    checkpoint_visited = False
    checkpoint_stop_start = 0.0
    detected_customer = "unknown"
    highest_match_score = -1.0
   
    delivery_done = False
    stop_sign_serviced = False  
    stop_sign_start_time = 0.0  




    state         = "WAIT_FOR_GREEN"
    period        = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick     = time.monotonic()




    print("[FSM] Waiting for GREEN traffic light...")




    while True:
        now = time.monotonic()




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




            try:
                robot.arm_elevator_to(ELEVATOR_HOME)          
                robot.arm_swing_to(TRAFFIC_LIGHT_SWING_STEPS)  
            except Exception as e:
                pass




            if traffic_light_color == "green":
                print("[VISION] Green — setting up planner and starting.")
                try:
                    robot.arm_swing_to(SWING_HOME)
                except Exception as e:
                    print(f"[WARN] Arm return to home failed: {e}")




                setup_planner(robot, path.copy())
                state = "MOVING"
                print("[FSM] → MOVING")




        # ══════════════════════════════════════════════════════════════════════
        # MOVING
        # ══════════════════════════════════════════════════════════════════════
        elif state == "MOVING":




            if delivery_done and not stop_sign_serviced and detect_stop_sign(robot):
                print("[VISION] Stop sign detected! Pausing for 3 seconds...")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                stop_sign_start_time = time.monotonic()
                state = "STOP_SIGN_PAUSE"
                print("[FSM] → STOP_SIGN_PAUSE")




            elif traffic_light_color == "red":
                print("[VISION] Red — pausing.")
                robot.stop()
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 255)
                state = "PAUSED"




            else:
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)




                current_x, current_y, _ = robot.get_pose()




                # ── Assembly Trigger ──────────────────────────────────────────
                if (
                    not assembly_done
                    and current_y >= ASSEMBLY_TRIGGER_Y_MM
                    and abs(current_x) <= ASSEMBLY_TRIGGER_X_MAX
                ):
                    print(f"[FSM] Assembly trigger at ({current_x:.0f}, {current_y:.0f}) mm → ASSEMBLY")
                    robot.stop()
                    state = "ASSEMBLY"




                # ── Normal Movement ───────────────────────────────────────────
                else:
                    # Check if the robot is in the correct X-channel (the third leg) 
                    # and has physically driven past your new waypoint at Y = 610.0
                    passed_lidar_trigger_point = (1000.0 <= current_x <= 1500.0) and (current_y >= 610.0)

                    if passed_lidar_trigger_point:
                        if not robot.planner.obstacle_avoidance:
                            print(f"[LIDAR] Passed waypoint (1325, 610) at current Y: {current_y:.1f} — Avoidance ENABLED")
                            robot.planner.obstacle_avoidance = True
                    else:
                        # Turn off avoidance if it leaves this leg or finishes the zone
                        # (Adjust the 3460.0 upper bound if you want it to turn off right at the next corner)
                        if robot.planner.obstacle_avoidance and current_y > 3460.0:
                            print("[LIDAR] Leg complete — Avoidance DISABLED (Pure Pursuit only)")
                            robot.planner.obstacle_avoidance = False

                    # Run navigation loop
                    result = robot._nav_follow_pp_path_loop()


                    if result == "IDLE":
                        # CRITICAL FIX: If the initial path finishes and we haven't visited the checkpoint yet,
                        # transition directly into the CUSTOMER_CHECKPOINT pause!
                        if not checkpoint_visited:
                            print(f"[NAV] Finished initial path segment at ({current_x:.1f}, {current_y:.1f})")
                            print("[FSM] Pausing for 3 seconds to scan customer ID face...")
                            robot.stop()
                            checkpoint_stop_start = time.monotonic()
                            checkpoint_visited = True
                            state = "CUSTOMER_CHECKPOINT"
                            print("[FSM] → CUSTOMER_CHECKPOINT")
                           
                        elif checkpoint_visited and not delivery_done:
                            print("[MOVING] Arrived at customer desk! Transitioning to DELIVERY.")
                            robot.stop()
                            state = "DELIVERY"
                            print("[FSM] → DELIVERY")
                           
                        elif delivery_done:
                            print("[MOVING] Final home waypoint reached!")
                            robot.stop()
                            state = "POST_DELIVERY_IDLE"
                            print("[FSM] → POST_DELIVERY_IDLE")




        # ══════════════════════════════════════════════════════════════════════
        # STOP_SIGN_PAUSE (3 Second Wait near Finish)
        # ══════════════════════════════════════════════════════════════════════
        elif state == "STOP_SIGN_PAUSE":
            robot.stop()
            robot.set_led(LED.ORANGE, 255)




            if time.monotonic() - stop_sign_start_time >= 3.0:
                print("[FSM] 3-second stop complete. Resuming path to final waypoint.")
                stop_sign_serviced = True
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.ORANGE, 0)
                state = "MOVING"
                print("[FSM] → MOVING")




        # ══════════════════════════════════════════════════════════════════════
        # CUSTOMER_CHECKPOINT (3 Second Pause to scan Face)
        # ══════════════════════════════════════════════════════════════════════
        elif state == "CUSTOMER_CHECKPOINT":
            robot.stop() # Force physical brake
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)




            # Continuous scan for customer tags while waiting
            for person in robot.get_detections("person"):
                attributes = person.get("attributes", {})
                if "customer_id" in attributes:
                    customer_attr = attributes.get("customer_id", {})
                    customer_id = customer_attr.get("value")
                    confidence = float(customer_attr.get("score", 0.0))




                    if customer_id in ("A", "B") and confidence >= PRODUCTION_THRESHOLD:
                        if confidence > highest_match_score:
                            highest_match_score = confidence
                            detected_customer = customer_id




            # Evaluate if the 3-second sampling timeline is complete
            if time.monotonic() - checkpoint_stop_start >= 3.0:
                print(f"[DECISION] Checkpoint pause complete. Target: Customer {detected_customer.upper()} "
                      f"(Confidence: {max(0.0, highest_match_score):.2f})")
               
                target_x, target_y = DELIVERY_LOCATIONS[detected_customer]
                print(f"[NAV] Planning route to customer drop-off coordinates: ({target_x}, {target_y})")
               
                # Build route from current position to the customer's desk location
                current_x, current_y, _ = robot.get_pose()
                customer_route = densify_polyline([(current_x, current_y), (target_x, target_y)], spacing=20.0)
               
                # Supply the planner with the new coordinates and kick-start it back to life
                robot.planner.set_path(customer_route)
               
                robot.set_led(LED.ORANGE, 0)
                state = "MOVING"
                print("[FSM] → MOVING (To Customer Drop-off)")




        # ══════════════════════════════════════════════════════════════════════
        # DELIVERY
        # ══════════════════════════════════════════════════════════════════════
        elif state == "DELIVERY":
            print(f"[FSM] Executing drop-off routine for Customer {detected_customer}...")
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)




            try:
                robot.deliver_burger(customer=detected_customer)
                delivery_done = True
                print("[FSM] Delivery complete — Stop sign detection active.")
            except Exception as e:
                print(f"[FSM] Delivery error: {e} — resuming path anyway.")
                delivery_done = True
                try:
                    robot.arm_safe_height()
                    robot.arm_open_gripper()
                    robot.arm_swing_to(0)
                except Exception:
                    pass




            print("[MOVING] Planning final return leg to home waypoint...")
            current_x, current_y, _ = robot.get_pose()
            return_route = densify_polyline([(current_x, current_y), FINAL_WAYPOINT], spacing=20.0)
            robot.planner.set_path(return_route)




            robot.set_led(LED.GREEN, 255)
            robot.set_led(LED.ORANGE, 0)
            state = "MOVING"
            print("[FSM] → MOVING (Heading Home)")




        # ══════════════════════════════════════════════════════════════════════
        # ASSEMBLY
        # ══════════════════════════════════════════════════════════════════════
        # ══════════════════════════════════════════════════════════════════════
        # ASSEMBLY
        # ══════════════════════════════════════════════════════════════════════
        elif state == "ASSEMBLY":
            print("[FSM] Skipping burger assembly (Commented Out)...")
           
            # We simply flag it as done and immediately switch back to moving
            assembly_done = True
            state = "MOVING"
            print("[FSM] → MOVING (resumed straight past assembly table)")




            # --- ORIGINAL ASSEMBLY CODE COMMENTED OUT BELOW ---
            # print("[FSM] Starting burger assembly...")
            # robot.set_led(LED.GREEN, 0)
            # robot.set_led(LED.ORANGE, 200)
            # try:
            #     robot.assemble_burger(home_on_start=False)
            #     try:
            #         robot.arm_safe_height()
            #     except Exception:
            #         pass
            #     assembly_done = True
            #     print("[FSM] Assembly complete — resuming path.")
            # except Exception as e:
            #     print(f"[FSM] Assembly error: {e} — resuming path anyway.")
            #     assembly_done = True
            #     try:
            #         robot.arm_safe_height()
            #         robot.arm_open_gripper()
            #     except Exception:
            #         pass
            #        
            # robot.set_led(LED.GREEN, 255)
            # robot.set_led(LED.ORANGE, 0)
            # state = "MOVING"
            # print("[FSM] → MOVING (resumed)")




        # ══════════════════════════════════════════════════════════════════════
        # PAUSED (Traffic Light)
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
        # POST_DELIVERY_IDLE
        # ══════════════════════════════════════════════════════════════════════
        elif state == "POST_DELIVERY_IDLE":
            robot.stop()




        # ── Tick rate control ─────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()




if __name__ == "__main__":
    pass