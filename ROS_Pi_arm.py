import rclpy
from rclpy.logging import get_logger
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import time
import math
import RPi.GPIO as GPIO
import numpy as np

# Config
JOINT_NAMES = ['1st', '2nd', '3rd', '4th', '5th', '6th']  # URDF virtual joints
DIR_PINS = [31, 36, 38, 33, 23, 26]
STEP_PINS = [32, 37, 40, 35, 24, 29]
EN_PIN = 22
STEPS_PER_REV = 200 * 8 # steppers at 1/8 microstepping
RAD_PER_REV = 2 * math.pi
GEAR_RATIOS = [150.0/15.0, 33.0/13.0*19.0, 24.0/16.0*19.0, 100.0/14.0, 80.0/12.0, 80.0/12.0]  # For motor1-4, then motor5,6 (wrist shared GR)
WRIST_DIFF_FACTOR = 2 * (25.0 / 13.0)  # E.g., pitch = (m5 + m6)/2, yaw = (m5 - m6)/2
VIRTUAL_MODE = True  # Plan in virtual joint space

def convert_virtual_to_motor(virtual_positions):
    """Convert [j1..j4, pitch, yaw] to motor1..6 positions (radians)"""
    j1, j2, j3, j4, pitch, yaw = virtual_positions
    m1 = j1 * GEAR_RATIOS[0]
    m2 = j2 * GEAR_RATIOS[1]
    m3 = j3 * GEAR_RATIOS[2]
    m4 = j4 * GEAR_RATIOS[3]
    m5 = (pitch + yaw) * GEAR_RATIOS[4] / WRIST_DIFF_FACTOR  # motor5 = (pitch + yaw)/2 * GR
    m6 = (pitch - yaw) * GEAR_RATIOS[5] / WRIST_DIFF_FACTOR  # motor6 = (pitch - yaw)/2 * GR
    return [m1, m2, m3, m4, m5, m6]

def convert_motor_to_virtual(motor_positions):
    """Convert motor1..6 to [j1..j4, pitch, yaw] for /joint_states"""
    m1, m2, m3, m4, m5, m6 = motor_positions
    j1 = m1 / GEAR_RATIOS[0]
    j2 = m2 / GEAR_RATIOS[1]
    j3 = m3 / GEAR_RATIOS[2]
    j4 = m4 / GEAR_RATIOS[3]
    pitch = (m5 + m6) * WRIST_DIFF_FACTOR / GEAR_RATIOS[4]  # pitch = (m5 + m6)/2
    yaw = (m5 - m6) * WRIST_DIFF_FACTOR / GEAR_RATIOS[5]   # yaw = (m5 - m6)/2
    return [j1, j2, j3, j4, pitch, yaw]

current_motor_positions = convert_virtual_to_motor([0.0,
                                                    -46.0*math.pi/180.0,
                                                    -93.0*math.pi/180.0,
                                                    -360.0*math.pi/180.0,
                                                    97.0*math.pi/180.0,
                                                    0.0*math.pi/180.0])  # In motor radians

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    for dir_pin, step_pin in zip(DIR_PINS, STEP_PINS):
        GPIO.setup(dir_pin, GPIO.OUT)
        GPIO.setup(step_pin, GPIO.OUT)
    GPIO.setup(EN_PIN, GPIO.OUT)

def cleanup_gpio():
    GPIO.cleanup()

def move_steppers(target_motor_positions, delta_time=0.0):
    global current_motor_positions
    deltas = [target - current for target, current in zip(target_motor_positions, current_motor_positions)]
    steps_list = [int(abs(delta * STEPS_PER_REV / RAD_PER_REV)) for delta in deltas]  # Motor rad to steps
    directions = [1 if delta > 0 else 0 for delta in deltas]

    for i, dir_pin in enumerate(DIR_PINS):
        GPIO.output(dir_pin, directions[i])

    max_steps = max(steps_list) if any(steps_list) else 0
    if max_steps == 0:
        current_motor_positions = target_motor_positions[:]
        return

    cycle_delay = delta_time / max_steps if delta_time > 0 else 0.001 * max_steps

    errors = [0.0] * 6
    increments = [steps / max_steps if max_steps > 0 else 0 for steps in steps_list]

    for macro_step in range(max_steps):
        start_cycle = time.time()
        for i in range(6):
            errors[i] += increments[i]
            if errors[i] >= 1.0:
                GPIO.output(STEP_PINS[i], 1)
                time.sleep(0.0001)
                GPIO.output(STEP_PINS[i], 0)
                errors[i] -= 1.0
        elapsed = time.time() - start_cycle
        if cycle_delay > elapsed:
            time.sleep(cycle_delay - elapsed)

    actual_steps = [round(inc * max_steps) for inc in increments]
    for i in range(6):
        delta_rad_motor = (actual_steps[i] * (1 if directions[i] else -1)) / (STEPS_PER_REV / RAD_PER_REV)
        current_motor_positions[i] += delta_rad_motor

def trajectory_callback(msg: JointTrajectory):
    logger = get_logger("pi_stepper_executor")
    if msg.joint_names != JOINT_NAMES:
        logger.error("Joint names mismatch!")
        return

    setup_gpio()
    prev_time = 0.0
    for point in msg.points:
        target_virt = point.positions
        target_motor = convert_virtual_to_motor(target_virt)
        delta_time = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) - prev_time
        move_steppers(target_motor, delta_time)

        # Publish virtual joint states for RViz
        js = JointState()
        js.header.stamp = rclpy.clock.Clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = convert_motor_to_virtual(current_motor_positions)
        joint_state_pub.publish(js)
        prev_time += delta_time

    cleanup_gpio()
    logger.info("Trajectory executed on steppers")

def main():

    global joint_state_pub
    rclpy.init()
    node = rclpy.create_node('stepper_arm_pi_executor')
    joint_state_pub = node.create_publisher(JointState, '/joint_states', 10)
    # Publish initial state
    initial_js = JointState()
    initial_js.header.stamp = node.get_clock().now().to_msg()
    initial_js.name = JOINT_NAMES
    initial_js.position = convert_motor_to_virtual(current_motor_positions)
    joint_state_pub.publish(initial_js)
    node.create_subscription(JointTrajectory, '/joint_trajectory', trajectory_callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()