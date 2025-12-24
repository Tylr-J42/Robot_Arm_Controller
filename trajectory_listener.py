#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import numpy as np
import time
import math
import RPi.GPIO as GPIO

# Config
JOINT_NAMES = ['1st', '2nd', '3rd', '4th', '5th', '6th']  # URDF virtual joints
DIR_PINS = [31, 36, 38, 33, 23, 22]
STEP_PINS = [32, 37, 40, 35, 21, 29]
EN_PIN = 11
STEPS_PER_REV = 200 * 8 # steppers at 1/8 microstepping
RAD_PER_REV = 2 * math.pi
GEAR_RATIOS = [-150.0/15.0, 33.0/13.0*19.0, -24.0/16.0*19.0, -100.0/14.0, -80.0/12.0*25.0/13.0, 80.0/12.0*25.0/13.0]  # For motor1-4, then motor5,6 (wrist shared GR)
WRIST_DIFF_FACTOR = 2 * (25.0 / 13.0)  # E.g., pitch = (m5 + m6)/2, yaw = (m5 - m6)/2
STEP_FREQUENCY = 1250

def convert_virtual_to_motor(virtual_positions):
    """Convert [j1..j4, pitch, yaw] to motor1..6 positions (radians)"""
    j1, j2, j3, j4, pitch, yaw = virtual_positions
    m1 = j1 * GEAR_RATIOS[0]
    m2 = j2 * GEAR_RATIOS[1]
    m3 = j3 * GEAR_RATIOS[2]
    m4 = j4 * GEAR_RATIOS[3]
    m5 = ((pitch + yaw*2) * GEAR_RATIOS[4] / 2) # motor5 = (pitch + yaw)/2 * GR
    m6 = ((pitch - yaw*2) * GEAR_RATIOS[5] / 2)  # motor6 = (pitch - yaw)/2 * GR

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

# starting down position
# current_motor_positions = convert_virtual_to_motor([0.0,
#                                                     -46.0*math.pi/180.0,
#                                                     -93.0*math.pi/180.0,
#                                                     -360.0*math.pi/180.0,
#                                                     97.0*math.pi/180.0,
#                                                     0.0*math.pi/180.0])  # In motor radians

# "neutral" position
neutral_position = convert_virtual_to_motor([0.0,
                                            0.0*math.pi/180.0,
                                            0.0*math.pi/180.0,
                                            0.0*math.pi/180.0,
                                            0.0*math.pi/180.0,
                                            0.0*math.pi/180.0])  # In motor radians

start_position = convert_virtual_to_motor([0.0,
                                            -45.0*math.pi/180.0,
                                            -90.0*math.pi/180.0,
                                            0.0*math.pi/180.0,
                                            -96.0*math.pi/180.0,
                                            0.0*math.pi/180.0])  # In motor radians

current_motor_positions = start_position

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(DIR_PINS[0], GPIO.OUT)
    GPIO.setup(DIR_PINS[1], GPIO.OUT)
    GPIO.setup(DIR_PINS[2], GPIO.OUT)
    GPIO.setup(DIR_PINS[3], GPIO.OUT)
    GPIO.setup(DIR_PINS[4], GPIO.OUT)
    GPIO.setup(DIR_PINS[5], GPIO.OUT)
    GPIO.setup(STEP_PINS[0], GPIO.OUT)
    GPIO.setup(STEP_PINS[1], GPIO.OUT)
    GPIO.setup(STEP_PINS[2], GPIO.OUT)
    GPIO.setup(STEP_PINS[3], GPIO.OUT)
    GPIO.setup(STEP_PINS[4], GPIO.OUT)
    GPIO.setup(STEP_PINS[5], GPIO.OUT)

    GPIO.setup(12, GPIO.OUT)

    GPIO.setup(EN_PIN, GPIO.OUT)

def cleanup_gpio():
    GPIO.cleanup()

def move_steppers(target_motor_positions, time_to_goal):
    """Move steppers to target motor positions."""
    global current_motor_positions
    deltas = [t - c for t, c in zip(target_motor_positions, current_motor_positions)]
    steps_list = [int(abs(d * STEPS_PER_REV / RAD_PER_REV)) for d in deltas]
    directions = [1 if d >= 0 else -1 for d in deltas]
    frequencies_list = [0, 0, 0, 0, 0, 0]

    for i in range(6):
        frequencies_list[i] = steps_list[i] / time_to_goal

    # Calculate pulse timing
    # period = 1.0 / STEP_FREQUENCY  # Time per step
    print(f"Moving to steps: {steps_list}, Directions: {directions}")

    count_list = [0, 0, 0, 0, 0, 0]

    for i in range(0,len(DIR_PINS)):
            if(directions[i]>0):
                GPIO.output(DIR_PINS[i], GPIO.HIGH)
            else:
                GPIO.output(DIR_PINS[i], GPIO.LOW)
            print(directions[i])

    prev_time_list = [0, 0, 0, 0, 0, 0]

    while max(count_list)<=max(steps_list)*2:
        
        for m in range(6):
            current_time = time.perf_counter()
            if(frequencies_list[m] != 0):
                if(current_time-prev_time_list[m] >= 1/(frequencies_list[m]*2)):
                    if(count_list[m]%2 == 1):
                        if(steps_list[m]*2>count_list[m]):
                            GPIO.output(STEP_PINS[m], GPIO.HIGH)

                    else:
                        GPIO.output(STEP_PINS[m], GPIO.LOW)

                    count_list[m] = count_list[m] + 1
                    prev_time_list[m] = current_time

    current_motor_positions = target_motor_positions
    print(current_motor_positions)

class TrajectoryListener(Node):
    def __init__(self):
        super().__init__('trajectory_listener')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.callback,
            10
        )
        self.get_logger().info("Listening for /joint_trajectory...")

    def callback(self, msg):
        self.get_logger().info(f"Received trajectory with {len(msg.points)} points")
        
        self.execute_trajectory(msg)

    def execute_trajectory(self, msg):
        """
    Called every time the controller publishes a new trajectory.
    Converts radians → degrees and streams to the robot.
    """
        self.get_logger().info(f"EXECUTING trajectory: {len(msg.points)} points")

        for i, point in enumerate(msg.points):
            # 1. ROS gives radians → convert to degrees
            joints_rad = point.positions
            joints_deg = [round(np.degrees(q), 2) for q in joints_rad]

            # 2. Pretty log
            t_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.get_logger().info(f"  [{i+1}/{len(msg.points)}] t={t_sec:.1f}s → "
                                f"[{', '.join(f'{d:6.1f}°' for d in joints_deg)}]")
            
            if(t_sec != 0):
                move_steppers(convert_virtual_to_motor(joints_rad), t_sec)

            print("done")

def main():
    setup_gpio()
    rclpy.init()
    node = TrajectoryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cleanup_gpio()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup_gpio()

if __name__ == '__main__':
    main()