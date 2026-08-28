#!/usr/bin/env python3
"""
Raspberry Pi arm node  (Option B — keep the existing step-generation).

Runs ON THE PI. Exposes to MoveIt:

  * a FollowJointTrajectory action server at
        /arm_controller/follow_joint_trajectory
    MoveIt (running on the dev machine) plans the whole motion and sends it
    here in ONE goal; we buffer it and drive the steppers locally.

  * continuous /joint_states  (needed by move_group + robot_state_publisher)

The gripper is NOT a MoveIt joint. It is a discrete open/close servo pair,
commanded separately over the /gripper_command topic (Int32: 0=open,
100=close, anything else=stop).

The stepper-driving code (move_steppers) and the kinematic gearing
(convert_virtual_to_motor / convert_motor_to_virtual) are unchanged from
trajectory_listener.py.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

import RPi.GPIO as GPIO
import lgpio

# ----------------------------------------------------------------------------
# Config  (identical to trajectory_listener.py)
# ----------------------------------------------------------------------------
# Canonical order the gearing math below expects: [j1, j2, j3, j4, pitch, yaw].
# MoveIt may send its points in a DIFFERENT joint order, so we remap by name.
ARM_JOINT_NAMES = ['1st', '2nd', '3rd', '4th', '5th', '6th']

DIR_PINS = [31, 36, 38, 33, 23, 22]
STEP_PINS = [32, 37, 40, 35, 21, 29]
EN_PIN = 11
STEPS_PER_REV = 200 * 8            # steppers at 1/8 microstepping
RAD_PER_REV = 2 * math.pi
GEAR_RATIOS = [-150.0/15.0, 33.0/13.0*19.0, -24.0/16.0*19.0, -100.0/14.0,
               -80.0/12.0*25.0/13.0, 80.0/12.0*25.0/13.0]
WRIST_DIFF_FACTOR = 2 * (25.0 / 13.0)

JOINT_STATE_RATE_HZ = 25.0         # how often we publish /joint_states


def convert_virtual_to_motor(virtual_positions):
    """Convert [j1..j4, pitch, yaw] to motor1..6 positions (radians)."""
    j1, j2, j3, j4, pitch, yaw = virtual_positions
    m1 = j1 * GEAR_RATIOS[0]
    m2 = j2 * GEAR_RATIOS[1]
    m3 = j3 * GEAR_RATIOS[2]
    m4 = j4 * GEAR_RATIOS[3]
    m5 = ((pitch + yaw*2) * GEAR_RATIOS[4] / 2)
    m6 = ((pitch - yaw*2) * GEAR_RATIOS[5] / 2)
    return [m1, m2, m3, m4, m5, m6]


def convert_motor_to_virtual(motor_positions):
    """Convert motor1..6 to [j1..j4, pitch, yaw] for /joint_states."""
    m1, m2, m3, m4, m5, m6 = motor_positions
    j1 = m1 / GEAR_RATIOS[0]
    j2 = m2 / GEAR_RATIOS[1]
    j3 = m3 / GEAR_RATIOS[2]
    j4 = m4 / GEAR_RATIOS[3]
    # True inverse of convert_virtual_to_motor's wrist mixing:
    #   m5 = (pitch + 2*yaw)*GR5/2 ;  m6 = (pitch - 2*yaw)*GR6/2
    # solving for pitch/yaw. The previous formula was NOT the inverse and
    # produced out-of-bounds joint states (e.g. 6th -> +6.44 rad), which made
    # MoveIt reject the start state on every plan.
    pitch = m5 / GEAR_RATIOS[4] + m6 / GEAR_RATIOS[5]
    yaw = (m5 / GEAR_RATIOS[4] - m6 / GEAR_RATIOS[5]) / 2
    return [j1, j2, j3, j4, pitch, yaw]


# ----------------------------------------------------------------------------
# Starting state
# ----------------------------------------------------------------------------
start_position = convert_virtual_to_motor([180.0*math.pi/180.0,
                                           -45.0*math.pi/180.0,
                                           -90.0*math.pi/180.0,
                                           0.0*math.pi/180.0,
                                           96.0*math.pi/180.0,
                                           0.0*math.pi/180.0])   # motor radians

current_motor_positions = start_position

# ----------------------------------------------------------------------------
# GPIO setup  (unchanged)
# ----------------------------------------------------------------------------
GPIO.setmode(GPIO.BOARD)
for pin in DIR_PINS + STEP_PINS:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(EN_PIN, GPIO.OUT)

h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, 23)
lgpio.gpio_claim_output(h, 24)


def cleanup_gpio():
    GPIO.cleanup()
    lgpio.gpiochip_close(h)


# ----------------------------------------------------------------------------
# Gripper (servos, driven by lgpio PWM)
# ----------------------------------------------------------------------------
# The two servos are mirrored: 23 increases as it closes, 24 decreases.
GRIPPER_PINS = (23, 24)
GRIPPER_OPEN_ANGLES = (0.0, 185.0)

# Angles at which the jaws just MEET the cube. Commanding exactly this is what
# dropped it: a positional servo only produces torque while it is short of its
# setpoint, so jaws that arrive at the cube's surface arrive and then stop
# pushing. Grip comes from commanding PAST the contact point, so the servo
# stays permanently short of its target and keeps driving into the cube.
GRIPPER_TOUCH_ANGLES = (80.0, 100.0)

# How far past contact to command. This is the grip-force knob. Too little and
# the cube slips; too much and the servos stall hard, which on this arm means
# a sustained current draw into a supply that already browns out -- so raise it
# in small steps and listen for the servos straining.
GRIPPER_SQUEEZE_DEG = 15.0

# A stalled servo holds only while it is being told to. lgpio's tx_pwm is
# software-timed, and the trajectory executor busy-waits a whole CPU core for
# the length of every move, so re-assert the setpoint rather than assuming one
# call holds for the entire retreat.
GRIPPER_REASSERT_HZ = 4.0

_gripper_duty = None      # last commanded (duty23, duty24), or None when limp


def _servo_duty(angle_deg):
    """Servo angle -> duty cycle percent at 50 Hz."""
    return angle_deg / 180.0 * 10.0 + 2.5


def _apply_gripper(duties):
    global _gripper_duty
    _gripper_duty = duties
    for pin, duty in zip(GRIPPER_PINS, duties):
        lgpio.tx_pwm(h, pin, 50, duty)


def open_gripper():
    _apply_gripper(tuple(_servo_duty(a) for a in GRIPPER_OPEN_ANGLES))


def close_gripper():
    a, b = GRIPPER_TOUCH_ANGLES
    _apply_gripper((_servo_duty(a + GRIPPER_SQUEEZE_DEG),
                    _servo_duty(b - GRIPPER_SQUEEZE_DEG)))


def hold_gripper():
    """Re-send the current setpoint. No-op while the gripper is released."""
    if _gripper_duty is not None:
        for pin, duty in zip(GRIPPER_PINS, _gripper_duty):
            lgpio.tx_pwm(h, pin, 50, duty)


def stop_gripper():
    global _gripper_duty
    _gripper_duty = None
    for pin in GRIPPER_PINS:
        lgpio.tx_pwm(h, pin, 50, 0)


# ----------------------------------------------------------------------------
# Stepper motion  (unchanged logic; guarded against divide-by-zero on dt)
# ----------------------------------------------------------------------------
def move_steppers(target_motor_positions, time_to_goal):
    """Move steppers to target motor positions over time_to_goal seconds."""
    global current_motor_positions
    deltas = [t - c for t, c in zip(target_motor_positions, current_motor_positions)]
    steps_list = [int(abs(d * STEPS_PER_REV / RAD_PER_REV)) for d in deltas]
    directions = [1 if d >= 0 else -1 for d in deltas]
    frequencies_list = [0, 0, 0, 0, 0, 0]

    if time_to_goal <= 0.0:
        time_to_goal = 1e-3            # avoid /0; MoveIt's t=0 first point etc.

    for i in range(6):
        frequencies_list[i] = steps_list[i] / time_to_goal

    print(f"Moving to steps: {steps_list}, Directions: {directions}")

    count_list = [0, 0, 0, 0, 0, 0]

    for i in range(0, len(DIR_PINS)):
        if directions[i] > 0:
            GPIO.output(DIR_PINS[i], GPIO.HIGH)
        else:
            GPIO.output(DIR_PINS[i], GPIO.LOW)

    prev_time_list = [0, 0, 0, 0, 0, 0]

    while max(count_list) <= max(steps_list) * 2 and max(steps_list) != 0:
        for m in range(6):
            current_time = time.perf_counter()
            if frequencies_list[m] != 0:
                if current_time - prev_time_list[m] >= 1/(frequencies_list[m]*2):
                    if count_list[m] % 2 == 1:
                        if steps_list[m]*2 > count_list[m]:
                            GPIO.output(STEP_PINS[m], GPIO.HIGH)
                    else:
                        GPIO.output(STEP_PINS[m], GPIO.LOW)
                    count_list[m] = count_list[m] + 1
                    prev_time_list[m] = current_time

    # Advance by what we ACTUALLY stepped, never by what we were asked for.
    #
    # steps_list truncates (int()), so every waypoint leaves a sub-step
    # remainder. Assigning the commanded target here threw that remainder away
    # each time, and because truncation always rounds toward zero the loss is
    # one-directional: it accumulates into a systematic UNDERSHOOT that grows
    # with the number of waypoints. A 0.15 m descent at lin_velocity_scaling
    # 0.01 is ~154 waypoints, which measured out to roughly 5 mm at the tip --
    # and the arm has no encoders, so nothing downstream could ever see it.
    # /joint_states reported the commanded value, so RViz showed the arm
    # arriving while the hardware sat short of the cube.
    #
    # Carrying the remainder into the next segment's delta keeps the total
    # error bounded under a single step forever, instead of growing with path
    # length. It also means the descent no longer has to be run slowly to stay
    # accurate -- slower made this worse, not better.
    step_rad = RAD_PER_REV / STEPS_PER_REV
    current_motor_positions = [
        c + (s if d >= 0 else -s) * step_rad
        for c, s, d in zip(current_motor_positions, steps_list, deltas)
    ]
    return max(steps_list) == 0    # True == no movement happened


# ----------------------------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------------------------
class ArmPiNode(Node):
    def __init__(self):
        super().__init__('arm_pi_node')
        cb = ReentrantCallbackGroup()

        # FollowJointTrajectory action server — this is what MoveIt calls.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=cb,
        )

        # Gripper command topic (0=open, 100=close, else=stop).
        self.create_subscription(Int32, '/gripper_command',
                                 self.gripper_callback, 10, callback_group=cb)

        # Continuous /joint_states so move_group knows where we are.
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / JOINT_STATE_RATE_HZ, self.publish_joint_states,
                          callback_group=cb)

        # Keep re-sending the gripper setpoint. execute_callback blocks for the
        # whole of a trajectory, so this only gets a chance to run because the
        # group is reentrant and the executor is multi-threaded -- same reason
        # /joint_states keeps publishing mid-move. Without it the servos are
        # commanded once and left, and the cube is released during the retreat.
        self.create_timer(1.0 / GRIPPER_REASSERT_HZ, hold_gripper,
                          callback_group=cb)

        self.get_logger().info(
            "arm_pi_node up: FollowJointTrajectory @ "
            "/arm_controller/follow_joint_trajectory")

    # -- gripper ------------------------------------------------------------
    def gripper_callback(self, msg: Int32):
        if msg.data == 0:
            self.get_logger().info("Gripper: OPEN")
            open_gripper()
        elif msg.data == 100:
            self.get_logger().info("Gripper: CLOSE")
            close_gripper()
        else:
            stop_gripper()

    # -- joint states -------------------------------------------------------
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINT_NAMES
        js.position = convert_motor_to_virtual(current_motor_positions)
        self.js_pub.publish(js)

    # -- trajectory execution ----------------------------------------------
    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        incoming = list(traj.joint_names)
        self.get_logger().info(
            f"Executing trajectory: {len(traj.points)} points, "
            f"joints={incoming}")

        # Map our canonical order -> the order MoveIt sent. Do NOT assume the
        # incoming order matches ARM_JOINT_NAMES.
        try:
            remap = [incoming.index(name) for name in ARM_JOINT_NAMES]
        except ValueError as e:
            self.get_logger().error(f"Joint name mismatch: {e}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        prev_t = 0.0
        for i, point in enumerate(traj.points):
            # MoveIt's time_from_start is CUMULATIVE from trajectory start.
            # move_steppers wants a per-segment duration, so take the delta.
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            prev_t = t

            ordered = [point.positions[remap[k]] for k in range(6)]
            target_motor = convert_virtual_to_motor(ordered)

            no_move = move_steppers(target_motor, dt)
            if no_move and dt > 0:
                # Nothing to step (e.g. the initial current-state point) but the
                # trajectory allots time for it — honor the dwell.
                time.sleep(dt)

            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = ARM_JOINT_NAMES
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory complete.")
        return result


def main():
    rclpy.init()
    node = ArmPiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Deliberately NOT stop_gripper(): releasing here drops whatever is in
        # the jaws on a Ctrl-C. cleanup_gpio() closes the chip a moment later,
        # which ends the PWM anyway, so the grip is lost on exit regardless --
        # this just avoids opening the jaws on purpose while holding a cube.
        node.destroy_node()
        rclpy.shutdown()
        cleanup_gpio()


if __name__ == '__main__':
    main()
