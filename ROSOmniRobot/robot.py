# import local package for PWM
import sys
from RPiPCA9685 import RPiPCA9685

# import board
from time import sleep
import math

# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Constants
MAX_SPEED = 1
REAR = 2
LEFT = 1
RIGHT = 0
REAR_MIN_PULSE = 1060
REAR_MAX_PULSE = 2075
LEFT_MIN_PULSE = 1060
LEFT_MAX_PULSE = 2075
RIGHT_MIN_PULSE = 1060
RIGHT_MAX_PULSE = 2075


# mapping function
def mapping(value, old_min=0, old_max=20000, new_min=0x0000, new_max=0xffff):
    if old_min == old_max:
        return new_min
    old_range = old_max - old_min
    new_range = new_max - new_min
    return int((((value - old_min) * new_range) / old_range) + new_min)


# Clamping function
def clamp(n, nmin, nmax):
    return max(min(n, nmax), nmin)


# Generate the angle triangle
def angle(a, p):
    theta = a - p
    return theta


# Determine the velocity of the motor
def get_speed(theta):
    vel = MAX_SPEED * math.sin(math.radians(theta))
    return vel


class Robot:
    def __init__(self, controller, alpha_r, alpha_l, alpha_re):
        self.controller = controller
        self.alpha_right = alpha_r
        self.alpha_left = alpha_l
        self.alpha_rear = alpha_re

    def move_angle(self, Phi, speed, rotation):
        theta1 = angle(self.alpha_right, Phi)
        theta2 = angle(self.alpha_left, Phi)
        theta3 = angle(self.alpha_rear, Phi)
        vel1 = clamp((get_speed(theta1) + rotation) * speed, -1, 1)
        vel2 = clamp((get_speed(theta2) + rotation) * speed, -1, 1)
        vel3 = clamp((get_speed(theta3) + rotation) * speed, -1, 1)
        self.controller.set_pwm(RIGHT, mapping(vel1, -1, 1, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
        self.controller.set_pwm(LEFT, mapping(vel2, -1, 1, LEFT_MIN_PULSE, LEFT_MAX_PULSE))
        self.controller.set_pwm(REAR, mapping(vel3, -1, 1, REAR_MIN_PULSE, REAR_MAX_PULSE))

    def halt(self):
        self.controller.set_pwm(RIGHT, 0)
        self.controller.set_pwm(LEFT, 0)
        self.controller.set_pwm(REAR, 0)


class BotNode(Node):
    def __init__(self, bot):

        super().__init__('bot_node')
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.robot = bot

    def callback(self, twist):
        x_vel = twist.linear.x
        y_vel = twist.linear.y
        vel = math.sqrt(x_vel * x_vel + y_vel * y_vel)  # Velocity Magnitude
        vel = clamp(vel, 0, 1)
        w = clamp(twist.angular.z, -1, 1)
        phi = math.degrees(math.atan2(y_vel, x_vel))
        if x_vel != 0 or y_vel != 0 or w != 0:
            self.get_logger().info(f'Bearing: {phi} ({x_vel},{y_vel}), Rotation: {w}')
            self.robot.move_angle(phi, vel, -w)
        else:
            self.get_logger().info('Robot Halted')
            self.robot.halt()


def main(args=None):
    alpha = [-60, 60, 180]
    i2c_address = 0x40
    i2c_port = 1
    pca = RPiPCA9685.PCA9685(i2c_address, i2c_port)
    pca.set_frequency(50)
    # Loop between different movement directions
    robot = Robot(pca, alpha[0], alpha[1], alpha[2])
    rclpy.init(args=args)
    bot_node = BotNode(robot)
    rclpy.spin(bot_node)
    bot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()