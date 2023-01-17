#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node

from nav_msgs.msg import Odometry

# For groundtruth information.
from scipy.spatial.transform import Rotation as R

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

ALL_ROBOTS = ['p0', 'p1', 'p2', 'b0', 'b1', 'b2']
POLICE = ['p0', 'p1', 'p2']
BADDIE = ['b0', 'b1', 'b2']
BADDIE_SPEED = .14
POLICE_SPEED = .1
CAPTURE_DIST = 0.15
EPSILON = .1
PREDICT_TIME = .5
POLICE_RADAR = 1.

PREFIX = {
    'p0': '/police/robot0/',
    'p1': '/police/robot1/',
    'p2': '/police/robot2/',
    'b0': '/baddie/robot3/',
    'b1': '/baddie/robot4/',
    'b2': '/baddie/robot5/',
}


class GroundtruthPose(Node):
    def __init__(self, name):
        super().__init__('groundtruth_pose')
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._name = name


    def callback(self, msg):
        self._pose[X] = msg.pose.pose.position.x
        self._pose[Y] = msg.pose.pose.position.y
        _, _, yaw = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]).as_euler(seq="XYZ")
        self._pose[YAW] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[X])

    @property
    def name(self):
        return self._name

    @property
    def pose(self):
        return self._pose


class BaddieTwist(Node):
    def __init__(self, name):
        super().__init__('baddie_twist')
        self._twist = np.array([np.nan, np.nan], dtype=np.float32)
        self._name = name


    def callback(self, msg):
        self._twist[0] = msg.linear.x
        self._twist[1] = msg.angular.z

    @property
    def ready(self):
        return not np.isnan(self._twist[X])

    @property
    def name(self):
        return self._name

    @property
    def twist(self):
        return self._twist


def feedback_linearized(pose, velocity, epsilon):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # MISSING: Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.

    # Solution:

    xp_dot = velocity[X]
    yp_dot = velocity[Y]
    theta = pose[YAW]

    u = xp_dot * np.cos(theta) + yp_dot * np.sin(theta)
    w = (1. / epsilon) * (-xp_dot * np.sin(theta) + yp_dot * np.cos(theta))

    return u, w

def estimate_goal(target_pose, linear_velocity):

    goal = np.zeros_like(target_pose)

    theta = target_pose[YAW]
    x_dot = linear_velocity * PREDICT_TIME * np.cos(theta)
    y_dot = linear_velocity * PREDICT_TIME * np.sin(theta)

    goal[X] = target_pose[X] + x_dot
    goal[Y] = target_pose[Y] + y_dot

    return goal
