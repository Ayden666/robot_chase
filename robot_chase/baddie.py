#!/usr/bin/env python

import argparse
import rclpy
import random
import math
import numpy as np
from rclpy.node import Node
from robot_chase.common import *

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Constants used for indexing.
X = 0
Y = 1
YAW = 2


class Baddie(Node):

    def __init__(self, args):
        super().__init__('baddie')
        self._robots = BADDIE

        captured = [False for _ in range(len(BADDIE))]
        self._captured = dict(zip(self._robots, captured))

        groundtruthposes = [GroundtruthPose(robot) for robot in ALL_ROBOTS]
        self._groundtruthposes = dict(zip(ALL_ROBOTS, groundtruthposes))
        pose_subscribers = [self.create_subscription(Odometry, PREFIX[robot] + 'odometry',
                                                           self._groundtruthposes[robot].callback, 10) for robot in ALL_ROBOTS]
        self._pose_subscribers = dict(zip(ALL_ROBOTS, pose_subscribers))


        velocities = [np.array([np.nan, np.nan], dtype=np.float32) for _ in range(len(BADDIE))]
        self._velocities = dict(zip(self._robots, velocities))

        cmd_publishers = [self.create_publisher(Twist, PREFIX[robot] + 'cmd_vel', 5) for robot in self._robots]
        self._cmd_publishers = dict(zip(self._robots, cmd_publishers))

        self._captured_publisher = self.create_publisher(String, '/captured', 5)

        self._cmd_timer = self.create_timer(timer_period_sec=0.1, callback=self.cmd_callback)
        self._new_velocity_timer = self.create_timer(timer_period_sec=3., callback=self.new_velocity_callback)

        self.get_logger().info('Baddie init finished')


    def new_velocity_callback(self):
        for r in self._robots:
            # Choose a random point on the unit circle as the direction
            angle = random.uniform(0, 2.0*math.pi)
            self._velocities[r][X] = math.cos(angle)
            self._velocities[r][Y] = math.sin(angle)
            self._velocities[r] *= BADDIE_SPEED


    def cmd_callback(self):
        for r in ALL_ROBOTS:
            if not self._groundtruthposes[r].ready:
                return
        
        for r in self._robots:
            if np.isnan(self._velocities[r][X]):
                return

        for r in self._robots:
            if self._captured[r]:
                vel_msg = Twist()
                self._cmd_publishers[r].publish(vel_msg)
                continue

            pose = self._groundtruthposes[r].pose
            for p in POLICE:
                police_pose = self._groundtruthposes[p].pose
                if np.linalg.norm(police_pose[:2] - pose[:2]) < CAPTURE_DIST:
                    self.get_logger().info('Baddie ' + r + ' captured by ' + p)
                    msg = String()
                    msg.data = r
                    self._captured_publisher.publish(msg)
                    self._captured[r] = True
                    break

            if self._captured[r]:
                continue

            u, w = feedback_linearized(pose, self._velocities[r], EPSILON)
            vel_msg = Twist()
            vel_msg.linear.x = float(u)
            vel_msg.angular.z = float(w)
            self._cmd_publishers[r].publish(vel_msg)


def run(args):
    rclpy.init()
    baddie_node = Baddie(args)

    rclpy.spin(baddie_node)

    baddie_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Baddie robot')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()