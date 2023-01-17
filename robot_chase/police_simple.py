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
from geometry_msgs.msg import PoseStamped

# Constants used for indexing.
X = 0
Y = 1
YAW = 2


class Police(Node):

    def __init__(self, args):
        super().__init__('police')
        self._robots = POLICE
        self._game_start_sent = False
        self._game_end_sent = False

        captured = [False for _ in range(len(BADDIE))]
        self._captured = dict(zip(BADDIE, captured))
        self._captured_subscriber = self.create_subscription(String, '/captured', self.captured_callback, 5)

        groundtruthposes = [GroundtruthPose(robot) for robot in ALL_ROBOTS]
        self._groundtruthposes = dict(zip(ALL_ROBOTS, groundtruthposes))
        pose_subscribers = [self.create_subscription(Odometry, PREFIX[robot] + 'odometry',
                                                           self._groundtruthposes[robot].callback, 10) for robot in ALL_ROBOTS]
        self._pose_subscribers = dict(zip(ALL_ROBOTS, pose_subscribers))

        baddie_twists = [BaddieTwist(robot) for robot in BADDIE]
        self._baddie_twists = dict(zip(BADDIE, baddie_twists))
        twist_subscribers = [self.create_subscription(Twist, PREFIX[robot] + 'cmd_vel',
                                                           self._baddie_twists[robot].callback, 10) for robot in BADDIE]
        self._twist_subscribers = dict(zip(BADDIE, twist_subscribers))

        cmd_publishers = [self.create_publisher(Twist, PREFIX[robot] + 'cmd_vel', 5) for robot in self._robots]
        self._cmd_publishers = dict(zip(self._robots, cmd_publishers))

        self._stat_publisher = self.create_publisher(PoseStamped, '/stat', 5)

        self._cmd_timer = self.create_timer(timer_period_sec=0.1, callback=self.cmd_callback)

        self.get_logger().info('Police init finished')


    def captured_callback(self, msg):
        self.get_logger().info('Baddie ' + msg.data + ' captured')
        self._captured[msg.data] = True

    
    def game_end(self):
        for r in BADDIE:
            if not self._captured[r]:
                return False
        return True


    def cmd_callback(self):
        for r in ALL_ROBOTS:
            if not self._groundtruthposes[r].ready:
                return
        
        for r in BADDIE:
            if not self._baddie_twists[r].ready:
                return

        if self.game_end(): 
            if not self._game_end_sent:
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.y = float(1)
                self._stat_publisher.publish(msg)
                self._game_end_sent = True

            vel_msg = Twist()
            self._cmd_publishers[r].publish(vel_msg)
            return

        if not self._game_start_sent:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(1)
            self._stat_publisher.publish(msg)
            self._game_start_sent = True

        for r in self._robots:
            for b in BADDIE:

                if self._captured[b]:
                    continue

                pose = self._groundtruthposes[r].pose
                baddie_pose = self._groundtruthposes[b].pose
                baddie_twist = self._baddie_twists[b].twist

                if np.linalg.norm(baddie_pose[:2] - pose[:2]) < POLICE_RADAR:

                    goal = estimate_goal(baddie_pose, baddie_twist[0])
                    direction = (goal[:2] - pose[:2]) / np.linalg.norm(goal[:2] - pose[:2])
                    v = direction * POLICE_SPEED

                    u, w = feedback_linearized(pose, v, EPSILON)
                    vel_msg = Twist()
                    vel_msg.linear.x = float(u)
                    vel_msg.angular.z = float(w)
                    self._cmd_publishers[r].publish(vel_msg)

                else:
                    vel_msg = Twist()
                    self._cmd_publishers[r].publish(vel_msg)


def run(args):
    rclpy.init()
    police_node = Police(args)

    rclpy.spin(police_node)

    police_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Police robot')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()