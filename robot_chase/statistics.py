#!/usr/bin/env python

import argparse
import rclpy
import numpy as np
from rclpy.node import Node
from robot_chase.common import *

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class Stat(Node):

    def __init__(self, args):
        super().__init__('stat')
        self._started = False
        self._start_time = None
        self._ended = False
        self._end_time = None

        self._stat_subscriber = self.create_subscription(PoseStamped, '/stat', self.stat_callback, 5)

        self.get_logger().info('Stat init finished')

        
    def stat_callback(self, msg):
        if not self._started and msg.pose.position.x == 1:
            self._started = True
            self._start_time = rclpy.time.Time.from_msg(msg.header.stamp)
            self.get_logger().info('Game started at ' + str(self._start_time))

        if not self._ended and msg.pose.position.y == 1:
            self._ended = True
            self._end_time = rclpy.time.Time.from_msg(msg.header.stamp)
            self.get_logger().info('Game ended at ' + str(self._end_time))
            self.get_logger().info('Game finished in ' + str((self._end_time.nanoseconds - self._start_time.nanoseconds) / 1000000000) + ' seconds')


def run(args):
    rclpy.init()
    stat_node = Stat(args)

    rclpy.spin(stat_node)

    stat_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Statistics node')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()