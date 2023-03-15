# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from random import choice, uniform
import sys
import numpy as np

import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from geometry_msgs.msg import Twist, Vector3, Point
from visualization_msgs.msg import Marker


class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            "/yoshi/hazard_detection",
            self.hazard_callback,
            qos.qos_profile_sensor_data
            )
        self.laser_subscription = self.create_subscription(
            LaserScan,
            "/yoshi/scan",
            self.laser_callback,
            qos.qos_profile_sensor_data
            )
        self.hazard_subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/yoshi/cmd_vel', 10)
        self.marker_publisher_ = self.create_publisher(Marker, '/yoshi/visualization_marker', 0)


        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.pub_cmd_vel_callback)
        
        self.twist = Twist()

        self.do_once_timer = None

        self.prev_hazard = []

        self.min_range = 0.1
        self.max_range = 2.0
        self.laser_angle_offset = np.pi
        self.wall_offset = 0.25

    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: "%s"' % self.twist)
    
    def do_once(self, func=None):
        self.do_once_timer.destroy()
        self.get_logger().info('DOONCE FUNC!: "%s"' % func)
        if(func) : func()
        self.get_logger().info('IS DOONCE TIMER DEAD!: "%s"' % self.do_once_timer)

    def makePoint(self, x,y,z):
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p


    def laser_callback(self, msg : LaserScan):
        ranges = np.vstack((np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges)), msg.ranges))
        # self.get_logger().info('A: {}\t D: {}'.format(a,d) )
        self.get_logger().info('RANGES_SHAPE {}'.format(ranges.shape) )

        filtered = ranges[:,(ranges[1] > self.min_range) & (ranges[1] < self.max_range)]
        
        xy = np.vstack((filtered[1] * np.cos(filtered[0] + self.laser_angle_offset),filtered[1] * np.sin(filtered[0] + self.laser_angle_offset)))
        # self.get_logger().info('XY {}'.format(xy) )

        marker = Marker()

        marker.points = [self.makePoint(r[0],r[1],0.0) for r in xy.T]

        marker.header.frame_id = "base_link"
        # marker.header.stamp = self.get_clock().now()
        marker.ns = "yoshi"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_publisher_.publish( marker )


    def hazard_callback(self, msg : HazardDetectionVector):
        # if(len(msg.detections) > 0):
        #     self.get_logger().info('hazardLen!: "%i"' % len(msg.detections))
        actions = {
            HazardDetection.BUMP: ("BUMP",self.handle_bump),
            HazardDetection.BACKUP_LIMIT : ("BACKUP_LIMIT",self.handle_backup_limit),
            HazardDetection.CLIFF: ("CLIFF",self.handle_bump),
            HazardDetection.WHEEL_DROP : ("WHEEL_DROP",self.stop),
            HazardDetection.STALL : ("STALL",self.handle_bump),
            HazardDetection.OBJECT_PROXIMITY : ("OBJECT_PROXIMITY",self.handle_bump)
        }
        newHazardList = []
        for hazard in msg.detections:
            newHazardList.append(hazard.type)
            if (hazard.type not in self.prev_hazard):
                hazardName, action = actions.get(hazard.type, None)
                self.get_logger().info('hazardType!: "%s"' % hazardName)
                self.get_logger().info('RUNNING ACTION!: "%s"' % action)
                if (action):
                    action()
        self.prev_hazard = newHazardList
        
            
    
    def stop(self):
        self.get_logger().info('STOP!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def reverse(self):
        self.get_logger().info('REVERSE!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = -0.01
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def forward(self):
        self.get_logger().info('FORWARD!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.1
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def turn(self):
        self.get_logger().info('TURN!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = choice([-1.0, 1.0])
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)

    def handle_bump(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        self.do_once_timer = self.create_timer(1.0, lambda : self.do_once(func = self.handle_backup_limit))
        self.reverse()
    
    def handle_backup_limit(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        delay = uniform(2.0943951, 4.1887902)
        self.get_logger().info('WAIT! : %f' % delay)
        self.do_once_timer = self.create_timer(delay, lambda : self.do_once(func = self.stop))
        self.turn()
    
    


def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()

    rclpy.spin(wanderer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
