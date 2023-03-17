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

class LaserFollow(Node):

    def __init__(self, walldist:float, wallSide:float):
        super().__init__('LaserFollow')
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
        self.max_range = 4.0 * walldist
        self.laser_angle_offset = np.pi
        self.wall_offset = walldist
        self.wall_side = wallSide
        self.wall_found = False
        self.twist.linear.x = 0.05
        self.get_logger().info('DONE INIT!')

    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        # self.get_logger().info('Publishing: "%s"' % self.twist)
    
    def do_once(self, func=None):
        self.do_once_timer.destroy()
        # self.get_logger().info('DOONCE FUNC!: "%s"' % func)
        if(func) : func()
        # self.get_logger().info('IS DOONCE TIMER DEAD!: "%s"' % self.do_once_timer)

    def makePoint(self, x,y,z):
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p


    def laser_callback(self, msg : LaserScan):
        self.twist.angular.z = 0.0

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges)) + self.laser_angle_offset
        angles[angles > np.pi] -= 2 * np.pi
        angles[angles < -np.pi] += 2 * np.pi
        ranges = np.vstack((angles, msg.ranges))
        # self.get_logger().info('A: {}\t D: {}'.format(a,d) )
        # self.get_logger().info('RANGES_SHAPE {}'.format(ranges.shape) )

        filtered_by_range = ranges[:,(ranges[1] > self.min_range) & (ranges[1] < self.max_range)]
        
        front_range_angle = filtered_by_range[:,(filtered_by_range[0] > (- np.pi / 3.0)) & (filtered_by_range[0] < (np.pi / 3.0))]

        if ((front_range_angle[1] < self.wall_offset).any()):
            self.wall_found = True
            self.twist.angular.z += self.wall_side
        
        side_range_angle = filtered_by_range[:,((-1 * self.wall_side * filtered_by_range[0]) > ( np.pi / 4.0)) & ((-1 * self.wall_side * filtered_by_range[0]) < (3.0 * np.pi / 4.0))]
        # self.get_logger().info('WALLCOUNT {}'.format(side_range_angle.shape) )
            
        if(side_range_angle.shape[1] > 10):
            avg_side_dist = np.average(side_range_angle[1,:])
            self.get_logger().info('DELTA {}'.format(self.wall_offset - avg_side_dist) )
            if(self.wall_found):
                self.twist.angular.z += self.wall_side * (self.wall_offset - avg_side_dist)
            

        front_xy = np.vstack((front_range_angle[1] * np.cos(front_range_angle[0]),front_range_angle[1] * np.sin(front_range_angle[0])))
        side_xy = np.vstack((side_range_angle[1] * np.cos(side_range_angle[0]),side_range_angle[1] * np.sin(side_range_angle[0])))
        
        # self.get_logger().info('XY {}'.format(xy.shape) )

        

        markerFront = Marker()

        markerFront.points = [self.makePoint(r[0],r[1],0.0) for r in front_xy.T]

        markerFront.header.frame_id = "base_link"
        # marker.header.stamp = self.get_clock().now()
        markerFront.ns = "yoshi"
        markerFront.id = 0
        markerFront.type = Marker.POINTS
        markerFront.action = Marker.ADD
        markerFront.scale.x = 0.05
        markerFront.scale.y = 0.05
        markerFront.scale.z = 0.05
        markerFront.color.a = 1.0 # Don't forget to set the alpha!
        markerFront.color.r = 0.0
        markerFront.color.g = 1.0
        markerFront.color.b = 0.0

        self.marker_publisher_.publish( markerFront )

        markerSide = Marker()

        markerSide.points = [self.makePoint(r[0],r[1],0.0) for r in side_xy.T]

        markerSide.header.frame_id = "base_link"
        # markerSide.header.stamp = self.get_clock().now()
        markerSide.ns = "yoshi"
        markerSide.id = 1
        markerSide.type = Marker.POINTS
        markerSide.action = Marker.ADD
        markerSide.scale.x = 0.05
        markerSide.scale.y = 0.05
        markerSide.scale.z = 0.05
        markerSide.color.a = 1.0 # Don't forget to set the alpha!
        markerSide.color.r = 1.0
        markerSide.color.g = 1.0
        markerSide.color.b = 1.0

        self.marker_publisher_.publish( markerSide )


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
                # if (action):
                #     action()
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
    wallDist = -1.0
    wallSide = 0.0

    while (wallDist <= 0 ):
        wallDist = float(input ("ENTER THE WALL DISTANCE IN METERS > 0: "))
    
    while (wallSide == 0):
        userInput = input("ENTER THE WALL SIDE<left|right>: ").lower()
        if (userInput == 'left' or userInput == 'l'):
            wallSide = -1.0
        elif (userInput == 'right' or userInput == 'r'):
            wallSide = 1.0

    print(wallDist, wallSide)
    laserfollower = LaserFollow(wallDist, wallSide)

    rclpy.spin(laserfollower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laserfollower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
