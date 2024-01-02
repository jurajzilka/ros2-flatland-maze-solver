#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class CustomRobotControllerNode(Node):


    def process_LaserScan(self,data):

        twist_msg = Twist()

        num_lasers = len(data.ranges)
        
        left_laser = data.ranges[67]
        left_front_laser = data.ranges[56] 
        front_laser = data.ranges[45]
        right_front_laser = data.ranges[37]
        
        twist_msg.linear.x = 0.0      
        twist_msg.angular.z = 0.0  
        
        d = 0.7 # distance from the wall, turning right - turning left +
        too_close = 0.5

        if right_front_laser > d and front_laser > d and left_front_laser > d and left_laser > d: 
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 1.0  # nothing seen - rotate to left

        elif right_front_laser > d and front_laser > d and left_front_laser > d and left_laser < d: #only left
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 0.0

        elif right_front_laser > d and front_laser > d and left_front_laser < d and left_laser > d: #only left-front
             twist_msg.angular.z = 0.0
             twist_msg.linear.x = 0.3

        elif right_front_laser > d and front_laser < d and left_front_laser > d and left_laser > d: #only front
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.0

        elif right_front_laser < d and front_laser > d and left_front_laser > d and left_laser > d: #only right 
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.0

        elif (right_front_laser > d and front_laser > d and left_front_laser < d and left_laser < d): ## left and left-front
              if (left_front_laser < too_close):
                     twist_msg.angular.z = -1.0
                     twist_msg.linear.x = 0.0
              else:
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 0.0

        elif right_front_laser > d and front_laser < d and left_front_laser > d and left_laser < d: ## left and front
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.0  # rotate right

        elif right_front_laser < d and front_laser > d and left_front_laser > d and left_laser < d: ## left and right
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 0.0

        elif right_front_laser > d and front_laser < d and left_front_laser < d and left_laser > d: ## left-front and front
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.3

        elif right_front_laser < d and front_laser > d and left_front_laser < d and left_laser > d: ## left-front and right
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 1.0

        elif right_front_laser < d and front_laser < d and left_front_laser > d and left_laser > d: ## front and right
               twist_msg.angular.z = 1.0
               twist_msg.linear.x = 0.3
         
        elif right_front_laser > d and front_laser < d and left_front_laser < d and left_laser < d: ### left and left-front and front
               twist_msg.linear.x = 0.0
               twist_msg.angular.z = -1.0
       
        elif right_front_laser < d and front_laser > d and left_front_laser < d and left_laser < d: ### left and left-front and right
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 1.0
       
        elif right_front_laser < d and front_laser < d and left_front_laser < d and left_laser > d: ### left-front and front and right
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.0
        
        elif right_front_laser < d and front_laser < d and left_front_laser > d and left_laser < d: ### left and front and right
               twist_msg.linear.x = 0.3
               twist_msg.angular.z = 1.0

        elif right_front_laser < d and front_laser < d and left_front_laser < d and left_laser < d: ####
               twist_msg.angular.z = -1.0
               twist_msg.linear.x = 0.0

        else:
          pass     

        self.twist_pub.publish(twist_msg)

    def __init__(self):
        super().__init__("custom_robot_controller")

        #self.scan_topic = "/simple_lidar"   # uncomment only one of these lines at a time, depending on which LiDAR you wish to use
        self.scan_topic = "/scan"   # "/static_laser" uses the complete LiDAR scan, "/simple_lidar" uses a simplified version

        self.twist_topic = "/cmd_vel"

        self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.process_LaserScan,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.twist_pub = self.create_publisher(
                                Twist,
                                self.twist_topic,
                                QoSProfile(depth=10))




if __name__ == '__main__':

    rclpy.init()
    custom_robot_controller = CustomRobotControllerNode()

    try:        
        rclpy.spin(custom_robot_controller)        

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
