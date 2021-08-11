from __future__ import division

import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class RobotConnection(Node):
    def __init__(self,id):
        self.id = id
        super().__init__('robot_'+str(self.id)+'_connection')
        self.twist_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.odom_sub = self.create_subscription(Twist,'odom',self.read_odom,10)

    def read_odom(self,msg):

        # Get the robot position
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        self.z_pose = msg.pose.pose.position.z

        # Get the robot orientation
        self.x_rot = msg.pose.pose.orientation.x
        self.y_rot = msg.pose.pose.orientation.y
        self.z_rot = msg.pose.pose.orientation.z
        self.w_rot = msg.pose.pose.orientation.w

        # Get the robot linear velocity
        self.x_linear = msg.twist.twist.linear.x
        self.y_linear = msg.twist.twist.linear.y
        self.z_linear = msg.twist.twist.linear.z

        # Get the robot angular velocity
        self.x_angular = msg.twist.twist.angular.x
        self.y_angular = msg.twist.twist.angular.y
        self.z_angular = msg.twist.twist.angular.z

    def set_velocity(self,linear_vel,angular_vel):
        # Creates a twist message
        msg = Twist()
        
        # Sets the linear velocity
        msg.linear.x = linear_vel[0]
        msg.linear.y = linear_vel[1]
        msg.linear.z = linear_vel[2]

        # Sets the angular volcity
        msg.angular.x = angular_vel[0]
        msg.angular.y = angular_vel[1]
        msg.angular.z = angular_vel[2]

        # Publishes the twist message
        self.twist_pub.publish(msg)

    def get_linear_vel(self):
        return self.x_linear,self.y_linear,self.z_linear
    
    def get_angular_vel(self):
        return self.x_angular,self.y_angular,self.z_angular
    
    def get_position(self):
        return self.x_pose,self.y_pose,self.z_pose

    def get_orientation_quaternion(self):
        return self.x_rot,self.y_rot,self.z_rot,self.w_rot
    
    def get_orientation_radians(self):
        raise NotImplementedError("Need to figure out the math for this")
    
    def get_orientation_degrees(self):
        raise NotImplementedError("Need to implment radians first")