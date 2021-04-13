#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from src.pathPlanning.trajectoryGenerator import Config
from src.pathPlanning.pathTarget import PathTargetList
from src.pathPlanning import pathGenerator


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        self.current_telem = None
        self.path_finnished = True
        self.seg_num = -1
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node("lab")
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        pass



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = angular_speed

        ### Publish the message
        self.cmd_vel.publish(vel_msg)

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

        print("X:" + str(self.px) + " Y:" + str(self.py) + " Theta:" + str(self.pth))




    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT

        goal_x = distance * math.cos(self.pth) + self.px
        goal_y = distance * math.sin(self.pth) + self.py

        goal_theta = self.pth

        speed_limited_config = Config(.001, linear_speed, 2.5, 10)

        path = PathTargetList()
        path.addTarget(self.px, self.py, self.pth)
        path.addTarget(goal_x, goal_y, goal_theta)

        self.current_telem = pathGenerator.generateFromPath(path, speed_limited_config)
        self.seg_num = 0
        self.path_finnished = False




    def run(self):
        rate = rospy.Rate(1) #1kHz
        rate.sleep()

        lastHeading = self.pth
        while not rospy.is_shutdown():
            #self.send_speed(.1, .05)
            if not self.path_finnished:
                self.send_speed(self.current_telem.get_segment(self.seg_num).vel, (self.current_telem.get_segment(self.seg_num).heading - lastHeading) / .0001)
                lastHeading = self.current_telem.get_segment(self.seg_num).heading
                self.seg_num += 1
                if self.seg_num == self.current_telem.get_num_segments():
                    self.path_finnished = True
                    self.seg_num = -1
            rate.sleep()

if __name__ == '__main__':
    try:
        Lab2().run()
    except rospy.ROSInterruptException:
        pass
