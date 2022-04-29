#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ros_sim.msg import Pose
from math import sqrt, pow

class Motion:
    def __init__(self):
        self.x_pos = 0
        self.y_pos = 0
        self.x_goal = 0
        self.y_goal = 0
        self.distance_2_goal = 100
        self.vel_pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    def pose_callback(self, msg):
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.distance_2_goal = sqrt(pow(self.x_goal - self.x_pos,2) + pow(self.y_goal - self.y_pos,2))

    def move_straight_line(self, x_goal_, speed_):
        self.x_goal = self.x_pos + x_goal_
        self.y_goal = self.y_pos + 0
        vel = Twist()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rospy.loginfo(self.distance_2_goal)
            if (self.distance_2_goal > 0.1):
                vel.linear.x = speed_
            else:
                vel.linear.x = 0
            self.vel_pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
   try:
        moves = Motion()
        rospy.Subscriber("/turtle1/pose", Pose, moves.pose_callback)
        rospy.init_node('straight_motion')
        moves.move_straight_line(3.0, 0.5)

   except rospy.ROSInterruptException:
       pass