#!/usr/bin/env python  
import rospy
import tf_conversions

import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped, Quaternion
import turtlesim.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path

class RobotBroadcaster:
    def __init__(self):
        rospy.Subscriber('/turtle1/pose', turtlesim.msg.Pose, self.handle_turtle_pose)
        self.path_pub = rospy.Publisher("/robot_path", Path, queue_size=1)
        self.path = Path()
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.stmpd_pose = PoseStamped()
        self.q = Quaternion()
        self.shown = 0

    def handle_turtle_pose(self,msg):
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "world"
        self.t.child_frame_id = "base_link"
        self.t.transform.translation.x = msg.x
        self.t.transform.translation.y = msg.y
        self.t.transform.translation.z = 0.0
        self.q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        self.t.transform.rotation.x = self.q[0]
        self.t.transform.rotation.y = self.q[1]
        self.t.transform.rotation.z = self.q[2]
        self.t.transform.rotation.w = self.q[3]

        
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"

        self.stmpd_pose.header.stamp = rospy.Time.now()
        self.stmpd_pose.header.frame_id = "world"
        self.stmpd_pose.pose.position.x = msg.x
        self.stmpd_pose.pose.position.y = msg.y
        self.stmpd_pose.pose.position.z = 0
        self.stmpd_pose.pose.orientation = self.t.transform.rotation
        self.path.poses.append(self.stmpd_pose)

        if(not self.shown):
            print(self.path)
            self.shown = 1

        self.br.sendTransform(self.t)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    robot = RobotBroadcaster()
    rospy.spin()