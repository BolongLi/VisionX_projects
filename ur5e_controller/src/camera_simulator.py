#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped

class CameraSimulator:
    def __init__(self):
        rospy.init_node("camera_simulator")
        self.pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)
        self.frame_id = "camera_link"  

        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback)
        rospy.loginfo("Camera Simulator started. Listening to /clicked_point and terminal input.")

        self.run_manual_input_loop()

    def clicked_point_callback(self, msg: PointStamped):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = msg.header.frame_id #"base_link"
        pose_msg.pose.position = msg.point
        pose_msg.pose.orientation.w = 1.0  # Default orientation

        self.pub.publish(pose_msg)
        rospy.loginfo("Published clicked point: (%.2f, %.2f, %.2f) from /clicked_point", 
                      msg.point.x, msg.point.y, msg.point.z)

    def run_manual_input_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                raw = input("Enter target position (x y z): ")
                x, y, z = map(float, raw.strip().split())

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = self.frame_id
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.w = 1.0

                self.pub.publish(pose_msg)
                rospy.loginfo("Published manual input: (%.2f, %.2f, %.2f)", x, y, z)

                rate.sleep()
            except Exception as e:
                rospy.logwarn("Invalid input: %s", e)

if __name__ == '__main__':
    try:
        CameraSimulator()
    except rospy.ROSInterruptException:
        pass
