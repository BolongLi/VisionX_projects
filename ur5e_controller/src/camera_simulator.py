#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("camera_simulator")
    pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)

    frame_id = "camera_link"  
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            raw = input("Enter target position (x y z): ")
            x, y, z = map(float, raw.strip().split())

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.w = 1.0 

            pub.publish(pose_msg)
            rospy.loginfo("Published pose: (%f, %f, %f) in %s", x, y, z, frame_id)

            rate.sleep()
        except Exception as e:
            rospy.logwarn("Invalid input: %s", e)

if __name__ == '__main__':
    main()