#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node("camera_marker_publisher")
    pub = rospy.Publisher("camera_marker", Marker, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "camera_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "camera"
    marker.id = 0
    marker.type = Marker.ARROW   
    marker.action = Marker.ADD

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2   # 箭头长度
    marker.scale.y = 0.05  # 箭头宽度
    marker.scale.z = 0.05  # 箭头高度

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    main()
