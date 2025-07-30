#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def generate_3d_grid(x_range, y_range, z_range, resolution):
    points = []
    x = x_range[0]
    while x <= x_range[1]:
        y = y_range[0]
        while y <= y_range[1]:
            z = z_range[0]
            while z <= z_range[1]:
                points.append((x, y, z))
                z += resolution
            y += resolution
        x += resolution
    return points


if __name__ == '__main__':
    rospy.init_node('static_pointcloud_publisher')

    pub = rospy.Publisher('/clickable_cloud', PointCloud2, queue_size=1, latch=True)

    header = Header()
    header.frame_id = "base_link"  

    grid1 = generate_3d_grid(x_range=(0.6, 0.9), y_range=(-0.15, 0.15), z_range=(0.4, 0.7), resolution=0.05)


    grid2 = generate_3d_grid(x_range=(-0.6, -0.4), y_range=(-0.15, 0.15), z_range=(0.9, 1.1), resolution=0.05)

    all_points = grid1 + grid2

    cloud_msg = pc2.create_cloud_xyz32(header, all_points)
    pub.publish(cloud_msg)

    rospy.loginfo("Static PointCloud Publshed /clickable_cloud")
    rospy.spin()
