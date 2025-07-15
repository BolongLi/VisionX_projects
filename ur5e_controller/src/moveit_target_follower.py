#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs

class UR5eTargetFollower:
    def __init__(self):
        # Setup moveit and ROS node
        moveit_commander.roscpp_initialize([])
        rospy.init_node('ur5e_target_follower', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create MoveGroupCommander object
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")

        self.arm_group.set_planning_time(5)

        rospy.Subscriber("/target_pose", PoseStamped, self.target_callback)

        rospy.loginfo("UR5e Target Follower with TF ready.")
        rospy.spin()

    def target_callback(self, msg):
        # rospy.loginfo("Received target pose: (%.2f, %.2f, %.2f)",
        #               msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
        try:
            # Transform from camera_link to base_link
            transformed = self.tf_buffer.transform(msg, "base_link", rospy.Duration(1.0))
            rospy.loginfo("Transformed pose to base_link: x=%.3f y=%.3f z=%.3f",
                          transformed.pose.position.x,
                          transformed.pose.position.y,
                          transformed.pose.position.z)

        except Exception as e:
            rospy.logwarn("TF transform failed: %s", e)
            return
    
        # saftey check
        test_pose = PoseStamped()
        test_pose.header = Header(frame_id="base_link")
        test_pose.pose = transformed.pose

        self.arm_group.set_pose_target(test_pose)
        #plan = self.arm_group.plan()
        success, plan, _, _ = self.arm_group.plan()

        if success and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Target is reachable. Executing...")
            self.arm_group.go(wait=True)
        else:
            rospy.logwarn("Target is not reachable or planning failed.")

if __name__ == '__main__':
    try:
        UR5eTargetFollower()
    except rospy.ROSInterruptException:
        pass
