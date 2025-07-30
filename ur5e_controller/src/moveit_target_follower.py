#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros
import math
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import Constraints, JointConstraint

class UR5eTargetFollower:
    def __init__(self):
        # Setup moveit and ROS node
        moveit_commander.roscpp_initialize([])
        rospy.init_node('ur5e_target_follower', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create MoveGroupCommander object
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")

        self.arm_group.set_planning_time(2)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_max_velocity_scaling_factor(1.0)       
        self.arm_group.set_max_acceleration_scaling_factor(1.0)    


        rospy.Subscriber("/target_pose", PoseStamped, self.target_callback)

        self.home()

        rospy.loginfo("UR5e Target Follower with TF ready.")
        rospy.spin()

    def home(self):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = math.radians(0)    # shoulder_pan
        joint_goal[1] = math.radians(-90)  # shoulder_lift
        joint_goal[2] = math.radians(90)   # elbow
        joint_goal[3] = math.radians(-90)
        joint_goal[4] = math.radians(-90)
        joint_goal[5] = math.radians(0)
        self.arm_group.set_joint_value_target(joint_goal)
        self.arm_group.go(wait=True)


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
        
        task_mode = "grasp"  # or "place"

        if task_mode == "grasp":
            qx, qy, qz, qw = quaternion_from_euler(0, math.pi, 0)  # 180 deg pitch
        elif task_mode == "place":
            qx, qy, qz, qw = quaternion_from_euler(0, 3.14 * 3 / 4, 0)  # ~135 deg pitch

        
            
        # saftey check
        test_pose = PoseStamped()
        test_pose.header = Header(frame_id="base_link")
        test_pose.pose.position = transformed.pose.position
        test_pose.pose.orientation.x = qx
        test_pose.pose.orientation.y = qy
        test_pose.pose.orientation.z = qz
        test_pose.pose.orientation.w = qw

        self.arm_group.set_pose_target(test_pose)
        #plan = self.arm_group.plan()
        success, plan, _, _ = self.arm_group.plan()

        joint_constraints = Constraints()
        joint_limit = JointConstraint()
        joint_limit.joint_name = "shoulder_lift_joint" 
        joint_limit.position = math.radians(-68)
        joint_limit.tolerance_above = math.radians(30)
        joint_limit.tolerance_below = math.radians(17)
        joint_limit.weight = 1.0
        joint_constraints.joint_constraints.append(joint_limit)

        self.arm_group.set_path_constraints(joint_constraints)

        if success and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Target is reachable. Executing...")
            self.arm_group.execute(plan, wait=True)
        else:
            rospy.logwarn("Target is not reachable or planning failed.")

if __name__ == '__main__':
    try:
        UR5eTargetFollower()
    except rospy.ROSInterruptException:
        pass
