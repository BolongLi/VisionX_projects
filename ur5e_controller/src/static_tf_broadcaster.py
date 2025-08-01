#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node("static_tf_broadcaster")
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_tf = geometry_msgs.msg.TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "base_link"
    static_tf.child_frame_id = "camera_link"

    static_tf.transform.translation.x = -0.2  
    static_tf.transform.translation.y = 0.0
    static_tf.transform.translation.z = 1.35  

    static_tf.transform.rotation.w = 1.0 

    broadcaster.sendTransform(static_tf)
    rospy.loginfo("Published static transform base_link -> camera_link")
    rospy.spin()

if __name__ == '__main__':
    main()