import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf2_ros
import tf, tf2_msgs.msg
import os
import json
import pickle



if __name__ == '__main__':
    rospy.init_node('molar_sanity_check_server', anonymous = False)
    rospy.loginfo("* Perception grabber server starting up")

    rospy.loginfo("pcd")
    point_cloud = rospy.wait_for_message("/head_camera/depth_registered/points",PointCloud2)
    rospy.loginfo("rgb")

    rgb_img = rospy.wait_for_message("/head_camera/rgb/image",Image)
    rospy.loginfo("depth")

    depth_img = rospy.wait_for_message("/head_camera/rgb/depth",Image)
    rospy.loginfo("cinfo")

    camera_info = rospy.wait_for_message("/head_camera/rgb/camera_info",CameraInfo)
    rospy.loginfo("robotpose")


    robot_pose = rospy.wait_for_message("/amcl_pose",PoseWithCovarianceStamped)
    rospy.loginfo("tf")


    tf = rospy.wait_for_message("/tf",TFMessage)
