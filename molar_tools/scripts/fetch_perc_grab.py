import roslib
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import tf2_ros
import tf, tf2_msgs.msg
import os
import json
import pickle
from collections import deque

transformations = deque([]) # use this to keep a big cache of tf msgs

def cb(transforms):
    time_window = rospy.Duration(2)
    for transform in transforms.transforms:
        #rospy.loginfo("Got transform: %s - > %s"% ( transform.header.frame_id, transform.child_frame_id))
        if len(transformations) > 2:
            l =  transformations.popleft()
            if (transform.header.stamp -  l.header.stamp) < time_window:
                transformations.appendleft(l)
        transformations.append(transform)

if __name__ == '__main__':
    rospy.init_node('molar_sanity_check_server', anonymous = False)
    rospy.loginfo("* Perception grabber server starting up")
    rospy.loginfo("pcd")
    point_cloud = rospy.wait_for_message("/head_camera/depth_registered/points",PointCloud2)
    rospy.loginfo("rgb")

    rgb_img = rospy.wait_for_message("/head_camera/rgb/image_rect_color",Image)
    rospy.loginfo("depth")

    depth_img = rospy.wait_for_message("/head_camera/depth/image_rect",Image)
    rospy.loginfo("cinfo")

    camera_info = rospy.wait_for_message("/head_camera/rgb/camera_info",CameraInfo)
    rospy.loginfo("robotpose")

    robot_pose = rospy.wait_for_message("/amcl_pose",PoseWithCovarianceStamped)
    rospy.loginfo("tf")

    transformations = deque([])
    sub = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, cb)
    rospy.sleep(2)
    sub.unregister()
