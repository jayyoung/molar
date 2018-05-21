#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import tf2_ros
import tf, tf2_msgs.msg
import os
import json
import pickle
import actionlib
import actionlib_msgs
from molar_tools.srv import *
from molar_controller.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from sensor_layer.srv import *
from collections import deque
class MolarControllerServer():
    def __init__(self):
        rospy.init_node('molar_controller_server', anonymous = False)
        rospy.loginfo("* Controller servers starting up")

        rospy.loginfo("\t* Starting learning sequence server")

        self.point_cloud_topic = rospy.get_param("molar_point_cloud_topic")
        self.rgb_img_topic = rospy.get_param("molar_rgb_img_topic")
        self.depth_img_topic = rospy.get_param("molar_depth_img_topic")
        self.camera_info_topic = rospy.get_param("molar_camera_info_topic")
        self.robot_pose_topic = rospy.get_param("molar_robot_pose_topic")
        self.tf_topic = rospy.get_param("molar_tf_topic")
        self.tf_cache_length = rospy.get_param("molar_tf_cache_length")
        self.transformations = deque([]) # use this to keep a big cache of tf msgs

        rospy.loginfo("\t * Using the following topics for sensor input: ")
        rospy.loginfo("\t \t Point Cloud Topic: " + self.point_cloud_topic)
        rospy.loginfo("\t \t RGB Image Topic: " + self.rgb_img_topic)
        rospy.loginfo("\t \t Depth Image Topic: " + self.depth_img_topic)
        rospy.loginfo("\t \t Camera Info Topic: " + self.camera_info_topic)
        rospy.loginfo("\t \t Robot Pose Topic: " + self.robot_pose_topic)
        rospy.loginfo("\t \t TF Topic: " + self.tf_topic)

        self.sequence_server = actionlib.SimpleActionServer('/molar/do_learning_sequence', LearningSequenceAction, self.execute_learning_sequence, False)
        self.sequence_server.start()

        rospy.loginfo("\t* Learning sequence server online")



        rospy.loginfo("* Controller servers online")
        rospy.spin()

    def execute_learning_sequence(self, goal):
        rospy.loginfo("Sequence Goal Received")
        begin_episode = rospy.ServiceProxy('/molar/begin_episode',Trigger)
        end_episode = rospy.ServiceProxy('/molar/end_episode',Trigger)
        #process_scene = rospy.ServiceProxy('/molar/sensor_input',MolarSensorInput)

        rospy.loginfo("---- BEGINNING LEARNING EPISODE ----")
        begin_episode()

        if(goal.robot_base_poses):
            rospy.loginfo("Target Views: " + str(len(goal.robot_base_poses)))
            for base_pose,ptu_pose in zip(goal.robot_base_poses,goal.robot_ptu_poses):
                rospy.loginfo("---- Performing Multi Perception ----")
                # move robot to pose
                self.do_percept()
                # do something?
        else:
            rospy.loginfo("No sequence specified, taking single view from current robot pose")
            rospy.loginfo("---- Performing Single Perception ----")
            self.do_percept()


        rospy.loginfo("---- VIEWS TAKEN, PERFORMING POST-PROCESSING ----")
        end_episode()
        rospy.loginfo("---- LEARNING EPISODE COMPLETE ----")

        self.sequence_server.set_succeeded()


    def do_percept(self):
        process_scene = rospy.ServiceProxy('/molar/sensor_input',MolarSensorInput)
        cur_time,default_meta_data,point_cloud,rgb_img,depth_img,camera_info,robot_pose,tf_msg = self.grab_sensor_data()
        perception_result = process_scene(cur_time,default_meta_data,point_cloud,rgb_img,depth_img,camera_info,robot_pose,tf_msg)
        #rospy.loginfo(perception_result)
        return perception_result


    def tf_cb(self,transforms):
        time_window = rospy.Duration(self.tf_cache_length)
        for transform in transforms.transforms:
            #rospy.loginfo("Got transform: %s - > %s"% ( transform.header.frame_id, transform.child_frame_id))
            if len(self.transformations) > self.tf_cache_length:
                l =  self.transformations.popleft()
                if (transform.header.stamp -  l.header.stamp) < time_window:
                    self.transformations.appendleft(l)
            self.transformations.append(transform)


    # TODO: do this properly so that everything is synchronous
    def grab_sensor_data(self):
        #try:
        topic_timeout = 3
        rospy.loginfo("Grabbing sensor data with a timeout of: " + str(topic_timeout) + " per topic")
        self.transformations = deque([]) # use this to keep a big cache of tf msgs
        cur_time = int(rospy.get_rostime().to_sec())
        default_meta_data = "{}"
        fake_debug = False
        if(fake_debug):
            point_cloud = PointCloud2()
            rgb_img = Image()
            depth_img = Image()
            camera_info = CameraInfo()
            robot_pose = Pose()
            sub = rospy.Subscriber(self.tf_topic, tf2_msgs.msg.TFMessage, self.tf_cb)
            rospy.sleep(self.tf_cache_length)
            sub.unregister()
            tf_msg = tf2_msgs.msg.TFMessage()
            tf_msg.transforms = self.transformations
            return cur_time,default_meta_data,point_cloud,rgb_img,depth_img,camera_info,robot_pose,tf_msg
        else:
            point_cloud = rospy.wait_for_message(self.point_cloud_topic,PointCloud2,timeout=topic_timeout)
            rgb_img = rospy.wait_for_message(self.rgb_img_topic,Image,timeout=topic_timeout)
            depth_img = rospy.wait_for_message(self.depth_img_topic,Image,timeout=topic_timeout)
            camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo,timeout=topic_timeout)
            robot_pose = rospy.wait_for_message(self.robot_pose_topic,PoseWithCovarianceStamped,timeout=topic_timeout)

            sub = rospy.Subscriber(self.tf_topic, tf2_msgs.msg.TFMessage, self.tf_cb)
            rospy.sleep(self.tf_cache_length)
            sub.unregister()
            tf_msg = tf2_msgs.msg.TFMessage()
            tf_msg.transforms = self.transformations

            return cur_time,default_meta_data,point_cloud,rgb_img,depth_img,camera_info,robot_pose,tf_msg
        #except Exception as e:
        #    rospy.logerr("Could not collect sensor data")
        #    rospy.logerr(e)



if __name__ == '__main__':
    s = MolarControllerServer()
