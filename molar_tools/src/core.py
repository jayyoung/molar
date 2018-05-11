#!/usr/bin/env python
import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf2_ros
import tf, tf2_msgs.msg
import os
import json
import pickle
import PyKDL
from molar_tools.srv import *

class MolarToolsServer():
    def __init__(self):
        rospy.init_node('molar_tools_server', anonymous = False)
        rospy.loginfo("* Tools server starting up")


        self.service = rospy.Service('/molar/tools/convert_cloud_camera_to_map', ConvertCloudCameraToMap, self.convert_cloud_camera_to_map_cb)


        rospy.loginfo("* Tools server online")
        rospy.spin()

    def convert_cloud_camera_to_map_cb(self,req):
        return ConvertCloudCameraToMapResponse(convert_cloud_camera_to_map(req.cloud,req.tf))


    def convert_cloud_camera_to_map(self, cloud, transform):
        target_camera_frame = "/map"
        starting_camera_frame = cloud.header.frame_id

        transformation_store = tf.TransformerROS()
        transformation_store.setTransform(transform)

        t = transformation_store.getLatestCommonTime(target_camera_frame, starting_camera_frame)
        tr_r = transformation_store.lookupTransform(target_camera_frame, starting_camera_frame, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = target_camera_frame
        tr_s.child_frame_id = starting_camera_frame
        tr_s.transform = tr


        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(cloud.header, cloud.fields, points_out)
        rospy.loginfo("done")
        return res

    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))


if __name__ == '__main__':
    s = MolarToolsServer()
