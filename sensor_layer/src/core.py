#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
from data_export import *
from segment_layer.srv import *
from sensor_layer.srv import *
from processing_layer.srv import *
import json

class SensorLayer:
    def __init__(self):
        rospy.init_node('molar_sensor_layer', anonymous = False)
        rospy.loginfo("* Sensor Layer Starting Up")

        self.data_exporter = LocalDiscDataExporter()
        self.service = rospy.Service('/molar/sensor_input', MolarSensorInput, self.sense_cb)
        rospy.loginfo("* Sensor Layer Ready")
        rospy.spin()

    def sense_cb(self,req):
        return self.read_scene(req)

    def read_scene(self,sensor_data):
        rospy.loginfo("* Received sensor data")

        # generate a unique id for this scene
        meta = json.loads(sensor_data.meta_data)
        meta['scene_id'] = str(uuid.uuid4())

        rospy.loginfo("* Generated ID for scene:"+meta['scene_id'])
        sensor_data.meta_data = json.dumps(meta)
        # timestamp
        # tf
        # scene cloud
        # rgb image
        # depth image
        # camera info
        # robot pose
        # meta data: source of perception action, current waypoint, episode id
        self.backup_data(sensor_data)
        rospy.loginfo("* Data Backed Up")
        rospy.loginfo("* Passing Data Through to Segmentation Layer")
        self.passthrough_data(sensor_data)
        rospy.loginfo("* Finished Processing Sensor Data")
        return MolarSensorInputResponse(True)

    def backup_data(self,sensor_data):
        #sensor_data['scene_id'] = uuid.uuid4()
        self.data_exporter.write_scene(sensor_data)

    def passthrough_data(self,sensor_data):
        # send to segment layer
        rospy.loginfo("* Waiting for segmentation wrapper service")
        rospy.wait_for_service("/molar/segmentation",timeout=30)
        rospy.loginfo("* Got it! Segmenting scene")
        segmentation_wrapper = rospy.ServiceProxy("/molar/segmentation",MolarSegmentScene)
        seg_response = segmentation_wrapper(sensor_data.cloud,sensor_data.rgb,sensor_data.depth)
        # send response to processing layer
        processing_wrapper = rospy.ServiceProxy("/molar/process_scene",MolarProcessScene)
        proc_response = processing_wrapper(seg_response.output)



if __name__ == '__main__':
    l = SensorLayer()
    #l.read_scene("what")
