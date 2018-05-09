#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
from segment_layer.srv import *
from segment_layer.msg import *

class SceneSegmentationLayer:
    def __init__(self):
        rospy.init_node('molar_segment_layer', anonymous = False)
        rospy.loginfo("* Segmentation Layer Starting Up")
        self.service = rospy.Service('/molar/segmentation', MolarSegmentScene, self.segment_cb)
        rospy.loginfo("* Segmentation Layer Ready")

        rospy.spin()

    def segment(self,req):
        pass

    def segment_cb(self,req):
        return self.segment(req)

class GenericSceneSegmentationLayer(SceneSegmentationLayer):
    def segment(self,req):
        rospy.loginfo("\t* Segmenting Scene with GENERIC")

        output = MolarSegmentResult()

        rospy.loginfo("\t* Done!")

        return MolarSegmentSceneResponse(output)



if __name__ == '__main__':
    l = GenericSceneSegmentationLayer()
    #r = l.segment_cb("butts")
    #print(r)
    #l = SensorLayer()
    #l.read_scene("what")