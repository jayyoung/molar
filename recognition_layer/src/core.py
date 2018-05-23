#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
import json
from filter_layer.srv import *
from filter_layer.msg import *
from recognition_layer.srv import *

class SegmentRecognitionLayer:
    def __init__(self):
        rospy.init_node('molar_recognition_layer', anonymous = False)
        rospy.loginfo("* Segment Recognition Layer Starting Up")
        self.service = rospy.Service('/molar/recognise_segments', MolarRecogniseSceneSegments, self.recognise_cb)

        rospy.spin()

    def recognise(self,req):
        rospy.logerr("** NO RECOGNITION WRAPPER SPECIFIED **")
        raise NotImplementedError

    def recognise_cb(self,req):
        return self.recognise(req)

class MovidiusSegmentRecognitionLayer(SegmentRecognitionLayer):
    def recognise(self,req):
        rospy.loginfo("\t* RECOGNITION LAYER: Using MOV recogniton")

        output = []

        for k in req.input.segment_masks:
            output.append("butts")

        rospy.loginfo("\t* Done!")

        return MolarRecogniseSceneSegmentsResponse(output)


if __name__ == '__main__':
    l = MovidiusSegmentRecognitionLayer()
    #l.read_scene("what")
