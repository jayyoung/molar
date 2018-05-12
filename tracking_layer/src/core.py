#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
from segment_layer.srv import *
from segment_layer.msg import *
from tracking_layer.srv import *

class TrackingLayer:
    def __init__(self):
        rospy.init_node('molar_tracking_layer', anonymous = False)
        rospy.loginfo("* Tracking Layer Starting Up")
        self.service = rospy.Service('/molar/tracking', MolarEpisodeCoherence, self.track_cb)
        rospy.loginfo("* Tracking Layer Ready")

        rospy.spin()

    def track(self,req):
        rospy.logerr("** NO SEGMENTATION WRAPPER SPECIFIED **")
        raise NotImplementedError

    def track_cb(self,req):
        return self.track(req)

class PassThroughTrackingLayer(TrackingLayer):
    def track(self,req):
        rospy.loginfo("\t* PASSTHROUGH TRACKING LAYER: Doing nothing")
        rospy.loginfo("\t* Not even trying to cohere: "+str(len(req.input))+" scenes")
        #output = MolarSceneFilterResult()
        # populate this with the results of the tracking
        rospy.loginfo("\t* Done!")

        return MolarEpisodeCoherenceResponse(req.input)



if __name__ == '__main__':
    l = PassThroughTrackingLayer()
    #r = l.segment_cb("butts")
    #print(r)
    #l = SensorLayer()
    #l.read_scene("what")
