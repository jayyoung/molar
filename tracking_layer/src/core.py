#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
from segment_layer.srv import *
from segment_layer.msg import *

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

class GenericTrackingLayer(TrackingLayer):
    def segment(self,req):
        rospy.loginfo("\t* Cohering episodes with naieve SIFT tracker")

        output = MolarSceneFilterResult()
        # populate this with the results of the tracking


        rospy.loginfo("\t* Done!")

        return MolarEpisodeCoherenceResponse(output)



if __name__ == '__main__':
    l = GenericSceneSegmentationLayer()
    #r = l.segment_cb("butts")
    #print(r)
    #l = SensorLayer()
    #l.read_scene("what")
