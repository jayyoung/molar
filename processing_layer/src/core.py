#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
import json

class ProcessingLayer:
    def __init__(self):
        rospy.init_node('molar_processing_layer', anonymous = False)
        rospy.loginfo("* Processing Layer Starting Up")
        self.service = rospy.Service('/molar/process_scene', MolarProcessScene, self.process_scene_cb)
        rospy.loginfo("* Processing Layer Online")

    def process_scene(self,scene):

        #
        #   send  to filter layer
        
        #   send result to tracking Layer
        #   do stuff
        #

        pass

    def process_scene_cb(self,req):
        return MolarProcessSceneResponse(self.process_scene(req.input))




if __name__ == '__main__':
    l = ProcessingLayer()
    #l.read_scene("what")
