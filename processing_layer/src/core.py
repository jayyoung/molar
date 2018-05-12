#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
import json
from processing_layer.srv import *
from filter_layer.srv import *
from tracking_layer.srv import *
from std_srvs.srv import Trigger, TriggerResponse

class ProcessingLayer:
    def __init__(self):
        rospy.init_node('molar_processing_layer', anonymous = False)
        rospy.loginfo("* Processing Layer Starting Up")
        self.service = rospy.Service('/molar/process_scene', MolarProcessScene, self.process_scene_cb)
        self.episode_start = rospy.Service('/molar/begin_episode',Trigger,self.begin_episode)
        self.episode_end = rospy.Service('/molar/end_episode',Trigger,self.end_episode)

        rospy.loginfo("* Processing Layer Online")
        rospy.spin()

    def begin_episode(self,trigger):
        self.cur_episode_scenes = []
        self.cur_episode_id = str(uuid.uuid4())
        return TriggerResponse(True,"MOLAR: Beginning new episode with ID: "+self.cur_episode_id)



    def process_scene_cb(self,req):
        return MolarProcessSceneResponse(self.process_scene(req.input))

    def process_scene(self,scene):
        rospy.loginfo("* Processing layer has received a scene")
        #   input: a segmented scene
        #   send to filter layer
        rospy.loginfo("\t* Passing to filter layer")
        filter_service = rospy.ServiceProxy("/molar/filter_scene",MolarFilterScene)
        filter_response = filter_service(scene) # send a segmentresult to the filter, get a filtered result back
        rospy.loginfo("response from filter layer:")
        rospy.loginfo(filter_response)
        self.cur_episode_scenes.append(filter_response.output)
        #   do more stuff?



    def end_episode(self,trigger):
        # cohere episode and store
        rospy.loginfo("\t * Performing coherence on episode with ID " + self.cur_episode_id)
        rospy.loginfo("\t * Num views taken: " + str(len(self.cur_episode_scenes)))
        tracking_service = rospy.ServiceProxy("/molar/tracking",MolarEpisodeCoherence)
        tracking_response = tracking_service(self.cur_episode_scenes) # send a segmentresult to the filter, get a filtered result back
        return TriggerResponse(True,"MOLAR: Episode with ID "+self.cur_episode_id+" finished.")


if __name__ == '__main__':
    l = ProcessingLayer()
    #l.read_scene("what")
