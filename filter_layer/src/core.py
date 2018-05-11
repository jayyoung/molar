#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import uuid
import json
from filter_layer.srv import *

class SceneFilterLayer:
    def __init__(self):
        rospy.init_node('molar_filter_layer', anonymous = False)
        rospy.loginfo("* Scene Filter Layer Starting Up")
        self.service = rospy.Service('/molar/filter_scene', MolarFilterScene, self.filter_cb)

        rospy.spin()

    def filter(self,req):
        rospy.logerr("** NO FILTER WRAPPER SPECIFIED **")
        raise NotImplementedError

    def filter_cb(self,req):
        return self.filter(req)

class GenericSceneFilterLayer(SceneFilterLayer):
    def segment(self,req):
        rospy.loginfo("\t* Filtering Scene with some generic stuff")

        output = MolarSceneFilterResult()
        # populate this with the results of the seg


        rospy.loginfo("\t* Done!")

        return MolarFilterSceneResponse(output)


if __name__ == '__main__':
    l = GenericSceneFilterLayer()
    #l.read_scene("what")
