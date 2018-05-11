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
import actionlib
import actionlib_msgs
from molar_tools.srv import *
from molar_controller.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from sensor_layer.srv import *

class MolarControllerServer():
    def __init__(self):
        rospy.init_node('molar_controller_server', anonymous = False)
        rospy.loginfo("* Controller servers starting up")

        rospy.loginfo("\t* Starting learning sequence server")

        self.sequence_server = actionlib.SimpleActionServer('/molar/do_learning_sequence', LearningSequenceAction, self.execute_learning_sequence, False)
        self.sequence_server.start()
        rospy.loginfo("\t* Learning sequence server online")

        rospy.loginfo("* Controller servers online")
        rospy.spin()

    def execute_learning_sequence(self, goal):
        begin_episode = rospy.ServiceProxy('/molar/begin_episode',Trigger)
        end_episode = rospy.ServiceProxy('/molar/end_episode',Trigger)
        process_scene = rospy.ServiceProxy('/molar/sensor_input',MolarSensorInput)

        begin_episode()
        for base_pose,ptu_pose in zip(goal.robot_base_poses,goal.robot_ptu_poses):
            # collect data required for sensor layer
            # update goal feedback
            # move to next pose
            pass
        end_episode()

        self.sequence_server.set_succeeded()


if __name__ == '__main__':
    s = MolarControllerServer()
