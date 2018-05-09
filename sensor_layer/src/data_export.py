import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
import os
import json
import pickle

class DataExporter:
    def __init__(self):
        rospy.loginfo("* Data Writer Online")

    def write_scene(self,sensor_data):
        pass

class LocalDiscDataExporter(DataExporter):
    def __init__(self,target=None):
        if(target is None):
            self.ROOT_DIR = os.path.dirname(os.path.realpath(__file__))+"/molar_data/"
        rospy.loginfo("* Using LocalDiscDataExporter Strategy")
        rospy.loginfo("* Writing to: "+self.ROOT_DIR)

    def write_scene(self,sensor_data):
        # just picke it in the right directory
        meta_data = json.loads(sensor_data.meta_data)
        episode_id = meta_data['episode_id']
        scene_id = meta_data['scene_id']
        target_dir = self.ROOT_DIR+episode_id+"/"
        target_file = target_dir+scene_id+".p"
        if(os.path.exists(target_dir)):
            pass
        else:
            os.makedirs(target_dir)
            rospy.loginfo("CREATED DIRECTORY: "+target_dir)
        rospy.loginfo("* WRITING FILE:"+target_file)

        output = open(target_file, 'wb')
        pickle.dump(sensor_data,output)
        output.close()
