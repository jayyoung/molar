import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
from sensor_layer.srv import *
import uuid


if __name__ == '__main__':
    rospy.init_node('molar_sanity_check_node', anonymous = False)
    sensor_layer = rospy.ServiceProxy("/molar/sensor_input",MolarSensorInput)
    sreq = MolarSensorInputRequest()
    sreq.meta_data = '{"episode_id" : "1", "waypoint" : "2", "somethingelse" : "3"}'
    sensor_layer(sreq)
    
