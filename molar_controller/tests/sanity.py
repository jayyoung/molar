import roslib
import rospy
import sensor_msgs
import geometry_msgs
import tf
from sensor_layer.srv import *
import uuid
import actionlib
import molar_controller.msg

if __name__ == '__main__':
    rospy.init_node('molar_sanity_check_node', anonymous = False)

    rospy.loginfo("setting up action client")
    client = actionlib.SimpleActionClient("/molar/do_learning_sequence",molar_controller.msg.LearningSequenceAction)
    # add some robot poses, or leave empty to just perceive whatever the robot is looking at right now

    rospy.loginfo("waiting for server")
    client.wait_for_server()

    rospy.loginfo("setting up empty goal")
    goal_action_wrapper = molar_controller.msg.LearningSequenceActionGoal()
    goal_action_wrapper.goal.robot_base_poses = []
    goal_action_wrapper.goal.robot_ptu_poses = []
    rospy.loginfo(goal_action_wrapper)

    rospy.loginfo("sending goal")
    client.send_goal(goal_action_wrapper.goal)

    rospy.loginfo("waiting for result")
    client.wait_for_result()

    rospy.loginfo("result is:")
    rospy.loginfo(client.get_result())
