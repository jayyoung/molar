#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
import geometry_msgs
import tf
import uuid
from segment_layer.srv import *
from segment_layer.msg import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random

class SceneSegmentationLayer:
    def __init__(self):
        rospy.init_node('molar_segment_layer', anonymous = False)
        rospy.loginfo("* Segmentation Layer Starting Up")
        self.service = rospy.Service('/molar/segmentation', MolarSegmentScene, self.segment_cb)
        rospy.loginfo("* Segmentation Layer Ready")

        self.blob_segment_pub = rospy.Publisher('/molar/segmentation/blob_cluster_image', Image, queue_size=10, latch=True)
        self.rgb_segment_pub = rospy.Publisher('/molar/segmentation/rgb_cluster_image', Image, queue_size=10, latch=True)

        #self.segment_pub.publish(std_msgs.msg.String("foo"))

        rospy.spin()

    def segment(self,req):
        rospy.logerr("** NO SEGMENTATION WRAPPER SPECIFIED **")
        raise NotImplementedError

    def segment_cb(self,req):
        return self.segment(req)

class PassThroughSceneSegmentationLayer(SceneSegmentationLayer):
    def segment(self,req):
        rospy.loginfo("\t* SEGMENTATION LAYER: Using passthrough segmentation")

        output = MolarSegmentResult()
        # populate this with the results of the seg


        rospy.loginfo("\t* Done!")

        return MolarSegmentSceneResponse(output)

class GraphCannySegmentationWrapper(SceneSegmentationLayer):
    def segment(self,req):
        rospy.loginfo("\t* SEGMENTATION LAYER: Using GRAPH CANNY segmentation")



        from canny_seg_wrapper.srv import CannySegWrapper

        rospy.loginfo("waiting for service")
        rospy.wait_for_service("/canny_seg_wrapper/segment")

        rospy.loginfo("setting up proxy")
        srv = rospy.ServiceProxy("/canny_seg_wrapper/segment",CannySegWrapper)

        output = MolarSegmentResult()

        response = srv(req.rgb,req.depth)

        output.segment_masks = response.output

        bridge = CvBridge()

        cv_col_ims = []
        cv_rgb_ims = []
        for k in output.segment_masks:
            i = bridge.imgmsg_to_cv2(k,desired_encoding="bgr8")
            cv_rgb_ims.append(i.copy())
            g_img = cv2.cvtColor( i, cv2.COLOR_RGB2GRAY )
            ret,thresh_img = cv2.threshold(g_img,127,255,cv2.THRESH_BINARY)
            thresh_img = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2RGB )
            col = [random.randint(0,255),random.randint(0,255),random.randint(0,255)]
            mask_idxs = np.where(thresh_img != 0)
            thresh_img[mask_idxs[0], mask_idxs[1], :] = col
            cv_col_ims.append(thresh_img)

        bs = sum(cv_col_ims)
        rs = sum(cv_rgb_ims)
        cv2.imwrite("segments_blob.png",bs)
        cv2.imwrite("segments_rgb.png",rs)

        self.rgb_segment_pub.publish(bridge.cv2_to_imgmsg(rs,"bgr8"))
        self.blob_segment_pub.publish(bridge.cv2_to_imgmsg(bs,"bgr8"))

        rospy.loginfo("\t* Done!")
        return MolarSegmentSceneResponse(output)



if __name__ == '__main__':
    l = GraphCannySegmentationWrapper()
    #r = l.segment_cb("butts")
    #print(r)
    #l = SensorLayer()
    #l.read_scene("what")
