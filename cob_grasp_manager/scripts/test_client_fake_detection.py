#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_grasp_manager')

import sys

import rospy
import random

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *

from gazebo.srv import *

from numpy import *
from numpy.linalg import inv

if __name__ == "__main__":

	rospy.wait_for_service('detect_object')
	try:
		add_two_ints = rospy.ServiceProxy('detect_object', DetectObjects)
		print "Ready to add two ints."
		req = DetectObjectsRequest()
		req.object_name.data='salt'
		req.roi.x_offset=10;
		req.roi.y_offset=10;
		req.roi.height=10;
		req.roi.width=10;
		req.roi.do_rectify=10;
		resp= add_two_ints(req)
		rospy.loginfo("label %d",len(resp.object_list.detections))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
