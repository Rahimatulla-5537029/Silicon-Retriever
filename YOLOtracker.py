#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

def image_callback(msg):
	cv_image = bridge.imgmsg_to_cv2(msg)
	np_image = np.asarray(cv_image)
	np_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)
	nonzeros = np.count_nonzero(np_image)
	
	if nonzeros > 0:
	
		results = model.track(np_image, persist=True)
		boxes = results[0].boxes.xywh.cpu()
		
		if results[0].boxes.id != None and results[0].boxes.cls != None:
		
			track_ids = results[0].boxes.id.int().cpu().tolist()
			cls = results[0].boxes.cls.int().cpu()
			
			cmd = Twist()
			cmd.angular.z = 0
			
			for box, cl, track_id in zip(boxes, cls, track_ids):
			
				x,y,w,h = box
				
				if cl == 39:
				
					cmd.angular.z = -(x - 320)/640

			pub2.publish(cmd)
			
		annotated_frame = results[0].plot()
		cv2.imshow("YOLOv8 Tracking", annotated_frame)
		cv2.waitKey(1)
	
if __name__ == '__main__':
	bridge = CvBridge()
	model = YOLO("/home/kyrian/ros_ws/src/tutorial_pkg/src/yolov8n.onnx")
	rospy.init_node("YOLOtracker")

	sub = rospy.Subscriber("/image", Image, callback=image_callback)
	pub1 = rospy.Publisher("/image_re", Image, queue_size=10)
	pub2 = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

	rospy.loginfo("YOLOtracker activated.")

	rospy.spin()
