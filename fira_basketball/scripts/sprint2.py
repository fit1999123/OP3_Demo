#! /usr/bin/env python

import rospy
from op3_ros_utils import *
from vision import *
import cv2


robot = Robot()

def init():
    
    robot.setGeneralControlModule("action_module")

   
    rospy.sleep(3.0)
    

rospy.init_node('camera_read', anonymous=False)
      

init()



class camera_1:
    
  def __init__(self):
    self.image_sub = rospy.Subscriber("/robotis_op3/camera/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image

    resized_image = cv2.resize(image, (360, 640)) 

    cv2.imshow("Camera output normal", image)
    # cv2.imshow("Camera output resized", resized_image)
    cv2.waitKey(1)


camera_1()


while not rospy.is_shutdown():
    
    

  
    robot.setGeneralControlModule("walking_module")

    rospy.sleep(5.0)


    robot.walkVelocities(x=32, th=0.0,z_move_amplitude=0.045,balance=True, hip_pitch=6) #36

    robot.walkStart()
    rospy.sleep(5.0)
    robot.walkStop()
    
    robot.setGeneralControlModule("none")

    break