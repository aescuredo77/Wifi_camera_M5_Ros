#!/usr/bin/env python3
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

        
def callback(ros_data):
    global image_np
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_v = cv2.flip(image_np, 0)
    
    
if __name__ == '__main__':
    #Inicialize ROS node and receiver module
    rospy.init_node('Image_viewer', anonymous=True)
    subscriber = rospy.Subscriber("/armpap00/Images",CompressedImage, callback)
    image_np = None
    while not rospy.is_shutdown():
        if image_np is not None:
            cv2.imshow("Image reciver", image_np)
            if cv2.waitKey(1) == ord("q"):
                cv2.destroyAllWindows()
                break
            
    
    
