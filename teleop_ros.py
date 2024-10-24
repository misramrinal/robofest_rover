#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import cv2
import numpy as np

def main():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('chatter', Int32, queue_size=10)

    image = np.zeros((10, 10, 3), dtype=np.uint8)
    cv2.imshow('Image', image)

    while not rospy.is_shutdown():
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):
            rospy.loginfo("You pressed 'q'. Exiting...")
            break
        else:
            key_str = chr(key) 
            pub.publish(key)
            rospy.loginfo(f"Key pressed: {key}")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
