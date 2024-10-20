#!/usr/bin/env python3
import serial
import time
import string
import pynmea2
import rospy
from sensor_msgs.msg import NavSatFix

class main:
    
	def __init__(self):
		rospy.init_node('gps_feedback_node')
		port = "/dev/ttyAMA0"
		self.ser = serial.Serial(port, baudrate=9600, timeout=0.5)
		self.dataout = pynmea2.NMEAStreamReader()
		self.gps_pub = rospy.Publisher('gps/data', NavSatFix, queue_size=10)
		while not rospy.is_shutdown():
			newdata = self.ser.readline()
			if newdata[0:6] == b"$GNGGA":
				newmsg = pynmea2.parse(newdata.decode())
				lat = newmsg.latitude
				lng = newmsg.longitude
				alt=newmsg.altitude
				gps = "Latitude = " + str(lat) + " and Longitude = " + str(lng)
				print(gps)
				gps_msg.latitude=float(lat)
				gps_msg.longitude=float(lng)
				gps_msg.altitude=float(alt)
				self.gps_subscriber.publish(gps_msg)

if __name__ == "__main__":
	gps_msg = NavSatFix()
	main()
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)