#! /usr/bin/env python
'''
Create on Sun October 12, 2019
@author: 2.009 Purple Team
Arduino code to relay data from serial monitor on arduino to python script
'''
import rospy
import serial
from fsr_readout.msg import forces 
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)

#Connecting to serial port

def publish_forces():

	arduino = serial.Serial("/dev/ttyACM1", 9600)
	print('Connected to Arduino')
	#Initializing variables
	pub = rospy.Publisher('forces', forces, queue_size = 10)
	rospy.init_node('one_fsr', anonymous = True)
	rate = rospy.Rate(10)
	go = True

	#Receiving data and print it out
	while not rospy.is_shutdown():
		# print 'here'
		try:
			force_raw = str(arduino.readline())
			force = int(force_raw.split("\r\n")[0])
			# print('Force:', force)
			heel = force
			force_msg = forces()
			force_msg.heel = heel
			pub.publish(force_msg)
			rate.sleep()
		except:
			print("Getting data")

if __name__ == "__main__":
	try:
		publish_forces()
	except rospy.ROSInterruptException:
		pass