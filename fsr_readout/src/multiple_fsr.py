#! /usr/bin/env python
'''
Create on Sun October 12, 2019
@author: 2.009 Purple Team
Arduino code to relay data from serial monitor on arduino to python script
'''
import rospy
import serial
from fsr_readout.msg import forces, forcesApp 
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)



def publish_forces():

	rospy.init_node('one_fsr', anonymous = True)
	#Connecting to serial port; if there is an error, the port may have changed
	arduino = serial.Serial("/dev/ttyACM0", 9600)
	rospy.loginfo('Connected to Arduino')

	#Initializing variables
	pub = rospy.Publisher('forces', forces, queue_size = 10)
	pub2 = rospy.Publisher('forcesNew', String, queue_size = 10)
	# rate = rospy.Rate(10)
	go = True
	count = 0
	heel = 0
	inner = 0
	outer = 0
	heelR = 0
	innerR = 0
	outerR = 0

	#Receiving data and print it out
	while go:
		# print 'here'
		# try:
		force_msg = forces()
		forceApp_msg = forcesApp()
		force_raw = str(arduino.readline())
		force = force_raw.split(":")
		if force[0] == 'Heel':
			heel = int(force[1])
		if force[0] == 'Inner':
			inner = int(force[1])
			
		if force[0] == 'Outer':
			outer = int(force[1])

		if force[0] == 'HeelR':
			heelR = int(force[1])
		if force[0] == 'InnerR':
			innerR = int(force[1])
			
		if force[0] == 'OuterR':
			outerR = int(force[1])

		force_msg.heel = heel
		force_msg.inner = inner
		force_msg.outer = outer
		force_msg.heelR = heelR
		force_msg.innerR = innerR
		force_msg.outerR = outerR
		pub.publish(force_msg)

		force_list = [str(heel), str(inner), str(outer), str(heelR), str(innerR), str(outerR)]
		force_string = ','.join(map(str, force_list))
		pub2.publish(force_string)
			# rate.sleep()
		# except:
		# 	rospy.loginfo("Getting data")

if __name__ == "__main__":
	try:
		publish_forces()
	except rospy.ROSInterruptException:
		pass