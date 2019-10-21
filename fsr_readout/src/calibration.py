#!/usr/bin/python
'''
Created on Sun Octoer 12, 2019
@author: 2.009 Purple
This subscribes to the data published by multiple_fsr.py and calibrates for a sepecific user
'''

import rospy
import getch
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

class Calibrate_and_Measure():
	def __init__(self):
		#Name the node
		rospy.init_node('calibration', anonymous=True) 

		self.calibrated = False
		self.first_loop = False
		self.data_collection = []
		self.previous_forces = [0,0,0]

		rospy.loginfo('Need to calibrate first')
		rospy.loginfo('Please step onto the pad and press enter when done')
		#The script will not continue unless the user presses enter
		user = raw_input()

		rospy.Subscriber('forces', forces, self.cb_forceReadout)

	def cb_forceReadout(self, data):
		'''
		Collects the raw data from the multiple_fsr script and puts it into a list
		'''

		#Defining the parts of the list
		forces = []
		forces.append(data.heel)
		forces.append(data.inner)
		forces.append(data.outer)
		print(forces)

		#Check to see if the user was calibrated; if not calibrated, calibrate; if calibrated, be prepared to provide feedback
		if not self.calibrated:
			# print('here')
			self.calibrate(forces)
		else:
			self.feedback()

	def calibrate(self, forces):
		'''
		This function is for calibrating for a specific user. 
		It asks the user to squat and collects the data accordingly.
		Lastly, it measures percentages throughout different points of the squat to see weight distribution.
		'''
		if not self.first_loop:
			rospy.loginfo('Begin squatting and get off pad when done')
			self.first_loop = True

		#Defining list for entire data_collection during calibration
		# rospy.loginfo(forces)
		self.data_collection.append(forces)

		if sum(forces) == 0 and sum(self.previous_forces)!=0:
			rospy.loginfo(self.data_collection)
			rospy.loginfo("Would you like to recalibrate? Enter 'yes' or 'no'.")
			user = raw_input()
			if user == 'yes':
				self.calibrated = False
				self.first_loop = False
				self.data_collection = []
				rospy.loginfo('Please step onto the pad and press enter when done')
				#The script will not continue unless the user presses enter
				user = raw_input()
			else:
				self.calibrated = True
		self.previous_forces = forces

	def feedback(self):
		rospy.loginfo('Feedback time')

if __name__ == '__main__':
    begin = Calibrate_and_Measure()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass