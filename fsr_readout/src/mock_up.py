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
		self.count=0
		self.max_count=100
		self.inner_add=0
		self.outer_add=0
		self.heel_add=0
		self.inner_percentage_calibrated = 0
		self.heel_percentage_calibrated = 0
		self.outer_percentage_calibrated = 0

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

		#Check to see if the user was calibrated; if not calibrated, calibrate; if calibrated, be prepared to provide feedback
		if not self.calibrated:
			# print('here')
			self.calibrate(forces)
		else:
			self.feedback(forces)

	def calibrate(self, forces):
		'''
		This function is for calibrating for a specific user. 
		It asks the user to squat and collects the data accordingly.
		Lastly, it measures percentages throughout different points of the squat to see weight distribution.
		'''

		#Defining list for entire data_collection during calibration
		# rospy.loginfo(forces)
		if self.count<self.max_count:
			self.inner_add+=forces[1]
			self.heel_add+=forces[0]
			self.outer_add+=forces[2]
			self.count+=1
			rospy.loginfo('Please wait, calibrating...')
		else:
			inner=self.inner_add/self.max_count
			outer=self.outer_add/self.max_count
			heel=self.heel_add/self.max_count
			total=float(inner+outer+heel)
			self.heel_percentage_calibrated = heel/total
			self.inner_percentage_calibrated = inner/total
			self.outer_percentage_calibrated = outer/total
			self.calibrated=True
			rospy.loginfo('Begin squatting')
			rospy.sleep(0.8)

	def feedback(self, forces):
		inner_thresh=.1
		total = float(sum(forces))
		#rospy.loginfo(forces)
		#rospy.loginfo(total)
		inner_percentage = forces[1]/total
		#rospy.loginfo(inner_percentage)

		if inner_percentage > self.inner_percentage_calibrated+inner_thresh:
			rospy.loginfo('BAD')
		else:
			rospy.loginfo('GOOD')

if __name__ == '__main__':
    begin = Calibrate_and_Measure()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass