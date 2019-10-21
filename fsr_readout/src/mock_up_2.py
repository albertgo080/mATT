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

		self.inner_addR=0
		self.outer_addR=0
		self.heel_addR=0

		self.inner_percentage_calibrated = 0
		self.heel_percentage_calibrated = 0
		self.outer_percentage_calibrated = 0

		self.inner_percentage_calibratedR = 0
		self.heel_percentage_calibratedR = 0
		self.outer_percentage_calibratedR = 0

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
		forces.append(data.heelR)
		forces.append(data.innerR)
		forces.append(data.outerR)

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
		calibration_distribution_thresh=.2#percentage off of 50 that the totals can be (.1=10% off of total, so 40 and 60)

		if self.count<self.max_count:
			inner=forces[1]
			heel=forces[0]
			outer=forces[2]
			innerR=forces[4]
			heelR=forces[3]
			outerR=forces[5]

			total=inner+heel+outer
			totalR=innerR+heelR+outerR
			total_both=float(total+totalR)
			diff=total/total_both
			#rospy.loginfo(diff)

			if abs(.5-diff)<calibration_distribution_thresh: #each are within thresh of 50%

				self.inner_add+=forces[1]
				self.heel_add+=forces[0]
				self.outer_add+=forces[2]
				self.inner_addR+=forces[4]
				self.heel_addR+=forces[3]
				self.outer_addR+=forces[5]
				self.count+=1
			else:
				rospy.loginfo('Not balanced while calibrating')
			rospy.loginfo('Please wait, calibrating...')
		else:
			inner=self.inner_add/self.max_count
			outer=self.outer_add/self.max_count
			heel=self.heel_add/self.max_count

			innerR=self.inner_addR/self.max_count
			outerR=self.outer_addR/self.max_count
			heelR=self.heel_addR/self.max_count

			total=float(inner+outer+heel)
			totalR=float(innerR+outerR+heelR)

			self.heel_percentage_calibrated = heel/total
			self.inner_percentage_calibrated = inner/total
			self.outer_percentage_calibrated = outer/total

			self.heel_percentage_calibratedR = heelR/totalR
			self.inner_percentage_calibratedR = innerR/totalR
			self.outer_percentage_calibratedR = outerR/totalR
			rospy.loginfo('Calibrated Data:')
			left=[self.heel_percentage_calibrated,self.inner_percentage_calibrated,self.outer_percentage_calibrated]
			right=[self.heel_percentage_calibratedR,self.inner_percentage_calibratedR,self.outer_percentage_calibratedR]
			rospy.loginfo(left)
			rospy.loginfo(right)
			rospy.loginfo('To begin squatting, press enter')
			
			user = raw_input()
			self.calibrated=True


	def feedback(self, forces):
		is_good=True
		inner_thresh=.1 #percentage difference
		heel_thresh = 0.15
		distribution_thresh=.3 #percetange of total weight 
		total = float(forces[0]+forces[1]+forces[2])
		totalR= float(forces[3]+forces[4]+forces[5])
		#rospy.loginfo(forces)
		#rospy.loginfo(total)
		inner_percentage = forces[1]/total
		inner_percentageR=forces[4]/totalR

		heel_percentage = forces[0]/total
		#rospy.loginfo(inner_percentage)
		total_total=total+totalR

		if heel_percentage < self.heel_percentage_calibrated-heel_thresh:
			is_good = False
			rospy.loginfo('LEANING TOO FAR FORWARD')

		#right or left inner ball
		if inner_percentage > self.inner_percentage_calibrated+inner_thresh:
			is_good=False
			rospy.loginfo("PRONATING LEFT")
		if inner_percentageR > self.inner_percentage_calibrated+inner_thresh:
			is_good=False
			rospy.loginfo("PRONATING RIGHT")

		#weight distribution across feet
		if abs(total-totalR)>(distribution_thresh)*total_total:
			is_good=False
			leg=False

			if totalR>total:
				leg="RIGHT"
			else:
				leg="LEFT"
			rospy.loginfo("OFF BALANCE! YOU ARE ON YOUR "+leg+ "LEG TOO MUCH!")

		if is_good:
			rospy.loginfo('GOOD')
		else:
			rospy.loginfo('BAD')

if __name__ == '__main__':
    begin = Calibrate_and_Measure()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass