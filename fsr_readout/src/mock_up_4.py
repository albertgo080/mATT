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
		self.count = 0
		self.max_count = 100

		#Left sensors and their placements
		self.inner_add = 0
		self.outer_add = 0
		self.heel_add  = 0

		#Right sensors and their placements
		self.inner_addR = 0
		self.outer_addR = 0
		self.heel_addR  = 0

		#Placement calibration parameters for left foot//Each calibration will be depenedent on the user
		self.inner_percentage_calibrated = 0
		self.heel_percentage_calibrated  = 0
		self.outer_percentage_calibrated = 0

		#Placement calibration parameters for left foot//Each calibration will be depenedent on the user
		self.inner_percentage_calibratedR = 0
		self.heel_percentage_calibratedR  = 0
		self.outer_percentage_calibratedR = 0

		#Determines when either the calibrate button or begin buttonw was pressed on the app
		self.calibrate_button_pressed = False
		self.begin_pressed = False

		#Publishing topic to give instructions
		self.instructions_topic = rospy.Publisher('/instructions', String, queue_size=1)

		#Publishing topic to give feedback
		self.feedback_topic = rospy.Publisher('/feedback', String, queue_size=1)

		rospy.loginfo('Need to calibrate first')
		rospy.loginfo('Please step onto the pad and press calibrate when done')
		
		#The script will not continue unless the user presses enter

		#Necessary subscriber topics; listens for when buttons were pressed or for force readouts
		rospy.Subscriber('forces', forces, self.cb_forceReadout)
		rospy.Subscriber('calibration', String, self.cb_calibration)
		rospy.Subscriber('begin', String, self.cb_begin)

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
		if self.calibrate_button_pressed:
			if not self.calibrated:
				# print('here')
				self.calibrate(forces)
			else:
				self.feedback(forces)
		else:
			self.instructions_topic.publish("Step onto mat and stand straight with equal weight distribution; then press calibrate.")

	def cb_begin(self, data):
		'''
		Waits and listens until Begin button is pressed on the app
		'''

		if data.data == 'yes':
			self.begin_pressed = True

	def cb_calibration(self, data):
		if data.data == 'yes':
			self.calibrate_button_pressed = True

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
				
				self.instructions_topic.publish('Please wait, calibrating...')
			else:
				rospy.loginfo('Not balanced while calibrating')
				self.instructions_topic.publish('Not balanced while calibrating')
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
			self.instructions_topic.publish("To begin squatting, press Begin")
			
			if self.begin_pressed:
				self.calibrated=True
			else:
				pass


	def feedback(self, forces):
		self.instructions_topic.publish("Begin squatting!")
		is_good=True
		inner_thresh=.1 #percentage difference
		heel_thresh = 0.08
		heelR_thresh = 0.08
		distribution_thresh=.3 #percetange of total weight 
		total = float(forces[0]+forces[1]+forces[2])
		totalR= float(forces[3]+forces[4]+forces[5])
		#rospy.loginfo(forces)
		#rospy.loginfo(total)
		inner_percentage = forces[1]/total
		inner_percentageR=forces[4]/totalR

		heel_percentage = forces[0]/total
		heelR_percentage = forces[3]/totalR
		total_total=total+totalR
		feed=""

		if heel_percentage < self.heel_percentage_calibrated-heel_thresh and heelR_percentage < self.heel_percentage_calibratedR-heelR_thresh:
			is_good = False
			self.feedback_topic.publish("LEANING TOO FAR FORWARD")
		#right or left inner ball
		elif inner_percentage > self.inner_percentage_calibrated+inner_thresh and inner_percentageR > self.inner_percentage_calibrated+inner_thresh:
			is_good=False
			# feed+="PRONATING LEFT "
			self.feedback_topic.publish("PRONATING BOTH KNEES")
		elif inner_percentage > self.inner_percentage_calibrated+inner_thresh:
			is_good=False
			# feed+="PRONATING LEFT "
			self.feedback_topic.publish("PRONATING LEFT")
		elif inner_percentageR > self.inner_percentage_calibrated+inner_thresh:
			is_good=False
			# feed+="PRONATING RIGHT "
			self.feedback_topic.publish("PRONATING RIGHT")

		#weight distribution across feet
		elif abs(total-totalR)>(distribution_thresh)*total_total:
			is_good=False
			leg=False

			if totalR>total:
				leg="RIGHT"
			else:
				leg="LEFT"
			# feed+="OFF BALANCE! "+leg+ "LEG TOO MUCH! "
			self.feedback_topic.publish("ON YOUR "+leg+ "LEG TOO MUCH!")

		if is_good:
			rospy.loginfo('GOOD')
			self.feedback_topic.publish("GOOD")
		else:
			rospy.loginfo('BAD')
			# self.feedback_topic.publish(feed)
		
if __name__ == '__main__':
    begin = Calibrate_and_Measure()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass