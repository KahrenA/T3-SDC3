#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `
waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and 
angular velocities.

One thing to keep in mind while building this node and the `twist_controller` 
class is the status of `dbw_enabled`. While in the simulator, its enabled all the 
time, in the real car, that will not be the case. This may cause your PID controller 
to accumulate error because the car could temporarily be driven by a human instead 
of your controller.

We have provided two launch files with this node. Vehicle specific values (like 
vehicle_mass, wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other 
utility classes. You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the 
various publishers that we have created in the `__init__` function.
'''

POINTS_TO_FIT = 10
class DBWNode(object):
    
	#-------------------------------------------------------------
	def __init__(self):
		rospy.init_node('dbw_node', log_level=rospy.DEBUG)

		self.twist_cmd = None
		self.twist_linear_vel = 0.0
		self.twist_angular_vel = 0.0
		self.velocity = None
		self.current_linear_vel = 0.0
		self.current_angular_vel = 0.0
		self.dbw_enabled = False
		self.waypoints = None # final waypoints

		self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
		self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
		self.brake_deadband = rospy.get_param('~brake_deadband', .1)
		self.decel_limit = rospy.get_param('~decel_limit', -5)
		self.accel_limit = rospy.get_param('~accel_limit', 1.)
		self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
		self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
		self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
		self.max_throttle = rospy.get_param('~max_throttle_proportional', 0.8)
		self.max_brake = rospy.get_param('~max_brake_proportional', -0.8)

		rospy.logwarn("DBW: wheel_base=%f, max_steer_ratio=%f, max_throttle=%f, max_brake=%f",
							self.wheel_base, self.steer_ratio, self.max_throttle, self.max_brake)	

		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
				                             					SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
				                                			ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
				                             					BrakeCmd, queue_size=1)

		# TODO: Create `TwistController` object
		self.controller = Controller(self.wheel_base, self.steer_ratio, 
																	self.max_lat_accel, self.max_steer_angle)

		# TODO: Subscribe to all the topics you need to
		rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
		rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

		rospy.logwarn ("In DBWNode:__init__ Done!")
		self.loop()


	#---------------------------------------
	# keep looping 
	#---------------------------------------
	def loop(self):

		rospy.logdebug("In dbw_node:loop\n")
		
		rate = rospy.Rate(50) # 50Hz
		while not rospy.is_shutdown():

			#-------------------------------------------------      
			# TODO:
			# wait for current_vel & dbw_enabled from sim; 
			# and proposed_linear from WPFollower
			#-------------------------------------------------

			if (self.current_linear_vel is 0.0) or (self.twist_linear_vel is 0.0) or (not self.dbw_enabled) :
				continue
						
			steer = self.controller.control_steer( self.twist_linear_vel,
       																				self.twist_angular_vel,
																	              self.current_linear_vel)
	
			throttle, brake = self.controller.control_throttle_brake( self.twist_linear_vel,
       																													self.current_linear_vel, 
																																self.max_throttle, 
																																self.max_brake) 																							
			self.publish(throttle, brake, steer)

#			rospy.logwarn("DWB-loop: Published throttle: %s, brake: %s, steering: %s", throttle, brake, steer)

			rate.sleep()


	#-----------------------------------------------------------------------
	# callback for subscribing to dbw_enabled published msg by the simulator
	#-----------------------------------------------------------------------
	def dbw_enabled_cb(self, msg):

		self.dbw_enabled = bool(msg.data)

		# Enabled self-driving mode will publish throttle, steer and brake mode.
		if self.dbw_enabled:
			rospy.logwarn("**** ============================ ****")
			rospy.logwarn("**** Self-Driving Mode Activated  ****")
			rospy.logwarn("**** ============================ ****")
		else:
			rospy.logwarn("**** ============================= ****")
			rospy.logwarn("**** Manual Driving Mode Activated ****")
			rospy.logwarn("**** ============================= ****")




	#----------------------------------------------------------------------------
	# callback for subscribing to current_velocity msg published by the simulator
	#----------------------------------------------------------------------------
	def current_velocity_cb(self, msg):

		self.velocity = msg.twist
		self.current_linear_vel = msg.twist.linear.x
		self.current_angular_vel = msg.twist.angular.z
#		rospy.logwarn("DBW:current_velocity_cb from simulator = %f", self.current_linear_vel)


	#---------------------------------------------------------------------
	# callback for subscribing to twist_cmd published by Waypoint Follower
	#---------------------------------------------------------------------
	def twist_cmd_cb(self, msg):

		self.twist_linear_vel = msg.twist.linear.x
		self.twist_angular_vel = msg.twist.angular.z

#		if self.twist_linear_vel > 0.5:
#			rospy.logwarn("In dbw:twist_cmd_cb : proposed_linear vel = %f, proposed_angular vel = %f", 
#											self.twist_linear_vel, self.twist_angular_vel)



	#--------------------------------------------------------------------
	# called from loop 
	#--------------------------------------------------------------------
	def publish(self, throttle, brake, steer):

		tcmd = ThrottleCmd()
		tcmd.enable = True
		tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
		tcmd.pedal_cmd = throttle
		self.throttle_pub.publish(tcmd)

		scmd = SteeringCmd()
		scmd.enable = True
		scmd.steering_wheel_angle_cmd = steer
		self.steer_pub.publish(scmd)

		bcmd = BrakeCmd()
		bcmd.enable = True
		bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
		bcmd.pedal_cmd = brake
		self.brake_pub.publish(bcmd)

#------------------------------------------------------------
if __name__ == '__main__':
    DBWNode()


