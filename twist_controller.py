
import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

	#--------------------------------------------------------------------------
	def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

		# create steering PID object
#		self.steer_adj_pid = pid.PID(kp=0.5, ki=0.005, kd=0.25, 
#																			-max_steer_angle, max_steer_angle)	

		# create a yaw controller object
		self.yaw_controller = YawController(wheel_base, steer_ratio, 0.0, 
																					max_lat_accel, max_steer_angle)

		self.timestamp = rospy.get_time()		
		rospy.logwarn("TwistC:__init__ Done!")
		return


	#---------------------------------------------------------------------------
	def control_steer(self, proposed_vel_linear, proposed_vel_angular, 
											current_linear_vel):

		new_timestamp = rospy.get_time()
		duration = new_timestamp - self.timestamp
		self.timestamp = new_timestamp						# save for next time 

#		rospy.logwarn("TC:steer : sugg_vel_linear=%f curr_vel_linear=%f", 
#																		proposed_vel_linear, current_linear_vel)


		yaw_steer = self.yaw_controller.get_steering( proposed_vel_linear, 
																									proposed_vel_angular,
																									current_linear_vel )
#		rospy.logwarn("TC:steer : yaw_steer=%f", yaw_steer)

		return yaw_steer
																									

	#---------------------------------------------------------------------------
	def control_throttle_brake(self, proposed_vel_linear, current_vel_linear, 
																	max_throttle, max_brake):

		vel_diff = proposed_vel_linear - current_vel_linear

#		rospy.logwarn("TC:t_b : sugg_vel_linear=%f curr_vel_linear=%f vel_diff = %f", 
#											proposed_vel_linear, current_vel_linear, vel_diff)

		throttle = 0.0
		brake = 0.0 

		# Let's prevent divide by 0
		if proposed_vel_linear > 0.5:
			if vel_diff > 0.1:
				throttle = vel_diff / proposed_vel_linear
				if throttle > max_throttle:
					throttle = max_throttle


			elif vel_diff < -0.1:
				brake = vel_diff / proposed_vel_linear 
				if brake < max_brake:
					brake = max_brake

#		rospy.logwarn("TC:t_b: throttle = %f --- brake = %f", throttle, brake)
			
		return throttle, brake	

