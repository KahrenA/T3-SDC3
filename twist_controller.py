
import rospy
from yaw_controller import YawController
import pid

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
PREDICTIVE_STEERING = 1.0  # value <0.0 to 1.0>


class Controller(object):

	#--------------------------------------------------------------------------
	def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):


		# create a yaw controller object
		self.yaw_controller = YawController(wheel_base, steer_ratio, 0.0, 
																					max_lat_accel, max_steer_angle)

		# create steering PID object
		self.steering_correction_pid = pid.PID(	kp=0.5, 
																					 	ki=0.004, 
																						kd=0.25, 
																						mn=-max_steer_angle, 
																						mx=max_steer_angle)



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

    # calculate new steering angle
		corrected_steer_angle = self.steering_correction_pid.step(proposed_vel_angular, 
																																					duration)

		steering_angle = corrected_steer_angle + PREDICTIVE_STEERING * yaw_steer

#		rospy.logwarn("TC:control_steer : cor_steer= %f, yaw_steer=%f ==> steer_angle =%f", 
#							corrected_steer_angle, yaw_steer, steering_angle)

		return steering_angle

																									

	#---------------------------------------------------------------------------
	def control_throttle_brake(self, proposed_vel_linear, current_vel_linear, 
																	max_throttle, max_brake):

		vel_diff = proposed_vel_linear - current_vel_linear

#		rospy.logwarn("TC:t_b : sugg_vel_linear=%f curr_vel_linear=%f vel_diff = %f", 
#											proposed_vel_linear, current_vel_linear, vel_diff)

		throttle = 0.0
		brake = 0.0 

		# Let's prevent divide by 0
		if proposed_vel_linear <= 0:
#			rospy.logwarn("TC:ctb-applying max brake!!!")
			brake = max_brake

		elif vel_diff > 0.1:
			throttle = vel_diff / proposed_vel_linear
			if throttle > max_throttle:
				throttle = max_throttle

		elif vel_diff < -0.1:
			brake = vel_diff / proposed_vel_linear 
			if brake < max_brake:
				brake = max_brake

			
		return throttle, brake	

