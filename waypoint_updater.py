#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node publishes waypoints from car's current position to some `x` distance ahead.

Once you have created dbw_node, update this node to use the status of traffic 
lights too.

Please note that our simulator also provides the exact location of traffic lights and 
their current status in `/vehicle/traffic_lights` message. You can use this message to 
build this node as well as to verify your TL classifier.1
'''

LOOKAHEAD_WPS = 100 				# Number of waypoints we will publish. was 200


class WaypointUpdater(object):

	#-------------------------------------------------------------
	def __init__(self):
		rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
	
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		# TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below
		self.car_pose = None
		self.base_waypoints = []
		self.frame_id = None
		self.light_i = None		# to be used when we get tf callback

		rospy.logwarn ("WaypointUpdater:__init__ done!")

		#rospy.spin()
		self.loop()


	#------------------------------------------------------------
	def loop(self):

		rospy.logdebug ("In WPU loop\n")
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():

			# wait for base_waypoints and pose info then 
			if self.base_waypoints is [] or self.car_pose is None:
				rate.sleep()
				continue
	
#			rospy.logwarn ("In WPU loop: have waypoints and car_pose")
			
			# get the index closest waypoint to car 
			closest_wp_index = get_closest_waypoint(self.base_waypoints, self.car_pose)
			rospy.logwarn("WPULoop - closest_wo_index = %d", closest_wp_index)

			if closest_wp_index == -1 :
				rate.sleep()
				continue

#---------------------------------------------------------------
			waypoints_ahead = []
			for i in range(LOOKAHEAD_WPS):
				ix = (closest_wp_index + i) % len(self.base_waypoints)
#				w.twist.twist.linear.x = 25.0				# set the speed of the wps 
				waypoints_ahead.append(self.base_waypoints[ix])

			# structure the data to match the expected styx_msgs/Lane form
			lane = Lane()
			lane.waypoints = waypoints_ahead  		# list of waypoints ahead of the car
			lane.header.stamp = rospy.Time.now()  # timestamp
			lane.header.frame_id = self.frame_id

			# publish Lane 
			rospy.logdebug ("In WPUloop: about to publish lane")
			self.final_waypoints_pub.publish(lane)

			rate.sleep()



	#=======================================================
	# Subscribed to /current_pose msg published by simulator
	# Simulator calls back with msg 
	##======================================================
	def pose_cb(self, msg):
		# TODO: Implement
		rospy.logdebug ("In WPU:pose_cb\n")

		#------------------------------------------------------------------------
		# the msg will be in PoseStamped format
		# std_msg/Header[seq, timestamp, frameId] 
		# geometry_msgs/Pose pose[geometry_msgs/Point position[x,y,x], 
		#													geometry_msgs/Quaternion orientation[x,y,z,w]]
		#------------------------------------------------------------------------
		self.header = msg.header
		self.car_pose = msg.pose					# this contains position and orientation
		self.frame_id = msg.header.frame_id

#		rospy.logwarn("WPU_cb: car_pose = %d:%d \n", self.car_pose.position.x, 
#																							self.car_pose.position.y)
	

	#================================================================
	# Subscribed to /base_waypoints published by Waypoint_Loader
	# Waypoint_Loader calls with waypoints[] 
	#================================================================
	def waypoints_cb(self, msg_wypts):
		
		# TODO: Implement
		rospy.logdebug("In WPUpdater:waypoints_cb")
		
		#------------------------------------------------------------------------
		# waypoints styx_msgs/Waypoint[] waypoints
  	#		geometry_msgs/PoseStamped pose
	  #	 		std_msgs/Header header
		#    		uint32 seq, time stamp, string frame_id
		#  		geometry_msgs/Pose pose
		#    		geometry_msgs/Point position
		#      		float64 x,y,z
		#    		geometry_msgs/Quaternion orientation
		#      		float64 x,y,z,w
		#		geometry_msgs/TwistStamped twist
		#  		std_msgs/Header header
		#    		uint32 seq, time stamp, string frame_id
		#  		geometry_msgs/Twist twist
		#    		geometry_msgs/Vector3 linear
		#      		float64 x,y,z
		#    		geometry_msgs/Vector3 angular
		#      		float64 x,y,z
		#------------------------------------------------------------------------
		self.base_waypoints = msg_wypts.waypoints

		rospy.logwarn("WPU: wpcb: # of waypoints = %d", len(msg_wypts.waypoints) )

		rospy.logdebug("position = %d:%d:%d \n", 
											self.base_waypoints[0].pose.pose.position.x, 
											self.base_waypoints[0].pose.pose.position.y,	
											self.base_waypoints[0].pose.pose.position.z)

		rospy.logdebug("orientation = %d:%d:%d:%d \n", 
											self.base_waypoints[0].pose.pose.orientation.x, 
											self.base_waypoints[0].pose.pose.orientation.y,	
											self.base_waypoints[0].pose.pose.orientation.z,
											self.base_waypoints[0].pose.pose.orientation.w)

		rospy.logdebug("twist linear = %d:%d:%d \n", 
											self.base_waypoints[0].twist.twist.linear.x, 
											self.base_waypoints[0].twist.twist.linear.y,	
											self.base_waypoints[0].twist.twist.linear.z)

		rospy.logdebug("twist angular = %d:%d:%d \n", 
											self.base_waypoints[0].twist.twist.angular.x, 
											self.base_waypoints[0].twist.twist.angular.y,	
											self.base_waypoints[0].twist.twist.angular.z)

		# can unsubscribe here
		self.base_wp_sub.unregister()
		 
		return

	#-------------------------------------------------------------
	# Callback for /traffic_waypoint message, once we subscribe
	#-------------------------------------------------------------
	def traffic_cb(self, msg):
		
#		rospy.logdebug("In WPU:traffic_cb")
		rospy.logwarn("In WPU:traffic_cb")

		self.light_i = msg.data
		

	#--------------------------------------------------------------
	# Callback for /obstacle_waypoint message, once we subscribe
	#--------------------------------------------------------------
	def obstacle_cb(self, msg):

		# TODO:  IMplement
#		rospy.logdebug("In WPU:obstacle_cb")
		rospy.logwarn("In WPU:obstacle_cb")



	#--------------------------------------------------------------
	# No caller yet? 
	#--------------------------------------------------------------
	def get_waypoint_velocity(self, waypoint):

		rospy.logwarn("In WPU:get_waypoint_velocity = %f", 
																	waypoint.twist.twist.linear.x)
		return waypoint.twist.twist.linear.x


	#--------------------------------------------------------------
	# No caller yet??
	#--------------------------------------------------------------
	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		rospy.logwarn("In WPUr:set_waypoint_velocity = %f", velocity)
		waypoints[waypoint].twist.twist.linear.x = velocity


	#--------------------------------------------------------------
	# No caller yet?? 
	#--------------------------------------------------------------
	def distance(self, waypoints, wp1, wp2):

		rospy.logdebug("In WPU:distance")

		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, 
								 waypoints[i].pose.pose.position)
			wp1 = i
		return dist


#======================================================================
def get_closest_waypoint(waypoints, pose):

	# initial variables
	closest_distance = 100000.0
	closest_point = 0
	closest_wp_index = -1;

	pose_x = pose.position.x
	pose_y = pose.position.y

#	orientation = msg.pose.orientation
#	euler = tf.transformations.euler_from_quaternion([
#            orientation.x,
#            orientation.y,
#            orientation.z,
#            orientation.w])
#	self.theta = euler[2]

	for i in range(len(waypoints)):
		wp_x = waypoints[i].pose.pose.position.x
		wp_y = waypoints[i].pose.pose.position.y

		# distance between car and waypoint[i]
		distance = math.sqrt((wp_x - pose_x) ** 2 + (wp_y - pose_y) ** 2)
		
		# take the shorter distance 
		if distance < closest_distance:
			closest_distance = distance
			closest_wp_index = i

	return closest_wp_index

#-------------------------------------------------------------
if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')


