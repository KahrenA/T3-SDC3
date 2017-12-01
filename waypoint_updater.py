#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node publishes waypoints from car's current position to some `x` distance ahead.

Once you have created dbw_node, update this node to use the status of traffic 
lights too.

Please note that our simulator also provides the exact location of traffic lights and 
their current status in `/vehicle/traffic_lights` message. You can use this message to 
build this node as well as to verify your TL classifier.1
'''

LOOKAHEAD_WPS = 200 				# Number of waypoints we will publish. was 200


class WaypointUpdater(object):

	#-------------------------------------------------------------
	def __init__(self):
		rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
		rospy.logdebug ("In WaypointUpdater:__init__\n")

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		# TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below
		self.car_pose = None
		self.base_waypoints = []

		#rospy.spin()
		self.loop()


	#------------------------------------------------------------
	def loop(self):

		rospy.logdebug ("In WPUpdater loop\n")
		rate = rospy.Rate(2)
		while not rospy.is_shutdown():

			rate.sleep()

			# wait for base_waypoints and pose info then 
			if self.base_waypoints is [] or self.car_pose is None:
				continue
	
			rospy.logdebug ("In loop: have waypoints and car_pose\n")
			
			# get the index closest waypoint to car 
			closest_wp_index = get_closest_waypoint(self.base_waypoints, self.car_pose)
			rospy.logdebug("closest_wo_index = %d\n", closest_wp_index)

			# Include 200 WPs starting at index
			waypoints_ahead = []
			for i in range(LOOKAHEAD_WPS):
				if closest_wp_index + i < len(self.base_waypoints):
				  waypoints_ahead.append(self.base_waypoints[closest_wp_index + i])

			# structure the data to match the expected styx_msgs/Lane form
			lane = Lane()
			lane.waypoints = waypoints_ahead  # list of waypoints ahead of the car
			lane.header.stamp = rospy.Time(0)  # timestamp

			# publish Lane 
			rospy.logdebug ("In loop: about to publish lane\n")
			self.final_waypoints_pub.publish(lane)

		return



	#=======================================================
	# Subscribed to /current_pose msg published by simulator
	# Simulator calls back with msg 
	##======================================================
	def pose_cb(self, msg):
		# TODO: Implement
		rospy.logdebug ("In WPUpdater:pose_cb\n")

		#------------------------------------------------------------------------
		# the msg will be in PoseStamped format
		# std_msg/Header[seq, timestamp, frameId] 
		# geometry_msgs/Pose pose[geometry_msgs/Point position[x,y,x], 
		#													geometry_msgs/Quaternion orientation[x,y,z,w]]
		#------------------------------------------------------------------------
		self.header = msg.header
		self.car_pose = msg.pose					# this contains position and orientation

		rospy.logdebug("car_pose = %d:%d \n", self.car_pose.position.x, 
																							self.car_pose.position.y)
																						
		return

	#================================================================
	# Subscribed to /base_waypoints published by Waypoint_Loader
	# Waypoint_Loader calls with waypoints[] 
	#================================================================
	def waypoints_cb(self, msg_wypts):
		# TODO: Implement
		rospy.logdebug("In WPUpdater:waypoints_cb\n")
		
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

		rospy.logdebug("len of waypoints = %d\n", len(msg_wypts.waypoints) )

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

		return

	#-------------------------------------------------------------
	# Callback for /traffic_waypoint message, once we subscribe
	#-------------------------------------------------------------
	def traffic_cb(self, msg):
		# TODO:  Implement
		rospy.logdebug("In WPUpdater:traffic_cb\n")
		pass


	#--------------------------------------------------------------
	# Callback for /obstacle_waypoint message, once we subscribe
	#--------------------------------------------------------------
	def obstacle_cb(self, msg):
		# TODO:  IMplement
		rospy.logdebug("In WPUpdater:obstacle_cb\n")
		pass


	#--------------------------------------------------------------
	# 
	#--------------------------------------------------------------
	def get_waypoint_velocity(self, waypoint):
		rospy.logdebug("In WPUpdater:get_waypoint_velocity\n")
		return waypoint.twist.twist.linear.x


	#--------------------------------------------------------------
	#
	#--------------------------------------------------------------
	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		rospy.logdebug("In WPUpdater:set_waypoint_velocity\n")
		waypoints[waypoint].twist.twist.linear.x = velocity


	#--------------------------------------------------------------
	#
	#--------------------------------------------------------------
	def distance(self, waypoints, wp1, wp2):

		rospy.logdebug("In WPUpdater:distance\n")

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

	# now we (hopefully) have the closest point
#	x = waypoints[closest_wp_index].pose.pose.position.x
#	y = waypoints[closest_wp_index].pose.pose.position.y

#	heading = np.arctan2((y-pose_y), (x-pose_x))
#	angle = np.abs(theta - heading)
#	if angle > np.pi/4.:
#		  closest_wp_index += 1
#		  if closest_wp_index >= len(waypoints):
#		      closest_wp_index = 0

	return closest_wp_index

#-------------------------------------------------------------
if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')


