#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):

		#----------------------------------------------------------------------
		def __init__(self):
		
			rospy.init_node('tl_detector')

			self.car_pose = None
			self.waypoints = []
			self.camera_image = None
			self.lights = []
			self.light_pose = None

			sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
			self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

			'''
			/vehicle/traffic_lights provides you with the location of the traffic light 
			 in 3D map space and helps you acquire an accurate ground truth data source
			 for the traffic light classifier by sending the current color state of all 
			 traffic lights in the simulator. When testing on the vehicle, the color state
			 will not be available. You'll need to rely on the position of the light and 
			 the camera image to predict it.
			'''
			sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, 
																															self.traffic_cb)

			sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

			config_string = rospy.get_param("/traffic_light_config")
			self.config = yaml.load(config_string)

			self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', 
																												Int32, queue_size=1)

			self.CVbridge = CvBridge()
#			self.light_classifier = TLClassifier(modelpath="./FrozenSyam.pb")
 
			self.listener = tf.TransformListener()

			self.state = TrafficLight.UNKNOWN
			self.last_state = TrafficLight.UNKNOWN
			self.last_wp = -1
			self.state_count = 0

			self.light_to_wp_map = []
			self.stop_wp = None
			self.light_state = TrafficLight.UNKNOWN

			rospy.logwarn("TLDetector_Init done!")
			rospy.spin()



		#-------------------------------------------------------------------
		# callback for subscribing to /current_pose msg by sim/car
		#-------------------------------------------------------------------
		def pose_cb(self, msg):

#			if self.car_pose != None :
#				if ( int(self.car_pose.pose.position.x) != int(msg.pose.position.x) ) \
#															or (int(self.car_pose.pose.position.y) != int(msg.pose.position.y) ):
#				
#					rospy.logwarn("tl_detector: Got car_pose = %d:%d", msg.pose.position.x, msg.pose.position.y)
		
			self.car_pose = msg		
		
	

		#-------------------------------------------------------------------
		# callback for subscribing to /base_waypoints msg Waypoint loader 
		#-------------------------------------------------------------------
		def waypoints_cb(self, waypoints):

			self.waypoints = waypoints.waypoints
			rospy.logwarn("tl_detector: Got wp!")
			rospy.logwarn("TL_D: # of waypoints = %d", len(self.waypoints) )

			self.sub2.unregister()



		#---------------------------------------------------------------------------
		# callback for subscribing to /vehicle/traffic_lights msg by light_publisher?
		#---------------------------------------------------------------------------
		def traffic_cb(self, msg):

#			rospy.logwarn("tl_detector: Got %d traffic_lights!", len(msg.lights))

			# Save reported lights
			self.lights = msg.lights

#			for i in range (len(self.lights)):
#				rospy.logwarn("light[%d] @ %d:%d ", i, 
#																						self.lights[i].pose.pose.position.x, 
#																						self.lights[i].pose.pose.position.y)

			# why not do it every time after we have valid WPs???
			if self.waypoints != [] and self.light_to_wp_map == []:

#				rospy.logwarn("TL_D:traffic_cb: have wps...")
		
				for i in range (len(self.lights)):
					wp_ix = self.get_closest_waypoint(self.lights[i].pose.pose)	
					self.light_to_wp_map.append(wp_ix)
#					rospy.logwarn("TlD: traffic_cb - adding %d to light_to_wp_map", wp_ix)

					#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#					rospy.logwarn("state of light %d is : %d", i, self.lights[i].state)
					#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#				rospy.logwarn("LT----------------------------------------->WP mapping done!")	
				
				# debug let's print out the wp_ix captured in light_to_wp_map
#				for i in range (len(self.light_to_wp_map)):
#					rospy.logwarn("self.light_to_wp_map[%d] = %d", i, self.light_to_wp_map[i])
			

			

		#--------------------------------------------------------------------
		# Identifies red lights in the incoming camera image and publishes the index
		#	of the waypoint closest to the red light's stop line to /traffic_waypoint
		#
		#	Args:
		#			msg (Image): image from car-mounted camera
		# callback for subscribing to /image_color msg from Car/simulator
		#--------------------------------------------------------------------
		def image_cb(self, msg):

			rospy.logwarn("tl_detector: Got an image!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
	
			self.has_image = True
			self.camera_image = msg

			#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			stop_wp, state = self.process_traffic_lights()
			#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

			# no need to publish for the same stoplight... could CHECK LFOR SAME LIGHT STATE!!
#			if self.stop_wp == stop_wp and self.light_state == state:		
#			if self.stop_wp == stop_wp :		
#				rospy.logwarn("Same stop_wp and same light state therefore will not re-publish!!!!")
#				return

			self.stop_wp = stop_wp
			self.light_state = state

			if state == TrafficLight.RED :
				stop_wp *= -1			# our way of indicating RED to wp_updater
		
			#^^^^^ Publish Red Light to prepare to stop  ^^^^^^^^^^^^^
			self.upcoming_red_light_pub.publish(Int32(stop_wp))
			# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	




		#----------------------------------------------------------------------
		# 
		# Finds closest visible traffic light, if one exists, and determines its
		#		location and color
		#
		#	Returns:
		#		int: index of waypoint closes to the upcoming stop line for a traffic light
		#						 (-1 if none exists)
		#		int: ID of traffic light color (specified in styx_msgs/TrafficLight)
		#
		# called from image_cb 
		# 
		#----------------------------------------------------------------------
		def process_traffic_lights(self):

	
			if self.car_pose is None:
				return -1, TrafficLight.UNKNOWN 
				
			car_closest_wpix = self.get_closest_waypoint(self.car_pose.pose)
			if car_closest_wpix == -1 :
				rospy.logerror("BAD RESULT! can't determine closest WP!")
				return -1, TrafficLight.UNKNOWN 
	
#			rospy.logwarn("process_traffic_light: closest wp index to car is %d", 
#													car_closest_wpix)

			for i in range (len(self.light_to_wp_map)):
				tl_closest_wpix = self.light_to_wp_map[i]			
#				rospy.logwarn("process_traffic_lights : self.light_to_wp_map[i] = %d", 
#													self.light_to_wp_map[i])
				if car_closest_wpix <= tl_closest_wpix :
					break
			
#			rospy.logwarn("Process_traffic_light: Closest TL to car is %d ", 
#													tl_closest_wpix)
						
			
			# List of positions that correspond line to stop in front of an intersection
			stop_line_positions = self.config['stop_line_positions']

			stop_pos = stop_line_positions[i]

			stop_pose = self.create_pose(stop_pos[0], stop_pos[1], 0)

			stop_line_wpix = self.get_closest_waypoint (stop_pose.pose)
#			rospy.logwarn("stop_line_wpix = %d", stop_line_wpix)

			#***********************************
			stop_line_wpix = tl_closest_wpix
			
			#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			state = self.get_light_state(self.lights[i])
			#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

			return stop_line_wpix, state



		#----------------------------------------------------------------------------
		# called from process_traffic_lights
		# Determines the current color of the traffic light
    # 	Args:
    #    	light (TrafficLight): light to classify	
		#
		#		Returns:
		#			int: ID of traffic light color (specified in styx_msgs/TrafficLight)
		#----------------------------------------------------------------------------
		def get_light_state(self, light):

			# -----------------------------------
			# When simulating w/o classification
			#------------------------------------
			# TrafficLight.YELLOW; TrafficLight.RED; TrafficLight.GREEN; TrafficLight.UNKNOWN

#			rospy.logwarn("GET_LIGHT_STATE:::::::::::::::::::::: TRAFFIC LIGHT STATE %d", light.state)
			return light.state 
		
	
 			# -----------------------------------
			# when we do classification ....  
			#------------------------------------      
			if(not self.has_image):
				self.prev_light_loc = None
				return TrafficLight.UNKNOWN

			cv_image = self.Cvbridge.imgmsg_to_cv2(self.camera_image, "bgr8")

			#^^^^^^^^^^^Get classification of light color^^^^^^^^^^^^
			return self.light_classifier.get_classification(cv_image)
			#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



		#-----------------------------------------------------------------------------
		#
		# Identifies the closest path waypoint to the given position
		#	https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
		#	Args:
		#			pose (Pose): position to match a waypoint to
		#
		#	Returns:
		#			int: index of the closest waypoint in self.waypoints
		#called from process_traffic_lights
		#-----------------------------------------------------------------------------
		def get_closest_waypoint(self, light_pose):
	
			# initial variables
			closest_distance = 100000.0
			closest_point = 0
			closest_wp_index = -1;

			pose_x = light_pose.position.x
			pose_y = light_pose.position.y

			for i in range(len(self.waypoints)):
				wp_x = self.waypoints[i].pose.pose.position.x
				wp_y = self.waypoints[i].pose.pose.position.y

				# distance between car and waypoint[i]
				distance = math.sqrt((wp_x - pose_x) ** 2 + (wp_y - pose_y) ** 2)
		
				# take the shorter distance 
				if distance < closest_distance:
					closest_distance = distance
					closest_wp_index = i

			return closest_wp_index

		#---------------------------------------------------------------------------
		# Create a PoseStamped message and init with data provided
		#---------------------------------------------------------------------------
		def create_pose(self, x, y, z, yaw=0.):

			pose = PoseStamped()

			pose.header = Header()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = '/world'

			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = z

			q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
			pose.pose.orientation = Quaternion(*q)

#			rospy.logdebug("pose.x=%f; pose_y=%f; pose_z=%f", x, y, z)

			return pose



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
