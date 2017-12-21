
import rospy

import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from dbw_mkz_msgs.msg import SteeringReport, ThrottleCmd, BrakeCmd, SteeringCmd
from std_msgs.msg import Float32 as Float
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from styx_msgs.msg import TrafficLight, TrafficLightArray
import numpy as np
from PIL import Image as PIL_Image
from io import BytesIO
import base64

import math
import random

TYPE = {
    'bool': Bool,
    'float': Float,
    'pose': PoseStamped,
    'pcl': PointCloud2,
    'twist': TwistStamped,
    'steer': SteeringReport,
    'trafficlights': TrafficLightArray,
    'steer_cmd': SteeringCmd,
    'brake_cmd': BrakeCmd,
    'throttle_cmd': ThrottleCmd,
    'image':Image
}


class Bridge(object):
	

	#------------------------------------------------------
	def __init__(self, conf, server):

		rospy.init_node('styx_server', log_level=rospy.DEBUG)

		self.server = server
		self.vel = 0.
		self.yaw = None
		self.angular_vel = 0.
		self.bridge = CvBridge()

		self.callbacks = { '/vehicle/steering_cmd': self.callback_steering,
          						 '/vehicle/throttle_cmd': self.callback_throttle,
						           '/vehicle/brake_cmd': self.callback_brake,
								     }

		self.subscribers = [rospy.Subscriber(e.topic, TYPE[e.type], 
															self.callbacks[e.topic]) for e in conf.subscribers]

		self.publishers = {e.name: rospy.Publisher(e.topic, TYPE[e.type], queue_size=1)
                           for e in conf.publishers}

		rospy.logwarn("Bridge:__init__Done!")
 
	#--------------------------------------------------------------------------
	# called from publish_traffic
	# creates a TrafficLight message 
	#--------------------------------------------------------------------------
	def create_light(self, x, y, z, yaw, state):

#		rospy.logdebug("In Bridge:create_light")

		light = TrafficLight()
		light.header = Header()
		light.header.stamp = rospy.Time.now()
		light.header.frame_id = '/world'

		light.pose = self.create_pose(x, y, z, yaw)
		light.state = state

		return light

 
	#---------------------------------------------------------------------------
	# Called from publish_odometry & publish_obstacles
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

#		rospy.logdebug("pose.x=%f; pose_y=%f; pose_z=%f", x, y, z)

		return pose


	#---------------------------------------------------------------------------
	# Called from publish_controls 
	# Creates a Float object
	#---------------------------------------------------------------------------
	def create_float(self, val):

#		rospy.logdebug("In Bridge:create_float")
		fl = Float()
		fl.data = val
		return fl





	#---------------------------------------------------------------------------
	# called from publish_odometry 
	# Creates a TwistStamped message and inits with provided lin and ang vels
	#--------------------------------------------------------------------------
	def create_twist(self, velocity, angular):

#		rospy.logdebug("In Bridge:create_twist")

		tw = TwistStamped()
		tw.twist.linear.x = velocity
		tw.twist.angular.z = angular
		return tw


	#---------------------------------------------------------------------------
	# called from publish_controls
	# creates a SteeringREport message and inits with 
	#---------------------------------------------------------------------------
	def create_steer(self, val):

#		rospy.logdebug("In Bridge:create_steer\n")

		st = SteeringReport()
		st.steering_wheel_angle_cmd = val * math.pi/180.
		st.enabled = True
		st.speed = self.vel
		return st


	#---------------------------------------------------------------------------
	# called from publish_odometry
	# calculate yaw 
	#---------------------------------------------------------------------------
	def calc_angular(self, yaw):

#		rospy.logdebug("In Bridge:calc_angular")

		angular_vel = 0.
		if self.yaw is not None:
			angular_vel = (yaw - self.yaw)/(rospy.get_time() - self.prev_time)
		self.yaw = yaw
		self.prev_time = rospy.get_time()
		return angular_vel


	#---------------------------------------------------------------------------
	# called from publish_lidar
	#---------------------------------------------------------------------------
	def create_point_cloud_message(self, pts):

#		rospy.logdebug("In Bridge:create_point_cloud_message")

		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/world'
		cloud_message = pcl2.create_cloud_xyz32(header, pts)
		return cloud_message


	#--------------------------------------------------------------------------
	# called from publish_odometry
	# What does this do?
	#--------------------------------------------------------------------------
	def broadcast_transform(self, name, position, orientation):

#		rospy.logdebug("In Bridge:broadcast_transform\n")

		br = tf.TransformBroadcaster()
		br.sendTransform(position, orientation,rospy.Time.now(), name, "world")

 
	#--------------------------------------------------------------------------
	# called from server.telemetry 
	# publish current_pose and current_velocity
	#--------------------------------------------------------------------------
	def publish_odometry(self, data):

		pose = self.create_pose(data['x'], data['y'], data['z'], data['yaw'])

		position = (data['x'], data['y'], data['z'])
		orientation = tf.transformations.quaternion_from_euler(0, 0, 
																													math.pi * data['yaw']/180.)
		self.broadcast_transform("base_link", position, orientation)

		#.............................................
		self.publishers['current_pose'].publish(pose)
		#.............................................

		self.vel = data['velocity']* 0.44704
		self.angular = self.calc_angular(data['yaw'] * math.pi/180.)
		#..........................................................................
		self.publishers['current_velocity'].publish(self.create_twist(self.vel, 
																																	self.angular))
		#...........................................................................

		#rospy.logdebug("Bridge:PubO incoming - lin_vel = %f, ang_vel = %f\n", 
		#													self.vel, self.angular)


	#-----------------------------------------------------------------------------------
	# called from server.control 
	# publish steering, throttle and brake messages
	#-----------------------------------------------------------------------------------
	def publish_controls(self, data):

		steering, throttle, brake = data['steering_angle'], data['throttle'], \
																data['brake']

		#.......................................................................
		self.publishers['steering_report'].publish(self.create_steer(steering))
		self.publishers['throttle_report'].publish(self.create_float(throttle))
		self.publishers['brake_report'].publish(self.create_float(brake))
		#.......................................................................

		rospy.logdebug("Incoming - steering angle = %s; throttle = %s, brake = %s\n" , 
												steering, throttle, brake) 


	#------------------------------------------------------------------------------------
	# called from server.obstacle 
	# create pose for each obstacle and publish them
	# create cloud and publish.... what's a cloud? 
	#------------------------------------------------------------------------------------
	def publish_obstacles(self, data):

#		rospy.logdebug("In Bridge:publish_obstacles")

		for obs in data['obstacles']:
			pose = self.create_pose(obs[0], obs[1], obs[2])
			self.publishers['obstacle'].publish(pose)

		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/world'
		cloud = pcl2.create_cloud_xyz32(header, data['obstacles'])
		#.................................................
		self.publishers['obstacle_points'].publish(cloud)
		#.................................................


	#-----------------------------------------------------------------------------------
	# called from server.obstacle or lidar
	#-----------------------------------------------------------------------------------
	def publish_lidar(self, data):

#		rospy.logdebug("In Bridge:publish_lidar")

		#...........................................................................
		self.publishers['lidar'].publish(
												self.create_point_cloud_message(zip(data['lidar_x'], 
																			  										 data['lidar_y'], 
																														 data['lidar_z'])))
		#...........................................................................


	#------------------------------------------------------------------------------------
	# Called from server.trafficlights
	#------------------------------------------------------------------------------------
	def publish_traffic(self, data):

#		rospy.logdebug("In Bridge:publish_traffic")
	
		x, y, z = data['light_pos_x'], data['light_pos_y'], data['light_pos_z'],

		yaw = [math.atan2(dy, dx) for dx, dy in zip(data['light_pos_dx'], 
																								data['light_pos_dy'])]
		status = data['light_state']

		lights = TrafficLightArray()
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/world'
		lights.lights = [self.create_light(*e) for e in zip(x, y, z, yaw, status)]
		#................................................
		self.publishers['trafficlights'].publish(lights)
		#................................................
 
	#---------------------------------------------------------------------------
	# called from server.telemetry
	#---------------------------------------------------------------------------
	def publish_dbw_status(self, data):

		rospy.logdebug("In Bridge:publish_dbw_status")
		self.publishers['dbw_status'].publish(Bool(data))


	#---------------------------------------------------------------------------
	# called from server.image
	# --------------------------------------------------------------------------
	def publish_camera(self, data):

		# reduce publishing frequency
		if random.uniform(0, 1) > 0.1 :
#			rospy.logwarn("will not publish image******************************************")
			return

		imgString = data["image"]
		image = PIL_Image.open(BytesIO(base64.b64decode(imgString)))
		image_array = np.asarray(image)

		image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
		#...............................................
		self.publishers['image'].publish(image_message)
		#...............................................
 
	#---------------------------------------------------------------------------------
	# Called from dbw_node publish (steer)
	# send steering data via send in server
	#---------------------------------------------------------------------------------
	def callback_steering(self, data):

		#rospy.logdebug("In Bridge:callback_steering")
		self.server('steer', data={'steering_angle': str(data.steering_wheel_angle_cmd)})

 
	#-------------------------------------------------------------------------------
	# Called from dbw_node publish(throttle)
	# send throttle data via send in server
	#-------------------------------------------------------------------------------
	def callback_throttle(self, data):

		#rospy.logdebug("In Bridge:callback_throttle")
		self.server('throttle', data={'throttle': str(data.pedal_cmd)})


	#-------------------------------------------------------------------------------
	# Called from dbw_node publish(brake) 
	#	send brake data via send in server
	#-------------------------------------------------------------------------------
	def callback_brake(self, data):

		#rospy.logdebug("In Bridge:callback_brake")
		self.server('brake', data={'brake': str(data.pedal_cmd)})


