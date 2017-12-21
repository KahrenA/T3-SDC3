#!/usr/bin/env python

import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

import rospy

sio = socketio.Server()
app = Flask(__name__)
msgs = []

dbw_enable = False

#------------------------------------------------------
@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

#------------------------------------------------------
def send(topic, data):
	s = 1
	msgs.append((topic, data))
	rospy.logdebug("In send. topic = %s; data = %s", topic, data)
	#sio.emit(topic, data=json.dumps(data), skip_sid=True)

#-------------------------------------------------------
bridge = Bridge(conf, send)

#------------------------------------------------------
@sio.on('telemetry')
def telemetry(sid, data):

	global dbw_enable
#	rospy.logdebug("In telemetry: data[dbw_enable] = %d; and dbw_enable flag = %d\n", 
#										data["dbw_enable"], dbw_enable)

	if data["dbw_enable"] != dbw_enable:
		dbw_enable = data["dbw_enable"]
		bridge.publish_dbw_status(dbw_enable)

	bridge.publish_odometry(data)

	for i in range(len(msgs)):
		topic, data = msgs.pop(0)
		sio.emit(topic, data=data, skip_sid=True)
		#rospy.logwarn("In telemetry-send. topic = %s; data = %s", topic, data)


#------------------------------------------------------
@sio.on('control')
def control(sid, data):
	rospy.logdebug("In server:control\n")
	bridge.publish_controls(data)


#------------------------------------------------------
@sio.on('obstacle')
def obstacle(sid, data):
	rospy.logdebug("In server:obstacle\n")
	bridge.publish_obstacles(data)


#------------------------------------------------------
@sio.on('lidar')
def obstacle(sid, data):
	rospy.logdebug("In server:lidar\n")
	bridge.publish_lidar(data)


#------------------------------------------------------
@sio.on('trafficlights')
def trafficlights(sid, data):
	rospy.logdebug("In server:trafficlights\n")
	bridge.publish_traffic(data)


#------------------------------------------------------
@sio.on('image')
def image(sid, data):
	rospy.logdebug("In server:image\n")
	bridge.publish_camera(data)


#------------------------------------------------------
if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
