#!/usr/bin/env python
"""
2015-05-20 KJ First version
"""

# imports
import rospy
from sensor_msgs.msg import NavSatFix
# http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
# http://docs.ros.org/api/sensor_msgs/html/msg/NavSatStatus.html
from uas_nordic_2015.msg import drone_pos
from math import pi
from transverse_mercator import tranmerc

# WGS-84 defines
wgs84_a = 6378137.0 # WGS84 semi-major axis of ellipsoid [m]
wgs84_f = 1/298.257223563 # WGS84 flattening of ellipsoid

# UTM defines
utm_false_easting = 500000.0
utm_scale_factor = 0.9996
utm_origin_latitude = 0.0

# UTM zone defines
false_northing = 0.0
central_meridian = 9.0

class ROSNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		self.first_time = True

		# defines
		self.update_rate = 100 # set update frequency [Hz]

		# transverse mercator conversion
		self.deg2rad = pi/180.0
		self.rad2deg = 180.0/pi
		self.tm = tranmerc()
		self.tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, central_meridian*self.deg2rad, utm_false_easting, false_northing, utm_scale_factor)

		# local variables

		# get parameters

		# get topic names
		gps_topic = rospy.get_param("~drone_gps_sub", "/mavros/gps/fix")
		self.drone_utm_topic = rospy.get_param("~drone_pos_pub", "/drone/pos")

		# setup utm topic publisher
		self.drone_pos_pub = rospy.Publisher(self.drone_utm_topic, drone_pos, queue_size=1)
		self.drone_pos = drone_pos()
		self.drone_pos.header.frame_id = 'gps'

		# setup subscription topic callbacks
		rospy.Subscriber(gps_topic, NavSatFix, self.on_gps_topic)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_gps_topic(self, msg):
		self.drone_pos.header.stamp = rospy.get_rostime()
		if msg.status.status == msg.status.STATUS_NO_FIX:
			self.drone_pos.fix = 0
		elif msg.status.status == msg.status.STATUS_FIX:
			self.drone_pos.fix = 1
		elif msg.status.status == msg.status.STATUS_SBAS_FIX or msg.status.status == msg.status.STATUS_GBAS_FIX:
			self.drone_pos.fix = 2
		else:
			self.drone_pos.fix = -1
		self.drone_pos.lat = msg.latitude
		self.drone_pos.lon = msg.longitude
		self.drone_pos.alt = msg.altitude
		(self.drone_pos.e, self.drone_pos.n) = self.tm.geodetic_to_tranmerc (msg.latitude*self.deg2rad, msg.longitude*self.deg2rad)
		self.publish_pos_messages()

	def publish_pos_messages(self):
		self.drone_pos_pub.publish (self.drone_pos)

	def updater(self):
		while not rospy.is_shutdown():

			if self.first_time:
				self.first_time = False

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('drone_pos_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSNode()
    except rospy.ROSInterruptException:
		pass


