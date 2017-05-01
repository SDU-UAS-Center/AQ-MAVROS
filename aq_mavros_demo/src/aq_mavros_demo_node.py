#!/usr/bin/env python
"""
2015-05-22 KJ First version
"""

# imports
import rospy
from math import pi, sqrt
from std_msgs.msg import Bool, Char, UInt8
from sensor_msgs.msg import NavSatFix, BatteryState
# http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
# http://docs.ros.org/api/sensor_msgs/html/msg/NavSatStatus.html
from aq_mavros.msg import drone_waypoint, IntStamped, UInt32Stamped
from aq_mavros_demo.msg import drone_pos
from transverse_mercator import tranmerc
from waypoint_files import qgc_wpl_120

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

# defines from mavlink package autoquad.h
MAV_CMD_NAV_WAYPOINT=16
MAV_CMD_NAV_LAND=21
MAV_CMD_NAV_TAKEOFF=22

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
		self.count = 0

		# defines
		self.update_rate = 100 # set update frequency [Hz]

		# parameters (should probably be in the launch file)
		self.timeout = 1.5 # [s]
		self.abs_alt_min = 33
		self.abs_alt_default = 38.0
		self.abs_alt_max = 55.0
		self.alt_step = 10
		self.alt = self.abs_alt_default

		# transverse mercator conversion
		self.deg2rad = pi/180.0
		self.rad2deg = 180.0/pi
		self.tm = tranmerc()
		self.tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, central_meridian*self.deg2rad, utm_false_easting, false_northing, utm_scale_factor)

		# keyboard interface
		self.KEY_ESC = 27
		self.KEY_SECOND = 91
		self.KEY_SPACE = 32
		self.KEY_h = 104
		self.KEY_k = 107
		self.KEY_l = 108
		self.KEY_r = 114
		self.KEY_t = 116
		self.KEY_u = 117
		self.KEY_ARROW_UP = 65
		self.KEY_ARROW_DOWN = 66
		self.KEY_ARROW_RIGHT = 67
		self.KEY_ARROW_LEFT = 68
		self.esc_key = False
		self.second_key = False

		# local variables
		self.next_status = 0.0
		self.voltage = 0.0
		self.drone_last_heard = 0.0
		self.drone_timeout = True
		self.dist_last_update = 0.0
		self.takeoff_lat = 0.0
		self.takeoff_lon = 0.0
		self.landing = False
		self.wpt_reached_last_status = 0.0
		self.target_within_threshold_last_update = 0.0

		# load waypoint list
		wpt_import = qgc_wpl_120()
		self.w = wpt_import.load('waypoints.txt')
		self.w_len = len(self.w)
		self.w_current = 1
		#for i in xrange(len(self.w)):
		#	print self.w[i]

		# get parameters

		# get topic names
		kbd_topic = rospy.get_param("~keyboard_sub", "/fmHMI/keyboard")
		gps_topic = rospy.get_param("~drone_gps_sub", "/mavros/global_position/raw/fix")
		battery_topic = rospy.get_param("~drone_battery_sub", "/mavros/battery")
		drone_utm_topic = rospy.get_param("~drone_pos_pub", "/drone/pos")
		wpt_topic = rospy.get_param("~drone_waypoint_pub", "/drone/waypoint")
		wpt_ack_topic = rospy.get_param("~drone_waypoint_ack_sub", "/drone/waypoint_ack")
		current_wpt_topic = rospy.get_param("~drone_set_current_waypoint_pub", "/drone/set_current_waypoint")
		wpt_reached_topic =rospy.get_param("~drone_waypoint_reached_sub", "/drone/waypoint_reached")
		wpt_clear_topic = rospy.get_param("~drone_clear_waypoints_pub",'/drone/clear_waypoints')

		# setup waypoint topic publisher
		self.drone_wpt_pub = rospy.Publisher(wpt_topic, drone_waypoint, queue_size=1)
		self.drone_wpt = drone_waypoint()
		self.drone_wpt_seq = 1

		self.wpt_queue = []
		# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
		self.wpt_ack_received = True
		self.wpt_retry = 0
		self.wpt_retry_max = 3
		self.wpt_timeout = 0.0
		self.wpt_timeout_define = 0.5 # [s]

		# setup current waypoint topic publisher
		self.drone_set_current_wpt_pub = rospy.Publisher(current_wpt_topic, IntStamped, queue_size=1)
		self.drone_current_wpt = IntStamped()

		# setup utm topic publisher
		self.drone_pos_pub = rospy.Publisher(drone_utm_topic, drone_pos, queue_size=1)
		self.drone_pos_timeout = True
		self.drone_pos_last_heard = 0.0
		self.drone_pos = drone_pos()
		self.drone_pos.header.frame_id = 'gps'

		# setup wpt clear topic publisher
		self.wpt_clear_pub = rospy.Publisher(wpt_clear_topic, Bool, queue_size=1)
		self.wpt_clear = Bool()

		# setup subscription topic callbacks
		rospy.Subscriber(kbd_topic, Char, self.on_kbd_topic)
		rospy.Subscriber(gps_topic, NavSatFix, self.on_gps_topic)

		rospy.Subscriber(battery_topic, BatteryState, self.on_battery_topic)
		rospy.Subscriber(wpt_ack_topic, UInt8, self.on_wpt_ack_topic)
		rospy.Subscriber(wpt_reached_topic, UInt32Stamped, self.on_wpt_reached_topic)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_kbd_topic(self, msg):
		if self.esc_key == False:
			if msg.data == self.KEY_ESC: # the following key will be a secondary key
				self.esc_key = True
			elif msg.data == self.KEY_SPACE:
				pass
			elif msg.data == self.KEY_h:
				rospy.loginfo(rospy.get_name() + ": Going home")
				self.landing = True
				# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
				radius = self.w[0][6]
				loiter = self.w[0][7]
				hori_vmax = self.w[0][8]
				yaw = self.w[0][9]
				self.drone_wpt_seq = self.w_len + 1
				self.queue_add_wpt(self.takeoff_lat, self.takeoff_lon, self.alt, radius, loiter, hori_vmax, yaw)
				self.w_current = self.w_len + 1
				self.publish_set_current_wpt_message (self.w_current)

			elif msg.data == self.KEY_k:
				rospy.loginfo(rospy.get_name() + ": Clear waypoint list")
				self.publish_wpt_clear_message()
				self.w_current = 0
			elif msg.data == self.KEY_l:
				rospy.loginfo(rospy.get_name() + ": Land")
 				#self.publish_wpt_clear_message()
				self.landing = True
				# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
				#radius = self.w[0][6]
				#loiter = self.w[0][7]
				#hori_vmax = self.w[0][8]
				#yaw = self.w[0][9]
				#self.drone_wpt_seq = 0
				# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
				#print self.takeoff_lat, self.takeoff_lon
				#self.queue_add_wpt(self.takeoff_lat, self.takeoff_lon, self.alt, radius, loiter, hori_vmax, yaw)
				alt = -1
				vert_vmax = 1.0
				hori_vmax = 1.0
				poi_alt = 0.0
				self.drone_wpt_seq = self.w_len + 2
				self.wpt_queue.append([self.drone_wpt_seq, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_LAND, self.takeoff_lat, self.takeoff_lon, alt, 0, vert_vmax, hori_vmax, poi_alt])
				self.w_current = self.w_len + 2
				self.publish_set_current_wpt_message (self.w_current)

			elif msg.data == self.KEY_r:
				rospy.loginfo(rospy.get_name() + ": Reset sequence")
				self.w_current = 1
				self.publish_set_current_wpt_message (self.w_current)
			elif msg.data == self.KEY_t:
				rospy.loginfo(rospy.get_name() + ": Takeoff")
				# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
				self.takeoff_lat = self.drone_pos.lat 
				self.takeoff_lon = self.drone_pos.lon
				lat = self.w[-1][3]
				lat = 0.0
				lon = self.w[-1][4]
				lon = 0.0
				alt = self.w[-1][5]
				radius = self.w[-1][6]
				loiter = self.w[-1][7]
				yaw = self.w[-1][9]
				vert_vmax = 1.0
				self.drone_wpt_seq = 0
				# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
				self.wpt_queue.append([self.drone_wpt_seq, MAV_FRAME_GLOBAL, MAV_CMD_NAV_TAKEOFF, lat, lon, alt, radius, loiter, yaw, vert_vmax])
				self.w_current = 0
				self.publish_set_current_wpt_message (self.w_current)

			elif msg.data == self.KEY_u:
				rospy.loginfo(rospy.get_name() + ": Upload waypoint list")
				self.drone_wpt_seq = 1
				for i in xrange(len(self.w)):
					if self.w[i][2] == MAV_CMD_NAV_WAYPOINT:
						self.queue_add_wpt(self.w[i][3], self.w[i][4], self.alt, self.w[i][6], self.w[i][7], self.w[i][8], self.w[i][9])

		else: # this key is a secondary key
			if msg.data == self.KEY_SECOND: # the following key will be a secondary key
				self.second_key = True
			elif self.second_key == True:
				self.esc_key = False
				self.second_key = False
				if msg.data == self.KEY_ARROW_UP:
					self.alt += self.alt_step
					if self.alt > self.abs_alt_max:
						self.alt = self.abs_alt_max
					self.wpt_update_alt()

				elif msg.data == self.KEY_ARROW_DOWN:
					self.alt -= self.alt_step
					if self.alt < self.abs_alt_min:
						self.alt = self.abs_alt_min
					self.wpt_update_alt()
				elif msg.data == self.KEY_ARROW_LEFT:
					pass
				elif msg.data == self.KEY_ARROW_RIGHT:
					pass


	def on_gps_topic(self, msg):
		self.drone_last_heard = rospy.get_time()
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
		if msg.latitude != 0 or msg.longitude != 0:
			(self.drone_pos.e, self.drone_pos.n) = self.tm.geodetic_to_tranmerc (msg.latitude*self.deg2rad, msg.longitude*self.deg2rad)
		else:
			self.drone_pos.e = 0.0
			self.drone_pos.n = 0.0
		self.publish_pos_messages()
		if self.drone_pos.fix > 0:
			self.drone_pos_last_heard = rospy.get_time()

	def on_wpt_reached_topic(self, msg):
		if True: # rospy.get_time() >= self.wpt_reached_last_status + 1.0:
			self.wpt_reached_last_status = rospy.get_time()
			rospy.loginfo (rospy.get_name() + ': Waypoint reached: %d counted (%d reported) lat %.7f lon %.7f' % (self.w_current, msg.data, self.drone_pos.lat, self.drone_pos.lon))

			#self.w_current += 1
			self.w_current = msg.data

			# reset navigation if end of waypoint list
			# print self.landing, self.w_current, self.w_len
			if 	self.landing == False and self.w_current == self.w_len:
				rospy.loginfo (rospy.get_name() + ': End of waypoint list reached, restarting')
				self.w_current = 0
				self.publish_set_current_wpt_message (self.w_current)

	def on_wpt_ack_topic(self, msg):
		if msg.data == 0:
			self.wpt_ack_received = True
			self.wpt_retry = 0
			if len(self.wpt_queue) > 0:
				self.wpt_queue.pop(0)
				if len(self.wpt_queue) == 0:
					rospy.loginfo (rospy.get_name() + ': All waypoints sent succesfully')
		else:
			rospy.logerr (rospy.get_name() + ': Waypoint NAK received')

	def on_battery_topic(self, msg):
		self.voltage = msg.voltage

	def publish_set_current_wpt_message(self, seq):
		self.drone_current_wpt.header.stamp =  rospy.get_rostime()
		self.drone_current_wpt.data = seq
		self.drone_set_current_wpt_pub.publish(self.drone_current_wpt)

	def publish_wpt_clear_message(self):
		self.wpt_clear = True
		self.wpt_clear_pub.publish (self.wpt_clear)

	def publish_pos_messages(self):
		self.drone_pos_pub.publish (self.drone_pos)


	def wpt_queue_send(self):
		if self.wpt_retry > 0:
			rospy.loginfo(rospy.get_name() + ": Send waypoint seq %d retry %d" % (self.wpt_queue[0][0], self.wpt_retry))
		self.wpt_timeout = rospy.get_time() + self.wpt_timeout_define
		self.drone_wpt.header.stamp = rospy.get_rostime()
		# seq,frame,command,lat,lon,alt,param1,param2,param3,param4
		self.drone_wpt.seq = self.wpt_queue[0][0]
		self.drone_wpt.frame = self.wpt_queue[0][1]
		self.drone_wpt.current = False
		self.drone_wpt.autocontinue = True
		self.drone_wpt.command = self.wpt_queue[0][2]
		self.drone_wpt.lat = self.wpt_queue[0][3]
		self.drone_wpt.lon = self.wpt_queue[0][4]
		self.drone_wpt.alt = self.wpt_queue[0][5]
		self.drone_wpt.param1 =  self.wpt_queue[0][6]
		self.drone_wpt.param2 =  self.wpt_queue[0][7]
		self.drone_wpt.param3 =  self.wpt_queue[0][8]
		self.drone_wpt.param4 =  self.wpt_queue[0][9]
		self.wpt_ack_received = False
		self.drone_wpt_pub.publish(self.drone_wpt)

	def queue_add_wpt(self, lat, lon, alt, radius, loiter, hori_vmax, yaw):
		# lat,lon,alt,param1,param2,param3,param4
		self.wpt_queue.append([self.drone_wpt_seq, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, lat, lon, alt, radius, loiter, hori_vmax, yaw])
		self.drone_wpt_seq += 1

	def wpt_queue_update(self):
		if self.wpt_ack_received == False: # if we are waiting for an ACK
			if rospy.get_time() >= self.wpt_timeout: # if the send failed	
				if self.wpt_retry > self.wpt_retry_max:
					rospy.logerr(rospy.get_name() + ": Waypoint send maximum retry exceeded")
					self.wpt_retry = 0
				else:				
					self.wpt_retry += 1
				self.wpt_queue_send()
		elif len(self.wpt_queue) > 0: # if there are more waypoints to send
			self.wpt_queue_send()

	def wpt_update_alt (self):
		rospy.loginfo (rospy.get_name() + ': New altitude %.0fm' % self.alt)
		self.drone_wpt_seq = 1
		for i in xrange(len(self.w)):
			if self.w[i][2] == MAV_CMD_NAV_WAYPOINT:
				self.queue_add_wpt(self.w[i][3], self.w[i][4], self.alt, self.w[i][6], self.w[i][7], self.w[i][8], self.w[i][9])
		self.publish_set_current_wpt_message (self.w_current)

	def updater(self):
		while not rospy.is_shutdown():
			if self.first_time:
				self.first_time = False
			self.count += 1
			
			self.wpt_queue_update()

			if self.count % 10 == 0:

				# drone timeout
				if self.drone_last_heard + self.timeout <= rospy.get_time():
					self.drone_timeout = True
				else:
					self.drone_timeout = False

				# drone position timeout
				if self.drone_pos_last_heard + self.timeout <= rospy.get_time():
					self.drone_pos_timeout = True
				else:
					self.drone_pos_timeout = False


				if self.next_status < rospy.get_time():
					self.next_status = rospy.get_time() + 2.0
					s = ''
					if self.drone_timeout == True:
						s = s + 'DRONE_LINK_ERR'
					else:
						if self.drone_pos_timeout == True:
							s = s + 'FIX_ERR'
						else:
							s = s + 'Drone alt: %.0fm ' % (self.drone_pos.alt)
						s += ', battery: %.1f Volt' % (self.voltage)
					rospy.logwarn (rospy.get_name() + ': ' +  s) 
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


