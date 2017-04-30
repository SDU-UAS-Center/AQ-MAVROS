#!/usr/bin/env python
"""
2015-05-20 KJ First version
"""


class qgc_wpl_120():
	def __init__(self):
		pass

	def load_action_22(self, d): # NAV: TakeOff
		lat = float(d[8])
		lon = float(d[9])
		alt = float(d[10])
		radius = float(d[4])
		loiter = float(d[5])/1000.0
		yaw = float(d[6])
		vert_vmax = float(d[7])
		return (lat, lon, alt, radius, loiter, yaw, vert_vmax)

	def load_action_16(self, d): # NAV: Waypoint
		lat = float(d[8])
		lon = float(d[9])
		alt = float(d[10])
		radius = float(d[4])
		loiter = float(d[5])/1000.0
		hori_vmax = float(d[6])
		yaw = float(d[7])
		return (lat, lon, alt, radius, loiter, hori_vmax, yaw)

	def load_action_21(self, d): # NAV: Land
		lat = float(d[8])
		lon = float(d[9])
		alt = float(d[10])
		dunno = float(d[4])
		vert_vmax = float(d[5])
		hori_vmax = float(d[6])
		yaw = float(d[7])
		return (lat, lon, alt, vert_vmax, hori_vmax, yaw)

	def load(self, filename):
		correct_format = False
		pts = []
		lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
		for i in xrange(len(lines)): # for all lines
			if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
				if lines[i][:3] == 'QGC':
					correct_format = True				
				elif correct_format == True:
					data = lines[i].split ('\t') # split into tab separated list
					count = int(data[0])
					dunno1 = int(data[1])
					frame = int(data[2]) # 0=Abs.alt, 3=Rel.alt
					action = int(data[3])
					if action == 22: # NAV: TakeOff
						(lat, lon, alt, radius, loiter, yaw, vert_vmax) = self.load_action_22(data)
						pts.append ([count, frame, action, lat, lon, alt, radius, loiter, yaw, vert_vmax])

					elif action == 16: # NAV: Waypoint
						(lat, lon, alt, radius, loiter, hori_vmax, yaw) = self.load_action_16(data)
						pts.append ([count, frame, action, lat, lon, alt, radius, loiter, hori_vmax, yaw])

					elif action == 21: # NAV: Land
						(lat, lon, alt, vert_vmax, hori_vmax, yaw) = self.load_action_21(data)
						pts.append ([count, frame, action, lat, lon, alt, vert_vmax, hori_vmax, yaw])
		return pts

