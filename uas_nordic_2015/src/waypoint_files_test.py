#!/usr/bin/env python
"""
2015-05-20 KJ First version
"""


from waypoint_files import qgc_wpl_120


wpt_import = qgc_wpl_120()


lst = wpt_import.load('waypoints.txt')

for i in xrange(len(lst)):
	print lst[i]

