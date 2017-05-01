


## QUESTIONS

Q: Why 15 hz ?
A:
	The frequency of the requestet positionstream has been set to 15 hz. Higher has not been testet. 15 hz is known to be the limit on pixhawk
	To see the frequency of positions/messages on a topic:

	    rostopic hz /drone/pose_gps

Q: Why no heatbeats to the UAV from autoquad_interface? 
	Transmission of heartbeats to the UAV is not implemented on autoquad and there by not in autoquad_interface.
	Autoquad transmits heartbeats with 1 hz. If the ground station stops receiving heartbeats, the connection has been lost.


Q: What's does the value on /drone/get_flight_status mean
A: 
	0  = AQ_NAV_STATUS_INIT     /* System is initializing | */
	1  = AQ_NAV_STATUS_STANDBY  /* System is standing by, not active | */
	2  = AQ_NAV_STATUS_MANUAL   /* Stabilized, under full manual control | */
	4  = AQ_NAV_STATUS_ALTHOLD  /* Altitude hold engaged | */
	8  = AQ_NAV_STATUS_POSHOLD  /* Position hold engaged | */
	16 = AQ_NAV_STATUS_DVH      /* Dynamic Velocity Hold is active | */
	32 = AQ_NAV_STATUS_MISSION  /* Autonomous mission execution mode | */



	The defines above needs to be AND'ed with the value from the topic to see the status.
	For most of the cases this is not necessary, and the defines above can be compared using == with the topic value.
		ex:
			Topic value: 12
			if 12 & AQ_NAV_STATUS_POSHOLD:
				print "Status: Poshold"
			if 12 & AQ_NAV_STATUS_ALTHOLD:
				print " and althold"

			Outputs:
				Status: Poshold and althold

Q: How to change current waypoint?
A: Write the seqid to /drone/set_current_waypoint of the waypoint you want the drone to approach

Q: How to change the flight_status?
A: This is UNTESTET.

	Send one or more defines below OR'ed together.

	ex.
		MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED is what autoquad sets when in mission_mode
			16 | 4 = 20

		write 20 on /drone/set_flight_status to go to mission_mode *UNTESTET*

	1   = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
		0b00000001 Reserved for future use.
	2   = MAV_MODE_FLAG_TEST_ENABLED
		0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. 
	4   = MAV_MODE_FLAG_AUTO_ENABLED
		0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	8   = MAV_MODE_FLAG_GUIDED_ENABLED
		0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	16  = MAV_MODE_FLAG_STABILIZE_ENABLED
		0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	32  = MAV_MODE_FLAG_HIL_ENABLED
		0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. 
	64  = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64
		0b01000000 remote control input is enabled. 
	128 = MAV_MODE_FLAG_SAFETY_ARMED
		0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. 
	129 = MAV_MODE_FLAG_ENUM_END

	0 = MAV_MODE_PREFLIGHT
		System is not ready to fly, booting, calibrating, etc. No flag is set.
	64 = MAV_MODE_MANUAL_DISARMED
		System is allowed to be active, under manual (RC) control, no stabilization
	66 = MAV_MODE_TEST_DISARMED
		UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	80 = MAV_MODE_STABILIZE_DISARMED
		System is allowed to be active, under assisted RC control. 
	88 = MAV_MODE_GUIDED_DISARMED
		System is allowed to be active, under autonomous control, manual setpoint 
	92 = MAV_MODE_AUTO_DISARMED=92
		System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	192 = MAV_MODE_MANUAL_ARMED=192
		System is allowed to be active, under manual (RC) control, no stabilization
	194 = MAV_MODE_TEST_ARMED=194
		UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	208 = MAV_MODE_STABILIZE_ARMED=208
		System is allowed to be active, under assisted RC control
	216 = MAV_MODE_GUIDED_ARMED
		System is allowed to be active, under autonomous control, manual setpoint 
	220 = MAV_MODE_AUTO_ARMED
		System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	221 = MAV_MODE_ENUM_END



Q: What is "bool current" in mission_item message?
A: Only has a value if mission_item message has been sent from the drone. Is true if the waypoint is active.


Q: What is "bool autocontinue" in mission_item message?
A: Description: "autocontinue to next wp", only sent from autoquad, always true(unknown reason).


Q: What is "frame" in mission_item?
A: Frame tells the drone wether the coordinates is global(lon, lat, alt) or local (x,y,z)

 - Defines for frame:
	0 = MAV_FRAME_GLOBAL
		Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
	
	3 = MAV_FRAME_GLOBAL_RELATIVE_ALT 
		Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. 
		First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */




Q: What is param1, param2, param3 and param4 in mission_item.
A: Each mission_item takes 4 extra parameters depending on the command.

	The following is implemented in autoquad:
	20 = RETURN_TO_LAUNCH
		targetRadius = param1
		loiterTime   = param2 [Sec]
		// param3 unused
		poiHeading   = param4

	21 = MAV_CMD_NAV_LAND
		//param1 unused
		maxVertSpeed = param2
		maxHorizSpeed = param3
		poiAltitude = param4

	16 = MAV_CMD_NAV_WAYPOINT
		targetRadius  = param1
		loiterTime    = param2 [Sec]
		maxHorizSpeed = param3
		poiHeading    = param4

	179 = MAV_CMD_DO_SET_HOME
		targetRadius  = param1
		loiterTime    = param2 [Sec]
		// param3 unused
		poiHeading    = param4

	22 = MAV_CMD_NAV_TAKEOFF
		targetRadius  = param1
		loiterTime    = param2 [Sec]
		poiHeading    = param3
		maxVertSpeed  = param4


Q: How does the seqid in mission_item work ?
A: At each mission_item, the seqid is incremented. If seqid = 25 (maximum defined by autoquad), it's set to zero. 
	**
		untested how the drones handles this overrun:
		From Maxim Paperno:
			 If it runs out of waypoints, it will drop into pos-hold mode.  If you add another mission item after this, 
			 then you will need to re-start the mission mode (AQ will start with the next waypoint it hasn't visited yet).
	**

changelog:
 23-05-2015:
    - cleaned CMakelist. Does not depend on mavlink-package + uses mavlink headers in include dir.
    - Added flight-status topic to see the current status of the drone.

 24-05-2015:
    - Update readme.txt
 03-05-2015:
    - Changed waypoint_Reached from bool to Uint32
