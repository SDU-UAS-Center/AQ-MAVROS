Header header
uint16 seq		    # Sequence - uniq identifier of each waypoint
uint8 frame         # The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
bool current        # false:0, true:1
bool autocontinue   # Autocontinue to next wp
uint16 command      # The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
float64 lat         # Latitude                
float64 lon         # Longtitude              
float64 alt         # Altitude                
float64 param1      # PARAM1, see MAV_CMD enum
float64 param2      # PARAM2, see MAV_CMD enum
float64 param3      # PARAM3, see MAV_CMD enum
float64 param4      # PARAM4, see MAV_CMD enum
