#include "ros/ros.h"
#include "std_msgs/String.h"
#include "autoquad_interface/Mavlink.h"
#include "autoquad_interface/drone_pos.h"
#include "autoquad_interface/drone_pose.h"
#include "autoquad_interface/drone_waypoint.h"
#include "autoquad_interface/IntStamped.h"
#include "autoquad_interface/UInt32Stamped.h"
#include "mavlink.h"
#include "autoquad/autoquad.h"
#include <sstream>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8.h"
#define ST_FIRST_HEARTBEAT  1
#define ST_SECOND_HEARTBEAT 2
#define ST_THIRD_HEARTBEAT  3

int seq = 0;
int total_number_of_waypoints = 0;
int drone_sysid = 0;
int state = ST_FIRST_HEARTBEAT;
int drone_heartbeat_watchdog_secs = 0;

bool debug = false;
bool send_data_request_after_reboot = true;
ros::Time last_heartbeat;

std::string drone_fused_position_pub, drone_gps_position_pub, drone_waypoint_sub, mavlink_from_sub, drone_set_mode_sub, mavlink_to_pub, drone_wp_reached_pub, \
 drone_flight_status_pub, drone_waypoint_ack_pub, drone_set_current_waypoint_sub, drone_clear_waypoints_sub;


autoquad_interface::Mavlink create_mission_request_list_msg();
autoquad_interface::Mavlink create_mission_request(int mission_seq);
autoquad_interface::Mavlink create_request_data_stream_msg(int req_stream_id, int req_msg_rate, int start_or_stop);
autoquad_interface::Mavlink create_mission_write_partial_list(int start, int end);
autoquad_interface::Mavlink create_mission_count(int missions);
autoquad_interface::Mavlink create_mission_item(int mission, int frame, int command, int current, int autocontinue, float x, float y, float z, float param1, float param2, float param3, float param4);

ros::Publisher pub_to_mavlink, pub_drone_fused_position, pub_drone_gps_position, pub_drone_wp_reached, pub_drone_flight_status, pub_waypoint_ack;
int drone_position_hz;

void on_mavlink_from(const autoquad_interface::Mavlink::ConstPtr& msg){
	static bool altitude_locked = false;
	static int setpoint_altitude = 0;

	mavlink_message_t mav_msg;


	mav_msg.len = msg->len;
	mav_msg.seq = msg->seq;
	mav_msg.sysid = msg->sysid;
	mav_msg.compid = msg->compid;
	mav_msg.msgid = msg->msgid;

	copy(msg->payload64.begin(), msg->payload64.end(), mav_msg.payload64);

	switch(msg->msgid){
		case MAVLINK_MSG_ID_MISSION_ACK:
		{
			mavlink_mission_ack_t mission_ack;
			mavlink_msg_mission_ack_decode(&mav_msg, &mission_ack);

			int mission_ack_type = mavlink_msg_mission_ack_get_type(&mav_msg);

			if(debug){
				ROS_WARN("Recv. Mission ack: %d", mission_ack_type);
			}

			std_msgs::UInt8 msg_out;
			msg_out.data = mission_ack_type;
			pub_waypoint_ack.publish(msg_out);
		}
		break;

		case MAVLINK_MSG_ID_HEARTBEAT:
       		{
			// Publish the drones system_state
			int system_status = mavlink_msg_heartbeat_get_system_status(&mav_msg);
			int base_mode = mavlink_msg_heartbeat_get_base_mode(&mav_msg);
			int custom_mode = mavlink_msg_heartbeat_get_custom_mode(&mav_msg);
//			if(debug){
//				std::cout << "System mode: " << system_status << std::endl;
//				std::cout << "Base_mode: " << base_mode << std::endl;
//				std::cout << "Custom_mode: " << custom_mode << std::endl;
//			}
			std_msgs::UInt32 msg_out;
			msg_out.data = custom_mode;
			pub_drone_flight_status.publish(msg_out);


			// Simple way to request the datastream when first heartbeat received.
			switch(state){
				case ST_FIRST_HEARTBEAT:
				{
					//save the drones sys id for later(needs to be in messages to the drone)
					drone_sysid = mav_msg.sysid;

					if(debug){
						ROS_INFO("Tx: Requesting global_position_stream, hz: %d", drone_position_hz);
					}

					// Request global_position_int, 1 = enable, 0 = disable
					autoquad_interface::Mavlink msg_out = create_request_data_stream_msg(MAV_DATA_STREAM_POSITION, drone_position_hz, 1);
					pub_to_mavlink.publish(msg_out);
					state = ST_SECOND_HEARTBEAT;
				}
				break;

				case ST_SECOND_HEARTBEAT:
				{
					state = ST_THIRD_HEARTBEAT;

//					autoquad_interface::Mavlink msg_out = create_mission_request_list_msg();
//					pub_to_mavlink.publish(msg_out);
				}
				break;
					default:
					break;
			}


                        //For the watchdog
                        last_heartbeat = ros::Time::now();
	  	}

	    	break;

		case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		{
			//Waypoint is reached, tell ROS
			autoquad_interface::UInt32Stamped msg_out;
			msg_out.data = mavlink_msg_mission_item_reached_get_seq(&mav_msg);
			pub_drone_wp_reached.publish(msg_out);
			if(debug){
				ROS_INFO("Recv: Mission item reached");
			}
		}


	        case MAVLINK_MSG_ID_STATUSTEXT:
	        {
			// Unpack message and get the message
			char * message;
			mavlink_msg_statustext_get_text(&mav_msg, message);
			std::cout << "Recv: " << message << std::endl;
	        }
	        break;

		case MAVLINK_MSG_ID_SYS_STATUS:
		{
			mavlink_sys_status_t sys_status;
			mavlink_msg_sys_status_decode(&mav_msg, &sys_status);
//			if(debug){
//				ROS_WARN("Recv: Sys status");
//			}
		}
		break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{

			/*
				The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
			*/


			autoquad_interface::drone_pose pose_out;

			// copy message to ROS message
			pose_out.time_boot_ms = mavlink_msg_local_position_ned_get_time_boot_ms(&mav_msg);
			pose_out.x = mavlink_msg_local_position_ned_get_x(&mav_msg);
			pose_out.y = mavlink_msg_local_position_ned_get_y(&mav_msg);
			pose_out.z = mavlink_msg_local_position_ned_get_z(&mav_msg);
			pose_out.vx = mavlink_msg_local_position_ned_get_vx(&mav_msg);
			pose_out.vy = mavlink_msg_local_position_ned_get_vy(&mav_msg);
			pose_out.vz = mavlink_msg_local_position_ned_get_vz(&mav_msg);

			pub_drone_fused_position.publish(pose_out);
		}
		break;


		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			/*
				The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value.
				See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
			*/

			mavlink_gps_raw_int_t gps_raw_int;
			mavlink_msg_gps_raw_int_decode(&mav_msg, &gps_raw_int);

			autoquad_interface::drone_pos pos_out;
			pos_out.header.stamp = ros::Time::now();
			pos_out.header.frame_id = "";
			pos_out.fix = gps_raw_int.fix_type;
			pos_out.lat = gps_raw_int.lat/(double)1e7;
			pos_out.lon = gps_raw_int.lon/(double)1e7;
			pos_out.alt = gps_raw_int.alt/1e3;
			pos_out.e = 0;
			pos_out.n = 0;


			pub_drone_gps_position.publish(pos_out);
		}
		break;


		case MAVLINK_MSG_ID_MISSION_COUNT:
		{
			mavlink_mission_count_t mission_count;
			mavlink_msg_mission_count_decode(&mav_msg, &mission_count);

			ROS_WARN("Number of wpts: %d", mission_count.count);

			// Save number of waypoints for later, begins with 0
			total_number_of_waypoints = mission_count.count -1;

			// Now that we got the total number of waypoints, let's request each one of them
			autoquad_interface::Mavlink msg_out = create_mission_request(total_number_of_waypoints);
			pub_to_mavlink.publish(msg_out);
			total_number_of_waypoints--;

		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM:
		{
			mavlink_mission_item_t mission_item;
			mavlink_msg_mission_item_decode(&mav_msg, &mission_item);


			std::cout << "Frame: " << mission_item.frame << std::endl;
			ROS_WARN("Frame is %d", mission_item.frame);

			std::cout << "Command: " << mission_item.command << std::endl;
			std::cout << "Current: " << mission_item.current << std::endl;
			ROS_WARN("current is %d", mission_item.current);
			std::cout << "Autocontinue: " << mission_item.autocontinue << std::endl;
			ROS_WARN("Autocontinue is %d", mission_item.autocontinue);
			std::cout << "Altitude: " << mission_item.x << std::endl;
			std::cout << "Longtitude: " << mission_item.y << std::endl;
			std::cout << "Altitude: " << mission_item.z << std::endl;

			std::cout << "param1: " << mission_item.param1 << std::endl;
			std::cout << "param2: " << mission_item.param2 << std::endl;
			std::cout << "param3: " << mission_item.param3 << std::endl;
			std::cout << "param4: " << mission_item.param4 << std::endl;


			if (total_number_of_waypoints >= 0){
				//If there is more waypoints left, fetch them
				autoquad_interface::Mavlink msg_out = create_mission_request(total_number_of_waypoints);
				pub_to_mavlink.publish(msg_out);
				total_number_of_waypoints--;
			}

		}
		break;

		case MAVLINK_MSG_ID_MISSION_REQUEST:
		{
			mavlink_mission_request_t mission_request;
			mavlink_msg_mission_request_decode(&mav_msg, &mission_request);

			// seq, frame, commnad, current, autocontinue, x,y,z,param1,param2,param3,param4
			autoquad_interface::Mavlink msg_out = create_mission_item(mission_request.seq, MAV_FRAME_GLOBAL, MAV_CMD_NAV_TAKEOFF,0, 1,15,25,30,1,2,3,4 );
			pub_to_mavlink.publish(msg_out);
		}
		break;
	}
}

autoquad_interface::Mavlink create_mission_item(int mission, int frame, int command, int current, int autocontinue, float x, float y, float z, float param1, float param2, float param3, float param4){

	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	if(debug){
		ROS_INFO("Tx: Adding mission: %d frame: %d", mission, frame);
	}

	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_ITEM_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_ITEM;
	msg_out.fromlcm=false;

	mavlink_mission_item_t packet;
	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.seq = mission;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;
	packet.param1 = param1;
	packet.param2 = param2,
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, MAVLINK_MSG_ID_MISSION_ITEM_LEN);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}
autoquad_interface::Mavlink create_mission_count(int missions){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	ROS_INFO("Number of missions to request: %d", missions);


	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_COUNT_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_COUNT;
	msg_out.fromlcm=false;

	mavlink_mission_count_t packet;
	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.count = missions;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}

autoquad_interface::Mavlink create_set_mode(int mode){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_SET_MODE_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_SET_MODE;
	msg_out.fromlcm=false;

	mavlink_set_mode_t packet;

	packet.target_system = drone_sysid;
	packet.base_mode = mode;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}

autoquad_interface::Mavlink create_mission_set_current(int seq){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_SET_CURRENT;
	msg_out.fromlcm=false;

	mavlink_mission_set_current_t packet;

	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.seq = seq;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}

autoquad_interface::Mavlink create_mission_write_partial_list(int start, int end){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	ROS_INFO("Writing waypoints start: %d, end: %d", start, end);


	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST;
	msg_out.fromlcm=false;

	mavlink_mission_write_partial_list_t packet;
	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.start_index = start;
	packet.end_index = end;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}


autoquad_interface::Mavlink create_request_data_stream_msg(int req_stream_id, int req_msg_rate, int start_or_stop){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
	msg_out.fromlcm=false;

	mavlink_request_data_stream_t packet;
	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.req_stream_id = req_stream_id;
	packet.req_message_rate = req_msg_rate;
	packet.start_stop = start_or_stop;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}


autoquad_interface::Mavlink create_mission_request(int mission_seq){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	ROS_INFO("Requesting waypoint: %d", mission_seq);


	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_REQUEST_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_REQUEST;
	msg_out.fromlcm=false;

	mavlink_mission_request_t packet;

	packet.target_system = drone_sysid;
	packet.target_component = 0;
	packet.seq = mission_seq;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}


autoquad_interface::Mavlink create_mission_request_list_msg(){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	ROS_INFO("Requesting waypoints");


	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
	msg_out.fromlcm=false;

	mavlink_mission_request_list_t packet;

	packet.target_system = drone_sysid;
	packet.target_component = 0;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, 33);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}
autoquad_interface::Mavlink create_mission_clear_all(){
	mavlink_message_t msg_first = {0};
	autoquad_interface::Mavlink msg_out;

	if(debug){
		ROS_INFO("Tx. Clearing all waypoints");
	}

	msg_out.seq=seq++;
	msg_out.len=MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN;
	msg_out.sysid=255;
	msg_out.compid=0;
	msg_out.msgid=MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
	msg_out.fromlcm=false;

	mavlink_mission_request_list_t packet;

	packet.target_system = drone_sysid;
	packet.target_component = 0;

	memcpy(_MAV_PAYLOAD_NON_CONST(&msg_first), &packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);

	for(int i = 0; i < 33; ++i){
		msg_out.payload64.push_back(msg_first.payload64[i]);
	}
	return msg_out;
}

/*
Run when waypoint is received
*/
void on_drone_waypoint(const autoquad_interface::drone_waypoint::ConstPtr& msg){

	if(debug){
		ROS_WARN("Adding waypoint to drone with seqid: %d, frame: %d", msg->seq, msg->frame);
	}

	//autoquad_interface::Mavlink create_mission_item(int mission, int frame, int command, int current, int autocontinue, int x, int y, int z, int param1, int param2, int param3, int param4);
	autoquad_interface::Mavlink msg_out = create_mission_item(msg->seq, msg->frame, msg->command, msg->current, msg->autocontinue, msg->lat, msg->lon, msg->alt, msg->param1, msg->param2, msg->param3, msg->param4 );
	pub_to_mavlink.publish(msg_out);

}

void on_drone_set_mode(const std_msgs::UInt32::ConstPtr& msg){
	if(debug){
		ROS_WARN("Setting mode: %d", msg->data);
	}
	autoquad_interface::Mavlink msg_out = create_set_mode(msg->data);
	pub_to_mavlink.publish(msg_out);
}


void on_drone_set_current_waypoint(const autoquad_interface::IntStamped::ConstPtr& msg){
	autoquad_interface::Mavlink msg_out = create_mission_set_current(msg->data);
	pub_to_mavlink.publish(msg_out);
}


/*
Run when clear_waypoints received
*/

void on_drone_clear_waypoints(const std_msgs::Bool::ConstPtr& msg){
	//If topic value is true, let's clear that list
	if(msg->data){
		autoquad_interface::Mavlink msg_out = create_mission_clear_all();
		pub_to_mavlink.publish(msg_out);
	}
}


void on_heartbeat_watchdog(const ros::TimerEvent&){
        if((ros::Time::now()-last_heartbeat).toSec() > drone_heartbeat_watchdog_secs){
		if(debug){
	                ROS_WARN("WD: Resend data_request_stream");
		}
		state = ST_FIRST_HEARTBEAT;
	}
}

int main(int argc, char **argv){
  ros::init(argc, argv, "mavlink_waypoint_node");
  ros::NodeHandle n("~");


  n.param<int>("drone_position_hz", drone_position_hz, 15);
  n.param<std::string>("drone_fused_position_pub", drone_fused_position_pub, "/drone/pose_fused");
  n.param<std::string>("drone_gps_position_pub", drone_gps_position_pub, "/drone/pose_gps");
  n.param<std::string>("drone_waypoint_sub", drone_waypoint_sub, "/drone/waypoint");
  n.param<std::string>("drone_waypoint_reached_pub", drone_wp_reached_pub, "/drone/waypoint_reached");
  n.param<std::string>("drone_flight_status_pub", drone_flight_status_pub, "/drone/get_flight_status");

  n.param<std::string>("drone_flight_status_sub", drone_set_mode_sub, "/drone/set_flight_status");
  n.param<std::string>("drone_clear_waypoints_sub", drone_clear_waypoints_sub, "/drone/clear_waypoints");

  n.param<std::string>("drone_set_current_waypoint_sub", drone_set_current_waypoint_sub, "/drone/set_current_waypoint");
  n.param<std::string>("drone_waypoint_ack_pub", drone_waypoint_ack_pub, "/drone/waypoint_ack");

  n.param<std::string>("mavlink_from_sub", mavlink_from_sub, "/mavlink/from");
  n.param<std::string>("mavlink_to_pub", mavlink_to_pub, "/mavlink/to");

  n.param<int>("drone_heartbeat_watchdog_secs", drone_heartbeat_watchdog_secs, 5);
  n.param<bool>("show_debug", debug, false);

  pub_to_mavlink = n.advertise<autoquad_interface::Mavlink>(mavlink_to_pub, 1000);

  pub_drone_fused_position = n.advertise<autoquad_interface::drone_pose>(drone_fused_position_pub, 1000);
  pub_drone_gps_position = n.advertise<autoquad_interface::drone_pos>(drone_gps_position_pub, 1000);
  pub_drone_wp_reached = n.advertise<autoquad_interface::UInt32Stamped>(drone_wp_reached_pub, 1000);
  pub_drone_flight_status = n.advertise<std_msgs::UInt32>(drone_flight_status_pub, 1000);
  pub_waypoint_ack = n.advertise<std_msgs::UInt8>(drone_waypoint_ack_pub, 1000);

  ros::Subscriber sub_from_mavlink    		  = n.subscribe(mavlink_from_sub, 1000, on_mavlink_from);
  ros::Subscriber sub_drone_waypoint  		  = n.subscribe(drone_waypoint_sub, 1000, on_drone_waypoint);
  ros::Subscriber sub_drone_set_mode  		  = n.subscribe(drone_set_mode_sub, 1000, on_drone_set_mode);
  ros::Subscriber sub_drone_set_current_waypoint  = n.subscribe(drone_set_current_waypoint_sub, 1000, on_drone_set_current_waypoint);
  ros::Subscriber sub_drone_clear_waypoints  = n.subscribe(drone_clear_waypoints_sub, 1000, on_drone_clear_waypoints);

  ros::Timer timer = n.createTimer(ros::Duration(1), on_heartbeat_watchdog);




  //Let the callback do the heavy stuff
  ros::spin();
  return 0;
}





