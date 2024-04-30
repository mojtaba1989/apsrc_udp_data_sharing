#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>


namespace ApsUDPMod {
struct waypoint_t { 
	float x = 0;
	float y = 0;
	float z = 0;
	float yaw = 0;
	float velocity = 0;
};// 20 bytes

struct waypoint_short_t { 
	int16_t waypoint_id = 0;
	int16_t velocity = 0;
	int16_t z = 0;
}; // 6 bytes

struct reply_t { // Waypoint_replanner report publisher
	uint8_t response_to_msg_id;
	uint8_t response_to_request_id;
	int32_t request_stamp[2];
	int32_t acknowledge_stamp[2];
	bool request_accomplished;
}; // 19 bytes

class header {//19 bytes
public:
	uint8_t msg_id;
	int32_t respond_stamp[2];
	int8_t info[10];

	int pack(std::vector<uint8_t> &buffer) {
		buffer[0] = msg_id;
		std::memcpy(&buffer[1], &respond_stamp, 8);
		std::memcpy(&buffer[9], &info, 10);
		return 19;
	}
};

class replyMsg{// 19 bytes
public:
	struct reply_t reply_array;

	int pack(std::vector<uint8_t> &buffer, int i){
		std::memcpy(&buffer[i], &reply_array, 19);
		return i+19;
	}
};

class waypointsArrayMsg {// (2000/9)=2009 bytes
public:
	int32_t closest_global_waypoint_id;
	uint8_t num_waypoints = 0;
	float path_curvature_score = 0;
	struct waypoint_t waypoints_array[100];

	int pack(std::vector<uint8_t> &buffer, int i) {
		std::memcpy(&buffer[i], &closest_global_waypoint_id, 4);
		buffer[i+4] = num_waypoints;
		std::memcpy(&buffer[i+5], &path_curvature_score, 4);
		std::memcpy(&buffer[i+9], waypoints_array, 2400);
		return i+2409;
	}
}; 

class leadDetectMsg { // 57 bytes
public:
	int8_t detected;
	double gap_lat; // Logan devel
	double gap_lng; // Logan devel
	double scale_x; // Logan devel
	double scale_y; // Logan devel
	double radar_lng; // apsrc_lead_vehicle_detection
	double lead_vel_mps_abs; // apsrc_lead_vehicle_detection
	double lead_vel_mps_rel; // apsrc_lead_vehicle_detection

	int pack(std::vector<uint8_t> &buffer, int i){
		buffer[i] = detected;
		std::memcpy(&buffer[i+1], &gap_lat, 8);
		std::memcpy(&buffer[i+9], &gap_lng, 8);
		std::memcpy(&buffer[i+17], &scale_x, 8);
		std::memcpy(&buffer[i+25], &scale_y, 8);
		std::memcpy(&buffer[i+33], &radar_lng, 8);
		std::memcpy(&buffer[i+41], &lead_vel_mps_abs, 8);
		std::memcpy(&buffer[i+49], &lead_vel_mps_abs, 8);
		return i+57;
	}
};

class Message_general { // published periodically 
public:
	ApsUDPMod::header header;
	ApsUDPMod::replyMsg replies_msg;
	ApsUDPMod::waypointsArrayMsg waypoints_array_msg;
	ApsUDPMod::leadDetectMsg lead_msg;
	uint32_t crc;
	

	std::vector<uint8_t> pack() {
		std::vector<uint8_t> buffer(4096);
		int cb = header.pack(buffer);
		cb = replies_msg.pack(buffer, cb);
		cb = waypoints_array_msg.pack(buffer, cb);
		cb = lead_msg.pack(buffer, cb);
		
		// Calculate CRC
		boost::crc_32_type msg_crc;
		msg_crc.process_bytes(&buffer[0], 4092);
		crc = msg_crc.checksum();
		std::memcpy(&buffer[4092], &crc, 4);
		return buffer;
	}
};

class FullWaypoint_Msg { // Andrew's devel -- needs work
public:
	ApsUDPMod::header header;
	struct waypoint_short_t wp_array[1361];
	uint32_t crc;
	uint16_t number_of_waypoints;
	uint16_t start_id;
	bool end_of_data = true;
	

	std::vector<uint8_t> pack() {
		std::vector<uint8_t> buffer(8192);
		int cb = header.pack(buffer);
		std::memcpy(&buffer[9], &number_of_waypoints, 2);
		std::memcpy(&buffer[11], &start_id, 2);
		buffer[13] = static_cast<uint8_t>(end_of_data);
		std::memcpy(&buffer[cb], wp_array, 8166);

		// Calculate CRC
		boost::crc_32_type msg_crc;
		msg_crc.process_bytes(&buffer[0], 8188);
		crc = msg_crc.checksum();
		std::memcpy(&buffer[8188], &crc, 4);
		return buffer;
	}     
};   
}// namespace APSMod
#endif  // PACKET_DEFINITIONS_H
