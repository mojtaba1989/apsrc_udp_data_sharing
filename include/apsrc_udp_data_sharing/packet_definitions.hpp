#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>


namespace ApsUDPMod {
    struct waypoint_t {
        float waypoint_id = 0;
        float x = 0;
        float y = 0;
        float z = 0;
        float yaw = 0;
        float velocity = 0;
    };// 24 bytes

    struct waypoint_short_t {
        int16_t waypoint_id = 0;
        int16_t velocity = 0;
        int16_t z = 0;
    }; // 6 bytes

    struct reply_t {
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

    class replyMsg{
    public:
        struct reply_t reply_array;

        int pack(std::vector<uint8_t> &buffer, int i){
            std::memcpy(&buffer[i], &reply_array, 19);
            return i+19;
        }
    };

    class statusMsg {//20 bytes
    public:
        int32_t closest_global_waypoint_id;
        uint16_t current_velocity;
        bool dbw_engaged = false; 
        bool lead_vehicle_detected = false;
        float vehicle_heading = 0;
        uint8_t path_curvature_score = 0;
        double lead_vehicle_speed = 0;

        int pack(std::vector<uint8_t> &buffer, int i) {
          std::memcpy(&buffer[i], &closest_global_waypoint_id, 4);
          std::memcpy(&buffer[i+4], &current_velocity, 2);
          std::memcpy(&buffer[i+6], &vehicle_heading, 4);
          buffer[i+10] = static_cast<uint8_t>(dbw_engaged);
          buffer[i+11] = static_cast<uint8_t>(lead_vehicle_detected);
          std::memcpy(&buffer[i+12], &path_curvature_score, 8);
          return i+20;
        }
    };

    class WaypointsArrayMsg {// (2400/1)=2401 bytes
    public:
        uint8_t num_waypoints = 0;
        struct waypoint_t waypoints_array[100];

        int pack(std::vector<uint8_t> &buffer, int i) {
          buffer[i] = num_waypoints;
          std::memcpy(&buffer[i+1], waypoints_array, 2400);
          return i+2401;
        }
    };

    class LatLongOffsetMsg { // 56 bytes
    public:
        double lat;
        double lon;
        double x;
        double y;
        double z;
        double log_lat;
        double log_long;

        int pack(std::vector<uint8_t> &buffer, int i){
            std::memcpy(&buffer[i], &lat, 8);
            std::memcpy(&buffer[i+8], &lon, 8);
            std::memcpy(&buffer[i+16], &x, 8);
            std::memcpy(&buffer[i+24], &y, 8);
            std::memcpy(&buffer[i+32], &z, 8);
            std::memcpy(&buffer[i+40], &log_lat, 8);
            std::memcpy(&buffer[i+48], &log_long, 8);
            return i+56;
        }
    };

    class ExtraMsg{// 9 bytes
    public:
        uint8_t offset_flag;
        int32_t obstacle_waypoint;
        float obstacle_lat_offset;

        int pack(std::vector<uint8_t> &buffer, int i){
            buffer[i] = offset_flag;
            std::memcpy(&buffer[i+1], &obstacle_waypoint, 4);
            std::memcpy(&buffer[i+5], &obstacle_lat_offset, 4);
            return i+9;            
        }
    };

    class RadarMsg{ // 12 bytes
    public:
        float range;
        float range_rate;
        float angle;

        int pack(std::vector<uint8_t> &buffer, int i){
            std::memcpy(&buffer[i], &range, 4);
            std::memcpy(&buffer[i+4], &range_rate, 4);
            std::memcpy(&buffer[i+8], &angle, 4);
            return i+12;            
        }

    };

    class Message_general {// 
    public:
        ApsUDPMod::header header;
        ApsUDPMod::replyMsg replies_msg;
        ApsUDPMod::WaypointsArrayMsg waypoints_array_msg;
        ApsUDPMod::statusMsg status_msg;
        ApsUDPMod::LatLongOffsetMsg lat_lon_msg;
        ApsUDPMod::ExtraMsg extra_msg;
        ApsUDPMod::RadarMsg radar_msg;
        uint32_t crc;
        

        std::vector<uint8_t> pack() {
          std::vector<uint8_t> buffer(4096);
          int cb = header.pack(buffer);
          cb = replies_msg.pack(buffer, cb);
          cb = status_msg.pack(buffer, cb);
          cb = waypoints_array_msg.pack(buffer, cb);
          cb = lat_lon_msg.pack(buffer, cb);
          cb = extra_msg.pack(buffer, cb);
          cb = radar_msg.pack(buffer, cb);
          
          // Calculate CRC
          boost::crc_32_type msg_crc;
          msg_crc.process_bytes(&buffer[0], 4092);
          crc = msg_crc.checksum();
          std::memcpy(&buffer[4092], &crc, 4);
          return buffer;
        }
    };

    class FullWaypoint_Msg {
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
