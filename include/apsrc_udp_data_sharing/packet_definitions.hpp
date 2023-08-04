#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>


namespace ApsUDPMod {
    struct waypoint_t {
        uint32_t waypoint_id = 0;
        float x = 0;
        float y = 0;
        float z = 0;
        float yaw = 0;
        float velocity = 0;
        uint32_t change_flag = 0;
    };//28 bytes

    class header {//24 bytes
    public:
        uint8_t msg_id;
        uint8_t request_id;
        int32_t time_stamp[2];
        ros::Time ros_time_stamp;
        uint32_t data_size_byte;
        uint8_t data_info[10];

        bool pack(std::vector<uint8_t> &buffer) {
          buffer[0] = msg_id;
          buffer[1] = request_id;
          std::memcpy(&buffer[2], &time_stamp, 8);
          std::memcpy(&buffer[10], &data_size_byte, 4);
          std::memcpy(&buffer[14], &data_info, 10);
          return true;
        }

        bool unpack(const std::vector<uint8_t> &buffer) {
          msg_id = buffer[0];
          request_id = buffer[1];
          std::memcpy(&ros_time_stamp, &buffer[2], 8);
          std::memcpy(&data_size_byte, &buffer[10], 4);
          std::memcpy(&data_info, &buffer[14], 10);
          return true;
        }
    };

    class statusMsg {
    public:
        int32_t closest_global_waypoint_id;
        uint16_t current_velocity;
        bool dbw_engaged = false; 

        void pack(std::vector<uint8_t> &buffer) {
          std::memcpy(&buffer[44], &closest_global_waypoint_id, 4);
          std::memcpy(&buffer[48], &current_velocity, 2);
          buffer[50] = static_cast<uint8_t>(dbw_engaged);
        }
    };

    class WaypointsArrayMsg {// (2800/1)=2081 bytes
    public:
        uint8_t num_waypoints = 0;
        struct waypoint_t waypoints_array[100];

        bool pack(std::vector<uint8_t> &buffer) {
          buffer[51] = num_waypoints;
          std::memcpy(&buffer[52], waypoints_array, 2800);
          return true;
        }

        void unpack(const std::vector<uint8_t> &buffer) {
          num_waypoints = buffer[51];
          std::memcpy(waypoints_array, &buffer[52], 2800);
        }
    };

    class Message {//
    public:
        ApsUDPMod::header header;
        ApsUDPMod::WaypointsArrayMsg waypoints_array_msg;
        ApsUDPMod::statusMsg status_msg;
        uint8_t message_content;
        float processing_time;
        uint32_t crc;
        

        std::vector<uint8_t> pack() {
          std::vector<uint8_t> buffer(4096);
          header.pack(buffer);
          waypoints_array_msg.pack(buffer);
          status_msg.pack(buffer);
          buffer[14] = message_content;

          // Calculate CRC
          boost::crc_32_type msg_crc;
          msg_crc.process_bytes(&buffer[0], 4092);
          crc = msg_crc.checksum();
          std::memcpy(&buffer[4092], &crc, 4);
          return buffer;
        }
    };
}// namespace APSMod
#endif  // PACKET_DEFINITIONS_H
