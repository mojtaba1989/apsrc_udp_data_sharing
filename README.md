# Packet Definitions

This C++ library defines a set of packet structures, classes, and methods for communication over UDP. It is part of a larger application and it specifically concerns the namespace `ApsUDPMod`.

## 📚 Dependencies

- `cstdint`
- `cstring`
- `vector`
- `boost/crc.hpp`
- `ros::Time` (Please ensure you have the ROS environment set up.)

## 🔧 Data Structures and Classes

### 1. `waypoint_t`

This structure represents a single waypoint. It contains:

- `waypoint_id`: An unsigned 32-bit integer representing the waypoint ID.
- `x`, `y`, `z`: Floats representing the waypoint coordinates.
- `yaw`: Float representing the waypoint's orientation.
- `velocity`: Float representing the waypoint's velocity.
- `change_flag`: An unsigned 32-bit integer representing the waypoint's change flag.

### 2. `header`

This class represents a packet header. It contains:

- `msg_id`: An 8-bit unsigned integer for the message ID.
- `request_id`: An 8-bit unsigned integer for the request ID.
- `time_stamp`: An array of two 32-bit integers for the time stamp.
- `ros_time_stamp`: A ROS time stamp.
- `data_size_byte`: An unsigned 32-bit integer representing the data size in bytes.
- `data_info`: An array of 10 8-bit unsigned integers for additional data information.

The `header` class also includes `pack` and `unpack` methods for packing the header data into a byte vector and unpacking it from a byte vector, respectively.

### 3. `statusMsg`

This class represents status messages that contain:

- `closest_global_waypoint_id`: A 32-bit integer representing the closest global waypoint ID.
- `current_velocity`: A 16-bit unsigned integer representing the current velocity.
- `dbw_engaged`: A boolean indicating whether or not dbw is engaged.

The `statusMsg` class includes a `pack` method to pack the status message data into a byte vector.

### 4. `WaypointsArrayMsg`

This class represents a waypoints array message. It contains:

- `num_waypoints`: An 8-bit unsigned integer representing the number of waypoints.
- `waypoints_array`: An array of `waypoint_t` structures, up to 100 waypoints.

The `WaypointsArrayMsg` class has `pack` and `unpack` methods for packing the waypoints array message data into a byte vector and unpacking it from a byte vector, respectively.

### 5. `Message`

The `Message` class represents a complete message. It contains:

- `header`: An object of the `header` class.
- `waypoints_array_msg`: An object of the `WaypointsArrayMsg` class.
- `status_msg`: An object of the `statusMsg` class.
- `message_content`: An 8-bit unsigned integer representing the message content.
- `processing_time`: A float representing the processing time.
- `crc`: An unsigned 32-bit integer representing the checksum for the message.

The `Message` class includes a `pack` method to pack the message data into a byte vector.

## 💻 Usage

Include this header file in your project and use the structures and classes to create and manipulate messages as necessary for your specific requirements.

```cpp
#include "packet_definitions.h"

ApsUDPMod::Message message;
//... initialize and manipulate message

std::vector<uint8_t> buffer = message.pack();
// ... send buffer over UDP
