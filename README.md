# Packet Definitions

This C++ library defines a set of packet structures, classes and methods for communication over UDP. It is part of a larger application and it specifically concerns the namespace `ApsUDPMod`.

## 📚 Dependencies

- `cstdint`
- `cstring`
- `vector`
- `boost/crc.hpp`
- `ros::Time` (Please ensure you have the ROS environment set up.)

## 🔧 Structures and Classes

### 1. `waypoint_t`

This is a structure that represents a single waypoint. It includes attributes for ID, coordinates (x, y, z), yaw, velocity, and change_flag.

### 2. `header`

This is a class that represents a packet header. It includes attributes for message ID, request ID, timestamp, ROS timestamp, data size, and additional data information.

The header also includes `pack` and `unpack` methods for packing the header data into a byte vector and unpacking it from a byte vector, respectively.

### 3. `statusMsg`

This class represents status messages that contain information about the closest global waypoint ID, current velocity, and whether or not dbw is engaged.

The `statusMsg` class includes a `pack` method to pack the status message data into a byte vector.

### 4. `WaypointsArrayMsg`

This class represents a waypoints array message that can contain up to 100 waypoints. It has `pack` and `unpack` methods for packing the waypoints array message data into a byte vector and unpacking it from a byte vector, respectively.

### 5. `Message`

The `Message` class represents a complete message that includes a header, a waypoints array message, a status message, message content, processing time, and a checksum for the message. 

The `Message` class includes a `pack` method to pack the message data into a byte vector.

## 💻 Usage

Include this header file in your project and use the structures and classes to create and manipulate messages as necessary for your specific requirements.

```cpp
#include "packet_definitions.h"

ApsUDPMod::Message message;
//... initialize and manipulate message

std::vector<uint8_t> buffer = message.pack();
// ... send buffer over UDP
