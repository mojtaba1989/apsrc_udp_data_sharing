# Packet Definitions

This repository contains a C++ header file defining packet structures and classes for a communication protocol used in the ApsUDPMod system.

## Overview

The `packet_definitions.h` file defines various packet structures and classes for handling different types of commands and data in the ApsUDPMod system. It includes the following structures and classes:

- `waypoint_t`: Structure defining waypoint parameters.
- `waypoint_short_t`: Structure defining short waypoint parameters.
- `reply_t`: Structure defining reply parameters.
- `header`: Class representing the header of a packet.
- `replyMsg`: Class representing a reply message packet.
- `waypointsArrayMsg`: Class representing a waypoints array message packet.
- `leadDetectMsg`: Class representing a lead detection message packet.
- `Message_general`: Class representing a general message packet.
- `FullWaypoint_Msg`: Class representing a full waypoint message packet.

## Structure Details

### `waypoint_t`

- `x`: float. X-coordinate of the waypoint.
- `y`: float. Y-coordinate of the waypoint.
- `z`: float. Z-coordinate of the waypoint.
- `yaw`: float. Yaw angle of the waypoint.
- `velocity`: float. Velocity of the waypoint.

### `waypoint_short_t`

- `waypoint_id`: int16_t. ID of the waypoint.
- `velocity`: int16_t. Velocity of the waypoint.
- `z`: int16_t. Z-coordinate of the waypoint.

### `reply_t`

- `response_to_msg_id`: uint8_t. Response to message ID.
- `response_to_request_id`: uint8_t. Response to request ID.
- `request_stamp`: int32_t[2]. Request timestamp.
- `acknowledge_stamp`: int32_t[2]. Acknowledge timestamp.
- `request_accomplished`: bool. Flag indicating if the request is accomplished.

### `header`

- `msg_id`: uint8_t. Message ID.
- `respond_stamp`: int32_t[2]. Respond timestamp.
- `info`: int8_t[10]. Information about the message.

### `replyMsg`

- `reply`: reply_t. Reply.

### `waypointsArrayMsg`

- `closest_global_waypoint_id`: int32_t. Closest Waypoiont.
- `num_waypoints`: uint8_t. Number of waypoints (up to 100).
- `path_curvature_score`: float. Path curvature score.
- `waypoints_array`: waypoint_t[100]. Waypoint array.

### `leadDetectMsg`

- `detected`: int8_t. Indicates if a lead is detected. (0:None, 1:Lidar perception only, 2:apsrc_lead_vehicle_detection only, 3:BOTH)
- `gap_lat`: double. Latitude gap for lead detection.
- `gap_lng`: double. Longitude gap for lead detection.
- `scale_x`: double. Fitted plane in the x-direction for lead detection.
- `scale_y`: double. Fitted plane in the y-direction for lead detection.
- `radar_lng`: double. Radar-based range of lead vehicle detection.
- `lead_vel_mps_abs`: double. Absolute velocity of the lead vehicle.
- `lead_vel_mps_rel`: double. Relative velocity of the lead vehicle.

### Classes for Packing

- `replyMsg`: Class for packing reply message packets.
- `waypointsArrayMsg`: Class for packing waypoints array message packets.
- `leadDetectMsg`: Class for packing lead detection message packets.
- `Message_general`: Class for packing general message packets.
- `FullWaypoint_Msg`: Class for packing full waypoint message packets.

### How to Unpack General Messages

Total length: 4096 bytes
1. header: `header` 19 bytes
2. body
	- apsrc_waypoint_replanner response: `replyMsg` 19 bytes
	- waypoints: `waypointsArrayMsg` 2009 bytes
	- lead detection: `leadDetectMsg` 57 bytes
	- NULL filler: 2007 bytes
3. `CRC`: 4 bytes
