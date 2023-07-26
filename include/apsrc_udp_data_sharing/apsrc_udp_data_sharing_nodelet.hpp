#ifndef APSRC_UDP_DATA_SharingNl_H
#define APSRC_UDP_DATA_SharingNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <thread>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>

namespace apsrc_udp_data_sharing 
{
class ApsrcUdpDataSharingNL : public nodelet::Nodelet
{
public:
  ApsrcUdpDataSharingNL();
  ~ApsrcUdpDataSharingNL();

private:


};
}
#endif //APSRC_UDP_DATA_SharingNl_H