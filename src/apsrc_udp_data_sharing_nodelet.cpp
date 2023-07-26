#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include "apsrc_udp_data_sharing/apsrc_udp_data_sharing_nodelet.hpp"

namespace apsrc_udp_data_sharing 
{
ApsrcUdpDataSharingNl::ApsrcUdpDataSharingNl()
{
}

ApsrcUdpDataSharingNl::~ApsrcUdpDataSharingNl()
{
}

void ApsrcUdpDataSharingNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  // Create a string message
  autoware_msgs::Lane msg;
  // Serialize the message into a byte stream
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  std::vector<uint8_t> buffer(serial_size);

  ros::serialization::OStream stream(buffer.data(), serial_size);
  ros::serialization::serialize(stream, msg);

}
}
PLUGINLIB_EXPORT_CLASS(apsrc_udp_data_sharing::ApsrcUdpDataSharingNl,
                      nodelet::Nodelet);