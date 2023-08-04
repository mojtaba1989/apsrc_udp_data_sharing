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
  if (udp_interface_.is_open()){
    udp_interface_.close();
  }
}

void ApsrcUdpDataSharingNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Subscribers
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &ApsrcUdpDataSharingNl::baseWaypointCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcUdpDataSharingNl::velocityCallback, this);
  vehicle_status_sub_ = nh_.subscribe("vehicle_status", 1, &ApsrcUdpDataSharingNl::vehicleStatusCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &ApsrcUdpDataSharingNl::closestWaypointCallback, this);

  if (openConnection())
  {
  } else {
    ros::requestShutdown(); // or maybe not
  }

}

bool ApsrcUdpDataSharingNl::openConnection(){
  AS::Network::ReturnStatuses status = udp_interface_.open(destination_ip_, destination_port_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not open UDP interface: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }
  else
  {
    ROS_INFO("UDP interface opened, sending to %s:%d", destination_ip_.c_str(), destination_port_);
    return true;
  }
}

void ApsrcUdpDataSharingNl::loadParams()
{
  pnh_.param<std::string>("/apsrc_udp_network/destination_ip", destination_ip_, "127.0.0.1");
  pnh_.param("/apsrc_udp_network/destination_port", destination_port_, 1552);
  pnh_.param("/apsrc_udp_network/frequency", duration_, 0.1);

  ROS_INFO("Parameters Loaded");
  return;
}

void ApsrcUdpDataSharingNl::timerCallback()
{
  if (udp_interface_.is_open()){
    udp_interface_.write(message_.pack());
    message_ = {};
    message_.header.msg_id = ++msg_id_;
    message_.header.time_stamp[0] = ros::Time::now().sec;
    message_.header.time_stamp[1] = ros::Time::now().nsec;
    message_.crc = 0;
    UDPStatusShare();
    UDPGlobalPathShare();
  }
}

void ApsrcUdpDataSharingNl::UDPStatusShare()
{
  std::unique_lock<std::mutex> msg_lock(msg_mts_);
  message_.status_msg.current_velocity = current_velocity_;
  message_.status_msg.dbw_engaged = dbw_engaged_;
  message_.status_msg.closest_global_waypoint_id = closest_waypoint_id_;
  message_.header.data_info[0] += 8;
}

void ApsrcUdpDataSharingNl::UDPGlobalPathShare()
{
  if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
    int32_t start_id = closest_waypoint_id_;
    autoware_msgs::Lane temp_waypoints = base_waypoints_;

    std::unique_lock<std::mutex> msg_lock(msg_mts_);
    message_.waypoints_array_msg.num_waypoints = (temp_waypoints.waypoints.size() - start_id < 100) ?
            temp_waypoints.waypoints.size() - start_id : 100;
    for (uint i = 0; i < message_.waypoints_array_msg.num_waypoints; i++) {
      uint wp_id = i + start_id;
      message_.waypoints_array_msg.waypoints_array[i].waypoint_id =
              temp_waypoints.waypoints[wp_id].gid;
      message_.waypoints_array_msg.waypoints_array[i].x =
              temp_waypoints.waypoints[wp_id].pose.pose.position.x;
      message_.waypoints_array_msg.waypoints_array[i].y =
              temp_waypoints.waypoints[wp_id].pose.pose.position.y;
      message_.waypoints_array_msg.waypoints_array[i].z =
              temp_waypoints.waypoints[wp_id].pose.pose.position.z;
      message_.waypoints_array_msg.waypoints_array[i].yaw =
              tf::getYaw(temp_waypoints.waypoints[wp_id].pose.pose.orientation);
      message_.waypoints_array_msg.waypoints_array[i].velocity =
              temp_waypoints.waypoints[wp_id].twist.twist.linear.x;
      message_.waypoints_array_msg.waypoints_array[i].change_flag =
              temp_waypoints.waypoints[wp_id].change_flag;
    }
    message_.header.data_info[0] += 16;
  }
}

void ApsrcUdpDataSharingNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x) * 1000.0));
  ApsrcUdpDataSharingNl::timerCallback();
}

void ApsrcUdpDataSharingNl::vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  dbw_engaged_ = (vehicle_status->drivemode == autoware_msgs::VehicleStatus::MODE_AUTO ||
                  vehicle_status->steeringmode == autoware_msgs::VehicleStatus::MODE_AUTO);
}

void ApsrcUdpDataSharingNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
}

void ApsrcUdpDataSharingNl::baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints)
{
  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
  base_waypoints_ = *base_waypoints;
  received_base_waypoints_ = true;
}
}
PLUGINLIB_EXPORT_CLASS(apsrc_udp_data_sharing::ApsrcUdpDataSharingNl, nodelet::Nodelet);