#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <apsrc_msgs/CommandAccomplished.h>
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
  base_waypoints_sub_       = nh_.subscribe("base_waypoints", 1, &ApsrcUdpDataSharingNl::baseWaypointCallback, this);
  closest_waypoint_sub_     = nh_.subscribe("closest_waypoint", 1, &ApsrcUdpDataSharingNl::closestWaypointCallback, this);

  if (!waypoint_only_){
    current_velocity_sub_     = nh_.subscribe("current_velocity", 1, &ApsrcUdpDataSharingNl::velocityCallback, this);
    vehicle_status_sub_       = nh_.subscribe("vehicle_status", 1, &ApsrcUdpDataSharingNl::vehicleStatusCallback, this);
    backplane_estimation_sub_ = nh_.subscribe("backplane_estimation/filtered_offsets", 1, &ApsrcUdpDataSharingNl::backPlaneEstimationCallback, this);
    backplane_marker_sub_     = nh_.subscribe("backplane_estimation/backplane_filtered_marker", 1, &ApsrcUdpDataSharingNl::backPlaneMarkerCallback, this);
    udp_report_sub_           = nh_.subscribe("apsrc_udp/received_commands_report", 1, &ApsrcUdpDataSharingNl::udpReceivedReportCallback, this);
  } else {
    udp_request_sub_           = nh_.subscribe("apsrc_udp/received_commands", 1, &ApsrcUdpDataSharingNl::udpReceivedCommandCallback, this);
  }
  
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
  pnh_.param<std::string>("destination_ip", destination_ip_, "127.0.0.1");
  pnh_.param("destination_port", destination_port_, 1552);
  pnh_.param("frequency", duration_, 0.1);
  pnh_.param("waypoint_only", waypoint_only_, false);

  ROS_INFO("Parameters Loaded");
  return;
}

void ApsrcUdpDataSharingNl::UDPDataSharingGeneral()
{
  if (udp_interface_.is_open()){
    udp_interface_.write(message_.pack());
    message_ = {};
    message_.header.info[0] = 0;
    UDPReportShare();
    UDPStatusShare();
    UDPGlobalPathShare();
    UDPBackPlaneEstimationShare();
    message_.header.msg_id = ++msg_id_;
    message_.header.respond_stamp[0] = ros::Time::now().sec;
    message_.header.respond_stamp[1] = ros::Time::now().nsec;
    message_.crc = 0;  
  }
}

void ApsrcUdpDataSharingNl::UDPReportShare()
{
  if (report_received_){
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.replies_msg.reply_array.response_to_msg_id = report_.msg_id;
    message_.replies_msg.reply_array.response_to_request_id = report_.request_id;
    message_.replies_msg.reply_array.request_stamp[0] = report_.request_stamp.toSec();
    message_.replies_msg.reply_array.request_stamp[1] = report_.request_stamp.toNSec();
    message_.replies_msg.reply_array.acknowledge_stamp[0] = report_.header.stamp.toSec();
    message_.replies_msg.reply_array.acknowledge_stamp[1] = report_.header.stamp.toNSec();
    message_.replies_msg.reply_array.request_accomplished = report_.request_accomplished;

    report_received_ = false;
  }
}

void ApsrcUdpDataSharingNl::UDPStatusShare()
{
  std::unique_lock<std::mutex> msg_lock(msg_mtx_);
  message_.status_msg.current_velocity = current_velocity_;
  message_.status_msg.dbw_engaged = dbw_engaged_;
  message_.status_msg.closest_global_waypoint_id = closest_waypoint_id_;
}

void ApsrcUdpDataSharingNl::UDPGlobalPathShare()
{
  if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
    int32_t start_id = closest_waypoint_id_;
    autoware_msgs::Lane temp_waypoints = base_waypoints_;

    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.waypoints_array_msg.num_waypoints = (temp_waypoints.waypoints.size() - start_id < 100) ?
            temp_waypoints.waypoints.size() - start_id : 100;
    for (uint i = 0; i < message_.waypoints_array_msg.num_waypoints; i++) {
      uint wp_id = i + start_id;
      message_.waypoints_array_msg.waypoints_array[i].waypoint_id =
              static_cast<float>(temp_waypoints.waypoints[wp_id].gid);
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
    }
  }
}

void ApsrcUdpDataSharingNl::UDPBackPlaneEstimationShare()
{
  if (received_estimation_ and received_marker_){
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.lat_lon_msg.lat = lat_long_offset_.lateral_offset;
    message_.lat_lon_msg.lon = lat_long_offset_.longitudinal_offset;
    message_.lat_lon_msg.x   = marker_.scale.x;
    message_.lat_lon_msg.y   = marker_.scale.y;
    message_.lat_lon_msg.z   = marker_.scale.z;

    lat_long_offset_ = {};
    marker_ = {};
    received_estimation_ = false;
    received_marker_ = false;
  }
}

void ApsrcUdpDataSharingNl::UDPFullPathShare()
{
  if (waypoint_only_ and received_base_waypoints_ and closest_waypoint_id_!=-1) {
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    while (!full_path_has_been_shared_)
    {
      ApsUDPMod::FullWaypoint_Msg msg = {};
      msg.start_id = last_shared_id_;
      msg.number_of_waypoints = static_cast<uint16_t>(base_waypoints_.waypoints.size());
      msg.end_of_data = (base_waypoints_.waypoints.size() - last_shared_id_) <= 1361 ? true:false;
      full_path_has_been_shared_ = msg.end_of_data;
      size_t end_id = msg.end_of_data ? (base_waypoints_.waypoints.size() - last_shared_id_) : 1361;
      for (size_t i = 0; i < end_id; ++i){
        msg.wp_array[i].waypoint_id = static_cast<int16_t>(base_waypoints_.waypoints[i+last_shared_id_].gid);
        msg.wp_array[i].velocity = static_cast<int16_t>(base_waypoints_.waypoints[i+last_shared_id_].twist.twist.linear.x*10);
        msg.wp_array[i].z = static_cast<int16_t>(base_waypoints_.waypoints[i+last_shared_id_].pose.pose.position.z*10);
      }
      last_shared_id_ = end_id;
      udp_interface_.write(msg.pack());
    }
    last_shared_id_ = 0;
    full_path_has_been_shared_ = false;
  }
}

void ApsrcUdpDataSharingNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> v_lock(status_data_mtx_);
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x) * 1000.0));
  ApsrcUdpDataSharingNl::UDPDataSharingGeneral();
}

void ApsrcUdpDataSharingNl::vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status)
{
  std::unique_lock<std::mutex> vs_lock(status_data_mtx_);
  dbw_engaged_ = (vehicle_status->drivemode == autoware_msgs::VehicleStatus::MODE_AUTO ||
                  vehicle_status->steeringmode == autoware_msgs::VehicleStatus::MODE_AUTO);
}

void ApsrcUdpDataSharingNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> cwp_lock(status_data_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
}

void ApsrcUdpDataSharingNl::baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints)
{
  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);
  base_waypoints_ = *base_waypoints;
  received_base_waypoints_ = true;
}

void ApsrcUdpDataSharingNl::udpReceivedReportCallback(const apsrc_msgs::CommandAccomplished::ConstPtr& command_accomplished)
{
  std::unique_lock<std::mutex> udp_lock(udp_mtx_);
  report_ = *command_accomplished;
  report_received_ = true;
}

void ApsrcUdpDataSharingNl::backPlaneEstimationCallback(const apsrc_msgs::LatLonOffsets::ConstPtr& back_plane_estimation)
{
  std::unique_lock<std::mutex> wf_lock(wf_mtx_);
  lat_long_offset_ = *back_plane_estimation;
  received_estimation_ = true;
}

void ApsrcUdpDataSharingNl::backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& back_plane_marker)
{
  std::unique_lock<std::mutex> wf_lock(wf_mtx_);
  marker_ = *back_plane_marker;
  received_marker_ = true;
}

void ApsrcUdpDataSharingNl::udpReceivedCommandCallback(const apsrc_msgs::CommandReceived::ConstPtr& command_received)
{
  std::unique_lock<std::mutex> rc_lock(udp_mtx_);
  apsrc_msgs::CommandReceived command = *command_received;
  if (command.request_id == 1){
    UDPFullPathShare();
  }
}

}
PLUGINLIB_EXPORT_CLASS(apsrc_udp_data_sharing::ApsrcUdpDataSharingNl, nodelet::Nodelet);