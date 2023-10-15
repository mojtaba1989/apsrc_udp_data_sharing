#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <apsrc_msgs/CommandAccomplished.h>
#include "apsrc_udp_data_sharing/apsrc_udp_data_sharing_nodelet.hpp"
#include <tf2_ros/transform_listener.h>

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
  tf2_ros::TransformListener tfListener(tfBuffer_);

  // Subscribers & Publishers
  base_waypoints_sub_       = nh_.subscribe("base_waypoints", 1, &ApsrcUdpDataSharingNl::baseWaypointCallback, this);
  closest_waypoint_sub_     = nh_.subscribe("closest_waypoint", 1, &ApsrcUdpDataSharingNl::closestWaypointCallback, this);

  if (!waypoint_only_){
    current_velocity_sub_   = nh_.subscribe("current_velocity", 1, &ApsrcUdpDataSharingNl::velocityCallback, this);
    vehicle_status_sub_     = nh_.subscribe("vehicle_status", 1, &ApsrcUdpDataSharingNl::vehicleStatusCallback, this);
    udp_report_sub_         = nh_.subscribe("apsrc_udp/received_commands_report", 1, &ApsrcUdpDataSharingNl::udpReceivedReportCallback, this);
    obj_mrk_array_sub_      = nh_.subscribe("detection/lidar_detector/objects_markers", 1, &ApsrcUdpDataSharingNl::objectMarkerArrayCallback, this);
    backplane_marker_sub_   = nh_.subscribe("backplane_estimation/backplane_filtered_marker", 1, &ApsrcUdpDataSharingNl::backPlaneMarkerCallback, this);
    vehicle_heading_sub_    = nh_.subscribe("gps/imu", 1, &ApsrcUdpDataSharingNl::vehicleHeadingCallback, this);
    obstacle_sub_           = nh_.subscribe("obstacle", 1, &ApsrcUdpDataSharingNl::obstacleCallback, this);
    obstacle_waypoints_sub_ = nh_.subscribe("obstacle_waypoint", 1, &ApsrcUdpDataSharingNl::obstacleWaypointsCallback, this);
    radar_track_sub_        = nh_.subscribe("/radar_fc/parsed_tx/radarvalid1", 1 ,&ApsrcUdpDataSharingNl::radarTrackCallback, this);
    timer_                  = nh_.createTimer(ros::Duration(1/frequency_), std::bind(&ApsrcUdpDataSharingNl::UDPDataSharingGeneral, this));
    lead_car_pub_           = nh_.advertise<visualization_msgs::Marker>("apsrc_vehicle_following/lead_car", 10, true);
  } else {
    udp_request_sub_        = nh_.subscribe("apsrc_udp/received_commands", 1, &ApsrcUdpDataSharingNl::udpReceivedCommandCallback, this);
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
  pnh_.param("frequency", frequency_, 10.0);
  pnh_.param("waypoint_only", waypoint_only_, false);
  pnh_.param("roi_lat", roi_lat_, 2.0);
  pnh_.param("roi_long", roi_long_, 40.0);
  pnh_.param("min_score", min_score_, 10);
  pnh_.param("path_eval_size", path_eval_size_, 20);

  ROS_INFO("Parameters Loaded");
  return;
}

void ApsrcUdpDataSharingNl::UDPDataSharingGeneral()
{
  if (udp_interface_.is_open()){
    // ROS_INFO("Message Publiched!");
    udp_interface_.write(message_.pack());
    message_ = {};
    message_.header.info[0] = 0;
    UDPReportShare();
    UDPStatusShare();
    UDPGlobalPathShare();
    UDPBackPlaneEstimationShare();
    UDPExtar();
    UDPRadar();
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

    message_.status_msg.path_curvature_score = ApsrcUdpDataSharingNl::path_curvature_score(path_eval_size_);
  }
}

void ApsrcUdpDataSharingNl::UDPBackPlaneEstimationShare()
{
  if (received_estimation_){
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.lat_lon_msg.lat = lead_.pose.position.y;
    message_.lat_lon_msg.lon = lead_.pose.position.x;
    message_.lat_lon_msg.x   = lead_.scale.x;
    message_.lat_lon_msg.y   = lead_.scale.y;
    message_.lat_lon_msg.z   = lead_.scale.z;

    received_estimation_ = false;
  }

  if (received_marker_){
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.lat_lon_msg.log_lat  = marker_.pose.position.y;
    message_.lat_lon_msg.log_long = marker_.pose.position.x - marker_.scale.y / 2;
    received_marker_ = false;
    message_.status_msg.lead_vehicle_detected = lidar_perception_score_ == min_score_;
  } else {
    message_.status_msg.lead_vehicle_detected = false;
  }
}

void ApsrcUdpDataSharingNl::UDPExtar()
{
  if (received_obstacle_ and received_obstacle_waypoints_){
    message_.extra_msg.offset_flag = 2;
    message_.extra_msg.obstacle_lat_offset = ApsrcUdpDataSharingNl::dist_to_waypoints();
    message_.extra_msg.obstacle_waypoint = obstacle_waypoints_;
    received_obstacle_ = false;
    received_obstacle_waypoints_ = false;
    return;
  }

  if (received_obstacle_waypoints_){
    message_.extra_msg.offset_flag = 1;
    message_.extra_msg.obstacle_waypoint = obstacle_waypoints_;
    received_obstacle_waypoints_ = false;
    return;
  }

  return;
}

void ApsrcUdpDataSharingNl::UDPRadar()
{
  if(received_radar_track_){
    message_.radar_msg.angle = radar_track_.lr_angle;
    message_.radar_msg.range = radar_track_.lr_range;
    message_.radar_msg.range_rate = radar_track_.lr_range_rate;

    received_radar_track_ = false;
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
        msg.wp_array[i].z = static_cast<int16_t>(base_waypoints_.waypoints[i+last_shared_id_].pose.pose.position.z*100);
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
  // ApsrcUdpDataSharingNl::UDPDataSharingGeneral();
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

void ApsrcUdpDataSharingNl::objectMarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array)
{
  std::unique_lock<std::mutex> cwp_lock(status_data_mtx_);
  bool found = false;
  if (marker_array->markers.empty()){
    tracking_strike_ = 0;
    return;
  }
  visualization_msgs::MarkerArray tmp_array = {};
  for (size_t i = 0; i < marker_array->markers.size(); i++){
    if (marker_array->markers[i].pose.position.x > 0){
      tmp_array.markers.push_back(marker_array->markers[i]);
    }
  }
  if (tmp_array.markers.empty()){
    tracking_strike_ = 0;
    tracked_available_ = false;
    lead_ = {};
    return;
  }
  if (!found and tracked_available_ and tracking_strike_ >= 8) {
    for (size_t i = 0; i < tmp_array.markers.size(); i++){
      if (tmp_array.markers[i].id == lead_.id){
        lead_= tmp_array.markers[i];
        tracking_strike_ = std::max({tracking_strike_ + 1, 10});
        found = true;
        break;
      }
    }
  } 
  if (!found and tracked_available_) {
    double nearest = ApsrcUdpDataSharingNl::dist_2(lead_, tmp_array.markers[0]);
    for (size_t i = 1; i < tmp_array.markers.size(); i++){
      if (ApsrcUdpDataSharingNl::dist_2(lead_, tmp_array.markers[i]) < nearest){
        if (lead_.id == tmp_array.markers[i].id){
          tracking_strike_++;
        }
        lead_ = tmp_array.markers[i];
        found = true;
      }
    }
  } 
  if (!found) {
    lead_ = tmp_array.markers[0];
    for (size_t i = 0; i < tmp_array.markers.size(); i++){
      if (tmp_array.markers[i].pose.position.x < lead_.pose.position.x){
        lead_ = tmp_array.markers[i];
      }
    }
    tracked_available_ = true;
    tracking_strike_ = 0;
  }
  
  lead_.type = visualization_msgs::Marker::CYLINDER;
  lead_car_pub_.publish(lead_);
  received_estimation_ = true;
  
  tfBuffer_.transform(lead_, lead_, "map");
}

void ApsrcUdpDataSharingNl::udpReceivedCommandCallback(const apsrc_msgs::CommandReceived::ConstPtr& command_received)
{
  std::unique_lock<std::mutex> rc_lock(udp_mtx_);
  apsrc_msgs::CommandReceived command = *command_received;
  if (command.request_id == 1){
    ApsrcUdpDataSharingNl::UDPFullPathShare();
  }
}

void ApsrcUdpDataSharingNl::backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& back_plane_marker)
{
  std::unique_lock<std::mutex> wf_lock(wf_mtx_);
  marker_ = *back_plane_marker;
  received_marker_ = true;
  if (back_plane_marker->pose.position.x <= roi_long_ and std::abs(back_plane_marker->pose.position.y) <= roi_lat_){
    lidar_perception_score_ = std::min(min_score_, lidar_perception_score_ + 1);
  } else {
    lidar_perception_score_ = 0;
  }
}

void ApsrcUdpDataSharingNl::vehicleHeadingCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  std::unique_lock<std::mutex> vs_lock(status_data_mtx_);
  vehicle_heading_ = static_cast<float>(tf::getYaw(imu->orientation));
}

void ApsrcUdpDataSharingNl::obstacleCallback(const visualization_msgs::Marker::ConstPtr& obstacle)
{
  std::unique_lock<std::mutex> obs_lock(status_data_mtx_);
  obstacle_ = *obstacle;
  received_obstacle_ = true;
}

void ApsrcUdpDataSharingNl::obstacleWaypointsCallback(const std_msgs::Int32::ConstPtr& obstacle_wp)
{
  std::unique_lock<std::mutex> obs_wp_lock(status_data_mtx_);
  obstacle_waypoints_ = obstacle_wp->data;
  received_obstacle_waypoints_ = true;
}

void ApsrcUdpDataSharingNl::radarTrackCallback(const apsrc_msgs::EsrValid::ConstPtr& radar_track)
{
  std::unique_lock<std::mutex> radar_tracking_lock(status_data_mtx_);
  radar_track_ = *radar_track;
  received_radar_track_ = true;
}

}
PLUGINLIB_EXPORT_CLASS(apsrc_udp_data_sharing::ApsrcUdpDataSharingNl, nodelet::Nodelet);