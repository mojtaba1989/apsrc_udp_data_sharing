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

  // Subscribers & Publishers
  if (gps_v2x_ && waypoint_only_){
    ROS_ERROR("More than one service requested from a single node");
    ros::requestShutdown();
  }

  if (waypoint_only_){
    here_app_sub_ = nh_.subscribe("here/route", 1, &ApsrcUdpDataSharingNl::hereAppCallback, this);
    ROS_INFO("Interface to HERE map enabled");
    goto EXIT;
  }

  if (gps_v2x_){
    gps_sub_                = nh_.subscribe("/gps/gps", 10, &ApsrcUdpDataSharingNl::gpsMKxCallback, this);
    current_velocity_sub_   = nh_.subscribe("/current_velocity", 1, &ApsrcUdpDataSharingNl::velocityCallback, this);
    imu_sub_                = nh_.subscribe("/current_pose", 10, &ApsrcUdpDataSharingNl::imuCallback, this);
    ROS_INFO("Interface to MKx enabled");
    goto EXIT;
  }
  
  base_waypoints_sub_     = nh_.subscribe("base_waypoints", 1, &ApsrcUdpDataSharingNl::baseWaypointCallback, this);
  closest_waypoint_sub_   = nh_.subscribe("closest_waypoint", 1, &ApsrcUdpDataSharingNl::closestWaypointCallback, this);
  timer_                  = nh_.createTimer(ros::Duration(1/frequency_), std::bind(&ApsrcUdpDataSharingNl::UDPDataSharingGeneral, this));
  SPaTnMAP_sub_           = nh_.subscribe("/v2x/SPaTnMAP", 1, &ApsrcUdpDataSharingNl::spatnmapCallback, this);
  udp_report_sub_         = nh_.subscribe("apsrc_udp/received_commands_report", 1, &ApsrcUdpDataSharingNl::udpReceivedReportCallback, this);
  backplane_marker_sub_   = nh_.subscribe("backplane_estimation/backplane_filtered_marker", 1, &ApsrcUdpDataSharingNl::backPlaneMarkerCallback, this);
  lead_vehicle_sub_       = nh_.subscribe("/lead_vehicle/track", 1, &ApsrcUdpDataSharingNl::leadVehicleCallback, this);
  gps_sub_                = nh_.subscribe("/gps/gps", 10, &ApsrcUdpDataSharingNl::gpsCallback, this);


EXIT: //check connection before exit
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
  pnh_.param("path_eval_size", path_eval_size_, 20);

  ROS_INFO("Parameters Loaded");
  return;
}

void ApsrcUdpDataSharingNl::UDPDataSharingGeneral()
{
  if (udp_interface_.is_open()){
    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    udp_interface_.write(message_.pack());
    message_ = {};
    message_.header.info[0] = 0;
    UDPReportShare();
    UDPGlobalPathShare();
    message_.header.msg_id = ++msg_id_;
    {
      std::unique_lock<std::mutex> gps_lock(gps_mtx_);
      message_.header.respond_stamp[0] = gps_.header.stamp.sec;
      message_.header.respond_stamp[1] = gps_.header.stamp.nsec;
    }
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

void ApsrcUdpDataSharingNl::UDPGlobalPathShare()
{
  if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
    int32_t start_id = closest_waypoint_id_;
    autoware_msgs::Lane temp_waypoints = base_waypoints_;

    std::unique_lock<std::mutex> msg_lock(msg_mtx_);
    message_.waypoints_array_msg.closest_global_waypoint_id = closest_waypoint_id_;
    message_.waypoints_array_msg.num_waypoints = (temp_waypoints.waypoints.size() - start_id < 100) ?
            temp_waypoints.waypoints.size() - start_id : 100;
    for (uint i = 0; i < message_.waypoints_array_msg.num_waypoints; i++) {
      uint wp_id = i + start_id;
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
    message_.waypoints_array_msg.path_curvature_score = ApsrcUdpDataSharingNl::path_curvature_score(path_eval_size_);
  }
}

void ApsrcUdpDataSharingNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> cwp_lock(cwp_mtx_);
  closest_waypoint_id_ = closest_waypoint_id->data;
}

void ApsrcUdpDataSharingNl::baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints)
{
  std::unique_lock<std::mutex> wp_lock(wp_mtx_);
  base_waypoints_ = *base_waypoints;
  received_base_waypoints_ = true;
}

void ApsrcUdpDataSharingNl::gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
{
    std::unique_lock<std::mutex> gps_lock(gps_mtx_);
    gps_ = *msg;
}

void ApsrcUdpDataSharingNl::udpReceivedReportCallback(const apsrc_msgs::CommandAccomplished::ConstPtr& command_accomplished)
{
  std::unique_lock<std::mutex> udp_lock(udp_mtx_);
  report_ = *command_accomplished;
  report_received_ = true;
}

void ApsrcUdpDataSharingNl::backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  std::unique_lock<std::mutex> bp_lock(msg_mtx_);
  if (message_.lead_msg.detected % 2 == 0){
    message_.lead_msg.detected += 1;
  } 

  message_.lead_msg.gap_lat = msg->pose.position.y;
  message_.lead_msg.gap_lng = msg->pose.position.x - msg->scale.x/2;
  message_.lead_msg.scale_x = msg->scale.x;
  message_.lead_msg.scale_y = msg->scale.y;
}

void ApsrcUdpDataSharingNl::leadVehicleCallback(const apsrc_msgs::LeadVehicle::ConstPtr& msg)
{
  std::unique_lock<std::mutex> ld_lock(msg_mtx_);
  if (msg->lead_detected){
    if (message_.lead_msg.detected < 2){
      message_.lead_msg.detected += 2;
    }
    message_.lead_msg.radar_lng = msg->range;
    message_.lead_msg.lead_vel_mps_abs = msg->speed_mps;
    message_.lead_msg.lead_vel_mps_rel = msg->relative_speed_mps;
  }
}

void ApsrcUdpDataSharingNl::spatnmapCallback(const apsrc_msgs::SPaTnMAP::ConstPtr& msg)
{
  std::unique_lock<std::mutex> bp_lock(msg_mtx_);
  message_.spat_msg.distance = msg->distance_to_stop;
  message_.spat_msg.phase = msg->phase;
  message_.spat_msg.time_to_change = msg->time_to_stop;
  message_.spat_msg.waypoint_id = msg->stop_waypoint;
}

void ApsrcUdpDataSharingNl::hereAppCallback(const apsrc_msgs::Response::ConstPtr& msg)
{
  ApsUDPMod::Here_Msg udp_msg;
  uint8_t number_of_routes = msg->routes.size();
  for (int ridx = 0; ridx < number_of_routes; ridx++){
    ros::Duration(1.0).sleep();
    udp_msg = {};
    udp_msg.route_id = ridx + 1;
    udp_msg.total_number_of_routes = number_of_routes;
    udp_msg.number_of_waypoint = static_cast<uint32_t>(msg->routes[ridx].waypoints.size()/3);
    if (udp_msg.number_of_waypoint > 2665){
      ROS_WARN("Number of Waypoints exceeded max capacity! Extra points dropped ...");
      udp_msg.number_of_waypoint = static_cast<uint32_t>(2665);
    }
    udp_msg.length = static_cast<uint32_t>(msg->routes[ridx].distance);
    udp_msg.duration = static_cast<uint32_t>(msg->routes[ridx].duration);
    udp_msg.base_duration = static_cast<uint32_t>(msg->routes[ridx].baseDuration);
  
    size_t max_id = udp_msg.number_of_waypoint;
    for (size_t idx = 0; idx < max_id; idx++){
      udp_msg.here_waypoints[idx].lat = msg->routes[ridx].waypoints[idx * 3];
      udp_msg.here_waypoints[idx].lng = msg->routes[ridx].waypoints[idx * 3 + 1];
      udp_msg.here_waypoints[idx].elv = msg->routes[ridx].waypoints[idx * 3 + 2];
    }

    for (size_t span_id = 0; span_id < msg->routes[ridx].spans.spans.size(); span_id++){
      if (msg->routes[ridx].spans.spans[span_id].offset < 2665){
        udp_msg.here_waypoints[msg->routes[ridx].spans.spans[span_id].offset].speed = msg->routes[ridx].spans.spans[span_id].trafficSpeed; 
        udp_msg.here_waypoints[msg->routes[ridx].spans.spans[span_id].offset].base_speed = msg->routes[ridx].spans.spans[span_id].baseSpeed;
      }
    }

    for (size_t span_id = 1; span_id < max_id; span_id++){
      if (udp_msg.here_waypoints[span_id].speed == -1){
        udp_msg.here_waypoints[span_id].speed = udp_msg.here_waypoints[span_id -1].speed;
        udp_msg.here_waypoints[span_id].base_speed = udp_msg.here_waypoints[span_id -1].base_speed;
      }
    }

    for (size_t action_id = 0; action_id < msg->routes[ridx].actions.actions.size(); action_id++){
      if (msg->routes[ridx].actions.actions[action_id].offset < 2665){
        std::string lower_instruction = toLowerCase(msg->routes[ridx].actions.actions[action_id].instruction);
        if (lower_instruction.find("ramp") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 2;
        } else  if (lower_instruction.find("turn left") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 3;
        } else  if (lower_instruction.find("turn right") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 4;
        } else  if (lower_instruction.find("exit") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 5;
        } else  if (lower_instruction.find("roundabout") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 6;
        } else  if (lower_instruction.find("u-turn") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 7;
        } else  if (lower_instruction.find("continue") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 8;
        } else  if (lower_instruction.find("keep") != std::string::npos){
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 8;
        } else {
          udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 1;
        }
      }
    }

    udp_interface_.write(udp_msg.pack());
    ROS_INFO("HERE map route (%d out of %d) has been shared", ridx + 1, number_of_routes);
  }
}

void ApsrcUdpDataSharingNl::imuCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::unique_lock<std::mutex> imu_lock(udp_mtx_);
  MKx_msg_.yaw = M_PI/2 + tf::getYaw(msg->pose.orientation);
}

void ApsrcUdpDataSharingNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> udp_lock(udp_mtx_);
  MKx_msg_.speed = current_velocity->twist.linear.x;
}

void ApsrcUdpDataSharingNl::gpsMKxCallback(const gps_common::GPSFix::ConstPtr& msg)
{
  std::unique_lock<std::mutex> cv_lock(udp_mtx_);
  MKx_msg_.latitude   = static_cast<float>(msg->latitude);
  MKx_msg_.longitude  = static_cast<float>(msg->longitude);
  MKx_msg_.elevation  = static_cast<float>(msg->altitude);
  MKx_msg_.time_ms    = static_cast<uint64_t>((msg->header.stamp.sec + msg->header.stamp.nsec)*1000);
  udp_interface_.write(MKx_msg_.pack());
  MKx_msg_ = {};
}


}
PLUGINLIB_EXPORT_CLASS(apsrc_udp_data_sharing::ApsrcUdpDataSharingNl, nodelet::Nodelet);