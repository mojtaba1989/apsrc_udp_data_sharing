#ifndef APSRC_UDP_DATA_SharingNl_H
#define APSRC_UDP_DATA_SharingNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/serialization.h>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <tf2_ros/transform_listener.h>


#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <apsrc_msgs/CommandAccomplished.h>
#include <apsrc_msgs/CommandReceived.h>
#include <apsrc_msgs/LeadVehicle.h>
#include <apsrc_msgs/Response.h>

#include "apsrc_udp_data_sharing/packet_definitions.hpp"



namespace apsrc_udp_data_sharing 
{
class ApsrcUdpDataSharingNl : public nodelet::Nodelet
{
public:
  ApsrcUdpDataSharingNl();
  ~ApsrcUdpDataSharingNl();

private:
virtual void onInit();
void loadParams();
ros::Timer timer_;

// Sunscriber Callbacks
void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints);
void closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id);
void udpReceivedReportCallback(const apsrc_msgs::CommandAccomplished::ConstPtr& command_accomplished);
void udpReceivedCommandCallback(const apsrc_msgs::CommandReceived::ConstPtr& command_received);
void backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
void leadVehicleCallback(const apsrc_msgs::LeadVehicle::ConstPtr& msg);
void hereAppCallback(const apsrc_msgs::Response::ConstPtr& msg);

// Util functions
bool openConnection();
void UDPDataSharingGeneral();
void UDPGlobalPathShare();
void UDPReportShare();

// Nodehandles
ros::NodeHandle nh_, pnh_;

// Subscribers
ros::Subscriber current_velocity_sub_;
ros::Subscriber closest_waypoint_sub_;
ros::Subscriber base_waypoints_sub_;
ros::Subscriber udp_report_sub_;
ros::Subscriber udp_request_sub_;
ros::Subscriber backplane_marker_sub_;
ros::Subscriber lead_vehicle_sub_;
ros::Subscriber here_app_sub_;

// Internal State
AS::Network::UDPInterface udp_interface_;
std::mutex waypoints_mtx_;
std::mutex status_data_mtx_;
std::mutex udp_mtx_;
std::mutex msg_mtx_;
ApsUDPMod::Message_general message_ = {};
uint8_t msg_id_                     = 0;

// Current base_waypoints
bool received_base_waypoints_   = false;
autoware_msgs::Lane base_waypoints_;
bool full_path_has_been_shared_ = false;
size_t last_shared_id_          = 0;

// Report
apsrc_msgs::CommandAccomplished report_ = {};
bool report_received_                   = false; 

// Closest global waypoint id
int32_t closest_waypoint_id_ = 0;

// Parameters
std::string destination_ip_;
int destination_port_;
double frequency_;
bool waypoint_only_;
int path_eval_size_;

// HERE Actions
std::string ramp__        = "ramp";
std::string turn_left__   = "turn left";
std::string turn_right__  = "turn right";
std::string exit__        = "exit";
std::string roundabout__  = "roundabout";
std::string uturn__       = "u-turn";
std::string continue__    = "continue";
std::string keep__        = "keep";

// Utils
float path_curvature_score(size_t num_of_wp)
{
  if (base_waypoints_.waypoints.size() < (closest_waypoint_id_ + num_of_wp)){
    return 0;
  }
  double dx = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.x - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.x;
  double dy = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.y - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.y;
  double dz = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.z - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.z;

  return static_cast<float>(sqrt(dx*dx + dy*dy + dz*dz)/num_of_wp);
}

std::string toLowerCase(const std::string& str) {
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lowerStr;
}
};
}
#endif //APSRC_UDP_DATA_SharingNl_H
