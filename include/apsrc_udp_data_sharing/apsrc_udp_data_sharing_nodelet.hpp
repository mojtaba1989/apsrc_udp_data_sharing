#ifndef APSRC_UDP_DATA_SharingNl_H
#define APSRC_UDP_DATA_SharingNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/serialization.h>

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
#include <apsrc_msgs/LatLonOffsets.h>
#include <apsrc_msgs/CommandReceived.h>

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
void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status);
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity);
void backPlaneEstimationCallback(const apsrc_msgs::LatLonOffsets::ConstPtr& back_plane_estimation);
void backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& back_plane_estimation);
void udpReceivedReportCallback(const apsrc_msgs::CommandAccomplished::ConstPtr& command_accomplished);
void udpReceivedCommandCallback(const apsrc_msgs::CommandReceived::ConstPtr& command_received);


// Util functions
bool openConnection();
void UDPDataSharingGeneral();
void UDPStatusShare();
void UDPGlobalPathShare();
void UDPBackPlaneEstimationShare();
void UDPReportShare();
void UDPFullPathShare();


// Nodehandles
ros::NodeHandle nh_, pnh_;

// Subscribers
ros::Subscriber current_velocity_sub_;
ros::Subscriber vehicle_status_sub_;
ros::Subscriber closest_waypoint_sub_;
ros::Subscriber base_waypoints_sub_;
ros::Subscriber backplane_estimation_sub_;
ros::Subscriber backplane_marker_sub_;
ros::Subscriber udp_report_sub_;
ros::Subscriber udp_request_sub_;


// Internal State
AS::Network::UDPInterface udp_interface_;
std::mutex waypoints_mtx_;
std::mutex status_data_mtx_;
std::mutex udp_mtx_;
std::mutex wf_mtx_;
std::mutex msg_mtx_;
ApsUDPMod::Message_general message_ = {};
uint8_t msg_id_ = 0;

// Current base_waypoints
bool received_base_waypoints_ = false;
autoware_msgs::Lane base_waypoints_;
bool full_path_has_been_shared_ = false;
size_t last_shared_id_ = 0;

// Current velocity of the vehicle (mm/s)
uint16_t current_velocity_ = 0;

// Report
apsrc_msgs::CommandAccomplished report_ = {};
bool report_received_ = false; 

// Vehicle Following
apsrc_msgs::LatLonOffsets lat_long_offset_;
visualization_msgs::Marker marker_;
bool received_marker_ = false;
bool received_estimation_ = false;

// DBW in manual or autonomy
bool dbw_engaged_ = false;

// Closest global waypoint id
int32_t closest_waypoint_id_ = 0;

// Parameters
std::string destination_ip_;
int destination_port_;
double duration_;
bool waypoint_only_ = false;
};

}
#endif //APSRC_UDP_DATA_SharingNl_H