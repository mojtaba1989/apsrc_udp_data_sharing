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


// Util functions
bool openConnection();
void timerCallback();
void UDPStatusShare();
void UDPGlobalPathShare();


// Nodehandles
ros::NodeHandle nh_, pnh_;

// Subscribers
ros::Subscriber current_velocity_sub_, vehicle_status_sub_, closest_waypoint_sub_, base_waypoints_sub_;

// Internal State
AS::Network::UDPInterface udp_interface_;
std::mutex waypoints_mtx_, status_data_mtx_, msg_mts_;
ApsUDPMod::Message message_ = {};
uint8_t msg_id_ = 0;

// Current base_waypoints
bool received_base_waypoints_ = false;
autoware_msgs::Lane base_waypoints_;

// Current velocity of the vehicle (mm/s)
uint16_t current_velocity_ = 0;

// DBW in manual or autonomy
bool dbw_engaged_ = false;

// Closest global waypoint id
int32_t closest_waypoint_id_ = 0;

// Parameters
std::string destination_ip_;
int destination_port_;
double duration_;
};

}
#endif //APSRC_UDP_DATA_SharingNl_H