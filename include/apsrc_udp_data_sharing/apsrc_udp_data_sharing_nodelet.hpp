#ifndef APSRC_UDP_DATA_SharingNl_H
#define APSRC_UDP_DATA_SharingNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/serialization.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>


#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <apsrc_msgs/CommandAccomplished.h>
#include <apsrc_msgs/LatLonOffsets.h>
#include <apsrc_msgs/CommandReceived.h>
#include <sensor_msgs/Imu.h>
#include <apsrc_msgs/EsrValid.h>

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

tf2_ros::Buffer tfBuffer_;


// Sunscriber Callbacks
void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& base_waypoints);
void closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id);
void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status);
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity);
void udpReceivedReportCallback(const apsrc_msgs::CommandAccomplished::ConstPtr& command_accomplished);
void udpReceivedCommandCallback(const apsrc_msgs::CommandReceived::ConstPtr& command_received);
void objectMarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array);
void backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& marker);
void vehicleHeadingCallback(const sensor_msgs::Imu::ConstPtr& imu);
void obstacleCallback(const visualization_msgs::Marker::ConstPtr& obstacle);
void obstacleWaypointsCallback(const std_msgs::Int32::ConstPtr& obstacle_wp);
void radarTrackCallback(const apsrc_msgs::EsrValid::ConstPtr& radar_track);

// Util functions
bool openConnection();
void UDPDataSharingGeneral();
void UDPStatusShare();
void UDPGlobalPathShare();
void UDPBackPlaneEstimationShare();
void UDPReportShare();
void UDPFullPathShare();
void UDPExtar();
void UDPRadar();


// Nodehandles
ros::NodeHandle nh_, pnh_;

// Subscribers
ros::Subscriber current_velocity_sub_;
ros::Subscriber vehicle_status_sub_;
ros::Subscriber closest_waypoint_sub_;
ros::Subscriber base_waypoints_sub_;
ros::Subscriber udp_report_sub_;
ros::Subscriber udp_request_sub_;
ros::Subscriber obj_mrk_array_sub_;
ros::Subscriber backplane_marker_sub_;
ros::Subscriber vehicle_heading_sub_;
ros::Subscriber obstacle_sub_;
ros::Subscriber obstacle_waypoints_sub_;
ros::Subscriber radar_track_sub_;

// Publisher
ros::Publisher lead_car_pub_;


// Internal State
AS::Network::UDPInterface udp_interface_;
std::mutex waypoints_mtx_;
std::mutex status_data_mtx_;
std::mutex udp_mtx_;
std::mutex wf_mtx_;
std::mutex msg_mtx_;
ApsUDPMod::Message_general message_ = {};
uint8_t msg_id_                     = 0;

// Current base_waypoints
bool received_base_waypoints_   = false;
autoware_msgs::Lane base_waypoints_;
bool full_path_has_been_shared_ = false;
size_t last_shared_id_          = 0;

// Current velocity of the vehicle (mm/s)
uint16_t current_velocity_ = 0;

// Report
apsrc_msgs::CommandAccomplished report_ = {};
bool report_received_                   = false; 

// Vehicle Following
visualization_msgs::Marker lead_    = {};
visualization_msgs::Marker marker_  = {};
bool received_marker_               = false;
bool tracked_available_             = false;
uint8_t tracking_strike_            = 0;
bool received_estimation_           = false;
uint8_t lidar_perception_score_     = 0;
double lead_speed_ = 0;

// Autoware messages
visualization_msgs::Marker obstacle_  = {};
bool received_obstacle_               = false;
float lat_offset_from_waypoints      = 0;
int32_t obstacle_waypoints_           = {};
bool received_obstacle_waypoints_     = false;


// Radar Tracking
apsrc_msgs::EsrValid radar_track_ = {};
bool received_radar_track_        = false;



// DBW in manual or autonomy
bool dbw_engaged_ = false;

// Closest global waypoint id
int32_t closest_waypoint_id_ = 0;

// Vehicle heading
float vehicle_heading_ = 0;

// Parameters
std::string destination_ip_;
int destination_port_;
double frequency_;
bool waypoint_only_;
double roi_long_;
double roi_lat_;
int min_score_;
int path_eval_size_;

// Utils
double dist_2(visualization_msgs::Marker first, visualization_msgs::Marker second)
{
  double dx = first.pose.position.x - second.pose.position.x;
  double dy = first.pose.position.y - second.pose.position.y;
  double dz = first.pose.position.z - second.pose.position.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

float dist_to_waypoints()
{
  size_t target_id = closest_waypoint_id_ + obstacle_waypoints_;
  if (base_waypoints_.waypoints.size() < target_id){
    return 0;
  }
     
  double yaw = tf::getYaw(base_waypoints_.waypoints[target_id].pose.pose.orientation);
  double dx  = base_waypoints_.waypoints[target_id].pose.pose.position.x - obstacle_.pose.position.x;
  double dy  = base_waypoints_.waypoints[target_id].pose.pose.position.y - obstacle_.pose.position.y;
  double d   = dy * std::cos(yaw) - dx * std::sin(yaw);

  return static_cast<float>(d);
}

double path_curvature_score(size_t num_of_wp)
{
  if (base_waypoints_.waypoints.size() < (closest_waypoint_id_ + num_of_wp)){
    return 0;
  }
  double dx = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.x - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.x;
  double dy = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.y - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.y;
  double dz = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.z - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.z;

  return sqrt(dx*dx + dy*dy + dz*dz)/num_of_wp;
}

double mean(const std::vector<double>& v) {
    double sum = 0.0;
    for (double value : v) {
        sum += value;
    }
    return sum / v.size();
}

double R2(const std::vector<double>& x, const std::vector<double>& y) {
    double x_mean = mean(x);
    double y_mean = mean(y);

    double numerator    = 0.0;
    double denominator1 = 0.0;
    double denominator2 = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        numerator += (x[i] - x_mean) * (y[i] - y_mean);
        denominator1 += (x[i] - x_mean) * (x[i] - x_mean);
        denominator2 += (y[i] - y_mean) * (y[i] - y_mean);
    }

    double correlation_coefficient = numerator / sqrt(denominator1 * denominator2);

    return correlation_coefficient * correlation_coefficient;
}

};
}
#endif //APSRC_UDP_DATA_SharingNl_H
