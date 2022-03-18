#ifndef VFH_CONTROLLER_H_
#define VFH_CONTROLLER_H_

//some useful stuff
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <numeric>

#include <ros/ros.h> 
#include <tf/tf.h>

//message types 
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h> 
#include <radar_pa_msgs/radar_msg.h> 
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace Eigen;
#define _USE_MATH_DEFINES

class VFHController
{
public:
    VFHController(ros::NodeHandle* nodehandle); 
    double node_frequency;
    void update_pers_map();
    void vfhController();
private:
    ros::NodeHandle nh_; 
    ros::Subscriber robot_pose_sub_; 
    ros::Subscriber path_point_sub_; 
    ros::Subscriber radar_points_sub_,radar_points_sub_2; 
    ros::Publisher  local_planner_pub_;
    ros::Publisher  cloud_pub_;
    
    double robot_pose_x; 
    double robot_pose_y; 
    double robot_pose_theta; 
	double ref_direction;
	double lateral_dist;
	int num_of_sector = 180;

    float obstacle_weight = 0.98;
    float target_dir_weight = 0.01;
    float prev_dir_weight = 0.01;
    float max_detection_dist = 8; // [m] distance that triggers the avoidance manouver (defined in vehicle frame)
    float stop_distance = 0.8; // [m] minimum distance to stop the vehicle (defined in vehicle frame)
    float sector_width = 360/(float)num_of_sector;
    float gaussian_weight_coeff = 0.5;
    float robot_radius = 0.6; // [m]
    float speed_upper_lim = 0.5; // [m/s]
    double direction_speed_lim = 5;
    double linear_speed_lim = 1;

	bool ctrl_word;
	double direction;
    double prev_direction;
	double speed_cmd;
    double speed_cmd_prev;

    vector<double> meas_raw_x;
    vector<double> meas_raw_y;
    vector<double> meas_x_filtered;
    vector<double> meas_y_filtered;
    Matrix<double, 3, 3> vehicle_to_radar;
    Matrix<double, 3, 3> map_to_vehicle;

    vector< vector<double> > pers_map;
    vector< int > close_points;
    int pers_index;
    double pers_time_th;
    double pers_dist_th;
    int consensus_th;
    double direction_gain;

    sensor_msgs::PointCloud2 debug_map;
    pcl::PointCloud<pcl::PointXYZ> debug_cloud;
        
    void initializeSubscribers(); 
    void initializePublishers();

    void robotPoseCallback(const nav_msgs::Odometry& msg); 
    void pathPointCallback(const nav_msgs::Odometry& msg); 
    void radarPointsCallback(const radar_pa_msgs::radar_msg& msg); 
    void radarPointsCallback_2(const sensor_msgs::LaserScan& msg); 
    void pathPointFake();
	
    void normpdf(const std::vector<int>& sector_array, int sector_index, double gaussian_weight, std::vector<double>& norm_distribution);
    int findSectorIdx(double angle, float sector_limits_up[]);
	void buildCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight, int w1, double w2);
	double findGaussianWeight(double coeff[]);
   
}; 

#endif  

