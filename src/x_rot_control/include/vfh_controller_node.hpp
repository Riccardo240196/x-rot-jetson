#ifndef VFH_CONTROLLER_H_
#define VFH_CONTROLLER_H_

//some useful stuff
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 
#include <tf/tf.h>

//message types 
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h> 
#include <radar_pa_msgs/radar_msg.h> 
#include <eigen3/Eigen/Dense>

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
    ros::Subscriber radar_points_sub_; 
    ros::Publisher  local_planner_pub_;
    
    double robot_pose_x; 
    double robot_pose_y; 
    double robot_pose_theta; 
	double ref_direction;
	double lateral_dist;
	int num_of_sector = 180;

	bool ctrl_word;
	double direction;
	double min_dist;
	double speed_cmd;

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
        
    void initializeSubscribers(); 
    void initializePublishers();

    void robotPoseCallback(const nav_msgs::Odometry& msg); 
    void pathPointCallback(const nav_msgs::Odometry& msg); 
    void radarPointsCallback(const radar_pa_msgs::radar_msg& msg); 
	
    void normpdf(const std::vector<int>& sector_array, int sector_index, double gaussian_weight, std::vector<double>& norm_distribution);
    int findSectorIdx(double direction, float sector_limits_up[]);
	void buildDirectionCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight);
	    
}; 

#endif  