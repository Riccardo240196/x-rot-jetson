#ifndef VFH_CONTROLLER_H_
#define VFH_CONTROLLER_H_

//some useful stuff
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <numeric>
#include <chrono>

#include <ros/ros.h> 
#include <tf/tf.h>

//message types 
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float64MultiArray.h> 
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/Range.h> 
#include <radar_pa_msgs/radar_msg.h> 
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <x_rot_control/x_rot_controlConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 

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
    void local_planner_pub();
private:
    ros::NodeHandle nh_; 
    ros::Subscriber robot_pose_sub_,robot_pose_sub_2; 
    ros::Subscriber path_point_sub_; 
    ros::Subscriber trajectory_sub_; 
    ros::Subscriber radar_points_sub_,radar_points_sub_2;
    ros::Subscriber sonar_dx_sub,sonar_sx_sub; 
    ros::Publisher  local_planner_pub_;
    ros::Publisher  cloud_pub_;
    ros::Publisher  overall_cost_pub_;
    ros::Publisher  debug_pub_;
    ros::Publisher  goal_pose_pub_, ref_dir_pose_pub_; 
    dynamic_reconfigure::Server<x_rot_control::x_rot_controlConfig> server;
    dynamic_reconfigure::Server<x_rot_control::x_rot_controlConfig>::CallbackType f;

    bool stop_mode;

    double robot_pose_x;                    // robot x in MAP frame (GPS) [m]
    double robot_pose_y;                    // robot y in MAP frame (GPS) [m]
    double robot_pose_theta;                // robot yaw in MAP frame (GPS) [rad]
    // persistency map parameters
    vector<double> meas_raw_x;              // vector of raw x measures extracted from radar sensor [m]
    vector<double> meas_raw_y;              // vector of raw y measures extracted from radar sensor [m]
    vector<double> meas_x_filtered;         // vector of filtered x measures [m]
    vector<double> meas_y_filtered;         // vector of filtered y measures [m]
    vector< vector<double> > pers_map;      // persistency matrix: 
                                                // 1st column: x measures [m]
                                                // 2nd column: y measures [m]    
                                                // 3rd column: last time in which each measure was updated (used to erase old measures)
                                                // 4th column: number of times each point was detected (used to confirm measures)
    vector< int > close_points;             // vector that contains points closer to each measure
    int pers_index;        // NON PIù USATO??
    double pers_time_th;                    // time threshold used to delete old measures 
    double pers_dist_th_same;               // distance threshold used to find closest point
    double pers_dist_th;                    // distance threshold used to determine close points  
    int consensus_th;                       // consensus threshold used to confirm measures 
    // local planner parameters - main
	double ref_direction;                   // target direction (GOAL) [deg]. When local planner has not control of the robot its value is 0.
	double lateral_dist;                    // lateral distance with respect to GOAL [m]. When local planner has not control of the robot its value is -1.
    float lateral_dist_sign;                // lateral distance sign [-]. When local planner has not control of the robot its value is 0.
    double dist_from_point;                 // euclidean distance from GOAL [m]. When local planner has not control of the robot its value is -1.
    bool path_point_received;               // TRUE when GOAL is received from CAN. FALSE when the GOAL is reached.
    bool path_point_requested;              // TRUE when GOAL is requested to gloabl panner, FALSE otherwise.
    bool ctrl_word;                         // TRUE when the measured distance is below 'max_detection_dist' or path_point_received is TRUE. FALSE when the GOAL is reached.
    bool alarm_on;                          // TRUE when the measured distance is below 'max_detection_dist'. FALSE when the GOAL is reached.
    bool stop;                              // TRUE when the measured distance is below 'stop_distance'. FALSE when the GOAL is reached.
    float max_detection_dist;               // distance that triggers the avoidance manouver (defined in vehicle frame) [m]
    float max_angle_dist;                   // max angle at which 'min_dist' should be to actually triggers the avoidance manouver [deg]
                                                // toghether with 'max_detection_dist' determine the frontal cone area in which an object should be to trigger the avoidance.
    float stop_distance;                    // minimum distance to stop the vehicle (defined in vehicle frame) [m]
    double min_dist;                        // minimum measured distance (defined in vehicle frame) [m]
	// local planner parameters - direction
    double direction;                       // direction chosen [deg]. When local planner has not control of the robot its value is 0.
    double prev_direction;                  // previous direction [deg]
    double direction_gain;                  // gain of the proportional controller used to steer the robot.
    double direction_gain_multi;            // multiplier of the direction gain function of ref speed.
    double direction_gain_offset;           // offset of the direction gain function of ref speed.
    double speed_gain_multi;                // multiplier of the speed gain function of ref speed.
    double speed_gain_offset;               // offset of the speed gain function of ref speed.
    double direction_speed_lim;             // limit of the possible robot turning speed [deg/s]
	// local planner parameters - linear speed
    float speed_upper_lim;                  // linear speed upper limit [m/s]
    float speed_lower_lim;                  // linear speed lower limit [m/s]
	double speed_cmd;                       // linear speed command [m/s]
    double speed_cmd_prev;                  // previous linear speed command [m/s]
    double linear_accel_lim;                // limit of the possible robot linear acceleration [m/s^2]
    float speed_gain;                       // gain of the linear speed.
    // local planner parameters - costs
	int num_of_sector = 180;                // number of sectors
    int window_size_param = 10;                 // parameter to select window size in obst cost propagation. it is multiplied by num_of_sectr/360
    float sector_width;                     // width of each sector [deg]. sector_width=360/num_of_sector.
    float obstacle_weight;                  // weight of the obstacle cost
    float target_dir_weight;                // weight of the target direction cost
    float prev_dir_weight;                  // weight of the previous direction cost
    float inflation_radius;                 // radius of the circle that is added to each measured point [m]
    std::vector<float> sector_limits_up;    // vector of sectors upper limits [deg]
    std::vector<float> sector_limits_down;  // vector of sectors lower limits [deg] 
    // local planner parameters - directions weights inversion
    double weight_inversion_lat_dist;       // lateral distance thresholds that triggers the 'target_dir_weight' and 'prev_dir_weight' inversion [m]
    bool weights_inverted;                  // TRUE if 'lateral_dist>weight_inversion_lat_dist'. FALSE when the GOAL is reached. 
    // local planner parameters - goal point
    vector< vector<double> > path_points;   // PATH from max_detection_dist to GOAL [m] MAP frame
    double goal_x;                          // GOAL x position in MAP frame [m]
    double goal_y;                          // GOAL y position in MAP frame [m]
    float angle_to_goal_th;                 // angle threshold that toghether with 'lateral_dist_th' determine the END of the avoidance manouver. [m]
    float lateral_dist_th;                  // distance threshold that toghether with 'dist_to_goal_th' determine the END of the avoidance manouver. [m]
    double path_direction;
    float goal_dist_th;
    float prev_goal_index;
    double dist_traj_end;
    // Transformation matrices 
    Matrix<double, 3, 3> vehicle_to_radar;  // vehicle to radar transformation matrix
    Matrix<double, 3, 3> map_to_vehicle;    // map to vehicle transformation matrix
    Matrix<double, 3, 3> vehicle_to_sonar_dx;  // vehicle to radar transformation matrix
    Matrix<double, 3, 3> vehicle_to_sonar_sx;  // vehicle to radar transformation matrix
    // verbose - DEBUG
    bool verbose;
    // ROS messages - DEBUG
    geometry_msgs::PoseStamped ref_dir_pose, goal_pose;
    sensor_msgs::PointCloud2 debug_map;
    pcl::PointCloud<pcl::PointXYZ> debug_cloud;
    //ROS messages - sonar sensors
    sensor_msgs::Range sonar_dx, sonar_sx;
    // sonar parameters FOV
    float sonar_hor_arc_lenght = 1;
    float sonar_hor_res = 0.1;
    float sonar_dx_angle = 0.1;
    float sonar_sx_angle = -0.1;
    vector<double> meas_raw_sonar_dx_x;
    vector<double> meas_raw_sonar_dx_y;
    vector<double> meas_raw_sonar_sx_x;
    vector<double> meas_raw_sonar_sx_y;
    int boundaries[2];

    // Methods to initialize publishers and subscribers
    void initializeSubscribers(); 
    void initializePublishers();
    // Methods to get robot pose, radar points and goal point 
    void robotPoseCallback(const nav_msgs::Odometry& msg); 
    void robotPoseCallback_2(const nav_msgs::Odometry& msg); 
    void pathPointCallback(const nav_msgs::Odometry& msg); 
    void trajectoryCallback(const nav_msgs::Path& msg); 
    void radarPointsCallback(const radar_pa_msgs::radar_msg& msg); 
    void radarPointsCallback_2(const sensor_msgs::LaserScan& msg); 
    void sonarDxCallback(const sensor_msgs::Range& msg);
    void sonarSxCallback(const sensor_msgs::Range& msg);
	// Methods for vector field histogram controller
    void normpdf(const std::vector<int>& sector_array, int sector_index, double gaussian_weight, std::vector<double>& norm_distribution);
    int findSectorIdx(double angle);
	void buildCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight, int w1, double w2);
	double findGaussianWeight(double coeff[]);
    int search_closest(const std::vector<int>& sorted_array, int value);
    void update_goal_position();
    // Methods for dynamic reconfiguration of parameters
    void reconfigureCallback(x_rot_control::x_rot_controlConfig &config, uint32_t level);
   
}; 

#endif  




