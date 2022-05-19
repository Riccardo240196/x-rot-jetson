#include "vfh_controller_node.hpp"

// CONSTRUCTOR
VFHController::VFHController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of VFHController");
    initializeSubscribers(); 
    initializePublishers();

    ros::NodeHandle private_nh("~");
    private_nh.param<bool>("stop_mode", stop_mode, false);

    // FOR DETAILS OF THE DEFINITION OF EACH PARAMETER SEE 'vfh_controller_node.hpp'
    node_frequency = 10;
    // persistency map parameters
    pers_index = 0;
    pers_time_th = 5;
    pers_dist_th = 0.5;
    pers_dist_th_same = 0.1;
    consensus_th = 1;
    // local planner parameters - main
    ref_direction = -1;
    lateral_dist = -1;
    dist_from_point = -1;
    path_point_received = false;
    ctrl_word = 0; 
    stop = 0;
    max_detection_dist = 6;
    max_angle_dist = 20;
    stop_distance = 2;
    min_dist = 100;
    // local planner parameters - direction
	direction = 0;
    prev_direction = 0;
    direction_gain = 0.9;
    direction_speed_lim = 10;
    // local planner parameters - linear speed
    speed_upper_lim = 0.5;
	speed_cmd = 0;
    speed_cmd_prev = 0;
    linear_accel_lim = 1;
    speed_lower_lim = 0;
    speed_gain = (speed_upper_lim - speed_lower_lim)/(max_detection_dist - stop_distance);
    // local planner parameters - costs
    num_of_sector = 180;
    sector_width = 360/(float)num_of_sector;
    obstacle_weight = 0.95;
    target_dir_weight = 0.1;
    prev_dir_weight = 0.2;
    inflation_radius = 0.7;
    // local planner parameters - directions weights inversion
    weight_inversion_lat_dist = 0.3;
    weights_inverted = false;
    // local planner parameters - goal point
    goal_x = 0.0;
    goal_y = 0.0;
    angle_to_goal_th = 15; // [m]
    lateral_dist_th = 1; // [m]
    goal_dist_th = 2.5; // [m]
    prev_goal_index = 0;
    // verbse
    verbose = true;
     
    // local planner - initialize sectors limits
    for (int i=0; i<num_of_sector; i++) {
        float sector_mean = i*sector_width;
        sector_limits_up.push_back(sector_mean + sector_width/2);
        sector_limits_down.push_back(sector_mean - sector_width/2);
    }

    // Transformation matrix 'vehicle_to_radar' - initialization
    vehicle_to_radar << 1,0,0.65,
                        0,1,0,
                        0,0,1;
    // 0 0.26 0.06 -0.1 0 0
    //
    
    vehicle_to_sonar_dx<< cos(sonar_dx_angle),-sin(sonar_dx_angle),0.65,
                        sin(sonar_dx_angle),cos(sonar_dx_angle),-0.26,
                        0,0,1;

    vehicle_to_sonar_sx<< cos(sonar_sx_angle),-sin(sonar_sx_angle),0.65,
                        sin(sonar_sx_angle),cos(sonar_sx_angle),0.26,
                        0,0,1;
    // Transformation matrix 'map_to_vehicle' - initialization
    map_to_vehicle = Matrix<double, 3, 3>::Identity();
    

    debug_cloud.height = 1;   

    f = boost::bind(&VFHController::reconfigureCallback, this,_1,_2);
    server.setCallback(f);
        
}

void VFHController::reconfigureCallback(x_rot_control::x_rot_controlConfig &config, uint32_t level) {
    
    // persistency map parameters
    pers_time_th = config.pers_time_th;
    pers_dist_th = config.pers_dist_th;
    pers_dist_th_same = config.pers_dist_th_same;
    consensus_th = config.consensus_th;
    // local planner costs parameters
    num_of_sector = config.num_of_sector;
    obstacle_weight = config.obstacle_weight;
    target_dir_weight = config.target_dir_weight;
    prev_dir_weight = config.prev_dir_weight;
    weight_inversion_lat_dist = config.weight_inversion_lat_dist;
    inflation_radius = config.inflation_radius;
    // local planner START/STOP parameters
    max_detection_dist = config.max_detection_dist;
    max_angle_dist = config.max_angle_dist;
    angle_to_goal_th = config.angle_to_goal_th;
    lateral_dist_th = config.lateral_dist_th;
    goal_dist_th = config.goal_dist_th;
    // linear speed cmd parameters
    stop_distance = config.stop_distance;
    speed_upper_lim = config.speed_upper_lim;
    linear_accel_lim = config.linear_accel_lim;
    // angular speed cmd parameters
    direction_speed_lim = config.direction_speed_lim;
    direction_gain = config.direction_gain;
    // verbose
    verbose = config.verbose;
    
    // local planner variables that need to be updated 
    sector_width = 360/(float)num_of_sector;
    sector_limits_up.clear();
    sector_limits_down.clear();
    for (int i=0; i<num_of_sector; i++) {
        float sector_mean = i*sector_width;
        sector_limits_up.push_back(sector_mean + sector_width/2);
        sector_limits_down.push_back(sector_mean - sector_width/2);
    }
    speed_lower_lim = 0;
    speed_gain = (speed_upper_lim - speed_lower_lim)/(max_detection_dist - stop_distance);

}

void VFHController::initializeSubscribers()
{
    robot_pose_sub_ = nh_.subscribe("/yape/odom_diffdrive", 1, &VFHController::robotPoseCallback,this);
    robot_pose_sub_2 = nh_.subscribe("/can_odometry", 1, &VFHController::robotPoseCallback_2,this);  
    path_point_sub_ = nh_.subscribe("/path_point", 1, &VFHController::pathPointCallback,this);  
    trajectory_sub_ = nh_.subscribe("/trajectory", 1, &VFHController::trajectoryCallback,this);  
    radar_points_sub_ = nh_.subscribe("/radar_messages", 1, &VFHController::radarPointsCallback,this);
    radar_points_sub_2 = nh_.subscribe("/radar", 1, &VFHController::radarPointsCallback_2,this);  
    sonar_dx_sub = nh_.subscribe("/sonar_dx", 1, &VFHController::sonarDxCallback,this);  
    sonar_sx_sub = nh_.subscribe("/sonar_sx", 1, &VFHController::sonarSxCallback,this);  
}

void VFHController::initializePublishers()
{
    local_planner_pub_ = nh_.advertise<geometry_msgs::Twist>("/yape/cmd_vel", 1, true); 
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/map_debug_output", 1);
    overall_cost_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/cost_debug_output", 1);
    debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/var_debug_output", 1);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("/goal_pose",1);
    ref_dir_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("/ref_dir_pose",1);
}

void VFHController::robotPoseCallback(const nav_msgs::Odometry& msg) { // GAZEBO
    robot_pose_x = msg.pose.pose.position.x;
    robot_pose_y = msg.pose.pose.position.y;
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_pose_theta = yaw;

    map_to_vehicle << cos(yaw), -sin(yaw), robot_pose_x,
                      sin(yaw), cos(yaw), robot_pose_y,
                      0, 0, 1;

    if (ctrl_word && robot_pose_x<16.8) {
        goal_x = 17;
        goal_y = 0;
        path_point_received = true;
    }
    else if (ctrl_word && robot_pose_x>17 && robot_pose_x<33.8) {
        goal_x = 34;
        goal_y = 0;
        path_point_received = true;
    }
    else if (ctrl_word && robot_pose_x>34 && robot_pose_x<51.8) {
        goal_x = 52;
        goal_y = 0;
        path_point_received = true;
    }

    if (path_point_received) {
        double delta_x = goal_x - robot_pose_x;
        double delta_y = goal_y - robot_pose_y;
        ref_direction = 180/M_PI*(atan2(delta_y,delta_x) - robot_pose_theta);

        dist_from_point = sqrt(pow(delta_x,2) + pow(delta_y,2));
        double ang = atan2(delta_y,delta_x);
        lateral_dist = abs(dist_from_point*sin(ang));

        if (dist_from_point<0.5) // ADDED JUST FOR GAZEBO SIMULATION
            ref_direction = 180/M_PI*(atan2(delta_y,delta_x+2) - robot_pose_theta); // ADDED JUST FOR GAZEBO SIMULATION

        if (ref_direction<0)
            ref_direction = 360 + ref_direction;     
    }
    else {
        ref_direction = 0;
        lateral_dist = -1;
        dist_from_point = -1;
    }
    
}

void VFHController::robotPoseCallback_2(const nav_msgs::Odometry& msg) {
    // Extract robot pose
    robot_pose_x = msg.pose.pose.position.x;
    robot_pose_y = msg.pose.pose.position.y;
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_pose_theta = yaw;

    // Build transformation matrix 'map_to_vehicle'
    map_to_vehicle << cos(yaw), -sin(yaw), robot_pose_x,
                      sin(yaw), cos(yaw), robot_pose_y,
                      0, 0, 1;

    // Calculate reference direction, lateral distance from path and distance from goal point
    if (path_point_received) {
        double delta_x = goal_x - robot_pose_x;
        double delta_y = goal_y - robot_pose_y;
        ref_direction = 180/M_PI*(atan2(delta_y,delta_x) - robot_pose_theta);

        dist_from_point = sqrt(pow(delta_x,2) + pow(delta_y,2));
        // double ang = atan2(delta_y,delta_x);
        // lateral_dist = abs(dist_from_point*cos(ang));

        double a = -tan(path_direction);
        double c = -a*goal_x - goal_y;
        lateral_dist = abs(robot_pose_x*a + robot_pose_y + c)/sqrt(a*a+1);

        if (ref_direction<0)
            ref_direction = 360 + ref_direction;      
    }
    else {
        ref_direction = 0;
        lateral_dist = -1;
        dist_from_point = -1;
    }

    
}

void VFHController::pathPointCallback(const nav_msgs::Odometry& msg) {

    double path_point_x = msg.pose.pose.position.x;
    double path_point_y = msg.pose.pose.position.y;
    
    goal_x = path_point_x;
    goal_y = path_point_y;

    path_point_received = true;
    // cout<< "path point! "<<path_point_x <<" " <<path_point_y <<endl;
}

void VFHController::trajectoryCallback(const nav_msgs::Path& msg) {

    path_points.clear();
    for (int i=0; i<msg.poses.size(); i++) {
        std::vector<double> xy_coord = {msg.poses[i].pose.position.x, msg.poses[i].pose.position.y};
        path_points.push_back(xy_coord);
    }
    
    // goal_x = path_point_x;
    // goal_y = path_point_y;

    path_point_received = true;

}

void VFHController::radarPointsCallback(const radar_pa_msgs::radar_msg& msg) {
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    // pcl::PointXYZ point;

    meas_raw_x.clear();
    meas_raw_y.clear();

    for(int i = 0; i< msg.data_A.size(); i++){
        if(msg.data_A[i].is_target){
            temp(0) = msg.data_A[i].distance*cos(msg.data_A[i].angle);
            temp(1) = msg.data_A[i].distance*sin(msg.data_A[i].angle);
            temp(2) = 1;
                    
            temp = map_to_vehicle*vehicle_to_radar*temp;

            meas_raw_x.push_back(temp(0));
            meas_raw_y.push_back(temp(1));      
        }
    }  
    
}

void VFHController::radarPointsCallback_2(const sensor_msgs::LaserScan& msg){ // GAZEBO
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    // pcl::PointXYZ point;

    meas_raw_x.clear();
    meas_raw_y.clear();

    double angle_min = msg.angle_min;
    double angle_increment = msg.angle_increment;
    double angle = angle_min;

    for(int i = 0; i< msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            temp(0) = msg.ranges[i]*cos(angle);
            temp(1) = msg.ranges[i]*sin(angle);
            temp(2) = 1;
                    
            temp = map_to_vehicle*vehicle_to_radar*temp;

            meas_raw_x.push_back(temp(0));
            meas_raw_y.push_back(temp(1));  

        }

        angle+=angle_increment; 
    }  
}

void VFHController::sonarDxCallback(const sensor_msgs::Range& msg){
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    Matrix<double, 2, 1> perp_dir, meas_pose, temp_pose;
    // pcl::PointXYZ point;

    meas_raw_sonar_dx_x.clear();
    meas_raw_sonar_dx_y.clear();
    
    temp << msg.range, 0, 1;

    temp = map_to_vehicle*vehicle_to_sonar_dx*temp; // xy coordinates in map frame
    meas_raw_sonar_dx_x.push_back(temp(0));
    meas_raw_sonar_dx_y.push_back(temp(1));
    meas_pose << temp(0), temp(1);

    perp_dir << cos(robot_pose_theta + sonar_dx_angle - M_PI/2), sin(robot_pose_theta + sonar_dx_angle - M_PI/2);

    for(float L = sonar_hor_res; L< (sonar_hor_arc_lenght/2); L+=sonar_hor_res){
        temp_pose = meas_pose + perp_dir*L;
        meas_raw_sonar_dx_x.push_back(temp_pose(0));
        meas_raw_sonar_dx_y.push_back(temp_pose(1));

        temp_pose = meas_pose - perp_dir*L;
        meas_raw_sonar_dx_x.push_back(temp_pose(0));
        meas_raw_sonar_dx_y.push_back(temp_pose(1));
    }
    // cout << "sonar dx size " << meas_raw_sonar_dx_x.size() << endl;
}

void VFHController::sonarSxCallback(const sensor_msgs::Range& msg){
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    Matrix<double, 2, 1> perp_dir, meas_pose, temp_pose;
    // pcl::PointXYZ point;

    meas_raw_sonar_sx_x.clear();
    meas_raw_sonar_sx_y.clear();
    
    temp << msg.range, 0, 1;

    temp = map_to_vehicle*vehicle_to_sonar_sx*temp; // xy coordinates in map frame
    meas_raw_sonar_sx_x.push_back(temp(0));
    meas_raw_sonar_sx_y.push_back(temp(1));
    meas_pose << temp(0), temp(1);

    perp_dir << cos(robot_pose_theta + sonar_sx_angle - M_PI/2), sin(robot_pose_theta + sonar_sx_angle - M_PI/2);

    for(float L = sonar_hor_res; L< (sonar_hor_arc_lenght/2); L+=sonar_hor_res){
        temp_pose = meas_pose + perp_dir*L;
        meas_raw_sonar_sx_x.push_back(temp_pose(0));
        meas_raw_sonar_sx_y.push_back(temp_pose(1));

        temp_pose = meas_pose - perp_dir*L;
        meas_raw_sonar_sx_x.push_back(temp_pose(0));
        meas_raw_sonar_sx_y.push_back(temp_pose(1));
    }
    // cout << "sonar sx size " << meas_raw_sonar_sx_x.size() << endl;
}

void VFHController::update_pers_map(){
    // add radar measurements to pers_map
    for(int i=0;i<meas_raw_x.size(); i++){

        close_points.clear();
        int closest_point = -1;
        double dist;
        double diff_x;
        double diff_y;
        double min_dist = 100;
        // check existence
        for(int ind=0; ind<pers_map.size(); ind++){
            diff_x = pers_map[ind][0] - meas_raw_x[i];
            diff_y = pers_map[ind][1] - meas_raw_y[i];
            dist = sqrt(diff_x*diff_x + diff_y*diff_y);
            if(dist< pers_dist_th ){
                close_points.push_back(ind);
                if(dist<pers_dist_th_same){
                    closest_point = ind;
                }
            }
        }
        
        if(closest_point!=-1){
            vector<double> to_insert = {meas_raw_x[i], meas_raw_y[i], 0, pers_map[closest_point][3]};
            pers_map[closest_point] = to_insert;
        }
        
        if(close_points.size()==0){
            vector<double> to_insert = {meas_raw_x[i], meas_raw_y[i], 0, 0};
            pers_map.push_back(to_insert);
        }
        for (int j=0; j<close_points.size(); j++){
            pers_map[close_points[j]][3]++; 
        }
        
    }
    // cout << "pers map size 1: " << pers_map.size() << endl;

    // add sonar sx measurements to pers_map
    for(int i=0;i<meas_raw_sonar_sx_x.size(); i++){

        close_points.clear();
        int closest_point = -1;
        double dist;
        double diff_x;
        double diff_y;
        double min_dist = 100;
        // check existence
        for(int ind=0; ind<pers_map.size(); ind++){
            diff_x = pers_map[ind][0] - meas_raw_sonar_sx_x[i];
            diff_y = pers_map[ind][1] - meas_raw_sonar_sx_y[i];
            dist = sqrt(diff_x*diff_x + diff_y*diff_y);
            if(dist< pers_dist_th ){
                close_points.push_back(ind);
                if(dist<pers_dist_th_same){
                    closest_point = ind;
                }
            }
        }
        
        if(closest_point!=-1){
            vector<double> to_insert = {meas_raw_sonar_sx_x[i], meas_raw_sonar_sx_y[i], 0, pers_map[closest_point][3]};
            pers_map[closest_point] = to_insert;
        }
        
        if(close_points.size()==0){
            vector<double> to_insert = {meas_raw_sonar_sx_x[i], meas_raw_sonar_sx_y[i], 0, 0};
            pers_map.push_back(to_insert);
        }
        for (int j=0; j<close_points.size(); j++){
            pers_map[close_points[j]][3]++; 
        }
        
    }

    // cout << "pers map size 2: " << pers_map.size() << endl;
    // add sonar dx measurements to pers_map
    for(int i=0;i<meas_raw_sonar_dx_x.size(); i++){

        close_points.clear();
        int closest_point = -1;
        double dist;
        double diff_x;
        double diff_y;
        double min_dist = 100;
        // check existence
        for(int ind=0; ind<pers_map.size(); ind++){
            diff_x = pers_map[ind][0] - meas_raw_sonar_dx_x[i];
            diff_y = pers_map[ind][1] - meas_raw_sonar_dx_y[i];
            dist = sqrt(diff_x*diff_x + diff_y*diff_y);
            if(dist< pers_dist_th ){
                close_points.push_back(ind);
                if(dist<pers_dist_th_same){
                    closest_point = ind;
                }
            }
        }
        
        if(closest_point!=-1){
            vector<double> to_insert = {meas_raw_sonar_dx_x[i], meas_raw_sonar_dx_y[i], 0, pers_map[closest_point][3]};
            pers_map[closest_point] = to_insert;
        }
        
        if(close_points.size()==0){
            vector<double> to_insert = {meas_raw_sonar_dx_x[i], meas_raw_sonar_dx_y[i], 0, 0};
            pers_map.push_back(to_insert);
        }
        for (int j=0; j<close_points.size(); j++){
            pers_map[close_points[j]][3]++; 
        }
        
    }

    // cout << "pers map size 3: " << pers_map.size() << endl;
    meas_x_filtered.clear();
    meas_y_filtered.clear();
    
    pcl::PointXYZ point;
    if (verbose) {
        debug_cloud.points.clear();
        debug_cloud.width = 0;
    }

    Matrix<double, 3, 1> temp;

    for(int i=0; i<pers_map.size(); i++){
        pers_map[i][2]++;
        if(pers_map[i][2]/node_frequency > pers_time_th){
            pers_map.erase(pers_map.begin()+i);
            i--;
        }
        else if(pers_map[i][3]>= consensus_th){
            temp << pers_map[i][0], pers_map[i][1], 1;
            
            temp = map_to_vehicle.inverse()*temp;

            meas_x_filtered.push_back(temp(0));
            meas_y_filtered.push_back(temp(1));            

            if (verbose) {
                point.x = temp(0);
                point.y = temp(1);
                point.z = 0.0;

                debug_cloud.points.push_back(point);
                debug_cloud.width++; 
            }
        }
    }

    if (verbose) {
        pcl::toROSMsg(debug_cloud,debug_map);  
        debug_map.header.frame_id = "chassis";
        debug_map.header.stamp = ros::Time::now();  
        cloud_pub_.publish (debug_map);
    }
}

void VFHController::normpdf(const std::vector<int>& sector_array, int sector_index, double gaussian_weight, std::vector<double>& norm_distribution) {
  double k;
  double t;
  int b_k;
  for (b_k = 0; b_k < num_of_sector; b_k = b_k + 1) {
    k = 1.0 + (((double)b_k) * 1.0);
    if (gaussian_weight > 0.0) {
      t = (sector_array[((int)k) - 1] - sector_index) / gaussian_weight;
      norm_distribution[((int)k) - 1] = exp((-0.5 * t) * t) / (2.5066282746310002 * gaussian_weight);
    } else {
      norm_distribution[((int)k) - 1] = 0.0;
    }
  }
}

int VFHController::findSectorIdx(double angle) {
    int sector_index = num_of_sector;
    if (angle>=sector_limits_up[num_of_sector-1]) {
        sector_index = 0;
    } else {
        for (int j=0; j<num_of_sector; j++) {
            if (sector_limits_up[j]>=angle) {
                sector_index = j;
                break;
            }
        }
    }
    return sector_index;
}

void VFHController::buildCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight, int w1, double w2) {
    double sector_value = sector_index-gaussian_shift;
    int max_idx = -1000;
    for (int j=0; j<num_of_sector; j++) {
        sector_array[j] = sector_value + j ;
        if (sector_array[j]>max_idx)
            max_idx = sector_array[j];
    }
    
    std::vector<double> gaussian_distribution(num_of_sector);
    normpdf(sector_array,sector_index,gaussian_weight,gaussian_distribution);
    if (max_idx < num_of_sector) {
        int start_ind = num_of_sector - max_idx - 1;
        for (int j=0; j<max_idx; j++) {
            cost_vec[j] = w1 + w2*gaussian_distribution[j+start_ind];
        }
        
        for (int j=max_idx; j<num_of_sector; j++) {
            cost_vec[j] = w1 + w2*gaussian_distribution[j-max_idx-1];
        }
    } else {
        int start_ind = max_idx-num_of_sector+1;
        for (int j=0; j<start_ind; j++) {
            cost_vec[j] = w1 + w2*gaussian_distribution[j+(num_of_sector-start_ind)];
        }
        
        for (int j=start_ind; j<num_of_sector; j++) {
            cost_vec[j] = w1 + w2*gaussian_distribution[j-start_ind];
        }
    }
}

double VFHController::findGaussianWeight(double coeff[]) {
    double gaussian_weight = coeff[0] + coeff[1]*exp(coeff[2]*(sector_width + coeff[3]));
    return gaussian_weight;
}

int VFHController::search_closest(const std::vector<int>& sorted_array, int value) {

    auto it = lower_bound(sorted_array.begin(), sorted_array.end(), value);
    if (it == sorted_array.end())
        return sorted_array.back();   // return last element
    auto found = *it;
    if (it != sorted_array.begin())
    {
        auto found2 = *(--it);
        if (abs(value - found2) < abs(value - found))
            found = found2;
    }
    return found;

}

void VFHController::update_goal_position(){
    std::vector<double> dists;

    if (path_points.size()==0) {
        return;
    }
    double path_x, path_x2, path_y, path_y2, diff_x, diff_y; 

    // cout << "\n----------------------------------------------------\n";
    // std::cout << "\nrobot poses \n";
    // std::cout << robot_pose_x << "," << robot_pose_y << "\t";
    // std::cout << "\npath_points \n";
    // for (int i=0; i<path_points.size(); i++) {
    //     std::cout << path_points[i][0] << "," << path_points[i][1] << "\t";
    // }
    // select goal at distance max_detection_dist
    double path_length=0;
    int goal_index=0;
    int closest_point_index=0;
    for(int i=0;i<path_points.size();i++){
        path_x = path_points[i][0];
        path_y = path_points[i][1];
        diff_x = path_x - robot_pose_x;
        diff_y = path_y - robot_pose_y;
        double ang = atan2(diff_y,diff_x) - robot_pose_theta;
        if(abs(ang*180/M_PI)<90){
            double dist_projected = sqrt(diff_y*diff_y + diff_x*diff_x) * cos(ang);
            dists.push_back(dist_projected);
        } 
        else {
            dists.push_back(1000);
        }
    }
    // std::cout << "\ndists vect \n";
    // //find min;
    // for (int i=0; i<dists.size(); i++) {
    //     std::cout << dists[i] << "\t";
    // }

    closest_point_index = min_element(dists.begin()+prev_goal_index,dists.end()) - dists.begin();
    path_length += dists[closest_point_index];
    // std::cout << "\npath_length: " << path_length << "\n";
    // std::cout << "closest_point_index: " << closest_point_index << "\n";

    goal_index = path_points.size()-1;

    goal_x = path_points[goal_index][0];
    goal_y = path_points[goal_index][1];

    path_x = path_points[goal_index-1][0];
    path_y = path_points[goal_index-1][1];

    path_x2 = path_points[goal_index][0];
    path_y2 = path_points[goal_index][1];

    diff_x = path_x2 - path_x;
    diff_y = path_y2 - path_y;

    path_direction = atan2(diff_y,diff_x);

    for (int i=closest_point_index; i<path_points.size()-1; i++) {
        path_x = path_points[i][0];
        path_y = path_points[i][1];

        path_x2 = path_points[i+1][0];
        path_y2 = path_points[i+1][1];

        diff_x = path_x2 - path_x;
        diff_y = path_y2 - path_y;

        double ds = sqrt(diff_y*diff_y + diff_x*diff_x);
        path_length += ds;

        if(path_length>=goal_dist_th){
            goal_index = i;
            goal_x = path_points[goal_index][0];
            goal_y = path_points[goal_index][1];
            break;
        }

    }
    // std::cout << "path_length: " << path_length << "\n";
    // std::cout << "goal_index: " << goal_index << "\n";
    // std::cout << "goal_x: " << goal_x << "\n";
    // std::cout << "goal_y: " << goal_y << "\n";
    prev_goal_index = goal_index;
    //update goal orientation
    if((goal_index-1)>=0 && (goal_index+1)<path_points.size()){
        //first ang
        path_x = path_points[goal_index-1][0];
        path_y = path_points[goal_index-1][1];

        path_x2 = path_points[goal_index][0];
        path_y2 = path_points[goal_index][1];

        diff_x = path_x2 - path_x;
        diff_y = path_y2 - path_y;

        double angle1 = atan2(diff_y,diff_x);
        
        //second ang
        path_x = path_points[goal_index][0];
        path_y = path_points[goal_index][1];

        path_x2 = path_points[goal_index+1][0];
        path_y2 = path_points[goal_index+1][1];

        diff_x = path_x2 - path_x;
        diff_y = path_y2 - path_y;

        double angle2 = atan2(diff_y,diff_x);

        path_direction = angle1/2 + angle2/2;

    }
    // std::cout << "path_direction: " << path_direction << "\n";
    // std::cout << "robot_pose_theta: " << robot_pose_theta << "\n";
    
}

void VFHController::vfhController() {
    int data_length = meas_x_filtered.size();
    direction = 0; // ref_direction==-1 ? 0 : ref_direction
    speed_cmd = 0; // 0

    if(data_length==0 && ctrl_word==0){
        return;
    }
    
    // cout << "\nlateral_dist: "<< lateral_dist << endl;
    // cout << "ang diff: "<< abs(robot_pose_theta-path_direction)*180/M_PI << endl;
    if (!stop && lateral_dist<lateral_dist_th && lateral_dist>0 && abs(robot_pose_theta-path_direction)*180/M_PI<angle_to_goal_th && weights_inverted){   
        // Reset local planner params     
        dist_from_point = -1;
        lateral_dist = -1;
        path_point_received = false;
        ctrl_word = 0;
        speed_cmd = 0;
        direction = prev_direction;
        // Invert again directions weights
        double temp = target_dir_weight;
        target_dir_weight = prev_dir_weight;
        prev_dir_weight = temp;
        weights_inverted = false;
        return;
    }

    // Invert directions weights
    if (lateral_dist > weight_inversion_lat_dist && !weights_inverted) {
        double temp = target_dir_weight;
        target_dir_weight = prev_dir_weight;
        prev_dir_weight = temp;
        weights_inverted = true;
    } 
        
    // Add circles to x,y points and FIND Min distance
    int circle_points = 90;
    double measures_x[data_length*(1+circle_points)];
    double measures_y[data_length*(1+circle_points)];
    double x_value = 1000;
    double y_value = 1000;
    double min_stop_dist = 100;
    min_dist = 100;    

    for (int i=0; i<data_length; i++) {

        double eucl_dist = sqrt(pow(meas_x_filtered[i],2) + pow(meas_y_filtered[i],2));
        double ang = 180/M_PI*atan2(meas_y_filtered[i],meas_x_filtered[i]); // VERIFICA CON PUNTI RADAR VERO
        
        if(abs(ang)<max_angle_dist && eucl_dist<min_dist)
            min_dist = eucl_dist;
        
        if(eucl_dist<min_stop_dist)
            min_stop_dist = eucl_dist;

        if (meas_x_filtered[i]==0 && meas_y_filtered[i]==0) {
            x_value = 1000;
            y_value = 1000;
        } else {
            x_value = meas_x_filtered[i];
            y_value = meas_y_filtered[i];
        }
        measures_x[i+i*circle_points] = x_value;
        measures_y[i+i*circle_points] = y_value;
        for (int j=1; j<circle_points+1; j++) {      
            measures_x[i+i*circle_points+j] = x_value + inflation_radius*cos(2*M_PI*(double)j/(double)circle_points);
            measures_y[i+i*circle_points+j] = y_value + inflation_radius*sin(2*M_PI*(double)j/(double)circle_points);
        }
    }  

    // set STOP and CTRL_WORD var
    if ((!stop && min_stop_dist < stop_distance)||
        (stop && min_stop_dist < (stop_distance+0.1))||
        (ref_direction>90 && ref_direction<270)){
        stop = 1;
        ctrl_word = 1;
    } else if (min_dist < max_detection_dist || path_point_received) {
        stop = 0;
        ctrl_word = 1;
    } else {
        stop = 0;
        ctrl_word = 0;
        path_point_received = false;
    }

    // TODO: if path_point_received FALSE -> set stop=1 and then return
    if (!path_point_received) {
        stop=1;
        return;
    }

    update_goal_position();
        
    // define sector limits 
    size_t data_size = sizeof(measures_x)/sizeof(measures_x[0]);

    // associate euclidean distance to each sector
    std::vector<double> dist_to_associate(num_of_sector,100);
    int sector_index = 0;
    for (int i=0; i<data_size; i++) {
        double eucl_dist = sqrt(pow(measures_x[i],2) + pow(measures_y[i],2));
        double ang = 180/M_PI*atan2(measures_y[i],measures_x[i]);
        if (ang < 0) 
            ang = ang + 360;
        
        sector_index = findSectorIdx(ang);
        if (dist_to_associate[sector_index] > eucl_dist && eucl_dist < (max_detection_dist+2) )
            dist_to_associate[sector_index] = eucl_dist;
        
    }
    
    // BUILD COSTS
    std::vector<int> sector_array(num_of_sector,0);
    std::vector<double> cost_target(num_of_sector,0);
    std::vector<double> cost_prev(num_of_sector,0);
    std::vector<double> cost_obstacle(num_of_sector,0);
    std::vector<double> cost_obst(num_of_sector,0);
    std::vector<double> overall_cost(num_of_sector,1000);

    // Gaussian properties
    double coeff[4] = {0.845015753269096,4.96041666063205,-0.188303172580222,-4.44023451783892};
    double gaussian_shift = round((float)num_of_sector/2);
    double gaussian_weight_target = findGaussianWeight(coeff);
    double gaussian_weight_prev_dir = findGaussianWeight(coeff);
    double coeff2[4] = {0.369389785763626,3.64914690021850,-0.216102762982391,-3.84497580829445};
    
    // build target direction cost
    sector_index = findSectorIdx(ref_direction);
    buildCost(cost_target,sector_index,gaussian_shift,sector_array,gaussian_weight_target,1,-gaussian_weight_target);
    
    // build previous direction cost
    sector_index = findSectorIdx(prev_direction);
    buildCost(cost_prev,sector_index,gaussian_shift,sector_array,gaussian_weight_prev_dir,1,-gaussian_weight_prev_dir);

    // build obstacle cost and overall cost
    int ind = 0;
    int window_idx = 0;
    int window_size = num_of_sector*30/360; // dispari
    if(window_size%2==0)
        window_size+=1;
        
    for (int k=0; k<num_of_sector; k++) {
        int ind = k;    
        for (int i = ind-(window_size/2); i < ind + (window_size/2)+1; i++) {
            if (i<0) window_idx = dist_to_associate.size()+i;
            else window_idx = (i % dist_to_associate.size());
            cost_obstacle[k] += (1/dist_to_associate[window_idx]);
        }
        cost_obstacle[k] = cost_obstacle[k] / window_size;    
        overall_cost[k] = obstacle_weight * cost_obstacle[k] + target_dir_weight * cost_target[k] + prev_dir_weight * cost_prev[k];   
    }

    // DEBUG - overall cost
    if (verbose) {
        std_msgs::Float64MultiArray overall_cost_debug;
        for (int i=0; i<num_of_sector; i++) {
            overall_cost_debug.data.push_back(overall_cost[i]);
        }
        overall_cost_pub_.publish(overall_cost_debug);
    }
  
    // get min cost and direction
    std::vector<int> vec;
    std::vector<int> bounds;
    float bound_ang = 40; // deg
    int boundaries[2] = {int(bound_ang/sector_width), num_of_sector - int(bound_ang/sector_width)};    
    
    if (ctrl_word && min_dist<max_detection_dist) {

        for (int i=1; i<num_of_sector; i++) {
            if (abs(overall_cost[i] - overall_cost[i-1])>1e-4)
                vec.push_back(i-1);
        }
        
        for (int i=0; i<vec.size()-1; i++) {
            if (abs(vec[i+1]-vec[i]) > 1) {
                bounds.push_back(vec[i]);
                bounds.push_back(vec[i+1]);
            }
        }
    
        int bound_1 = search_closest(bounds,boundaries[0]);
        int bound_2 = search_closest(bounds,boundaries[1]);
        if (bound_1 < boundaries[0] && bound_1!=0) 
            boundaries[0] = bound_1;
        if (bound_2 > boundaries[1] && bound_2!=num_of_sector)
            boundaries[1] = bound_2;
    }

    for (int i=1; i<num_of_sector; i++) {
        if (i>boundaries[0] && i<boundaries[1])
            overall_cost[i]=1000;
    }
    int idx = min_element(overall_cost.begin(), overall_cost.end()) - overall_cost.begin();
  
    if (stop)
        direction = 0;
    else if (ctrl_word && !stop_mode)
        direction = sector_limits_up[idx]-sector_width/2;
    else
        direction = 0;

    if (direction > 180)
        direction = direction - 360;
    
    if( abs(direction-prev_direction)*node_frequency > direction_speed_lim){
        double sign = (direction-prev_direction)/abs(direction-prev_direction);
        direction = prev_direction + sign*direction_speed_lim/node_frequency;
    }
    prev_direction = direction;

    // DEBUG - reference direction and goal pose
    if (verbose) {
        // fill ref dir and goal pose 
        ref_dir_pose.header.frame_id = "chassis";
        ref_dir_pose.header.stamp = ros::Time::now();
        ref_dir_pose.pose.position.x = 0;
        ref_dir_pose.pose.position.y = 0;
        tf::Quaternion q;
        q.setRPY(0, 0, (direction)*M_PI/180.0);
        ref_dir_pose.pose.orientation.x = q.x();
        ref_dir_pose.pose.orientation.y = q.y();
        ref_dir_pose.pose.orientation.z = q.z();
        ref_dir_pose.pose.orientation.w = q.w();

        goal_pose.header.frame_id = "chassis";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.y = (goal_x - robot_pose_x);
        goal_pose.pose.position.x = -(goal_y - robot_pose_y);
        q.setRPY(0, 0, (ref_direction+180)*M_PI/180.0);
        goal_pose.pose.orientation.x = q.x();
        goal_pose.pose.orientation.y = q.y();
        goal_pose.pose.orientation.z = q.z();
        goal_pose.pose.orientation.w = q.w();

        ref_dir_pose_pub_.publish(ref_dir_pose);
        goal_pose_pub_.publish(goal_pose);
    }

}

void VFHController::local_planner_pub() {
    
    // DEBUG - overall cost
    if (verbose) {
        if (!ctrl_word) {
            std::vector<double> overall_cost(num_of_sector,0.075);
            std_msgs::Float64MultiArray overall_cost_debug;
            for (int i=0; i<num_of_sector; i++) {
                overall_cost_debug.data.push_back(overall_cost[i]);
            }
            overall_cost_pub_.publish(overall_cost_debug);
        }
    }
    
    // evaluate speed cmd
    double angular_speed = direction_gain*direction*M_PI/180.0;
    if (abs(angular_speed) > speed_upper_lim/2) 
        angular_speed = angular_speed/abs(angular_speed)*speed_upper_lim/2; 

    double min_dist_speed = (speed_gain * (min_dist - stop_distance));

    double goal_dist_speed = speed_upper_lim;
    if (dist_from_point>0)
        goal_dist_speed = (speed_upper_lim/5)*dist_from_point;

    if (ctrl_word) 
        speed_cmd = min(min(min_dist_speed,speed_upper_lim-abs(angular_speed)),goal_dist_speed);
        
    if( abs(speed_cmd-speed_cmd_prev)*node_frequency > linear_accel_lim){
        double sign = (speed_cmd-speed_cmd_prev)/abs(speed_cmd-speed_cmd_prev);
        speed_cmd = speed_cmd_prev + sign*linear_accel_lim/node_frequency;
    }
    
    if (speed_cmd > speed_upper_lim)
        speed_cmd = speed_upper_lim;
    if (speed_cmd < 0)
        speed_cmd = 0;

    if (stop){
        direction = 0;
        speed_cmd = 0;
    }

    speed_cmd_prev = speed_cmd;

    // Publish DEBUG message    
    if (verbose) {
        std_msgs::Float64MultiArray var_debug;
        var_debug.data.push_back(ctrl_word);
        var_debug.data.push_back(stop);
        var_debug.data.push_back(min_dist);
        var_debug.data.push_back(lateral_dist);
        var_debug.data.push_back(ref_direction);
        var_debug.data.push_back(direction);
        var_debug.data.push_back(min(min(speed_cmd,speed_upper_lim-abs(angular_speed)),goal_dist_speed));
        var_debug.data.push_back(goal_x);
        var_debug.data.push_back(goal_y);
        var_debug.data.push_back(dist_from_point);
        var_debug.data.push_back(path_point_received);
        debug_pub_.publish(var_debug);
    }

    // Publish local planner message    
    geometry_msgs::Twist planner_msg;
    planner_msg.linear.x = speed_cmd;
    planner_msg.linear.z = ctrl_word;
    planner_msg.angular.z = angular_speed;
    local_planner_pub_.publish(planner_msg);

}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "VFHController"); 

    ros::NodeHandle nh; 

    VFHController VFHController(&nh); 
    ros::Rate loop_rate(VFHController.node_frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        auto start = std::chrono::steady_clock::now();
        VFHController.update_pers_map();
        VFHController.vfhController();
        VFHController.local_planner_pub();
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;
        // std::cout << "Duration [seconds]: " << diff.count() << std::endl;
        loop_rate.sleep();
    }

    return 0;
} 





