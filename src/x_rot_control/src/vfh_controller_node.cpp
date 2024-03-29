#include "vfh_controller_node.hpp"

// CONSTRUCTOR
VFHController::VFHController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of VFHController");
    initializeSubscribers(); 
    initializePublishers();

    ros::NodeHandle private_nh("~");
    private_nh.param<bool>("stop_mode", stop_mode, false);

    private_nh.param<float>("speed_upper_lim", speed_upper_lim, 0.6);
    private_nh.param<int>("window_size_param_max", window_size_param_max, 30);
    private_nh.param<int>("num_of_sector", num_of_sector, 180);
    private_nh.param<float>("k_ref_dir", k_ref_dir, 0.05);
    private_nh.param<float>("prev_dir_weight", prev_dir_weight, 0.1);
    private_nh.param<float>("target_dir_weight_initial", target_dir_weight_initial, 0.1);
    private_nh.param<float>("obstacle_weight", obstacle_weight, 0.97);
    private_nh.param<float>("inflation_radius", inflation_radius, 0.6);
    private_nh.param<float>("bound_ang", bound_ang, 90);
    private_nh.param<double>("direction_speed_lim", direction_speed_lim, 10);
    private_nh.param<double>("direction_gain_multi", direction_gain_multi, 0.1);
    private_nh.param<double>("direction_gain_offset", direction_gain_offset, 0.3);
    private_nh.param<double>("speed_gain_multi", speed_gain_multi, 0.03);
    private_nh.param<double>("speed_gain_offset", speed_gain_offset, 0.2);
    private_nh.param<float>("max_detection_dist", max_detection_dist, 4);
    private_nh.param<float>("stop_distance", stop_distance, 1.3);

    // FOR DETAILS OF THE DEFINITION OF EACH PARAMETER SEE 'vfh_controller_node.hpp'
    node_frequency = 10;
    // persistency map parameters
    pers_index = 0;
    pers_time_th = 5;
    pers_dist_th = 0.5;
    pers_dist_th_same = 0.1;
    consensus_th = 10;
    // local planner parameters - main
    ref_direction = -1;
    lateral_dist = -1;
    dist_from_point = -1;
    path_point_received = false;
    target_goal_exist = false;
    path_point_requested = false;
    ctrl_word = 0; 
    alarm_on = 0;
    stop = 0;
    angle_diff_from_path = 0.0;
    // max_detection_dist = 4;
    max_angle_dist = 20;
    stop_distance = 1.3;
    min_dist = 100;
    // local planner parameters - direction
	direction = 0;
    prev_direction = 0;
    direction_gain = 0.7;
    // direction_gain_multi = 0.5;
    // direction_gain_offset = 0.3;
    direction_gain_max = 1;
    // speed_gain_multi = 0.03;
    // speed_gain_offset = 0.2;
    direction_speed_lim = 10;
    // local planner parameters - linear speed
    // speed_upper_lim = 0.8;
	speed_cmd = 0;
    speed_cmd_prev = 0;
    linear_accel_lim = 1;
    speed_lower_lim = 0;
    speed_gain = (speed_upper_lim - speed_lower_lim)/(max_detection_dist - stop_distance);
    // local planner parameters - costs
    // num_of_sector = 180;
    window_size_param = 1;
    sector_width = 360/(float)num_of_sector;
    // obstacle_weight = 0.95;
    // target_dir_weight = 0.1;
    // k_ref_dir = 0.05;
    // prev_dir_weight = 0.2;
    // inflation_radius = 0.7;
    // local planner parameters - directions weights inversion
    weight_inversion_lat_dist = 0.3;
    weights_inverted = false;
    // local planner parameters - goal point
    goal_x = 0.0;
    goal_y = 0.0;
    angle_to_goal_th = 25; // [m]
    lateral_dist_th = 1; // [m]
    goal_dist_th = 3; // [m]
    prev_goal_index = 0;
    dist_traj_end = 15; // [m]
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
    window_size_param_max = config.window_size_param_max;
    obstacle_weight = config.obstacle_weight;
    target_dir_weight = config.target_dir_weight;
    k_ref_dir = config.k_ref_dir;
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
    direction_gain_multi = config.direction_gain_multi;
    direction_gain_offset = config.direction_gain_offset;
    direction_gain_max = config.direction_gain_max;
    speed_gain_multi = config.speed_gain_multi;
    speed_gain_offset = config.speed_gain_offset;
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
}

void VFHController::initializePublishers()
{
    local_planner_pub_ = nh_.advertise<geometry_msgs::Twist>("/yape/cmd_vel", 1, true); 
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/map_debug_output", 1);
    overall_cost_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/cost_debug_output", 1);
    obst_cost_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/obst_debug_output", 1);
    target_cost_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/target_debug_output", 1);
    prev_cost_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("/prev_debug_output", 1);

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

    if (path_point_received) {
        double delta_x = goal_x - robot_pose_x;
        double delta_y = goal_y - robot_pose_y;
        ref_direction = 180/M_PI*(atan2(delta_y,delta_x) - robot_pose_theta);

        dist_from_point = sqrt(pow(delta_x,2) + pow(delta_y,2));
        double a = -tan(path_direction);
        double c = -a*goal_x - goal_y;
        lateral_dist = abs(robot_pose_x*a + robot_pose_y + c)/sqrt(a*a+1);
        lateral_dist_sign = -((robot_pose_x*a + robot_pose_y + c)/sqrt(a*a+1))/lateral_dist;

        if (ref_direction<0)
            ref_direction = 360 + ref_direction;     
    }
    else {
        ref_direction = 0;
        lateral_dist = -1;
        lateral_dist_sign = 0;
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

        double a = -tan(path_direction);
        double c = -a*goal_x - goal_y;
        lateral_dist = abs(robot_pose_x*a + robot_pose_y + c)/sqrt(a*a+1);
        lateral_dist_sign = -((robot_pose_x*a + robot_pose_y + c)/sqrt(a*a+1))/lateral_dist;

        if (ref_direction<0)
            ref_direction = 360 + ref_direction;      
    }
    else {
        ref_direction = 0;
        lateral_dist = -1;
        lateral_dist_sign = 0;
        dist_from_point = -1;
    }

    
}

void VFHController::pathPointCallback(const nav_msgs::Odometry& msg) {

    double path_point_x = msg.pose.pose.position.x;
    double path_point_y = msg.pose.pose.position.y;
    
    // goal_x = path_pont_x;
    // goal_y = path_point_y;i

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
    std::cout << "PATH POINT RECEIVED \n\n";

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

void VFHController::update_pers_map(){
    // add new measurements to pers_map
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

void VFHController::normpdf(const std::vector<int>& sector_array, int sector_index, double gaussian_weight, std::vector<double>& norm_distribution, bool target) {
  double k;
  double t;
  int b_k;
  for (b_k = 0; b_k < num_of_sector; b_k = b_k + 1) {
    k = 1.0 + (((double)b_k) * 1.0);
    if (gaussian_weight > 0.0) {
      t = (sector_array[((int)k) - 1] - sector_index) / gaussian_weight;
      if(target)
        norm_distribution[((int)k) - 1] = exp((-0.5 * t) * t) / (2.5066282746310002 * gaussian_weight/2);
      else
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

void VFHController::buildCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight, int w1, double w2,bool target=false) {
    double sector_value = sector_index-gaussian_shift;
    int max_idx = -1000;
    for (int j=0; j<num_of_sector; j++) {
        sector_array[j] = sector_value + j ;
        if (sector_array[j]>max_idx)
            max_idx = sector_array[j];
    }
    
    std::vector<double> gaussian_distribution(num_of_sector);
    normpdf(sector_array,sector_index,gaussian_weight,gaussian_distribution,target);
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

    if (path_point_requested) {prev_goal_index = 0;}
    double path_length=0;
    int goal_index=0;
    int closest_point_index=0;
    for(int i=0;i<path_points.size();i++){
        path_x = path_points[i][0];
        path_y = path_points[i][1];
        diff_x = path_x - robot_pose_x;
        diff_y = path_y - robot_pose_y;
        double ang = atan2(diff_y,diff_x) - robot_pose_theta;
        if(abs(atan2(diff_y,diff_x)*180/M_PI)<45){
            // double dist = sqrt(diff_y*diff_y + diff_x*diff_x)*cos(ang);
            double dist = sqrt(diff_y*diff_y + diff_x*diff_x);
            dists.push_back(dist);
        } 
        else {
            dists.push_back(1000);
        }
    }
    // cout<<endl;
    // for(int i=0; i<dists.size(); i++)
    //     cout<<dists[i]<<' ';
    // cout<<endl;

    closest_point_index = min_element(dists.begin()+prev_goal_index,dists.end()) - dists.begin();
    dist_traj_end = max_element(dists.begin()+prev_goal_index,dists.end()) - dists.begin();
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

            target_goal_exist = true;
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
    
}

void VFHController::vfhController() {
    int data_length = meas_x_filtered.size();
    direction = 0; // ref_direction==-1 ? 0 : ref_direction
    speed_cmd = 0; // 0

    if(data_length==0 && ctrl_word==0){
        return;
    }
    
    if (!stop && min_dist>max_detection_dist && lateral_dist<lateral_dist_th && lateral_dist>0 && lateral_dist_sign*(robot_pose_theta-path_direction)*180/M_PI<angle_to_goal_th && lateral_dist_sign*(robot_pose_theta-path_direction)*180/M_PI>0){   
        // Reset local planner params     
        dist_from_point = -1;
        lateral_dist = -1;
        path_point_received = false;
        target_goal_exist = false;
        ctrl_word = 0;
        cout<<"x_rot_vfh_controller :: exit for conditions at line 611"<<endl;
        speed_cmd = 0;
        direction = prev_direction;
        return;
    }
        
    // Add circles to x,y points and FIND Min distance
    int circle_points = 90;
    double measures_x[data_length*(1+circle_points)];
    double measures_y[data_length*(1+circle_points)];
    double x_value = 1000;
    double y_value = 1000;
    double min_stop_dist = 100;
    min_dist = 100;
    min_dist_allFOV = 100;    

    // processing of radar filtered data
    for (int i=0; i<data_length; i++) {

        double eucl_dist = sqrt(pow(meas_x_filtered[i],2) + pow(meas_y_filtered[i],2));
        double ang = 180/M_PI*atan2(meas_y_filtered[i],meas_x_filtered[i]); // VERIFICA CON PUNTI RADAR VERO
        
        if(eucl_dist<min_dist_allFOV)
            min_dist_allFOV = eucl_dist;

        if(abs(ang)<max_angle_dist && eucl_dist<min_dist)
            min_dist = eucl_dist;

        if(abs(ang)<45 && eucl_dist<min_stop_dist)
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

    // Local planner take control conditions
    if(!ctrl_word && min_dist <= (max_detection_dist-0.05)){
        alarm_on = 1;
        ctrl_word = 1;
        path_point_requested = true;
    }

    if(min_dist >= (max_detection_dist+0.05)){
        alarm_on = 0;
        path_point_requested = false;
    }

    // Request new path points while in control
    if(ctrl_word && min_dist < max_detection_dist && !path_point_requested){
        alarm_on = 1;
        ctrl_word = 1;
        path_point_requested = true;
        path_point_received = false;
    }

    // Stop conditions
    if(!stop && (min_dist < stop_distance || (!target_goal_exist))){ //path_point_received && path_point_requested
        stop = 1;
        speed_cmd = 0;
        direction = 0;
    }
    else if(min_dist > stop_distance+0.1)
        stop = 0;
      
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
        if (dist_to_associate[sector_index] > eucl_dist && eucl_dist < (max_detection_dist*2) )
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
    buildCost(cost_target,sector_index,gaussian_shift,sector_array,gaussian_weight_target,1,-gaussian_weight_target,true);
    
    // build previous direction cost
    double prev_direction_for_cost = prev_direction;
    if(prev_direction_for_cost<0){
        prev_direction_for_cost = 360 + prev_direction_for_cost;
    }
    sector_index = findSectorIdx(prev_direction_for_cost);
    buildCost(cost_prev,sector_index,gaussian_shift,sector_array,gaussian_weight_prev_dir,1,-gaussian_weight_prev_dir);

    // build obstacle cost and overall cost
    int ind = 0;
    int window_idx = 0;

    
    // window_size_param = window_size_param_max;

    window_size_param = window_size_param_max+3 - min_dist*window_size_param_max/max_detection_dist;
    if(min_dist_allFOV < 1)
        window_size_param = window_size_param_max;
    
    if(min_dist_allFOV >= max_detection_dist)
        window_size_param = 3;

    // cout<<window_size_param<<endl;

    int window_size = num_of_sector*window_size_param/360; // dispari
    if(window_size%2==0)
        window_size+=1;
    
    double ref_dir_weight=0;
    if (ref_direction > 180)
        ref_dir_weight = abs(ref_direction - 360);
    else 
        ref_dir_weight = ref_direction;

    target_dir_weight = target_dir_weight_initial + k_ref_dir * (ref_dir_weight/180*M_PI * ref_dir_weight/180*M_PI);

    for (int k=0; k<num_of_sector; k++) {
        int ind = k;    
        for (int i = ind-(window_size/2); i < ind + (window_size/2)+1; i++) {
            if (i<0) window_idx = dist_to_associate.size()+i;
            else window_idx = (i % dist_to_associate.size());
            cost_obstacle[k] += (1/(dist_to_associate[window_idx]));
        }
        cost_obstacle[k] = cost_obstacle[k] / window_size;    
        overall_cost[k] = obstacle_weight * cost_obstacle[k] + (target_dir_weight) * cost_target[k] + prev_dir_weight * cost_prev[k];   
    }

    // DEBUG - overall cost
    if (verbose) {
        std_msgs::Float64MultiArray overall_cost_debug;
        std_msgs::Float64MultiArray obst_cost_debug;
        std_msgs::Float64MultiArray target_cost_debug;
        std_msgs::Float64MultiArray prev_cost_debug;
        for (int i=0; i<num_of_sector; i++) {
            overall_cost_debug.data.push_back(overall_cost[i]);
            obst_cost_debug.data.push_back(sqrt(cost_obstacle[i]));
            target_cost_debug.data.push_back(cost_target[i]);
            prev_cost_debug.data.push_back(cost_prev[i]);
        }
        overall_cost_pub_.publish(overall_cost_debug);
        obst_cost_pub_.publish(obst_cost_debug);
        target_cost_pub_.publish(target_cost_debug);
        prev_cost_pub_.publish(prev_cost_debug);
    }
  
    // get min cost and direction
    std::vector<int> vec;
    std::vector<int> bounds;
    
    boundaries[0] = int(bound_ang/sector_width);
    boundaries[1] = num_of_sector - int(bound_ang/sector_width);
    
    if (ctrl_word && min_dist<max_detection_dist) {

        for (int i=1; i<num_of_sector; i++) {
            if (abs(overall_cost[i] - overall_cost[i-1])>1e-4)
                vec.push_back(i-1);
        }
        
        for (int i=0; i<vec.size()-1; i++) {
            if (abs(vec[i+1]-vec[i]) >= 1) {
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

        goal_pose.header.frame_id = "odom";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = goal_x;
        goal_pose.pose.position.y = goal_y;
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
            // overall_cost_pub_.publish(overall_cost_debug);
        }
    }
    
    double ref_dir;
    if (ref_direction > 180)
        ref_dir = ref_direction - 360;
    else 
        ref_dir = ref_direction;

    direction_gain = direction_gain_offset+direction_gain_multi*abs(ref_dir-direction)*M_PI/180;
    if (direction_gain>direction_gain_max)
        direction_gain = direction_gain_max;

    double angular_speed = direction_gain*direction*M_PI/180.0;
    if (abs(angular_speed) > speed_upper_lim/2) 
        angular_speed = angular_speed/abs(angular_speed)*speed_upper_lim/2; 
    
    speed_gain = speed_gain_offset - speed_gain_multi*abs(ref_dir-direction)*M_PI/180;
    if (speed_gain<0.05)
        speed_gain = 0.05;

    double goal_dist_speed = speed_upper_lim;
    if (dist_from_point>0)
        goal_dist_speed = speed_gain*dist_from_point;

    speed_gain = (speed_upper_lim - speed_lower_lim)/(max_detection_dist - stop_distance);
    double min_dist_speed = (speed_gain * (min_dist - stop_distance));

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

    // if (stop && !path_point_received){
    //     direction = 0;
    //     angular_speed = direction_gain*direction*M_PI/180.0;
    //     speed_cmd = 0.5;
    // }
    // else if (stop && path_point_received){
    //     direction = 0;
    //     speed_cmd = 0;
    // }
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
        var_debug.data.push_back(speed_cmd);
        var_debug.data.push_back(goal_x);
        var_debug.data.push_back(goal_y);
        var_debug.data.push_back(angle_diff_from_path);
        var_debug.data.push_back(path_point_received);
        var_debug.data.push_back(boundaries[0]);
        var_debug.data.push_back(boundaries[1]);
        var_debug.data.push_back(alarm_on);
        var_debug.data.push_back(direction_gain);
        var_debug.data.push_back(speed_gain);
        var_debug.data.push_back(prev_direction);
        var_debug.data.push_back(target_goal_exist);
        var_debug.data.push_back(target_dir_weight);
        debug_pub_.publish(var_debug);
    }

    // Publish local planner message    
    geometry_msgs::Twist planner_msg;
    planner_msg.linear.x = speed_cmd;
    planner_msg.linear.y = alarm_on;
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






