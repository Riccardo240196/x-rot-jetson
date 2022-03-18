#include "vfh_controller_node.hpp"

// CONSTRUCTOR
VFHController::VFHController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of VFHController");
    initializeSubscribers(); 
    initializePublishers();
    
    ref_direction = -1;
    lateral_dist = -1;
    ctrl_word = 0;
	direction = 0;
    prev_direction = 0;
	speed_cmd = 0;
    speed_cmd_prev = 0;
    direction_gain = 0.5;

    vehicle_to_radar = Matrix<double, 3, 3>::Identity();

    // vehicle_to_radar << 1, 0, 0.65,
    //                     0, 1, 0,
    //                     0, 0, 1;   
    
    map_to_vehicle = Matrix<double, 3, 3>::Identity();
    
    vector<double> temp = {-1, -1, -1, -1};
    for(int i=0; i<300; i++){
        pers_map.push_back(temp);
    }
    pers_index = 0;
    node_frequency = 10;
    pers_time_th = 5;
    pers_dist_th = 0.2;
    consensus_th = 1;

    debug_cloud.height = 1;   
        
}

void VFHController::initializeSubscribers()
{
    robot_pose_sub_ = nh_.subscribe("/yape/odom_diffdrive", 1, &VFHController::robotPoseCallback,this);  
    path_point_sub_ = nh_.subscribe("/path_point", 1, &VFHController::pathPointCallback,this);  
    radar_points_sub_ = nh_.subscribe("/radar_messages", 1, &VFHController::radarPointsCallback,this);
    radar_points_sub_2 = nh_.subscribe("/radar_scan", 1, &VFHController::radarPointsCallback_2,this);  
}

void VFHController::initializePublishers()
{
    local_planner_pub_ = nh_.advertise<geometry_msgs::Twist>("/yape/cmd_vel", 1, true); 
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/debug_output", 1);
}

void VFHController::robotPoseCallback(const nav_msgs::Odometry& msg) {
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
    
    pathPointFake();
}

void VFHController::pathPointCallback(const nav_msgs::Odometry& msg) {

    double path_point_x = msg.pose.pose.position.x;
    double path_point_y = msg.pose.pose.position.y;

    double delta_x = path_point_x - robot_pose_x;
    double delta_y = path_point_y - robot_pose_y;
    ref_direction = 180/M_PI*(atan2(delta_y,delta_x) - robot_pose_theta);

    double dist_from_point = sqrt(pow(delta_x,2) + pow(delta_y,2));
    double ang = atan2(delta_y,delta_x);
    lateral_dist = abs(dist_from_point*cos(ang));
    if(dist_from_point<0.5)
        lateral_dist = 0.0;

    if (ref_direction<0)
        ref_direction = 360 + ref_direction;     

    if (ctrl_word==0)
        lateral_dist = -1;
}

void VFHController::pathPointFake() {

    double path_point_x = 30;
    double path_point_y = 0;

    double delta_x = path_point_x - robot_pose_x;
    double delta_y = path_point_y - robot_pose_y;
    ref_direction = 180/M_PI*(atan2(delta_y,delta_x) - robot_pose_theta);

    double dist_from_point = sqrt(pow(delta_x,2) + pow(delta_y,2));
    double ang = atan2(delta_y,delta_x);
    lateral_dist = abs(dist_from_point*cos(ang));
    if(dist_from_point<0.5)
        lateral_dist = 0.0;

    if (ref_direction<0)
        ref_direction = 360 + ref_direction;     

    if (ctrl_word==0)
        lateral_dist = -1;
}

void VFHController::radarPointsCallback(const radar_pa_msgs::radar_msg& msg) {
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    // pcl::PointXYZ point;

    meas_raw_x.clear();
    meas_raw_y.clear();

    for(int i = 0; i< msg.data_A.size(); i++){
        temp(0) = msg.data_A[i].distance*cos(msg.data_A[i].angle);
        temp(1) = msg.data_A[i].distance*sin(msg.data_A[i].angle);
        temp(2) = 1;
                
        temp = map_to_vehicle*vehicle_to_radar*temp;

        meas_raw_x.push_back(temp(0));
        meas_raw_y.push_back(temp(1));      

    }  
    
}

void VFHController::radarPointsCallback_2(const sensor_msgs::LaserScan& msg){
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;
    // pcl::PointXYZ point;

    meas_raw_x.clear();
    meas_raw_y.clear();

    double angle_min = msg.angle_min;
    double angle_increment = msg.angle_increment;
    double angle=angle_min;

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
        for(int ind=0; ind<pers_index; ind++){
            diff_x = pers_map[ind][0] - meas_raw_x[i];
            diff_y = pers_map[ind][1] - meas_raw_y[i];
            dist = sqrt(diff_x*diff_x + diff_y*diff_y);
            if(dist< pers_dist_th){
                close_points.push_back(ind);
                if(dist<min_dist){
                    closest_point = ind;
                }
            }
        }
        
        vector<double> to_insert = {meas_raw_x[i], meas_raw_y[i], 0, 0};
        if(closest_point!=-1)
            pers_map[closest_point] = to_insert;
        else{
            pers_map[pers_index] = to_insert;
            if((pers_index+1)<pers_map.size())
                pers_index++;
        }
        for (int j=0; j<close_points.size(); j++){
            pers_map[close_points[j]][3]++; 
        }
        
    }

    vector<double> null={-1, -1, -1, -1};
    pcl::PointXYZ point;
    meas_x_filtered.clear();
    meas_y_filtered.clear();
    
    debug_cloud.points.clear();
    debug_cloud.width = 0;

    Matrix<double, 3, 1> temp;

    for(int i=0; i<pers_index; i++){
        pers_map[i][2]++;
        if(pers_map[i][2]/node_frequency > pers_time_th){
            pers_map.erase(pers_map.begin()+i);
            pers_map.push_back(null);
            pers_index --;
            i--;
        }
        else if(pers_map[i][3]>= consensus_th){
            temp << pers_map[i][0], pers_map[i][1], 1;
            
            temp = map_to_vehicle.inverse()*temp;

            meas_x_filtered.push_back(temp(0));
            meas_y_filtered.push_back(temp(1));            

            point.x = temp(0);
            point.y = temp(1);
            point.z = 0.0;

            debug_cloud.points.push_back(point);
            debug_cloud.width++; 
        }
    }

    pcl::toROSMsg(debug_cloud,debug_map);  
    debug_map.header.frame_id = "laser_frame";
    debug_map.header.stamp = ros::Time::now();  
    cloud_pub_.publish (debug_map);
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

int VFHController::findSectorIdx(double angle, float sector_limits_up[]) {
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
    for (int j=0; j<num_of_sector; j++) {
        sector_array[j] = sector_value + j ; // - 1
    }
    
    std::vector<double> gaussian_distribution(num_of_sector);
    normpdf(sector_array,sector_index,gaussian_weight,gaussian_distribution);
    int max_idx = round(*max_element(sector_array.begin(), sector_array.end()));
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

void VFHController::vfhController() {
    int data_length = meas_x_filtered.size();
    direction = 0;
    speed_cmd = 0;

    if(data_length==0)
        return;

    float min_lateral_dist = 0.2; // [m]
    if (lateral_dist < min_lateral_dist) {
        target_dir_weight = 0.01;
        prev_dir_weight = 0.015;
    } else {
        prev_dir_weight = 0.01;
        target_dir_weight = 0.015;
    }
        
    // Add circles to x,y points
    int circle_points = 90;
    double measures_x[data_length*(1+circle_points)];
    double measures_y[data_length*(1+circle_points)];
    double x_value = 1000;
    double y_value = 1000;
    for (int i=0; i<data_length; i++) {
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
            measures_x[i+i*circle_points+j] = x_value + robot_radius*cos(2*M_PI*(double)j/(double)circle_points);
            measures_y[i+i*circle_points+j] = y_value + robot_radius*sin(2*M_PI*(double)j/(double)circle_points);
        }
    }  

    // define sector limits 
    size_t data_size = sizeof(measures_x)/sizeof(measures_x[0]);
    
    float sector_mean = 0;
    float sector_limits_up[num_of_sector];   
    float sector_limits_down[num_of_sector];   
    for (int i=0; i<num_of_sector; i++) {
        sector_mean = i*sector_width;
        sector_limits_up[i] = sector_mean + sector_width/2;
        sector_limits_down[i] = sector_mean - sector_width/2;
    }

    // associate euclidean distance to each sector
    std::vector<double> dist_to_associate(num_of_sector,100);
    int sector_index = 0;
    for (int i=0; i<data_size; i++) {
        double eucl_dist = sqrt(pow(measures_x[i],2) + pow(measures_y[i],2));
        double ang = 180/M_PI*atan2(measures_y[i],measures_x[i]);
        if (ang < 0) 
            ang = ang + 360;
        
        sector_index = findSectorIdx(ang,sector_limits_up);
        if (dist_to_associate[sector_index] > eucl_dist) 
            dist_to_associate[sector_index] = eucl_dist;
        
    }
    
    // get min distance
    int ang_lim = round(15/sector_width); // deg
    double min_dist = *min_element(dist_to_associate.begin(), dist_to_associate.begin()+ang_lim);
    double min_dist_2 = *min_element(dist_to_associate.end()-ang_lim, dist_to_associate.end());
    if (min_dist_2 < min_dist)
        min_dist = min_dist_2;
       

    bool stop = 0;
    if (min_dist < stop_distance) {
        stop = 1;
        ctrl_word = 1;
    } else if (min_dist < max_detection_dist || lateral_dist > 0.1 ) {
        stop = 0;
        ctrl_word = 1;
    } else {
        stop = 0;
        ctrl_word = 0;
    }
    
    if (min_dist >= max_detection_dist) 
        min_dist = max_detection_dist;
    
    // evaluate speed cmd
    float dist_upper_lim = max_detection_dist;
    float dist_lower_lim = 0.7;
    float speed_lower_lim = 0;
    float m = (speed_upper_lim - speed_lower_lim)/(dist_upper_lim - dist_lower_lim);
    
    if (stop) {
        speed_cmd = 0;
    } 
    else {
        if (min_dist <= 1) {
            speed_cmd = (m * (1 - dist_lower_lim));
        } else {
            speed_cmd = (m * (min_dist - dist_lower_lim));
        }
    }
    if (speed_cmd > speed_upper_lim)
        speed_cmd = speed_upper_lim;
    
    if( abs(speed_cmd-speed_cmd_prev)*node_frequency > linear_speed_lim){
        double sign = (speed_cmd-speed_cmd_prev)/abs(speed_cmd-speed_cmd_prev);
        speed_cmd = speed_cmd_prev + sign*linear_speed_lim/node_frequency;
    }
    speed_cmd_prev = speed_cmd;
    
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
    double gaussian_weight_obs = findGaussianWeight(coeff2)*gaussian_weight_coeff;
       
    // build target direction cost
    sector_index = findSectorIdx(ref_direction,sector_limits_up);
    buildCost(cost_target,sector_index,gaussian_shift,sector_array,gaussian_weight_target/2,1,-gaussian_weight_target/2);
    
    // build previous direction cost
    sector_index = findSectorIdx(prev_direction,sector_limits_up);
    buildCost(cost_prev,sector_index,gaussian_shift,sector_array,gaussian_weight_prev_dir/2,1,-gaussian_weight_target/2);
    
    // build obstacle cost and overall cost
    for (int k=0; k<num_of_sector; k++) {
        std::vector<double> cost_obst_full;
        for (int i=0; i<num_of_sector; i++) {
            double cost = 1/dist_to_associate[i];
            buildCost(cost_obst,i,gaussian_shift,sector_array,gaussian_weight_obs,0,cost);
            cost_obst_full.push_back(cost_obst[k]);
        }
        cost_obstacle[k] = std::accumulate(cost_obst_full.begin(), cost_obst_full.end(), 0.0);
        overall_cost[k] = obstacle_weight * cost_obstacle[k] + target_dir_weight * cost_target[k] + prev_dir_weight * cost_prev[k];
    }
  
    // get min cost and direction
    std::vector<int> vec;
    std::vector<int> bounds;
    float bound_ang = 40; // deg
    int boundaries[2] = {int(bound_ang/sector_width), num_of_sector - int(bound_ang/sector_width)};    
    
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
    
    for (int i=1; i<num_of_sector; i++) {
        if (i>boundaries[0] && i<boundaries[1])
            overall_cost[i]=1000;
    }
    int idx = min_element(overall_cost.begin(), overall_cost.end()) - overall_cost.begin();

  
    if (stop)
        direction = 0;
    else if (ctrl_word)
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
    geometry_msgs::Twist planner_msg;
    planner_msg.linear.x = speed_cmd;
    // planner_msg.linear.z = ctrl_word;
    planner_msg.angular.z = direction_gain*direction*M_PI/180.0;
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
        
        VFHController.update_pers_map();
        VFHController.vfhController();
    
        loop_rate.sleep();
    }

    return 0;
} 


