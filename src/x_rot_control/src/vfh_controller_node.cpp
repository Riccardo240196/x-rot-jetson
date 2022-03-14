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

    vehicle_to_radar << 1, 0, 0.65,
                        0, 1, 0,
                        0, 0, 1;   
    
    map_to_vehicle = Matrix<double, 3, 3>::Identity();
    
    vector<double> temp = {-1, -1, -1, -1};
    for(int i=0; i<300; i++){
        pers_map.push_back(temp);
    }
    pers_index = 0;
    node_frequency = 10;
    pers_time_th = 5;
    pers_dist_th = 0.2;
    consensus_th = 2;
    
}

void VFHController::initializeSubscribers()
{
    robot_pose_sub_ = nh_.subscribe("/can_odometry", 1, &VFHController::robotPoseCallback,this);  
    path_point_sub_ = nh_.subscribe("/path_point", 1, &VFHController::pathPointCallback,this);  
    radar_points_sub_ = nh_.subscribe("/radar_messages", 1, &VFHController::radarPointsCallback,this);  
}

void VFHController::initializePublishers()
{
    local_planner_pub_ = nh_.advertise<geometry_msgs::Twist>("/x_rot/cmd_vel", 1, true); 
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

void VFHController::radarPointsCallback(const radar_pa_msgs::radar_msg& msg) {
    // spacchetta i messagi del radar e trasforma i punti in X e Y
    Matrix<double, 3, 1> temp;

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
    // aggiorna i timer delle righe non nulle
    // elimina le righe che hanno superato il limite del timer
    // estrai righe non nulle e restituisci le matrici delle misure filtrate

    
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
        for(int ind=0; ind<300; ind++){
            diff_x = pers_map[i][0] - meas_raw_x[i];
            diff_y = pers_map[i][1] - meas_raw_y[i];
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
            if(pers_index+1<pers_map.size())
                pers_index++;
        }
        for (int j=0; j<close_points.size(); j++){
            pers_map[close_points[j]][3]++; 
        }
        
    }

    vector<double> null={-1, -1, -1, -1};
    meas_x_filtered.clear();
    meas_y_filtered.clear();
    Matrix<double, 3, 1> temp;

    for(int i=0; i<=pers_index; i++){
        pers_map[i][2]++;
        if(pers_map[i][2]*1/node_frequency > pers_time_th){
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
        }
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

int VFHController::findSectorIdx(double direction, float sector_limits_up[]) {
    int sector_index = num_of_sector;
    if (direction>=sector_limits_up[num_of_sector]) {
        sector_index = 0;
    } else {
        for (int j=0; j<num_of_sector; j++) {
            if (sector_limits_up[j]>=direction) {
                sector_index = j;
                break;
            }
        }
    }
    return sector_index;
}

void VFHController::buildDirectionCost(std::vector<double>& cost_vec, int sector_index, double gaussian_shift, std::vector<int>& sector_array, double gaussian_weight) {
    double sector_value = sector_index-gaussian_shift;
    for (int j=0; j<num_of_sector; j++) {
        sector_array[j] = sector_value + j - 1;
    }
    
    std::vector<double> gaussian_distribution(num_of_sector);
    normpdf(sector_array,sector_index,gaussian_weight/2,gaussian_distribution);
    for (int i=0; i<num_of_sector; i++) 
        gaussian_distribution[i] = gaussian_distribution[i]*gaussian_weight/2;

    int max_idx = round(*max_element(sector_array.begin(), sector_array.end()));
    if (max_idx < num_of_sector) {
        int start_ind = num_of_sector - max_idx;
        for (int j=0; j<max_idx; j++) {
            cost_vec[j] = 1 - gaussian_distribution[j+start_ind];
        }
        
        for (int j=max_idx; j<num_of_sector; j++) {
            cost_vec[j] = 1 - gaussian_distribution[j-max_idx];
        }
    } else {
        int start_ind = max_idx-num_of_sector;
        for (int j=0; j<start_ind; j++) {
            cost_vec[j] = 1 - gaussian_distribution[j+(num_of_sector-start_ind)];
        }
        
        for (int j=start_ind; j<num_of_sector; j++) {
            cost_vec[j] = 1 - gaussian_distribution[j-start_ind];
        }
    }
}

void VFHController::vfhController() {
    
    float obstacle_weight = 0.98;
    float target_dir_weight, prev_dir_weight;
    float min_lateral_dist = 0.2; // [m]
    if (lateral_dist < min_lateral_dist) {
        target_dir_weight = 0.01;
        prev_dir_weight = 0.015;
    } else {
        prev_dir_weight = 0.01;
        target_dir_weight = 0.015;
    }
    float max_detection_dist = 8; // [m]
    int sector_width = 360/(double)num_of_sector;
    float gaussian_weight_coeff = 0.5;
    float robot_radius = 0.6;
    int circle_points = 90;
        
    // Add circles to x,y points
    int data_length = meas_x_filtered.size();//*(1+circle_points);
    double measures_x[data_length*(1+circle_points)];
    double measures_y[data_length*(1+circle_points)];
    
    for (int i=0; i<data_length; i++) {
        if (meas_x_filtered[i]!=0 && meas_y_filtered[i]!=0) {
            measures_x[i+i*circle_points] = meas_x_filtered[i];
            measures_y[i+i*circle_points] = meas_y_filtered[i];
            for (int j=1; j<circle_points+1; j++) {      
                measures_x[i+i*circle_points+j] = meas_x_filtered[i] + robot_radius*cos(2*M_PI*(double)j/(double)circle_points);
                measures_y[i+i*circle_points+j] = meas_y_filtered[i] + robot_radius*sin(2*M_PI*(double)j/(double)circle_points);
            }
        }
    }   
    /*
    % define sector limits 
    data_size=length(measures.x);
    num_of_sector = 360/sector_width;
    overall_cost = 1000*ones(num_of_sector,1);   
    if num_of_sector>360
        num_of_sector = 360;
    }
    sector_mean = zeros(num_of_sector,1);
    sector_limits_up = zeros(num_of_sector,1);
    sector_limits_down = zeros(num_of_sector,1);
    for i=1:num_of_sector
        sector_mean[i] = (i-1)*sector_width;
        sector_limits_up[i] = sector_mean[i]+sector_width/2;
        sector_limits_down[i] = sector_mean[i]-sector_width/2;
    }
    dist_to_associate = ones(num_of_sector,1)*30;
 
    % associate euclidean distance to each sector
    for i=1:data_size
        eucl_dist = sqrt(measures.x[i]^2 + measures.y[i]^2);
        ang = 180/pi*atan2(measures.y[i],measures.x[i]);
        if(ang<0)
            ang = ang + 360;
        }
        
        sector_index = find_sector_idx(ang,sector_limits_up,num_of_sector);
        if (dist_to_associate(sector_index) > eucl_dist)
            dist_to_associate(sector_index) = eucl_dist;
        }          
        
    }
    
    % get min distance
    ang_lim = round(15/sector_width); % deg
    indexes = [1:ang_lim,(num_of_sector+1-ang_lim):num_of_sector];
    [min_dist, sector] = min(dist_to_associate(indexes));
        
    if (min_dist<0.8)
        stop = 1;
        ctrl_word = 1;
    elseif (min_dist<max_detection_dist || lateral_dist>0.1 )
        stop = 0;
        ctrl_word = 1;
    else
        stop = 0;
        ctrl_word = 0;
    }
    
    if (min_dist>=max_detection_dist)
        min_dist = max_detection_dist;
    }
    
    % evaluate speed cmd
    dist_upper_lim = max_detection_dist;
    dist_lower_lim = 0.7;
    speed_upper_lim = 0.3;
    speed_lower_lim = 0;
    m = (speed_upper_lim - speed_lower_lim)/(dist_upper_lim - dist_lower_lim);
    
    if (stop)
        speed = 0;
    else
        if (min_dist <= 1)
            speed = (m * (1 - dist_lower_lim));
        else
            speed = (m * (min_dist - dist_lower_lim));
            speed = speed_prev + 0.02*(speed-speed_prev); %m/10     0.0001
        }
    }
    if speed > 0.3
        speed = 0.3;
    }
    
    % BUILD COSTS
    sector_array = zeros(num_of_sector,1);
    cost_target = zeros(num_of_sector,1);
    cost_prev = zeros(num_of_sector,1);
    cost_obstacle = zeros(num_of_sector,1);
    cost_obst = zeros(num_of_sector,num_of_sector);
    
    % Gaussian properties
    coeff = [0.845015753269096,4.96041666063205,-0.188303172580222,-4.44023451783892];
    fun = @(B,x)  B(1) + B(2).*exp(B(3).*(x + B(4)));
    gaussian_shift = round(num_of_sector/2);
    gaussian_weight_target = fun(coeff,sector_width);
    gaussian_weight_prev_dir = fun(coeff,sector_width);
    coeff = [0.369389785763626,3.64914690021850,-0.216102762982391,-3.84497580829445];
    fun = @(B,x)  B(1) + B(2).*exp(B(3).*(x + B(4)));
    gaussian_weight_obs = fun(coeff,sector_width)*gaussian_weight_coeff;
       
    % build target direction cost
    sector_index = find_sector_idx(ref_direction,sector_limits_up,num_of_sector);
    cost_target = build_direction_cost(cost_target,num_of_sector,sector_index,gaussian_shift,sector_array,gaussian_weight_target);
    
    % build previous direction cost
    sector_index = find_sector_idx(prev_direction,sector_limits_up,num_of_sector);
    cost_prev = build_direction_cost(cost_prev,num_of_sector,sector_index,gaussian_shift,sector_array,gaussian_weight_prev_dir);
    
    % build obstacle cost
    cost = 0.25;
    for k=1:num_of_sector
        sector_value = k-gaussian_shift;
        for j=1:num_of_sector
            sector_array(j) = sector_value + j - 1;
        }
        cost = 1./dist_to_associate(k);
        gaussian_obst = normpdf(sector_array,k,gaussian_weight_obs);
        max_idx = round(max(sector_array));
        if (max_idx < num_of_sector)
            start_ind = num_of_sector - max_idx;
            for j=1:max_idx
                cost_obst(j,k) = cost*gaussian_obst(j+start_ind);
            }

            for j=max_idx+1:num_of_sector
                cost_obst(j,k) = cost*gaussian_obst(j-max_idx);
            }
        else
            start_ind = max_idx-num_of_sector;
            for j=1:start_ind
                cost_obst(j,k) = cost*gaussian_obst(j+(num_of_sector-start_ind));
            }

            for j=start_ind+1:num_of_sector
                cost_obst(j,k) = cost*gaussian_obst(j-start_ind);
            }
        }
    }    
    for k=1:num_of_sector
        cost_obstacle(k) = sum(cost_obst(k,:));
    }
    
    % build overall cost
    for k=1:num_of_sector
        overall_cost(k) = w1 * cost_obstacle(k) + w2 * cost_target(k) + w3 * cost_prev(k);
    }
    
    % get min cost and direction
    vec = zeros(num_of_sector,1);
    boundaries1 = zeros(2,1); 
    boundaries2 = zeros(2,1); 
    boundaries = zeros(2,1);    
    
    for i=2:num_of_sector
        if abs(overall_cost[i] - overall_cost(i-1))>1e-4
            vec[i] = i-1;
        }
    }
    indexes = maxk(abs(diff(vec)),2);
    bound_ang = 40; % deg
    if (length(indexes)==2)
        boundaries1(1) = indexes(2);% - floor(bound_width/5);
        boundaries1(2) = indexes(1);% + floor(bound_width/5); 
        boundaries2(1) = round(bound_ang/sector_width);
        boundaries2(2) = num_of_sector - round(bound_ang/sector_width); 
        boundaries(1) = min(boundaries1(1),boundaries2(1));
        boundaries(2) = max(boundaries1(2),boundaries2(2));
       
    else
        boundaries(1) = round(bound_ang/sector_width);
        boundaries(2) = num_of_sector - round(bound_ang/sector_width); 
    }
    
    for i=1:num_of_sector
        if i>boundaries(1) && i<boundaries(2)
            overall_cost[i]=1000;
        }
    }
    [min_cost, idx]=min(overall_cost);
    
    if stop
        direction = 0;
    elseif ctrl_word
        direction = sector_limits_up(idx)-sector_width/2;
    else
        direction = 0;
    }

}
*/
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
    
        loop_rate.sleep();
    }

    return 0;
} 