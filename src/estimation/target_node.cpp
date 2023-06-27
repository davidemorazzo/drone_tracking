#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <math.h>
#include <random>
#include <Eigen/Core>
#include <Eigen/Dense>

int N = 3;

struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
    }
};
  
mavros_msgs::State current_state_t;
void state_cb_t(const mavros_msgs::State::ConstPtr& msg){
    current_state_t = *msg;
}

 
 geometry_msgs::PoseStamped target_state;
void self_state_cb_t(const geometry_msgs::PoseStamped::ConstPtr& msg){
    target_state = *msg;
}
 geometry_msgs::TwistStamped target_vel;
void self_vel_cb_t(const geometry_msgs::TwistStamped::ConstPtr& msg){
    target_vel = *msg;
}
 
  geometry_msgs::PoseStamped uav0_state;
void uav0_state_cb_t(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav0_state = *msg;
}
  geometry_msgs::PoseStamped uav1_state;
void uav1_state_cb_t(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_state = *msg;
}
  geometry_msgs::PoseStamped uav2_state;
void uav2_state_cb_t(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_state = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node_target");
    ros::NodeHandle nh;
 
    ros::Subscriber state_sub_t = nh.subscribe<mavros_msgs::State>
            ("target/mavros/state", 10, state_cb_t);
    ros::Subscriber self_state_sub_t = nh.subscribe<geometry_msgs::PoseStamped>
            ("target/mavros/local_position/pose", 10, self_state_cb_t);   
    ros::Subscriber self_vel_sub_t = nh.subscribe<geometry_msgs::TwistStamped>
            ("target/mavros/local_position/velocity_local", 10, self_vel_cb_t);   
            
    ros::Subscriber uav0_state_sub_t = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 10, uav0_state_cb_t);  
    ros::Subscriber uav1_state_sub_t = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, uav1_state_cb_t);  
    ros::Subscriber uav2_state_sub_t = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, uav2_state_cb_t);     
            
    ros::Publisher start_flag_pub = nh.advertise<std_msgs::Bool>
            ("target/start_flag",10);             
            
    ros::Publisher mimic_sensor_pub = nh.advertise<std_msgs::Float64MultiArray>
            ("target/mimic_sensor", 10);
    ros::Publisher local_pos_pub_t = nh.advertise<geometry_msgs::PoseStamped>
            ("target/mavros/setpoint_position/local", 10);
    ros::Publisher raw_pub_t = nh.advertise<mavros_msgs::PositionTarget>
            ("target/mavros/setpoint_raw/local", 10);           
    ros::ServiceClient arming_client_t = nh.serviceClient<mavros_msgs::CommandBool>
            ("target/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_t = nh.serviceClient<mavros_msgs::SetMode>
            ("target/mavros/set_mode");
            
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && !current_state_t.connected){
        ros::spinOnce();
        rate.sleep();
    }
     // Commands to Target before mission start--> position command --> reach a prescribed altitude of 4m from your initial position
    geometry_msgs::PoseStamped t_pos;
    t_pos.pose.position.x = 0;
    t_pos.pose.position.y = 0;        
    t_pos.pose.position.z = 4;
    
    // Commands to UAV after mission start --> position in z and velocity in x and y 
    t_raw.position.z = 4;
    t_raw.velocity.x = 0.5;
    t_raw.velocity.y = 0.5; 
    t_raw.coordinate_frame = 1;
    t_raw.type_mask = 1 | 2 | 32 | 64 | 128 | 256 | 256 | 512 | 1024 | 2048;
    
    ros::Time alt_reached;

    std_msgs::Bool start_flag;
    start_flag.data = 0;
    
    bool start_target = 0;
    double  time = 0, x0[3] = {3,2,-2}, y0[3] = {4, -2, 1}, rho, theta;
    std::vector<double> noise;
    
    std_msgs::Float64MultiArray rho_theta;

    
    float  uav_position_x[N], uav_position_y[N], R[2][2] = {1.4, 0, 0, 0.1};    
    
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0,sqrt(R[1][1]));
 	
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_t.publish(t_pos);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now(); 
    
    while(ros::ok()){
        if( current_state_t.mode != "OFFBOARD" && 
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client_t.call(offb_set_mode) && 
                offb_set_mode.response.mode_sent){
                ROS_INFO("Enabling Offboard");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state_t.armed && 
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client_t.call(arm_cmd) && 
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }                           
          	// from here to line 191 it's just a check to make sure all the UAV have reached an altitude close to the desired one, 
            // and that a sufficient amount of time (10 s) passed before the mission actually starts
        if((uav0_state.pose.position.z < 2 || uav1_state.pose.position.z < 2 || uav2_state.pose.position.z < 2) && start_target == 0){ 
              local_pos_pub_t.publish(t_pos);
              alt_reached = ros::Time::now();
              }
        else{ 
          start_target = 1;
      	  if(ros::Time::now() - alt_reached < ros::Duration(10.0))
      	     {local_pos_pub_t.publish(t_pos);}
         else{	
            rho_theta.data.clear();
            start_flag.data = 1;
            
            //here the mission actually starts
            //collect x and y positions from UAVs
            uav_position_x[0] = uav0_state.pose.position.x;
            uav_position_x[1] = uav1_state.pose.position.x;
            uav_position_x[2] = uav2_state.pose.position.x;
            
            uav_position_y[0] = uav0_state.pose.position.y;
            uav_position_y[1] = uav1_state.pose.position.y;
            uav_position_y[2] = uav2_state.pose.position.y;
            
            for (int i = 0; i < N; ++i){
              
              //compute the real range and bearing
              rho = sqrt(pow(target_state.pose.position.x - (uav_position_x[i] + x0[i]),2) + pow(target_state.pose.position.y - (uav_position_y[i] + y0[i]),2));
              theta = atan2(target_state.pose.position.y - (uav_position_y[i] + y0[i]), target_state.pose.position.x - (uav_position_x[i] +x0[i]));
	      
	      //compute real covariance matrix
		Eigen::MatrixXd covar(2,2);
		covar << 0.4*pow(cos(theta),2) + 0.1*pow(rho*sin(theta),2), (-0.4+0.1)*(cos(theta)*sin(theta)),
      			  (-0.4+0.1)*(cos(theta)*sin(theta)), 0.4*pow(sin(theta),2) + 0.1*pow(rho*cos(theta),2);

		normal_random_variable sample { covar };   
		
		//std::cout << sample()<< std::endl;
		
		noise.resize(2);
		Eigen::Map<Eigen::VectorXd>(noise.data(), noise.size()) = sample();
		ROS_INFO("[%f], [%f]", noise[0], noise[1]);
	      //send out measurements affected by white noise errors   
     rho_theta.data.push_back( sqrt(pow(target_state.pose.position.x - (uav_position_x[i] + x0[i]),2) + pow(target_state.pose.position.y - (uav_position_y[i] + y0[i]),2)) +noise[0]);// distribution(generator)); //
     rho_theta.data.push_back( atan2(target_state.pose.position.y - (uav_position_y[i] + y0[i]), target_state.pose.position.x - (uav_position_x[i] +x0[i]))+ noise[1]);//distribution(generator)); //

         }
         
		//also send out timestamp of the last measurement for computing DT
         rho_theta.data.push_back(ros::Time::now().toSec());
           

            mimic_sensor_pub.publish(rho_theta);         
            
            //follow a sinusoidal path along y axis
            t_raw.velocity.y = 0.5*sin(0.1*time); //instead of constant velocity, travel along sinusoidal path in y coordinate
            raw_pub_t.publish(t_raw);
            time = time + 0.05;
            

            
            }}
              
        
	start_flag_pub.publish(start_flag);
	 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
 