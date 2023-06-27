#include <ros/ros.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <bits/stdc++.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

 std::string self = "0";
 std::string nA = "1"; // neighbour A
 std::string nB = "2"; // neighbour B
 std::string target = "3";
 
 int me = std::stoi(self);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 sensor_msgs::NavSatFix self_state_global;
void self_state_global_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    self_state_global = *msg;
}
 geometry_msgs::PoseStamped self_state_local;
void self_state_local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    self_state_local = *msg;
}
 geometry_msgs::TwistStamped self_vel;
void self_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    self_vel = *msg;
}


 sensor_msgs::NavSatFix neigh_stateA;
void neigh_state_cbA(const sensor_msgs::NavSatFix::ConstPtr& msg){
    neigh_stateA  = *msg;
}
 geometry_msgs::TwistStamped neigh_velA;
void neigh_vel_cbA(const geometry_msgs::TwistStamped::ConstPtr& msg){
    neigh_velA = *msg;
}
 std_msgs::Float64MultiArray info_nA;
void info_nA_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    info_nA = *msg;
}

 sensor_msgs::NavSatFix neigh_stateB;
void neigh_state_cbB(const sensor_msgs::NavSatFix::ConstPtr& msg){
    neigh_stateB = *msg;
}
 geometry_msgs::TwistStamped neigh_velB;
void neigh_vel_cbB(const geometry_msgs::TwistStamped::ConstPtr& msg){
    neigh_velB = *msg;
}
 std_msgs::Float64MultiArray info_nB;
void info_nB_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    info_nB = *msg;
}


 std_msgs::Float64MultiArray rho_theta;
void mimic_sensor_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    rho_theta = *msg;
}  
   std_msgs::Bool start_flag;
void target_start_cb(const std_msgs::Bool::ConstPtr& msg){
    start_flag = *msg;
}



double sigma_norm(double dist){

	double r , eps = 0.1; 

	r = (1/eps)*(sqrt(1+eps*pow(dist,2))-1);

	return r;
}



double rho_p(double z){
	double h = 0.2;
	if(z >= 0 && z < h){	return 1;}
	if(z>=h && z<= 1){
		double r;
		r = (1/2)*(1+cos(M_PI*(z-h)/(1-h)));
		return r;}
	else{return 0;}	
}


double rho_vel(double z){
	double h = 0.85;
	if(z >= 0 && z < h){	return 1;}
	if(z>=h && z<= 1){
		double r;
		r = (1/2)*(1+cos(M_PI*(z-h)/(1-h)));
		return r;}
	else{return 0;}	
}

double rho_int(double z){
       double r;
       r = 4*atan(z);
       return r;
}

double bump(double z){
	if(z<=1){return 1;}
	else {return 0;}
}

double phi_p(double z){
	double r;
	r = 2*z/(sqrt(1+pow(z,2)));
	return r;
}


int sign(double val){
	if(val > 0){return 1;}
	if(val < 0){return -1;}
	else {return 0;}
}
 
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node" + self);
    ros::NodeHandle nh;    
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav" + self + "/mavros/state", 10, state_cb);
    ros::Subscriber self_state_global_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav" + self + "/mavros/global_position/global", 10, self_state_global_cb); 
    ros::Subscriber self_state_local_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav" + self + "/mavros/local_position/pose", 10, self_state_local_cb);            
    ros::Subscriber self_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("uav" + self + "/mavros/local_position/velocity_local", 10, self_vel_cb);   
            
    ros::Subscriber neigh_state_subA = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav" + nA + "/offb/global", 10, neigh_state_cbA); 
    ros::Subscriber neigh_state_subB = nh.subscribe<sensor_msgs::NavSatFix>
            ("uav" + nB + "/offb/global", 10, neigh_state_cbB);
    ros::Subscriber neigh_vel_subA = nh.subscribe<geometry_msgs::TwistStamped>
            ("uav" + nA + "/offb/vel", 10, neigh_vel_cbA); 
    ros::Subscriber neigh_vel_subB = nh.subscribe<geometry_msgs::TwistStamped>
            ("uav" + nB + "/offb/vel", 10, neigh_vel_cbB);   
    ros::Subscriber information_subA = nh.subscribe<std_msgs::Float64MultiArray>
            ("uav" + nA + "/information", 10, info_nA_cb);
    ros::Subscriber information_subB = nh.subscribe<std_msgs::Float64MultiArray>
            ("uav" + nB + "/information", 10, info_nB_cb);           
                          
      
    ros::Subscriber mimic_sensor_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("target/mimic_sensor", 10, mimic_sensor_cb);
    ros::Subscriber target_start_sub = nh.subscribe<std_msgs::Bool>
            ("target/start_flag", 10, target_start_cb);              
                         
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav" + self + "/mavros/setpoint_position/local", 10);
    ros::Publisher raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("uav" + self + "/mavros/setpoint_raw/local", 10); 
            
    ros::Publisher information_pub = nh.advertise<std_msgs::Float64MultiArray>
            ("uav" + self + "/information", 10);
    ros::Publisher estimation_pub = nh.advertise<std_msgs::Float64MultiArray>
            ("uav" + self + "/estimation", 10);
    ros::Publisher my_global = nh.advertise<sensor_msgs::NavSatFix>
            ("uav" + self + "/offb/global", 10); 
    ros::Publisher my_vel = nh.advertise<geometry_msgs::TwistStamped>
            ("uav" + self + "/offb/vel", 10);                           
    ros::Publisher distances = nh.advertise<geometry_msgs::PoseStamped>
            ("uav" + self + "/chatter", 10);  
                  
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav" + self + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav" + self + "/mavros/set_mode");
            
            
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    

    geometry_msgs::PoseStamped dist_plot;     
    geometry_msgs::PoseStamped self_pose;    
   
    // Commands to UAV before mission start--> position command --> reach a prescribed altitude of 2.5m from your initial position
    self_pose.pose.position.x = 0;
    self_pose.pose.position.y= 0;
    self_pose.pose.position.z = 2.5;
    
    
    // Commands to UAV after mission start --> position in z and acceleration in x and y --> stay at 2.5m altitude and follow the flocking algorithm in x and y
    mavros_msgs::PositionTarget cmd_raw;
    cmd_raw.position.z = 2.5;
    cmd_raw.acceleration_or_force.x = 0;
    cmd_raw.acceleration_or_force.y = 0;
    cmd_raw.coordinate_frame = 1;
    cmd_raw.type_mask = 1 | 2 | 8 | 16 | 32 | 256 | 1024 | 2048;
   
    // Variable inizialization
    std::vector<double> lambda = {0, 0, 0, 0}, phi = {0, 0, 0, 0}, dist= {0, 0, 0, 0},  dist_phi = {0, 0, 0, 0}, dist_lambda = {0, 0, 0, 0};
    std::vector<double> u_alpha = {0, 0, 0}, u_x = {0, 0, 0}, u_y = {0, 0, 0}, e_r = {0, 0, 0};   
    
    // x0 and y0 are the initial positions of the UAVs in the global coordinate frame. DT is the step time of the estimation algorithm (DT = 0.05 s = 1/20 Hz)
    double e_vx_0t = 0, e_vy_0t= 0, u_vx, u_vy, x0[3] = {3, 2, -2}, y0[3] = {4, -2, 1}, DT = 0.05;
    
    // Control parameters, tuned for good performances
    double R_E = 6371008.7714, k_I= 0.001 , k_P = 1, k_d = 2, c_1 = 0.5, c_2 = 0.75, c_int = 0.05, d_int = 3; // c_1 = 0.75 // k_I= 0.001 , k_P = 1, k_d = 0.5, c_1 = 0.5, c_2 = 0.1, c_int = 0.075   
       
    Matrix4d Y_pred, Y_est, F, Q, P_pred, I_meas, I_meas_nA, I_meas_nB;
    
    // Matrix inizialization for estimation algorithm
    
    I_meas_nA << 0,0,0,0,
    		0,0,0,0,
    		0,0,0,0,
    		0,0,0,0;    
    				
    I_meas_nB << 0,0,0,0,
    		0,0,0,0,
    		0,0,0,0,
    		0,0,0,0;
    
    Y_est << 0.001, 0.0  , 0.0  , 0.0,
             0.0  , 0.001, 0.0  , 0.0, 
             0.0  , 0.0  , 0.001, 0.0, 
             0.0  , 0.0  , 0    , 0.001 ;   
             
    F    << 1,0,DT,0,
            0,1,0,DT, 
            0,0,1,0, 
            0,0,0,1;
            
    Q   << 1e-3,0,0,0,
      	    0,1e-3,0,0, 
    	    0,0,1e-3,0, 
    	    0,0,0,1e-3;
    
    Matrix2d R;
    
    R	  << 0.5,0,
    	     0,0.5;
    
    Matrix < double , 2 , 4 >  grad_h; 
    
    Vector4d  y_pred, y_est, i_meas, x_pred, x_est, i_meas_nA, i_meas_nB;
    
     y_est << 0,0,0,0;
     i_meas_nA << 0,0,0,0;
     i_meas_nB << 0,0,0,0;
 
    Vector2d z_meas, h_meas;
  
    int n = 4, k_abs = 0, count = 0 ;
    double X, Y;
    double prev_time = 0;
    
    std_msgs::Float64MultiArray my_info, my_est;
    
 
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(self_pose);
        ros::spinOnce();
        rate.sleep();    
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();

 
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && 
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && 
                offb_set_mode.response.mode_sent){
                ROS_INFO("Enabling Offboard");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && 
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        } 
        
           
       // the start_flag is provided by the target node. The algorithm starts 10 seconds after the UAVs have reached the desired altitude.    	
       if(start_flag.data == 1){ 
       
              if(prev_time!=0){
       // the estimation algorithm needs the precise DT in matrix F to properly estimate the target's velocity.
	 F  << 1,0,DT,0,
            0,1,0,DT, 
            0,0,1,0, 
            0,0,0,1;
            
            
       
       ///////////////////////////// PREDICTION////////////////////////////// 
        
	// eq. (16) of EASN
	P_pred = F*Y_est.inverse()*F.transpose() + Q;	  
	x_pred = F*Y_est.inverse()*y_est;		
	  
	//////////////////////// MEASUREMENTS //////////////////////////////
	
	X = x_pred(0)- (self_state_local.pose.position.x + x0[me]); //relative position between predicted target and UAV along x axis 
	Y = x_pred(1)- (self_state_local.pose.position.y + y0[me]); //relative position between predicted target and UAV along y axis 

	grad_h << X/sqrt(pow(X,2)+pow(Y,2)), Y/sqrt(pow(X,2)+pow(Y,2)), 0, 0,
		 -Y/(pow(X,2)+pow(Y,2))    , X/(pow(X,2)+pow(Y,2))    , 0, 0; // compute gradient of h to use in eq. (17) of EASN
	
	h_meas << sqrt(pow(X,2)+pow(Y,2)), atan2(Y,X) ; // compute h_i to use in eq. (17) of EASN
	
	z_meas << rho_theta.data[2*me], rho_theta.data[2*me+1]; //get the actual measurement from the sensor
	    	
	// eq. (17) of EASN    	
	I_meas = grad_h.transpose()*R.inverse()*grad_h;    	      
	i_meas = grad_h.transpose()*R.inverse()*(z_meas - h_meas + grad_h*x_pred); 
	
	
	
	//////////////////////////////UPDATE///////////////////////////////    
	
	//get the information matrix and vector from neighbors
	if (info_nA.data.size() != 0 && info_nB.data.size() != 0){	
	i_meas_nA << info_nA.data[0], info_nA.data[1], info_nA.data[2], info_nA.data[3];
	i_meas_nB << info_nB.data[0], info_nB.data[1], info_nB.data[2], info_nB.data[3];
	
	I_meas_nA << info_nA.data[4], info_nA.data[5], info_nA.data[6], info_nA.data[7],
		info_nA.data[8], info_nA.data[9], info_nA.data[10], info_nA.data[11],
		info_nA.data[12], info_nA.data[13], info_nA.data[14], info_nA.data[15],
		info_nA.data[16], info_nA.data[17], info_nA.data[18], info_nA.data[19];
		
	I_meas_nB << info_nB.data[4], info_nB.data[5], info_nB.data[6], info_nB.data[7],
		info_nB.data[8], info_nB.data[9], info_nB.data[10], info_nB.data[11],
		info_nB.data[12], info_nB.data[13], info_nB.data[14], info_nB.data[15],
		info_nB.data[16], info_nB.data[17], info_nB.data[18], info_nB.data[19];		
		}
	
	// eq. (18) of EASN 
	Y_est = P_pred.inverse() + I_meas + I_meas_nA + I_meas_nB;	
	y_est = P_pred.inverse() * x_pred + i_meas + i_meas_nA + i_meas_nB; 
	
	// compute the actual estimate vector
	x_est = Y_est.inverse()*y_est;
	
	// prepare the information matrix and vector to send to neighbors
	my_info.data.clear();	
	for(int i = 0; i < n; i++){my_info.data.push_back(i_meas(i));}
	for(int i = 0; i < n; i++){for(int j = 0; j < n; j++){my_info.data.push_back(I_meas(i,j));}}
	
	
	// this is just for plotting the results
	my_est.data.clear();	
	for(int i = 0; i < n; i++){my_est.data.push_back(x_est(i));}
	my_est.data.push_back(self_state_local.pose.position.x);
	my_est.data.push_back(self_state_local.pose.position.y);
	my_est.data.push_back(self_state_local.pose.position.z);
	my_est.data.push_back(self_vel.twist.linear.x );
	my_est.data.push_back(self_vel.twist.linear.y );
	
	
	information_pub.publish(my_info);
	estimation_pub.publish(my_est);
	
	// in rho_theta.data[6] we find the time instant in which the last measurement was performed. This is crucial to obtain precise velocity estimate.
	prev_time = rho_theta.data[6];



	
	///////////////////////////////   FLOCKING ALGORITHM   ////////////////////////////////////

       // transform the longitudes from degrees to rad
        lambda[std::stoi(self)] = self_state_global.longitude * M_PI/180;
        lambda[std::stoi(nA)] = neigh_stateA.longitude * M_PI/180;
        lambda[std::stoi(nB)] = neigh_stateB.longitude * M_PI/180;
        
        
        // transform the latitudes from degrees to rad
        phi[std::stoi(self)] = self_state_global.latitude* M_PI/180;
        phi[std::stoi(nA)] = neigh_stateA.latitude * M_PI/180;
        phi[std::stoi(nB)] = neigh_stateB.latitude * M_PI/180;
        

        
        for (int i = 0; i < 3; ++i){
        
        	if( i != me){
                	
                	// compute relative distance through Haversine formula (eq. (13) of ICUAS), and then compute relative x and y positions with respect to neighbors UAVs	 
        	       dist[i] = 2*R_E*asin(sqrt(pow(sin((phi[i]-phi[me])/2),2) + cos(phi[me])*cos(phi[i])*pow(sin((lambda[i]-lambda[me])/2),2)));
      		       dist_phi[i] = sign(phi[i]-phi[me])*2*R_E*asin(sqrt(pow(sin((phi[i]-phi[me])/2),2)));
       	       dist_lambda[i] = sign(lambda[i]-lambda[me])*sqrt(pow(dist[i],2) - pow(dist_phi[i],2));
        		
        		//compute distance regulator input + integral action on the relative distance --> eq(3) + eq. (8) of EASN
        	        u_alpha[i] = (rho_p(sigma_norm(dist[i])/sigma_norm(4.8))*k_P * phi_p(sigma_norm(dist[i])-sigma_norm(4)) + k_I * e_r[i])/sqrt(1 + 0.1*pow(dist[i],2));  
        	        //compute x and y components of u_alpha      
        		u_x[i] = u_alpha[i]*dist_lambda[i];
        		u_y[i] = u_alpha[i]*dist_phi[i];
        		//compute the discrete integral of error in relative distance (eq. (7) of EASN).  the "bump" function is needed to reset the error when communication is lost
       		 e_r[i] = (phi_p(sigma_norm(dist[i])-sigma_norm(4)) + e_r[i])*bump(sigma_norm(dist[i])/sigma_norm(4.8)); 
        		}}
         
        //compute x component of eq. (5) in EASN    
        u_vx = -(self_vel.twist.linear.x - neigh_velA.twist.linear.x) * rho_vel(sigma_norm(dist[std::stoi(nA)])/sigma_norm(4.8)) - (self_vel.twist.linear.x - neigh_velB.twist.linear.x) * rho_vel(sigma_norm(dist[std::stoi(nB)])/sigma_norm(4.8)); 
        //compute y component of eq. (5) in EASN 
        u_vy = -(self_vel.twist.linear.y - neigh_velA.twist.linear.y) * rho_vel(sigma_norm(dist[std::stoi(nA)])/sigma_norm(4.8)) - (self_vel.twist.linear.y - neigh_velB.twist.linear.y) * rho_vel(sigma_norm(dist[std::stoi(nB)])/sigma_norm(4.8));
               
       
       // compute relative distance and relative x and y positions between UAV and target
       dist_lambda[3] = x_est(0) - (self_state_local.pose.position.x + x0[me]);
       dist_phi[3] =    x_est(1) - (self_state_local.pose.position.y + y0[me]);
       dist[3] = sqrt(pow(x_est(0) - (self_state_local.pose.position.x + x0[me]),2) + pow(x_est(1) - (self_state_local.pose.position.y + y0[me]),2)); 
            
        
        
        if(count > 20){k_abs = 1;} //start chasing the target after 20 iterations (1 second). This is because the initial estimates are very far from actual target states.
        count = count +1;
        
        
        //compute control input along x axis
         cmd_raw.acceleration_or_force.x = 0.25 * (u_x[std::stoi(nA)]+u_x[std::stoi(nB)]+ k_d*u_vx + k_abs*(c_1*dist_lambda[3]*rho_int(dist[3]/d_int) + c_2*(x_est(2)-self_vel.twist.linear.x) + c_int*e_vx_0t)); 
         
        //compute control input along y axis
         cmd_raw.acceleration_or_force.y = 0.25 * (u_y[std::stoi(nA)]+u_y[std::stoi(nB)]+ k_d*u_vy + k_abs*(c_1*dist_phi[3]   *rho_int(dist[3]/d_int) + c_2*(x_est(3)-self_vel.twist.linear.y)  + c_int*e_vy_0t));
        
        
        //discrete version of eq. (9) of EASN.  velocity error between UAV and target. 
        e_vx_0t = k_abs * ((x_est(2) - self_vel.twist.linear.x)) + e_vx_0t;
        e_vy_0t = k_abs * ((x_est(3) - self_vel.twist.linear.y)) + e_vy_0t;
               

        
        /*        dist_plot.pose.orientation.x = dist[std::stoi(nA)];
        dist_plot.pose.orientation.y = dist[std::stoi(nB)];
        dist_plot.pose.orientation.z = dist[3];
        dist_plot.pose.orientation.w = x_est(3);*/
        	

        raw_pub.publish(cmd_raw);
        }
        else {local_pos_pub.publish(self_pose);}
        
        
        my_global.publish(self_state_global);
        my_vel.publish(self_vel);        
        distances.publish(dist_plot);
	 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}