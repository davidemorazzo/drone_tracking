/*ROS 2*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs//msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

/* PX4 MSGS */
#include "px4_msgs/msg/vehicle_odometry.hpp"

/* STD LIBRARIES */
#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
using std::placeholders::_1;

rclcpp::QoS px4_sub_qos = rclcpp::QoS(0)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);


class EASN_estimation : public rclcpp::Node
{
public:
	EASN_estimation() : Node("EASN_estimation"){

		// Declare parameters to parametrize the node
		this->declare_parameter("self_id", "0");
		this->declare_parameter("nA_id", "1");
		this->declare_parameter("nB_id", "2");
		this->me = this->get_parameter("self_id").get_parameter_value().get<std::string>();
		this->nA = this->get_parameter("nA_id").get_parameter_value().get<std::string>();
		this->nB = this->get_parameter("nB_id").get_parameter_value().get<std::string>();
			
		this->generate_subscribers();
		this->generate_publishers();

		// Variable initialization
		this->lambda = {0, 0, 0, 0};     
		this->phi = {0, 0, 0, 0};     
		this->dist= {0, 0, 0, 0};      
		this->dist_phi = {0, 0, 0, 0};     
		this->dist_lambda = {0, 0, 0, 0};    
    	this->u_alpha = {0, 0, 0};     
		this->u_x = {0, 0, 0};     
		this->u_y = {0, 0, 0};     
		this->e_r = {0, 0, 0}; 

		this->e_vx_0t = 0;   
		this->e_vy_0t= 0;      
		this->DT = 0.05;     

		/* Matrix initialization for the estimation */  
		this->I_meas_nA << 0,0,0,0,
    		0,0,0,0,
    		0,0,0,0,
    		0,0,0,0;    
    				
		this->I_meas_nB << 0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0;
		
		this->Y_est << 0.001, 0.0  , 0.0  , 0.0,
				0.0  , 0.001, 0.0  , 0.0, 
				0.0  , 0.0  , 0.001, 0.0, 
				0.0  , 0.0  , 0    , 0.001 ;   
				
		this->F<< 1,0,DT,0,
				  0,1,0,DT, 
				  0,0,1,0, 
				  0,0,0,1;
				
		this->Q<< 1e-3,0,0,0,
				  0,1e-3,0,0, 
				  0,0,1e-3,0, 
				  0,0,0,1e-3;     
		
		this->R<< 0.5,0,
    	          0,0.5;

		this->y_est << 0,0,0,0;
     	this->i_meas_nA << 0,0,0,0;
     	this->i_meas_nB << 0,0,0,0;
	}

	/*Estimation algorithm. At the end publish the information and estimation topics*/
	void estimation(){
			// the estimation algorithm needs the precise DT in matrix F to properly estimate the target's velocity.
			F  << 1,0,this->measured_DT,0,
					0,1,0,this->measured_DT, 
					0,0,1,0, 
					0,0,0,1;
			
			///////////////////////////// PREDICTION////////////////////////////// 
				
			// eq. (16) of EASN
			P_pred = F*Y_est.inverse()*F.transpose() + Q;	  
			x_pred = F*Y_est.inverse()*y_est;		
			
			//////////////////////// MEASUREMENTS //////////////////////////////
			
			X = x_pred(0) - (self_odom.position[0]); //relative position between predicted target and UAV along x axis 
			Y = x_pred(1) + (self_odom.position[1]); //relative position between predicted target and UAV along y axis 

			grad_h << X/sqrt(pow(X,2)+pow(Y,2)), Y/sqrt(pow(X,2)+pow(Y,2)), 0, 0,
				-Y/(pow(X,2)+pow(Y,2))    , X/(pow(X,2)+pow(Y,2))    , 0, 0; // compute gradient of h to use in eq. (17) of EASN
			
			h_meas << sqrt(pow(X,2)+pow(Y,2)), atan2(Y,X) ; // compute h_i to use in eq. (17) of EASN
			
			//get the actual measurement from the sensor
			z_meas << this->last_sensor_info.data[0], this->last_sensor_info.data[1];
					
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
			self_info.data.clear();	
			for(int i = 0; i < n; i++){self_info.data.push_back(i_meas(i));}
			for(int i = 0; i < n; i++){for(int j = 0; j < n; j++){self_info.data.push_back(I_meas(i,j));}}
			
			
			// this is just for plotting the results
			my_est.data.clear();	
			for(int i = 0; i < n; i++){my_est.data.push_back(x_est(i));}
			my_est.data.push_back(self_odom.position[0]); // Position X [m]
			my_est.data.push_back(self_odom.position[1]); // Position Y [m]
			my_est.data.push_back(self_odom.position[2]); // Position Z [m]
			my_est.data.push_back(self_odom.velocity[0]); // Velocity X [m/s]
			my_est.data.push_back(self_odom.velocity[1]); // Velocity Y [m/s]
			
			
			this->self_info_pub->publish(self_info);
			this->self_estimate_pub->publish(my_est);
			
			/* in rho_theta.data[6] we find the time instant in which 
			the last measurement was performed. This is crucial to obtain precise velocity estimate. */
			// prev_time = rho_theta.data[6];
			// prev_time = this->last_sensor_info.data[3];
			
	};
	
	/*Flocking Algorithm*/
	void compute_flocking_cmd(){	
		///////////////////////////////   FLOCKING ALGORITHM   ////////////////////////////////////

		// FIXME: possibile errore tra scambio di X e Y
		// lambda is the X coordinate (lo spero...) 
		lambda[std::stoi(me)] = this->self_odom.position[0];
		lambda[std::stoi(nA)] = this->nA_odom.position[0];
		lambda[std::stoi(nB)] = this->nB_odom.position[0];
		// phi is the y coordinate (lo spero parte 2 ...)
		phi[std::stoi(me)] = -this->self_odom.position[1];
		phi[std::stoi(nA)] = -this->nA_odom.position[1];
		phi[std::stoi(nB)] = -this->nB_odom.position[1];
			
				
		for (int i = 0; i < 3; ++i){
		
			if( i != std::stoi(me)){
				// Relative distance from "me" to drone i-th
				double diff_vec[3] = {
					this->self_odom.position[0] - this->lambda[i],	// X
					-this->self_odom.position[1] - this->phi[i],	// Y
					0.0												// Z    //this->self_odom.position[0] - this->lambda[i]
				};

				dist[i] = this->vector3_norm(diff_vec);
				dist_phi[i] = this->phi[std::stoi(me)] - this->phi[i];				// FIXME: la distanza deve avere segno o no?
				dist_lambda[i] = this->lambda[std::stoi(me)] - this->lambda[i];		// FIXME: la distanza deve avere segno o no?
				
				//compute distance regulator input + integral action on the relative distance --> eq(3) + eq. (8) of EASN
				u_alpha[i] = (rho_p(sigma_norm(dist[i])/sigma_norm(4.8))*k_P * phi_p(sigma_norm(dist[i])-sigma_norm(4)) 
							+ k_I * e_r[i])/sqrt(1 + 0.1*pow(dist[i],2));  
				//compute x and y components of u_alpha      
				u_x[i] = u_alpha[i]*dist_lambda[i];
				u_y[i] = u_alpha[i]*dist_phi[i];
				//compute the discrete integral of error in relative distance (eq. (7) of EASN).  the "bump" function is needed to reset the error when communication is lost
				e_r[i] = (phi_p(sigma_norm(dist[i])-sigma_norm(4)) + e_r[i])*bump(sigma_norm(dist[i])/sigma_norm(4.8)); 
			}}
		
		//compute x component of eq. (5) in EASN    
		u_vx = -(this->self_odom.velocity[0] - this->nA_odom.velocity[0]) * rho_vel(sigma_norm(dist[std::stoi(nA)])/sigma_norm(4.8)) 
			   -(this->self_odom.velocity[0] - this->nB_odom.velocity[0]) * rho_vel(sigma_norm(dist[std::stoi(nB)])/sigma_norm(4.8)); 
		//compute y component of eq. (5) in EASN 
		u_vy = -(this->self_odom.velocity[1] - this->nA_odom.velocity[1]) * rho_vel(sigma_norm(dist[std::stoi(nA)])/sigma_norm(4.8))
			   -(this->self_odom.velocity[1] - this->nB_odom.velocity[1]) * rho_vel(sigma_norm(dist[std::stoi(nB)])/sigma_norm(4.8));
			
	
		// compute relative distance and relative x and y positions between UAV and TARGET
		// Y has to be inverted because self_odom ref frame is FRD and x_est is FLU
		dist_lambda[3] = x_est(0) - (this->self_odom.position[0]); // + x0[std::stoi(me)]);
		dist_phi[3] =    x_est(1) + (this->self_odom.position[1]); // + y0[std::stoi(me)]);
		dist[3] = sqrt(pow(x_est(0) - (this->self_odom.position[0]),2) + // + x0[std::stoi(me)]),2) + 
						pow(x_est(1) + (this->self_odom.position[1]),2));		// + y0[std::stoi(me)]),2)); 
			
		
		
		if(count > 20){k_abs = 1;} //start chasing the target after 20 iterations (1 second). This is because the initial estimates are very far from actual target states.
		count = count +1;
		
		
		//compute control input along x axis
		this->cmd_force_x = 
				0.25 * (
				  u_x[std::stoi(nA)]
				+ u_x[std::stoi(nB)]
				+ k_d*u_vx 
				+ k_abs*(c_1*dist_lambda[3]*rho_int(dist[3]/d_int)
				+ c_2*(x_est(2)-this->self_odom.velocity[0])
				+ c_int*e_vx_0t)); 
		
		//compute control input along y axis
		this->cmd_force_y = 
				0.25 * (
				  u_y[std::stoi(nA)]
				+ u_y[std::stoi(nB)]
				+ k_d*u_vy 
				+ k_abs*(c_1*dist_phi[3]*rho_int(dist[3]/d_int) 
				+ c_2*(x_est(3)-this->self_odom.velocity[1])  
				+ c_int*e_vy_0t));
		
		
		//discrete version of eq. (9) of EASN.  velocity error between UAV and target. 
		e_vx_0t = k_abs * ((x_est(2) - this->self_odom.velocity[0])) + e_vx_0t;
		e_vy_0t = k_abs * ((x_est(3) - this->self_odom.velocity[1])) + e_vy_0t;
			

		
		/*        dist_plot.pose.orientation.x = dist[std::stoi(nA)];
		dist_plot.pose.orientation.y = dist[std::stoi(nB)];
		dist_plot.pose.orientation.z = dist[3];
		dist_plot.pose.orientation.w = x_est(3);*/
			

		// raw_pub.publish(cmd_raw);
		// }
		// else {local_pos_pub.publish(self_pose);}
		
		
		// my_global.publish(self_state_global);
		// my_vel.publish(self_vel);        
		// distances.publish(dist_plot);
	
		// ros::spinOnce();
		// rate.sleep();
	}

private:
	/* -------------------------------------------------------------------------------*/
	/*---------------------------- ROS 2 COMMUNICATION -------------------------------*/

	// std_msgs::msg::Float64MultiArray rho_theta;

	std::string me = "0";
	std::string nA = "1";
	std::string nB = "2";
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr self_odometry_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr nA_odometry_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr nB_odometry_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nA_info_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nB_info_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_sub;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr self_info_pub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr self_estimate_pub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drone_command_pub;

	px4_msgs::msg::VehicleOdometry nA_odom, nB_odom, self_odom;
	std_msgs::msg::Float64MultiArray info_nA, info_nB, self_info;
	std_msgs::msg::Float64MultiArray my_est;
	std_msgs::msg::Float64MultiArray last_sensor_info;

	void generate_subscribers(){
		/* Odometry topics */
		this->self_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/drone" + std::to_string(std::stoi(this->me)+2) + "/fmu/out/vehicle_odometry", px4_sub_qos,
			std::bind(& EASN_estimation::self_odom_cb, this, _1)); 
		this->nA_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/drone" + std::to_string(std::stoi(this->nA)+2) + "/fmu/out/vehicle_odometry", px4_sub_qos,
			std::bind(& EASN_estimation::nA_odom_cb, this, _1)); 
		this->nB_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/drone" + std::to_string(std::stoi(this->nB)+2) + "/fmu/out/vehicle_odometry", px4_sub_qos,
			std::bind(& EASN_estimation::nB_odom_cb, this, _1));
		
		/* Estimator topics */
		this->sensor_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/drone" + std::to_string(std::stoi(this->me)+2) + "/sensor_measure", 10,
			std::bind(& EASN_estimation::sensor_cb, this, _1)); 
		this->nA_info_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/drone" + std::to_string(std::stoi(this->nA)+2) + "/information", 10,
			std::bind(& EASN_estimation::nA_info_cb, this, _1)); 
		this->nB_info_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/drone" + std::to_string(std::stoi(this->nB)+2) + "/information", 10,
			std::bind(& EASN_estimation::nB_info_cb, this, _1)); 
	}

	void generate_publishers(){
		std::string self_topic_id = std::to_string(std::stoi(this->me)+2);
		this->self_info_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drone" + self_topic_id + "/information", 10);
		this->self_estimate_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drone" + self_topic_id + "/estimation", 10);
		this->drone_command_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drone" + self_topic_id + "/acceleration_cmd",10);
	}

	void self_odom_cb(const px4_msgs::msg::VehicleOdometry msg){this->self_odom = msg;};
	void nA_odom_cb(const px4_msgs::msg::VehicleOdometry msg){this->nA_odom = msg;};
	void nB_odom_cb(const px4_msgs::msg::VehicleOdometry msg){this->nB_odom = msg;};
	
	void nA_info_cb(const std_msgs::msg::Float64MultiArray msg){this->info_nA = msg;};
	void nB_info_cb(const std_msgs::msg::Float64MultiArray msg){this->info_nB = msg;};

	/* Execute the estimation and compute the flocking command each time a new sensor 
		message is available */
	void sensor_cb(const std_msgs::msg::Float64MultiArray msg){
		if(this->last_sensor_info.data.empty()){
			this->last_sensor_info = msg;
		}
		// Calculate DT in seconds
		this->measured_DT = this->last_sensor_info.data[2] - msg.data[2];
		this->last_sensor_info = msg;

		// Do estimation
		this->estimation();

		if(this->me == "1"){
			// RCLCPP_INFO(get_logger(), "x=%.3f y=%.3f vx=%.3f vy=%.3f X=%.3f Y=%.3f", x_est[0], x_est[1],
			// 	x_est[2], x_est[3], this->X, this->Y);
			// RCLCPP_INFO(get_logger(), "H_meas: %.3f m %.3f rad  Z_meas: %.3f m %.3f rad",
			// 	h_meas[0], h_meas[1], z_meas[0], z_meas[1]);

		}
		this->compute_flocking_cmd();
		if(this->me == "0"){
			// RCLCPP_INFO(get_logger(), "acc_x=%.3f m/s2 acc_y=%.3f m/s^2", cmd_force_x, cmd_force_y);
			RCLCPP_INFO(get_logger(), "u_d[1]=(%.3f  %.3f) u_v=(%.3f  %.3f)", u_x[1], u_y[1], u_vx, u_vy);
		}

		// Publish acceleration command 
		std_msgs::msg::Float64MultiArray acc_msg;
		acc_msg.data.clear();
		acc_msg.data.push_back(this->cmd_force_x);
		acc_msg.data.push_back(this->cmd_force_y);
		this->drone_command_pub->publish(acc_msg);

	}	
	/* -------------------------------------------------------------------------------*/
	/*-------------------------- ESTIMATION VARIABLES --------------------------------*/


	/* x0 and y0 are the initial positions of the UAVs in the 
	  global coordinate frame. DT is the step time of the 
	  estimation algorithm (DT = 0.05 s = 1/20 Hz) */
	double e_vx_0t, e_vy_0t, u_vx, 
		u_vy, 
		x0[3] = {0.5, -0.5, -0.5}, 		// [m] x starting points of drones, used as local origin for the drone
		y0[3] = {0.5,  0.5, -0.5}, 		// [m] y starting points of drones, used as local origin for the drone
		DT;								// [s] timestep between rho measurements


	Matrix4d Y_pred, Y_est, F, Q, P_pred, I_meas, I_meas_nA, I_meas_nB;
	Matrix2d R;
	Matrix<double, 2, 4> grad_h;
	Vector4d  y_pred, y_est, i_meas, x_pred, x_est, i_meas_nA, i_meas_nB;
	Vector2d z_meas, h_meas;
	int n = 4, k_abs = 0, count = 0 ;
    double X, Y;
    // double prev_time = 0;
	double measured_DT;			// [s] seconds between 2 sensor measurements measured from timestamp

	/*-----------------------------------------------------------------------*/
	/*----------------- REGULATOR VARIABLES ---------------------------------*/

	// Control parameters, tuned for good performances
    double R_E = 6371008.7714;
	double k_I= 0.001;
	double k_P = 1;
	double k_d = 2;
	double c_1 = 0.5;
	double c_2 = 0.75;
	double c_int = 0.05;
	double d_int = 3; 

	std::vector<double> lambda;			// [rad] latitude
	std::vector<double> phi;			// [rad] longitude
	std::vector<double> dist; 			// [m] relative distance to other drones and target
	std::vector<double> dist_phi;
	std::vector<double> dist_lambda;
	std::vector<double> u_alpha, u_x, u_y, e_r;
	double cmd_force_x = 0, 			// [m/s^2] x acceleration command for the drone
			cmd_force_y = 0;			// [m/s^2] y acceleration command for the drone
	
	/*----------------------------------------------------------------------*/
	double vector3_norm(double vector[3]){
		return sqrt(
			pow(vector[0],2)+
			pow(vector[1],2)+
			pow(vector[2],2)
		);
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
};




int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EASN_estimation>());
	rclcpp::shutdown();
	return 0;
}