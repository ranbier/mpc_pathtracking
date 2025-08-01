#ifndef ROSCONTROL_H
#define ROSCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>	
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <math.h>
#include <numeric>
#include <std_msgs/Bool.h>


#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/CollisionData.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "ros/time.h"

#include <mpc.h>

#define INTER_TIME_PLUS 1000000000
#define INTER_TIME_MIN 90000000
#define INTER_TIME_MAX 200000000
#define INTER_SPEED_TIME_MAX 3600000000
#define SOME_MIN 0.04
#define SOME_MAX 0.2

class ROSCONTROL: public MPC {
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

    const double WHEEL_BASE = 3.00;
	const double MAX_SEARCH_DIST = 10.0;

    double ex_x_, ex_y_;
    unsigned int ex_time_;
    int inter_time_;

    double cur_course_;
    double cur_speed_;
	double cur_accel_;

    int waypoints_size_;
	int waypoint_min_;
	int closest_waypoint_;

    bool is_pose_;
    bool is_lane_;
    bool is_course_;
	bool is_control_;

	double dist_;
	int current_mission_state_;
	int loader_number_;

	double cte_cost_weight_info;
	double epsi_cost_weight_info;
	double v_weight_info;
	double delta_cost_weight_info;
	double v_target_cost_weight_info;
	double delta_change_cost_weight_info;
	double v_target_change_cost_weight_info;
	double referance_v_info;
	double k_v_info;
	double dt_info;
	int mpc_step_info;



    geometry_msgs::PoseStamped cur_pose_;

    vector<waypoint_maker::Waypoint> waypoints_;

	ros::Publisher ackermann_pub_;
	ros::Publisher rviz_predicted_path_;
	ros::Publisher vis_pub_;

	ros::Subscriber odom_sub_;
	ros::Subscriber lane_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber imu_sub_;

	morai_msgs::CtrlCmd ackermann_msg_;

	tf::TransformBroadcaster tf_broadcaster_; // TF Broadcaster 선언

	// MPC Variables
	// float dt = 0.1;
	float steer;

	float velocity_actual;
    float steering_angle;
	double target_velocity;


public:
	ros::Time ex_stamp;
    ROSCONTROL() 
	{
		initSetup();
	}

	~ROSCONTROL() 
	{
		waypoints_.clear();
	}

    void initSetup()
    {
		ros::Time::init();
		MPC mpc;
		

		

        ex_x_ = 0.0;
		ex_y_ = 0.0;
		ex_time_ = 0;
        inter_time_ = 0;


        cur_course_ = .0;
        cur_speed_ = .0;

        waypoints_size_ = 0;
		waypoint_min_ = -1;
		closest_waypoint_ = -1;

        is_pose_ = false;
        is_lane_ = false;
        is_course_ = false;
		is_control_ = false;

		dist_ = 100.0;
		current_mission_state_ = -1;
		loader_number_ = 0;

		steer = 0.0;
		steering_angle = 0.0;
		target_velocity = 0.0;

		ackermann_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
		rviz_predicted_path_ = nh_.advertise<nav_msgs::Path>("/rviz_predicted_path", 1);
		vis_pub_ = nh_.advertise<visualization_msgs::Marker>("pose",1);

		odom_sub_ = nh_.subscribe("odom", 1, &ROSCONTROL::OdomCallback, this);
		lane_sub_ = nh_.subscribe("/local_path", 1, &ROSCONTROL::PathCallback, this);
		state_sub_ = nh_.subscribe("gps_state",1,&ROSCONTROL::StateCallback,this);
		imu_sub_ = nh_.subscribe("imu",1,&ROSCONTROL::ImuCallback,this);

    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) 
	{
		ros::Time cur_stamp = odom_msg->header.stamp;
		cur_pose_.pose.position = odom_msg->pose.pose.position;
		broadcastTransform(); // TF broadcast 호출
		double cur_time_sec = cur_stamp.toSec();
		double ex_time_sec = ex_stamp.toSec();

		double dt = cur_time_sec - ex_time_sec;
		if (dt < 0){
			dt = 0.1;
		}

		if (dt > SOME_MIN && dt < SOME_MAX){
			cur_speed_ = getSpeed(ex_x_, ex_y_, odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, dt);
	
		}

		ex_stamp = cur_stamp;
		ex_x_ = cur_pose_.pose.position.x;
		ex_y_ = cur_pose_.pose.position.y;
		is_pose_ = true;


		// visualization_msgs::Marker marker;
		// marker.header.frame_id = "map";
		// marker.header.stamp = ros::Time();
		// marker.ns = "my_namespace";
		// marker.id = 0;
		// marker.type = visualization_msgs::Marker::SPHERE;
		// marker.action = visualization_msgs::Marker::ADD;
		// marker.pose.position.x = odom_msg->pose.pose.position.x;
		// marker.pose.position.y = odom_msg->pose.pose.position.y;
		// marker.pose.position.z = odom_msg->pose.pose.position.z;
		// marker.pose.orientation.x = 0.0;
		// marker.pose.orientation.y = 0.0;
		// marker.pose.orientation.z = 0.0;
		// marker.pose.orientation.w = 1.0;
		// marker.scale.x = 0.3;
		// marker.scale.y = 0.3;
		// marker.scale.z = 0.1;
		// marker.color.a = 1.0; // Don't forget to set the alpha!
		// marker.color.r = 1.0;
		// marker.color.g = 0.0;
		// marker.color.b = 0.0;

		// vis_pub_.publish( marker );
	}

    // void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg) 
	// {
	// 	waypoints_.clear();
	// 	vector<waypoint_maker::Waypoint>().swap(waypoints_);
	// 	waypoints_ = lane_msg->waypoints;
	// 	waypoints_size_ = waypoints_.size();
	// 	if (waypoints_size_ != 0) is_lane_ = true;
	// }
	void PathCallback(const nav_msgs::Path::ConstPtr& path_msg)
	{
	    waypoints_.clear();
	    std::vector<waypoint_maker::Waypoint>().swap(waypoints_);

	    int index = 0;
	    for (const auto& pose_stamped : path_msg->poses) {
	        waypoint_maker::Waypoint wp;

	        wp.waypoint_index = index++;
	        wp.pose = pose_stamped;  // geometry_msgs::PoseStamped 그대로 복사
	        

	        waypoints_.push_back(wp);
	    }

	    waypoints_size_ = waypoints_.size();
	    is_lane_ = !waypoints_.empty();
	}

void broadcastTransform()
{

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();  
    transform.header.frame_id = "map";          
    transform.child_frame_id = "base_link";     


    transform.transform.translation.x = cur_pose_.pose.position.x;
    transform.transform.translation.y = cur_pose_.pose.position.y;
    transform.transform.translation.z = cur_pose_.pose.position.z;


    geometry_msgs::Quaternion orientation = cur_pose_.pose.orientation;


    if (std::isnan(orientation.x) || std::isnan(orientation.y) ||
        std::isnan(orientation.z) || std::isnan(orientation.w) ||
        (orientation.x == 0 && orientation.y == 0 &&
         orientation.z == 0 && orientation.w == 0)) {
        // ROS_WARN("[TF BROADCASTER] Invalid quaternion detected. Using identity quaternion.");
        

        double yaw = cur_course_; 
        

        tf::Quaternion tf_quat;
        tf_quat.setRPY(0.0, 0.0, yaw);  
        
        orientation.x = tf_quat.x();
        orientation.y = tf_quat.y();
        orientation.z = tf_quat.z();
        orientation.w = tf_quat.w();
    }

    transform.transform.rotation = orientation;

    tf_broadcaster_.sendTransform(transform);
}





	void StateCallback(const waypoint_maker::State::ConstPtr &state_msg)
	{
		dist_ = state_msg->dist;
		current_mission_state_ = state_msg->current_state;
		loader_number_ = state_msg->lane_number;
	}

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
	{
		tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
			imu_msg->orientation.z, imu_msg->orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll,pitch,yaw);
		cur_course_ = yaw;
		//cout << "cur_course is : " << cur_course_ << endl;
		// cur_course_ = yaw * (180.0 / M_PI);
		is_course_ = true;
	}

    void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) 
	{
		if (!waypoints_.empty()) 
		{
			double dist_min = MAX_SEARCH_DIST;
			for (int i = 0; i < waypoints_.size(); i++)
			{
				double dist = calcPlaneDist(current_pose, waypoints_[i].pose);
				if (dist < dist_min) 
				{
					dist_min = dist;
					waypoint_min_ = i;
				}
			}
			closest_waypoint_ = waypoint_min_;
		}
		else cout << "------ NO CLOSEST WAYPOINT -------" << endl;
	}

	double calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) 
	{
		return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y - pose2.pose.position.y, 2));
	}

	double getSpeed(double& ex_x, double& ex_y, double cur_x, double cur_y, double dt)
	{
        double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
        double speed = distance / dt;
		

		//cout << "cur_speed is : " << speed << endl;
		//cout << "---------------" << endl;
 
        // cout << "distance   :: " << distance << endl;
        // cout << "time   "  << inter_time_ <<  "    ::    speed  " << speed << endl;
        return speed;
	}

	double getAccel(double& ex_x, double& ex_y, double cur_x, double cur_y) {
		double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
		double speed = distance / inter_time_;
		double a = speed / inter_time_ * INTER_SPEED_TIME_MAX;

		return a;
	}

	


	void modelPredictiveController() {

    	getClosestWaypoint(cur_pose_);

    	if(is_pose_ && is_course_ && is_lane_) 
    	{
    	    is_control_ = true;

    	    double px = ex_x_;
    	    double py = ex_y_;
			

    	    double psi = cur_course_;
    	    double velocity = cur_speed_; // 10km/h -> m/s
			double dt_ = 0.1;

    	    VectorXd vehicle_waypoints_x(waypoints_size_);
    	    VectorXd vehicle_waypoints_y(waypoints_size_);

    	    for (int i = 0; i < waypoints_size_; i++) 
    	    {
    	        double diff_x = waypoints_[i].pose.pose.position.x - px;
    	        double diff_y = waypoints_[i].pose.pose.position.y - py;
    	        vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
    	        vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
    	    }

    	    auto coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);
    	    const double cte = polyeval(coeffs, 0);
    	    const double epsi = -atan(coeffs[1]); // const add

		

    	    Eigen::VectorXd state(6);
    	    state = predict_FutureState(velocity, steering_angle, cte, epsi, target_velocity, dt_);

    	    auto output = Solve(state, coeffs);

    	    steering_angle = output[0];
			target_velocity = output[1];

			std::vector<double> predicted_x;
			std::vector<double> predicted_y;

			for (size_t i = 2; i < output.size(); i += 2) {
    			predicted_x.push_back(output[i]);        // x-coordinate
    			predicted_y.push_back(output[i + 1]);    // y-coordinate
			}

			nh_.param<double>("cost_weight/cte_cost_weight", cte_cost_weight_info, 1000);
    		nh_.param<double>("cost_weight/epsi_cost_weight", epsi_cost_weight_info, 1000);
    		nh_.param<double>("cost_weight/v_start_weight", v_weight_info, 1);
    		nh_.param<double>("cost_weight/delta_cost_weight", delta_cost_weight_info, 500);
    		nh_.param<double>("cost_weight/v_target_cost_weight", v_target_cost_weight_info, 1);
    		nh_.param<double>("cost_weight/delta_change_cost_weight", delta_change_cost_weight_info, 10000);
    		nh_.param<double>("cost_weight/v_target_change_cost_weight", v_target_change_cost_weight_info, 1);
			nh_.param<double>("ref_v", referance_v_info, 20.0);
			nh_.param<double>("k_v", k_v_info, 0.4);
			nh_.param<double>("dt", dt_info, 0.1);
			nh_.param<int>("N", mpc_step_info, 50);
			ROS_INFO("\n-----cost_weight-----\n"
         			"cte: %.2f\n"
         			"epsi: %.2f\n"
         			"v: %.2f\n"
         			"delta: %.2f\n"
         			"v_target: %.2f\n"
         			"delta_change: %.2f\n"
         			"v_target_change: %.2f\n"
					"horizon: %d\n"
         			"---------------------\n\n"
					"-----info-----\n"
					"referance_vel: %.2f\n"
					"k_v: %.2f\n"
					"dt: %.2f\n"
					"---------------------\n\n"
         			"Initial State:\n"
         			"x: %.2f, y: %.2f\npsi: %.2f, v: %.2f\ncte: %.2f, epsi: %.2f\n\n"
         			"Steering Angle: %.2f rad\n"
         			"Target Velocity: %.2f m/s\n"
					"Current Speed: %.2f km/h\n"
         			"Current Velocity: %.2f m/s\n"
					"Cost: %.3f\n"
         			"---------------------",
         			cte_cost_weight_info, 
					epsi_cost_weight_info, 
					v_weight_info, 
					delta_cost_weight_info, 
         			v_target_cost_weight_info, 
					delta_change_cost_weight_info, 
					v_target_change_cost_weight_info, 
					mpc_step_info,
					referance_v_info, 
					k_v_info, 
					dt_info,
         			state[0], 
					state[1], 
					state[2], 
					state[3], 
					state[4], 
					state[5], 
         			steering_angle, 
					target_velocity, 
					cur_speed_ * 3.6, 
					velocity, 
					cost);


    	    // 차량 좌표계 변환된 경로 rviz
    	    nav_msgs::Path predicted_path_rviz;
			predicted_path_rviz.header.frame_id = "base_link";
			predicted_path_rviz.header.stamp = ros::Time::now();

    	    if(N!=0)
			{
				int idx_rowsize = 0;

    	        for(int i=0;i<N;i++)
    	        {
    	            geometry_msgs::PoseStamped loc;
    	            loc.header.frame_id = "base_link";
    	            loc.header.stamp = ros::Time::now();
    	            loc.pose.position.x = predicted_x[i];
    	            loc.pose.position.y = predicted_y[i];

    	            loc.pose.position.z = 0.0;
    	            // if(i % 10 == 0){
    	                predicted_path_rviz.poses.emplace_back(loc);
    	            // }
    	        }
			}
			rviz_predicted_path_.publish(predicted_path_rviz);
    	}

    	if (is_control_)
    	{
    	    ackermann_msg_.longlCmdType = 2;
    	    ackermann_msg_.velocity = target_velocity * 3.6; // km/h
    	    ackermann_msg_.steering = -steering_angle;
			// if (accelation >= 0) {
			// 	ackermann_msg_.accel = accelation;
			// 	ackermann_msg_.brake = 0;
			// }
			// else {
			// 	ackermann_msg_.brake = accelation;
			// 	ackermann_msg_.accel = 0;
			// }
			
			
	
    	    ackermann_pub_.publish(ackermann_msg_);
    	}
    	is_pose_ = false;
		is_course_ = false;


	}
};



#endif
