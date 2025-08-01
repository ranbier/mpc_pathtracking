#include <mpc_path_tracking.h>
#include <cmath>

ROSCONTROL::ROSCONTROL() {
    initSetup();
}

ROSCONTROL::~ROSCONTROL() {
    waypoints_.clear();
}

void ROSCONTROL::initSetup() {
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

    odom_sub_ = nh_.subscribe("odom", 1, &ROSCONTROL::OdomCallback,this);
    lane_sub_ = nh_.subscribe("/local_path", 1, &ROSCONTROL::PathCallback, this);
    state_sub_ = nh_.subscribe("gps_state",1,&ROSCONTROL::StateCallback,this);
    imu_sub_ = nh_.subscribe("imu",1,&ROSCONTROL::ImuCallback,this);
}

void ROSCONTROL::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    ros::Time cur_stamp = odom_msg->header.stamp;
    cur_pose_.pose.position = odom_msg->pose.pose.position;
    broadcastTransform();
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
}

void ROSCONTROL::PathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    waypoints_.clear();
    std::vector<waypoint_maker::Waypoint>().swap(waypoints_);

    int index = 0;
    for (const auto& pose_stamped : path_msg->poses) {
        waypoint_maker::Waypoint wp;
        wp.waypoint_index = index++;
        wp.pose = pose_stamped;
        waypoints_.push_back(wp);
    }

    waypoints_size_ = waypoints_.size();
    is_lane_ = !waypoints_.empty();
}

void ROSCONTROL::broadcastTransform() {
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

void ROSCONTROL::StateCallback(const waypoint_maker::State::ConstPtr &state_msg) {
    dist_ = state_msg->dist;
    current_mission_state_ = state_msg->current_state;
    loader_number_ = state_msg->lane_number;
}

void ROSCONTROL::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                     imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);
    cur_course_ = yaw;
    is_course_ = true;
}

void ROSCONTROL::getClosestWaypoint(geometry_msgs::PoseStamped current_pose) {
    if (!waypoints_.empty()) {
        double dist_min = MAX_SEARCH_DIST;
        for (int i = 0; i < waypoints_.size(); i++) {
            double dist = calcPlaneDist(current_pose, waypoints_[i].pose);
            if (dist < dist_min) {
                dist_min = dist;
                waypoint_min_ = i;
            }
        }
        closest_waypoint_ = waypoint_min_;
    } else {
        std::cout << "------ NO CLOSEST WAYPOINT -------" << std::endl;
    }
}

double ROSCONTROL::calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
    return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                pow(pose1.pose.position.y - pose2.pose.position.y, 2));
}

double ROSCONTROL::getSpeed(double& ex_x, double& ex_y, double cur_x, double cur_y, double dt) {
    double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
    double speed = distance / dt;
    return speed;
}

double ROSCONTROL::getAccel(double& ex_x, double& ex_y, double cur_x, double cur_y) {
    double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
    double speed = distance / inter_time_;
    double a = speed / inter_time_ * INTER_SPEED_TIME_MAX;
    return a;
}

void ROSCONTROL::modelPredictiveController() {
    getClosestWaypoint(cur_pose_);

    if(is_pose_ && is_course_ && is_lane_) {
        is_control_ = true;

        double px = ex_x_;
        double py = ex_y_;
        double psi = cur_course_;
        double velocity = cur_speed_;
        double dt_ = 0.1;

        VectorXd vehicle_waypoints_x(waypoints_size_);
        VectorXd vehicle_waypoints_y(waypoints_size_);

        for (int i = 0; i < waypoints_size_; i++) {
            double diff_x = waypoints_[i].pose.pose.position.x - px;
            double diff_y = waypoints_[i].pose.pose.position.y - py;
            vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
            vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
        }

        auto coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);
        const double cte = polyeval(coeffs, 0);
        const double epsi = -atan(coeffs[1]);

        Eigen::VectorXd state(6);
        state = predict_FutureState(velocity, steering_angle, cte, epsi, target_velocity, dt_);

        auto output = Solve(state, coeffs);

        steering_angle = output[0];
        target_velocity = output[1];

        std::vector<double> predicted_x;
        std::vector<double> predicted_y;

        for (size_t i = 2; i < output.size(); i += 2) {
            predicted_x.push_back(output[i]);
            predicted_y.push_back(output[i + 1]);
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

        nav_msgs::Path predicted_path_rviz;
        predicted_path_rviz.header.frame_id = "base_link";
        predicted_path_rviz.header.stamp = ros::Time::now();

        if(N!=0) {
            for(int i=0;i<N;i++) {
                geometry_msgs::PoseStamped loc;
                loc.header.frame_id = "base_link";
                loc.header.stamp = ros::Time::now();
                loc.pose.position.x = predicted_x[i];
                loc.pose.position.y = predicted_y[i];
                loc.pose.position.z = 0.0;
                predicted_path_rviz.poses.emplace_back(loc);
            }
        }
        rviz_predicted_path_.publish(predicted_path_rviz);
    }

    if (is_control_) {
        ackermann_msg_.longlCmdType = 2;
        ackermann_msg_.velocity = target_velocity * 3.6;
        ackermann_msg_.steering = -steering_angle;
        ackermann_pub_.publish(ackermann_msg_);
    }
    is_pose_ = false;
    is_course_ = false;
}
