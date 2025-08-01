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
#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/CollisionData.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/time.h>
#include <vector>
#include <mpc.h>

#define INTER_TIME_PLUS 1000000000
#define INTER_TIME_MIN 90000000
#define INTER_TIME_MAX 200000000
#define INTER_SPEED_TIME_MAX 3600000000
#define SOME_MIN 0.04
#define SOME_MAX 0.2

class ROSCONTROL : public MPC {
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
    std::vector<waypoint_maker::Waypoint> waypoints_;

    ros::Publisher ackermann_pub_;
    ros::Publisher rviz_predicted_path_;
    ros::Publisher vis_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber lane_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber imu_sub_;

    morai_msgs::CtrlCmd ackermann_msg_;
    tf::TransformBroadcaster tf_broadcaster_;

    float steer;
    float velocity_actual;
    float steering_angle;
    double target_velocity;

public:
    ros::Time ex_stamp;

    ROSCONTROL();
    ~ROSCONTROL();

    void initSetup();
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void PathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    void broadcastTransform();
    void StateCallback(const waypoint_maker::State::ConstPtr &state_msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
    void getClosestWaypoint(geometry_msgs::PoseStamped current_pose);
    double calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2);
    double getSpeed(double& ex_x, double& ex_y, double cur_x, double cur_y, double dt);
    double getAccel(double& ex_x, double& ex_y, double cur_x, double cur_y);
    void modelPredictiveController();
};

#endif
