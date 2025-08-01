#include <ros/ros.h>
#include <mpc_path_tracking.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "mpc_path_tracking");

	ROSCONTROL mpc;

	ros::Rate loop_rate(10);

	while(ros::ok()) 
	{
		ros::spinOnce();
		mpc.modelPredictiveController();
		loop_rate.sleep();
	}
	return 0;
}