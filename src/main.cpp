#include <ros/ros.h>
#include <mpc_path_tracking.h>
#include <new_platform_control.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "mpc_path_tracking");

	ROSCONTROL mpc;
	PlatformConnector pc;

	ros::Rate loop_rate(20);

	while(ros::ok()) 
	{
		ros::spinOnce();
		mpc.modelPredictiveController();
		pc.setFromWF(mpc.getSpeed(), mpc.getSteer()); // set pc_spd
		pc.inputSerial();
		loop_rate.sleep();
	}
	return 0;
}