
#include "ros2can_node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros2can_node");
	ros::NodeHandle nh;

	Ros2Can ros2can(&nh);

	while (ros::ok()) {
		ros::spinOnce();
	}

	return 0;
}