#include <mpc_planning_node.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_planning_node");
	ros::NodeHandle nh;

    Planner* planner = new Planner(&nh);

    ros::Rate r(20); // 20 Hz
    while (ros::ok()){
        if (planner->start_plan == true){
            planner->solve_OP();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
