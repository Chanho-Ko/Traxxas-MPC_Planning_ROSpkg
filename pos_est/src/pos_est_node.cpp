#include <TimeVaryingKF_heading.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pos_est_node");
   ros::NodeHandle nh;
   TimeVaryingKF TVKF(&nh);

   ros::spin();

   return 0;
}

