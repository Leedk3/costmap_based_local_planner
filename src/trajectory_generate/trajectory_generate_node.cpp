
#include "trajectory_generate/trajectory_generate.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "behavior_planner");
  TrajectoryGenerator behavior_planner;
  
  ros::Rate loop_rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    behavior_planner.run();
    loop_rate.sleep();
  }

  return 0;
}
