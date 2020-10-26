#include "voronoi_planner/voronoi_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voronoi_planner");
  VoronoiPlanner planner;
  
  ros::Rate loop_rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    planner.run();
    loop_rate.sleep();
  }

  return 0;
}
