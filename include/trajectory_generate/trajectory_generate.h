#ifndef COSTMAP_BASED_BEHAVIOR_PLANNER_H
#define COSTMAP_BASED_BEHAVIOR_PLANNER_H

// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>

// headers in local directory
#include "vector_map/vector_map.h"
#include "autoware_msgs/DetectedObjectArray.h"

// headers in op_planner and op_ros_helpers
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/TrajectoryDynamicCosts.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/DecisionMaker.h"
#include "op_planner/PlannerH.h"

// headers in STL
#include <memory>
#include <cmath>

class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  ~TrajectoryGenerator();

  void init();
  void run();

  void CallbackGlobalWptThroughAutowareMsg(const autoware_msgs::LaneArrayConstPtr& msg);
  void CallbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pubLocalTrajectories;
  ros::Publisher pubLocalTrajectoriesRviz;  
  
  ros::Subscriber subGlobalWaypoint;
  ros::Subscriber subOdom;
  ros::Subscriber subClosestWaypoint;
  
  std::shared_ptr<autoware_msgs::LaneArray> m_GlobalWaypointAutowareMsg_ptr;
  std::shared_ptr<nav_msgs::OccupancyGrid> m_OccupancyGrid_ptr;
  std::shared_ptr<nav_msgs::Odometry> m_Odometry_ptr;


  bool bOdometry;
  bool bGlobalWaypoint;
  bool bClosestWaypoint;
  
  int m_ClosestIndex;

  PlannerHNS::PlannerH m_Planner;
  PlannerHNS::PlanningParams m_PlanningParams;
  std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
  std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
  std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
  std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_RollOuts;
  PlannerHNS::WayPoint m_CurrentPos;
  
  

};

#endif  // COSTMAP_BASED_BEHAVIOR_PLANNER_H
