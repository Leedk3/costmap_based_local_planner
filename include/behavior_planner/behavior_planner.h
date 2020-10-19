/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

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

// headers in STL
#include <memory>
#include <cmath>

class BehaviorPlanner
{
public:
  BehaviorPlanner();
  ~BehaviorPlanner();

  void init();
  void run();
  void CallbackLaneArray(const autoware_msgs::LaneArrayConstPtr& msg);
  void CallbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg);
  void CallbackBaseWaypoints(const autoware_msgs::LaneConstPtr & msg);
  void CallbackTrajectoryCost(const autoware_msgs::LaneConstPtr & msg);
  void CallbackObstacles(const visualization_msgs::MarkerArrayConstPtr &msg);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_LocalPath;
  ros::Publisher pub_BehaviorState;
  ros::Publisher pub_BehaviorStateRviz;
  ros::Publisher pub_BlockedScreen;
  ros::Publisher pub_AllPathBlocked;
  
  ros::Subscriber sub_lane_array_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_closest_index_;
  ros::Subscriber sub_base_waypoints_;
  ros::Subscriber sub_trajectory_cost_;
  ros::Subscriber sub_obstacles_;
  tf::TransformListener m_tf_listener;

  std::shared_ptr<autoware_msgs::LaneArray> m_LaneArray_ptr;
  std::shared_ptr<nav_msgs::OccupancyGrid> m_OccupancyGrid_ptr;
  std::shared_ptr<nav_msgs::Odometry> m_Odometry_ptr;
  std::shared_ptr<autoware_msgs::Lane> m_BaseWaypoints_ptr;
  std::shared_ptr<autoware_msgs::Lane> m_FinalPathCost_ptr;
  std::shared_ptr<visualization_msgs::MarkerArray> m_Obstacles_ptr;
  std::vector<std::vector<PlannerHNS::WayPoint>> m_RollOuts;
  PlannerHNS::DecisionMaker m_BehaviorGenerator;
  autoware_msgs::Lane m_UpdatedLane;

  bool bOccupancyGrid;
  bool bPathCandidates;
  bool bOdometry;
  bool bClosestIndex;
  bool bBaseWaypoints;
  bool bRollOuts;
  bool bBestRollOutCost;
  bool bAllPathBlocked;
  bool bObstacles;
  bool bCurrentPathColliding;
  
  std::string m_base_frame_id;
  double m_local_update_horizon;
  double m_AccumulatedPose;
  double m_obstacle_padding;
  PlannerHNS::WayPoint m_VehiclePosePrev;
  PlannerHNS::WayPoint m_VehiclePoseCurrent;

  double* m_RollOutCost_ptr;

  int m_ClosestIndex;
  int m_MinimumCostIndex;

  void initCost();
  void finalize();
  void GetPoseAccumulated();
  void SendLocalPlanningTopics();
  void AllPathBlockedSituation();
  void ReplanIfColliding(const autoware_msgs::Lane& CurrentLane);
  tf::Transform getTransform(const std::string& from, const std::string& to);
  inline geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const tf::Transform& tf)
  {
    // Convert ROS pose to TF pose
    tf::Pose tf_pose;
    tf::poseMsgToTF(pose, tf_pose);

    // Transform pose
    tf_pose = tf * tf_pose;

    // Convert TF pose to ROS pose
    geometry_msgs::Pose ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose);

    return ros_pose;
  }

};

#endif  // COSTMAP_BASED_BEHAVIOR_PLANNER_H
