#include "behavior_planner/behavior_planner.h"
#include <string>
#include <assert.h>

// Constructor
BehaviorPlanner::BehaviorPlanner() : nh_(""), private_nh_("~")
{

  private_nh_.param("base_frame_id", m_base_frame_id, std::string("base_link"));
  private_nh_.param("local_update_horizon", m_local_update_horizon, double(10));
  private_nh_.param("obstacle_padding", m_obstacle_padding, double(2));

  pub_LocalPath = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 1,true);
  pub_BehaviorState = nh_.advertise<geometry_msgs::TwistStamped>("current_behavior", 1);
  pub_BehaviorStateRviz = nh_.advertise<visualization_msgs::MarkerArray>("behavior_state", 1);
  pub_BlockedScreen = nh_.advertise<visualization_msgs::Marker>("blocked_screen",1);
  pub_AllPathBlocked = nh_.advertise<std_msgs::Bool>("all_path_blocked", 1);

  sub_lane_array_ = nh_.subscribe("/local_weighted_trajectories", 1, &BehaviorPlanner::CallbackLaneArray, this);
  sub_odom_ = nh_.subscribe("/Odometry/ekf_estimated", 1, &BehaviorPlanner::CallbackOdometry, this);
  sub_closest_index_ = nh_.subscribe("/closest_waypoint", 1, &BehaviorPlanner::CallbackClosestIndex, this);
  sub_base_waypoints_ = nh_.subscribe("/base_waypoints",1, &BehaviorPlanner::CallbackBaseWaypoints, this);
  sub_trajectory_cost_ = nh_.subscribe("local_trajectory_cost", 1, &BehaviorPlanner::CallbackTrajectoryCost, this);
  sub_obstacles_ = nh_.subscribe("/MarkerArray/obstacles", 1,&BehaviorPlanner::CallbackObstacles, this);

  init();
}

BehaviorPlanner::~BehaviorPlanner(){}

void BehaviorPlanner::init()
{
  ROS_INFO("init");

  bPathCandidates = false;
  bOdometry = false;
  bClosestIndex = false;
  bBaseWaypoints = false;
  bRollOuts = false;
  bBestRollOutCost = false;
  bObstacles = false;
  
  
  m_VehiclePosePrev.pos.x = 0;
  m_VehiclePosePrev.pos.y = 0;
  m_VehiclePoseCurrent.pos.x = 0;
  m_VehiclePoseCurrent.pos.y = 0;
  m_AccumulatedPose = 0;
  m_MinimumCostIndex = -1;
}

void BehaviorPlanner::run()
{
  // std::cout << "----------------------" << std::endl;
  if(bPathCandidates && bOdometry && bRollOuts)
  {
    initCost();
    GetPoseAccumulated();
    ReplanIfColliding(m_UpdatedLane);
    SendLocalPlanningTopics();
    AllPathBlockedSituation();
    finalize();
  }
}

void BehaviorPlanner::initCost()
{
  m_RollOutCost_ptr = (double*)malloc( m_LaneArray_ptr->lanes.size() * sizeof(double));
  for (unsigned int i =0; i < m_LaneArray_ptr -> lanes.size(); i++)
  {
    *(m_RollOutCost_ptr + i) = m_LaneArray_ptr->lanes.at(i).cost;
    // std::cout << "i: "<< i << ", cost: " << *(m_RollOutCost_ptr + i) << std::endl;
  }
  bAllPathBlocked = false;
  bCurrentPathColliding = false;
  
}

void BehaviorPlanner::finalize()
{
  free(m_RollOutCost_ptr);
}

void BehaviorPlanner::CallbackOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  m_Odometry_ptr = std::make_shared<nav_msgs::Odometry>(*msg);
  bOdometry = true;
}

void BehaviorPlanner::CallbackLaneArray(const autoware_msgs::LaneArrayConstPtr& msg)
{
  if(msg->lanes.size() > 0)
  {
    m_LaneArray_ptr = std::make_shared<autoware_msgs::LaneArray>(*msg);
  
    m_RollOuts.clear();
    bool all_path_blocked_check_ = true;
    for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
    {
      std::vector<PlannerHNS::WayPoint> path;
      PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
      m_RollOuts.push_back(path);
         
      if (msg->lanes.at(i).is_blocked && all_path_blocked_check_)
        bAllPathBlocked = true;
      else
      {
        all_path_blocked_check_ = false;
        bAllPathBlocked = false;
      }
    }
    bRollOuts = true;
    std_msgs::Bool AllPathBlocked_msg;
    AllPathBlocked_msg.data = bAllPathBlocked;
    pub_AllPathBlocked.publish(AllPathBlocked_msg);
  }
  bPathCandidates = true;
}

void BehaviorPlanner::CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg)
{
  m_ClosestIndex = msg->data;
  bClosestIndex = true;
}

void BehaviorPlanner::CallbackBaseWaypoints(const autoware_msgs::LaneConstPtr& msg)
{
  m_BaseWaypoints_ptr = std::make_shared<autoware_msgs::Lane>(*msg);
  bBaseWaypoints = true;
}

void BehaviorPlanner::CallbackTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
  m_FinalPathCost_ptr = std::make_shared<autoware_msgs::Lane>(*msg);
  bBestRollOutCost = true;
}
void BehaviorPlanner::CallbackObstacles(const visualization_msgs::MarkerArrayConstPtr &msg)
{
  m_Obstacles_ptr = std::make_shared<visualization_msgs::MarkerArray>(*msg);
  bObstacles = true;
}
void BehaviorPlanner::SendLocalPlanningTopics()
{
  //Send Trajectory Data to path following nodes
  autoware_msgs::Lane lane;
  
  m_MinimumCostIndex = m_FinalPathCost_ptr->lane_index;

  if (m_AccumulatedPose > m_local_update_horizon || bCurrentPathColliding)
  {
    m_UpdatedLane = m_LaneArray_ptr ->lanes.at(m_FinalPathCost_ptr->lane_index);
    m_AccumulatedPose = 0;
  }
  lane = m_UpdatedLane;
  pub_LocalPath.publish(lane);
}

void BehaviorPlanner::ReplanIfColliding(const autoware_msgs::Lane& CurrentLane)
{
  if (bObstacles)
  {
    tf::Transform global2baselink = getTransform(m_base_frame_id, m_Odometry_ptr->header.frame_id);
    for(unsigned int i =0; i < m_Obstacles_ptr->markers.size(); i++)
    {
      for(unsigned int j=0; j < CurrentLane.waypoints.size(); j++)
      {
        geometry_msgs::Pose pose_local_buffer;
        geometry_msgs::Pose pose_global_buffer;
        pose_global_buffer.position.x = CurrentLane.waypoints.at(j).pose.pose.position.x;
        pose_global_buffer.position.y = CurrentLane.waypoints.at(j).pose.pose.position.y;
        pose_local_buffer = transformPose(pose_global_buffer, global2baselink);

        PlannerHNS::WayPoint wp;
        wp.pos.x = pose_local_buffer.position.x;
        wp.pos.y = pose_local_buffer.position.y;      
        double dist_from_path_to_obstacle = sqrt(pow(m_Obstacles_ptr->markers.at(i).pose.position.x - pose_local_buffer.position.x ,2) + 
                                                 pow(m_Obstacles_ptr->markers.at(i).pose.position.y - pose_local_buffer.position.y,2));
        if (dist_from_path_to_obstacle < m_obstacle_padding)
        {
          bCurrentPathColliding = true;
          // std::cout << "minimum index: " << m_FinalPathCost_ptr->lane_index << std::endl;
          // std::cout << "bCurrentPathColliding: "<< bCurrentPathColliding << std::endl;
        }
      }

    }
  }
}

void BehaviorPlanner::GetPoseAccumulated()
{
  //Accumulate a pose of the vehicle to update the local path
  
  m_VehiclePoseCurrent.pos.x = m_Odometry_ptr->pose.pose.position.x;
  m_VehiclePoseCurrent.pos.y = m_Odometry_ptr->pose.pose.position.y;

  if (m_VehiclePosePrev.pos.x != 0 && m_VehiclePosePrev.pos.y != 0){
    m_AccumulatedPose = m_AccumulatedPose + 
                      sqrt(pow(m_VehiclePoseCurrent.pos.x - m_VehiclePosePrev.pos.x, 2) + 
                      pow(m_VehiclePoseCurrent.pos.y - m_VehiclePosePrev.pos.y, 2));

  }
  // std::cout << "m_AccumulatedPose: " << m_AccumulatedPose << std::endl;
  m_VehiclePosePrev.pos.x = m_VehiclePoseCurrent.pos.x;
  m_VehiclePosePrev.pos.y = m_VehiclePoseCurrent.pos.y;

}

void BehaviorPlanner::AllPathBlockedSituation()
{
  tf::Transform global2baselink = getTransform(m_base_frame_id, m_Odometry_ptr->header.frame_id);
  PlannerHNS::WayPoint BlockedPoseNext;
  PlannerHNS::WayPoint BlockedPoseBefore; 
  if (bAllPathBlocked || bCurrentPathColliding)
  {
    double closest_distance = m_FinalPathCost_ptr->closest_object_distance;
    bool found_ = false;
    for (unsigned int i =0; i <  m_LaneArray_ptr->lanes.at(m_FinalPathCost_ptr->lane_index).waypoints.size(); i++)
    {
      double RollOut_x = m_LaneArray_ptr->lanes.at(m_FinalPathCost_ptr->lane_index).waypoints.at(i).pose.pose.position.x;
      double RollOut_y = m_LaneArray_ptr->lanes.at(m_FinalPathCost_ptr->lane_index).waypoints.at(i).pose.pose.position.y;
      geometry_msgs::Pose BlockedPoseNextLocal;
      geometry_msgs::Pose BlockedPoseNextGlobal;
      BlockedPoseNextGlobal.position.x = RollOut_x;
      BlockedPoseNextGlobal.position.y = RollOut_y;
      BlockedPoseNextLocal = transformPose(BlockedPoseNextGlobal, global2baselink);
      double RollOutDistanceFromOrigin = sqrt(pow(BlockedPoseNextLocal.position.x, 2) + pow(BlockedPoseNextLocal.position.y, 2));
      if (RollOutDistanceFromOrigin > closest_distance && found_ == false)
      {
        if (i < 1)
          return;
        geometry_msgs::Pose BlockedPoseBeforeLocal;
        geometry_msgs::Pose BlockedPoseBeforeGlobal;
        BlockedPoseBeforeGlobal.position.x = m_LaneArray_ptr->lanes.at(m_FinalPathCost_ptr->lane_index).waypoints.at(i-1).pose.pose.position.x;
        BlockedPoseBeforeGlobal.position.y = m_LaneArray_ptr->lanes.at(m_FinalPathCost_ptr->lane_index).waypoints.at(i-1).pose.pose.position.y;
        BlockedPoseBeforeLocal = transformPose(BlockedPoseBeforeGlobal, global2baselink);

        BlockedPoseNext.pos.x = BlockedPoseNextLocal.position.x;
        BlockedPoseNext.pos.y = BlockedPoseNextLocal.position.y;
        BlockedPoseBefore.pos.x = BlockedPoseBeforeLocal.position.x;
        BlockedPoseBefore.pos.y = BlockedPoseBeforeLocal.position.y;

        // std::cout << " BlockedPoseNext.pos.x: " <<  BlockedPoseNext.pos.x << ", BlockedPoseNext.pos.y:" << BlockedPoseNext.pos.y << std::endl;
        // std::cout << " BlockedPoseBefore.pos.x: " <<  BlockedPoseBefore.pos.x << ", BlockedPoseBefore.pos.y:" << BlockedPoseBefore.pos.y << std::endl;
        found_ = true;  
      }
    }
    double BlockedX = BlockedPoseNext.pos.x - BlockedPoseBefore.pos.x;
    double BlockedY = BlockedPoseNext.pos.y - BlockedPoseBefore.pos.y;
    double BlockedYaw = atan2(BlockedY,  BlockedX+ 0.0000001); //perpendicular to the path.
    geometry_msgs::Quaternion QautMsgBlocked = tf::createQuaternionMsgFromRollPitchYaw(0, 0, BlockedYaw);

    visualization_msgs::Marker BlockedMarker;
    BlockedMarker.header.frame_id = m_base_frame_id;
    BlockedMarker.header.stamp = ros::Time();
    BlockedMarker.ns = "BlockedScreen";
    BlockedMarker.type = visualization_msgs::Marker::CUBE;
    BlockedMarker.action = visualization_msgs::Marker::ADD;
    BlockedMarker.scale.x = 0.1, BlockedMarker.scale.y = 4, BlockedMarker.scale.z = 3;
    BlockedMarker.color.a = 0.4, BlockedMarker.color.r = 1.0, BlockedMarker.color.g = 0.0, BlockedMarker.color.b = 0.0;
    BlockedMarker.frame_locked = false;
    BlockedMarker.pose.position.x = BlockedPoseBefore.pos.x;
    BlockedMarker.pose.position.y = BlockedPoseBefore.pos.y;
    BlockedMarker.pose.orientation = QautMsgBlocked;
    BlockedMarker.lifetime = ros::Duration(1);
    pub_BlockedScreen.publish(BlockedMarker);
  } 
}

tf::Transform BehaviorPlanner::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    m_tf_listener.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}
