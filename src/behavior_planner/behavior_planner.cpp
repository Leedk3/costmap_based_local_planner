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
  pub_LocalCoordPath = nh_.advertise<autoware_msgs::Lane>("final_waypoints/local_coordinate", 1,true);
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
  m_VehiclePosePrev.pos.a = 0; // sw: temporary using for yaw angle(rad)
  m_VehiclePoseCurrent.pos.x = 0;
  m_VehiclePoseCurrent.pos.y = 0;
  m_VehiclePoseCurrent.pos.a = 0; // sw: temporary using for yaw angle(rad)
  m_VehiclePoseLocalPathRef.pos.x = 0;
  m_VehiclePoseLocalPathRef.pos.y = 0;
  m_VehiclePoseLocalPathRef.pos.a = 0; // sw: temporary using for yaw angle(rad)

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

  // sw: cf@m_AccumulatedPose
  if (m_AccumulatedPose > m_local_update_horizon || bCurrentPathColliding)
  {
    m_UpdatedLane = m_LaneArray_ptr ->lanes.at(m_FinalPathCost_ptr->lane_index);
    m_VehiclePoseLocalPathRef.pos.x = m_VehiclePoseCurrent.pos.x;
    m_VehiclePoseLocalPathRef.pos.y = m_VehiclePoseCurrent.pos.y;
    m_VehiclePoseLocalPathRef.pos.a = m_VehiclePoseCurrent.pos.a;
    m_AccumulatedPose = 0;
  }
  lane = m_UpdatedLane;
  pub_LocalPath.publish(lane); // piece of global waypoints

  autoware_msgs::Lane temp_global_local_lane;
  HoldLocalPathInGlobalFrame(m_UpdatedLane, temp_global_local_lane, m_VehiclePoseLocalPathRef, m_VehiclePoseCurrent);
  pub_LocalCoordPath.publish(temp_global_local_lane); // local coordinate local waypoints
}

/* 
 * @brief: local(m_VehiclePoseLocalPathRef = m_VehiclePoseCurrent) -> global(m_VehiclePoseCurrent) -> local(m_VehiclePoseCurrent)
 */
void BehaviorPlanner::HoldLocalPathInGlobalFrame(const autoware_msgs::Lane& CurrentLane, autoware_msgs::Lane& temp_global_local_lane, 
                                                  const PlannerHNS::WayPoint& poseLocalRef, const PlannerHNS::WayPoint& poseCurrent){
  // convert m_UpdatedLane to global frame with m_VehiclePoseLocalPathRef(local->global)
  autoware_msgs::Lane temp_global_lane;
  temp_global_lane.header.frame_id = "odom";
  int waypoint_length = CurrentLane.waypoints.size(); // uint into int
  for (int i = 0; i < waypoint_length; i++){
    double local_wpt_x = CurrentLane.waypoints[i].pose.pose.position.x;
    double local_wpt_y = CurrentLane.waypoints[i].pose.pose.position.y;
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.header.stamp    = CurrentLane.header.stamp;
    // TODO: turn frame_id variable
    waypoint.pose.header.frame_id = temp_global_lane.header.frame_id;
    // (local->global)
    waypoint.pose.pose.position.x = local_wpt_x * cos(poseLocalRef.pos.a) - local_wpt_y * sin(poseLocalRef.pos.a) + poseLocalRef.pos.x;
    waypoint.pose.pose.position.y = local_wpt_x * sin(poseLocalRef.pos.a) + local_wpt_y * cos(poseLocalRef.pos.a) + poseLocalRef.pos.y;
    temp_global_lane.waypoints.push_back(waypoint);
  }

  // convert back to local frame centered at m_VehiclePoseCurrent(global->local)
  autoware_msgs::Lane temp_local_lane;
  temp_local_lane.header.frame_id = "base_link";
  for (int i = 0; i < waypoint_length; i++){
    double global_wpt_x = temp_global_lane.waypoints[i].pose.pose.position.x;
    double global_wpt_y = temp_global_lane.waypoints[i].pose.pose.position.y;
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.header.stamp    = temp_global_lane.header.stamp;
    // TODO: turn frame_id variable
    waypoint.pose.header.frame_id = temp_local_lane.header.frame_id;
    // (global->local)
    waypoint.pose.pose.position.x = (global_wpt_x - poseCurrent.pos.x) * cos(-poseCurrent.pos.a)
                                    - (global_wpt_y - poseCurrent.pos.y) * sin(-poseCurrent.pos.a);
    waypoint.pose.pose.position.y = (global_wpt_x - poseCurrent.pos.x) * sin(-poseCurrent.pos.a)
                                    + (global_wpt_y - poseCurrent.pos.y) * cos(-poseCurrent.pos.a);
    temp_local_lane.waypoints.push_back(waypoint);
  }
  // TODO sw: turn the two transforms (l->g, g->l) into a single transform.
  // https://kaistackr-my.sharepoint.com/personal/seungwook1024_kaist_ac_kr/_layouts/15/Doc.aspx?sourcedoc={b23cfce8-9f17-4c2e-add4-b0e1de1fb6c1}&action=edit&wd=target%28%EA%B0%9C%EC%9D%B8%EC%97%B0%EA%B5%AC.one%7C49ee15d7-dd1c-4ad3-a940-da1f33d54bfa%2FHold%20local%20path%20algorithm%7Cdca3394f-adf6-364f-aa25-51634ca38ea3%2F%29&wdorigin=703
  temp_global_local_lane = temp_local_lane;

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
  double roll, pitch, yaw;
  quarternionToRPY(m_Odometry_ptr->pose.pose.orientation, roll, pitch, yaw);
  m_VehiclePoseCurrent.pos.a = yaw;

  if (m_VehiclePosePrev.pos.x != 0 && m_VehiclePosePrev.pos.y != 0){
    m_AccumulatedPose = m_AccumulatedPose +
                        sqrt(pow(m_VehiclePoseCurrent.pos.x - m_VehiclePosePrev.pos.x, 2) + 
                             pow(m_VehiclePoseCurrent.pos.y - m_VehiclePosePrev.pos.y, 2));
  }
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
