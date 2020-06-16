#include "collision_check_with_path/collision_check_with_path.h"
#include <string>
#include "collision_check_with_path/collision_check.cuh"
#include <assert.h>
#include <std_msgs/String.h>

// Constructor
CollisionChecker::CollisionChecker() : nh_(""), private_nh_("~")
{
  private_nh_.param("base_frame_id", m_base_frame_id, std::string("base_link"));
  private_nh_.param("obstacle_radius", m_obstacle_radius, double(0.8));

  private_nh_.param("collision_cost", m_collision_cost, double(10));
  private_nh_.param("curvature_cost", m_curvature_cost, double(100));
  private_nh_.param("trasition_cost", m_trasition_cost, double(10));

  private_nh_.param("weight_curvature", m_WeightCurvature, double(1));
  private_nh_.param("weight_trasition", m_WeightTrasition, double(0.5));
  private_nh_.param("weight_consistancy", m_WeightConsistancy, double(0.5));
  private_nh_.param("weight_collision", m_WeightCollision, double(1));
  private_nh_.param("path_block_threshold", m_PathBlockedThres, double(1));

  private_nh_.param("/op_common_params/rollOutDensity", m_rollOutDensity, double(0.5));
  private_nh_.param("/op_common_params/rollOutsNumber", m_rollOutNumber, double(7));

  pub_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("/MarkerArray/obstacles", 1);
  pub_occupied_grid_ = nh_.advertise<visualization_msgs::MarkerArray>("/MarkerArray/occupied_grid", 1);
  pub_TrajectoryCost_ = nh_.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
  pub_LocalWeightedTrajectoriesRviz_ = nh_.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);
  pub_LocalWeightedTrajectories_ = nh_.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);

  sub_occupancy_grid_ = nh_.subscribe("/semantics/costmap_generator/occupancy_grid", 1, &CollisionChecker::CallbackOccupancyGrid, this);
  sub_path_ = nh_.subscribe("/local_paths_tmp", 1, &CollisionChecker::CallbackPath, this);
  sub_lane_array_ = nh_.subscribe("/local_trajectories", 1, &CollisionChecker::CallbackLaneArray, this);
  sub_odom_ = nh_.subscribe("/Odometry/ekf_estimated", 1, &CollisionChecker::CallbackOdometry, this);
  sub_closest_index_ = nh_.subscribe("/closest_waypoint", 1, &CollisionChecker::CallbackClosestIndex, this);
  sub_base_waypoints_ = nh_.subscribe("/base_waypoints",1, &CollisionChecker::CallbackBaseWaypoints, this);

  init();
}

CollisionChecker::~CollisionChecker(){}

void CollisionChecker::init()
{
  bOccupancyGrid = false;
  bPathCandidates = false;
  bOdometry = false;
  bClosestIndex = false;
  bBaseWaypoints = false;

  m_ClosestIndex = 0;

  ROS_INFO("init");
}

void CollisionChecker::run()
{
  if(bPathCandidates && bOdometry && bOccupancyGrid)
  {
    m_obstacle_markers.markers.clear();
    GlobalCoordToLocalCoord(*m_LaneArray_ptr, *m_Odometry_ptr);
    initRollOutCost(*m_LaneArray_ptr);
    CalculateCost(*m_OccupancyGrid_ptr, m_ConvertedLocalPaths);
    BestPath(*m_LaneArray_ptr, m_RollOutCost_ptr, m_PathBlocked_ptr, m_ConvertedLocalPaths);
    // GridMapCheckWithPath_GPU(*m_OccupancyGrid_ptr, m_ConvertedLocalPaths);
  }
}

void CollisionChecker::CallbackOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg)
{
  m_OccupancyGrid_ptr = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
  bOccupancyGrid = true;
}

void CollisionChecker::CallbackPath(const nav_msgs::PathConstPtr& msg)
{
  // ROS_INFO("Read Path");
}

void CollisionChecker::CallbackOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  m_Odometry_ptr = std::make_shared<nav_msgs::Odometry>(*msg);
  bOdometry = true;
}

void CollisionChecker::CallbackLaneArray(const autoware_msgs::LaneArrayConstPtr& msg)
{
  if(msg->lanes.size() > 0)
  {
    m_LaneArray_ptr = std::make_shared<autoware_msgs::LaneArray>(*msg);
  }
  bPathCandidates = true;
}

void CollisionChecker::CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg)
{
  m_ClosestIndex = msg->data;
  bClosestIndex = true;
}

void CollisionChecker::CallbackBaseWaypoints(const autoware_msgs::LaneConstPtr& msg)
{
  m_BaseWaypoints_ptr = std::make_shared<autoware_msgs::Lane>(*msg);
  bBaseWaypoints = true;
}

void CollisionChecker::initRollOutCost(const autoware_msgs::LaneArray &LaneArray)
{
  m_current_rollout_index = floor((double)(m_rollOutNumber/2));
  m_best_rollout_index = floor((double)(m_rollOutNumber/2));

  m_RollOutCost_ptr = (double*)malloc( LaneArray.lanes.size() * sizeof(double));
  m_CurvatureCost_ptr = (double*)malloc(LaneArray.lanes.size() * sizeof(double));
  m_TrasitionCost_ptr = (double*)malloc(LaneArray.lanes.size() * sizeof(double));
  m_ConsistancyCost_ptr = (double*)malloc(LaneArray.lanes.size() * sizeof(double));
  m_CollisionCost_ptr = (double*)malloc((LaneArray.lanes.size() * sizeof(double)));
  m_PathBlocked_ptr = (bool*)malloc((LaneArray.lanes.size() * sizeof(bool)));
  for (unsigned int i =0; i < LaneArray.lanes.size(); i++)
  {
    *(m_RollOutCost_ptr + i)  = 0;
    *(m_CurvatureCost_ptr + i) = 0;
    *(m_TrasitionCost_ptr +i) = 0;
    *(m_ConsistancyCost_ptr + i) = 0;
    *(m_CollisionCost_ptr + i) = 0;
    *(m_PathBlocked_ptr + i ) = false;
  }

  m_IntegCurvatureCost = 0;
  m_IntegTrasitionCost = 0;

  m_minimum_distance_to_obstacle = DBL_MAX;
  m_minimum_cost = DBL_MAX;
  m_AllPathBlocked = false;
  
}

void CollisionChecker::GlobalCoordToLocalCoord(const autoware_msgs::LaneArray &LaneArray, const nav_msgs::Odometry &Odometry)
{ 
  tf::Transform global2baselink = getTransform(m_base_frame_id, Odometry.header.frame_id);
  m_ConvertedLocalPaths.clear();

  for (unsigned int i = 0; i < LaneArray.lanes.size(); i++)
  {
    std::vector<PlannerHNS::WayPoint> local_path;
    local_path.clear();
    for(unsigned int j=0; j < LaneArray.lanes.at(i).waypoints.size(); j++)
    {
      geometry_msgs::Pose pose_local_buffer;
      geometry_msgs::Pose pose_global_buffer;
      pose_global_buffer.position.x = LaneArray.lanes.at(i).waypoints.at(j).pose.pose.position.x;
      pose_global_buffer.position.y = LaneArray.lanes.at(i).waypoints.at(j).pose.pose.position.y;
      pose_local_buffer = transformPose(pose_global_buffer, global2baselink);

      PlannerHNS::WayPoint wp;
      wp.pos.x = pose_local_buffer.position.x;
      wp.pos.y = pose_local_buffer.position.y;
      wp.pos.z = pose_local_buffer.position.z;
      local_path.push_back(wp);
    }
    m_ConvertedLocalPaths.push_back(local_path);
  }

}
void CollisionChecker::CalculateTrasitionCost(double* CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut ,const int& currTrajectoryIndex)
{
  for(unsigned int ic = 0; ic< RollOut.size(); ic++)
  {
    
    double trasition_cost = fabs(m_rollOutDensity * ((int)ic - currTrajectoryIndex)) * m_trasition_cost;
    *(CostArray + ic) =  trasition_cost;
    m_IntegTrasitionCost = m_IntegTrasitionCost + trasition_cost;
    // std::cout << "i: " << ic <<",  Trasition: " << trasition_cost << std::endl;
  }
}

void CollisionChecker::CalculateConsistancyCost(double* CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut, const int& MinimumCostIndex)
{
  double consistancy_cost;
  for(unsigned int ic = 0; ic< RollOut.size(); ic++)
  {
    if (ic == MinimumCostIndex)
    {
      consistancy_cost = 0;
    }
    else
    {
      consistancy_cost = 1;
    }
    *(CostArray + ic) =  consistancy_cost;
  }
}

void CollisionChecker::CalculateCurvatureCost(double* CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut)
{
  if (bClosestIndex && bBaseWaypoints)
  {
    for(unsigned int ic = 0; ic< RollOut.size(); ic++)
    {
      double lookAheadDistance = getLookAheadDistance(*m_Odometry_ptr, RollOut.at(ic), 30);
      double lookAheadAngle = getLookAheadAngleconst(*m_Odometry_ptr, RollOut.at(ic), 30);
      double lookAheadRadius = 0.5*(lookAheadDistance / sin(lookAheadAngle));
      double curvature_cost = 1/fabs(lookAheadRadius) * m_curvature_cost;

      *(CostArray + ic) = curvature_cost;
      m_IntegCurvatureCost = m_IntegCurvatureCost + curvature_cost; 
      // std::cout << "i: " << ic <<",  Curvature: " << curvature_cost << std::endl;
    }
  }

}

void CollisionChecker::CalculateCost(const nav_msgs::OccupancyGrid& Grid, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut)
{
  m_Obstacles.clear();
  GridMap * grid_map_ = (GridMap*)malloc(m_OccupancyGrid_ptr->info.width * sizeof(GridMap));
  // int grid_id_ = 0;
  for (unsigned int width = 0; width < m_OccupancyGrid_ptr->info.width; width++){
    grid_map_[width].occupied_intensity_ = (int*)malloc(m_OccupancyGrid_ptr->info.height * sizeof(int));
    for (unsigned int height = 0; height < m_OccupancyGrid_ptr->info.height; height++)
    {
      if(m_OccupancyGrid_ptr->data[height * m_OccupancyGrid_ptr->info.width + width] > 80)
      { 
        *(grid_map_[width].occupied_intensity_ + height) = 255;
        geometry_msgs::Pose obstacle;
        obstacle.position.x = width * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.x;
        obstacle.position.y = height * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.y;
        m_Obstacles.push_back(obstacle);

        // visualization_msgs::Marker occupied_grid;
        // occupied_grid.header.stamp = ros::Time::now();
        // occupied_grid.header.frame_id = m_base_frame_id;
        // occupied_grid.type = visualization_msgs::Marker::CUBE;
        // occupied_grid.action = visualization_msgs::Marker::ADD;
        // occupied_grid.ns = "grid";
        // occupied_grid.id = grid_id_++;
        // occupied_grid.pose.position.x = obstacle.position.x, occupied_grid.pose.position.y = obstacle.position.y, occupied_grid.pose.position.z = 0;
        // occupied_grid.pose.orientation.w = 1;
        // occupied_grid.scale.x = 0.3, occupied_grid.scale.y = 0.3, occupied_grid.scale.z = 0.1;
        // occupied_grid.color.r = 0, occupied_grid.color.b = 1, occupied_grid.color.g = 0, occupied_grid.color.a = 1;
        // occupied_grid.lifetime = ros::Duration(0.05);
        // m_occupied_grid_markers.markers.push_back(occupied_grid);       
      }
      else
      {
        *(grid_map_[width].occupied_intensity_ + height) = 0;
      }
    }
  }
  // pub_occupied_grid_.publish(m_occupied_grid_markers);

  std::cout << "-------------------------------------" <<std::endl;
  int once = -1;
  int marker_id_ = 0;
  for (int k =0; k < (int)m_Obstacles.size(); k++)
  {
    for (unsigned int i =0; i < RollOut.size(); i++)
    {
      for (unsigned int j = 0; j < RollOut.at(i).size(); j++)
      {
        double x_wpt_ = RollOut.at(i).at(j).pos.x;
        double y_wpt_ = RollOut.at(i).at(j).pos.y;      
        double dist_from_path_to_obstacle = sqrt(pow(m_Obstacles.at(k).position.x - x_wpt_ ,2) + pow(m_Obstacles.at(k).position.y - y_wpt_,2));
        //Collision check
        if (dist_from_path_to_obstacle < m_obstacle_radius)
        {
          if (once == k)
          {
            double collision_cost = 1;
            *(m_CollisionCost_ptr + i) = collision_cost;
            continue;
          }
          if (sqrt(pow(m_Obstacles.at(k).position.x ,2) + pow(m_Obstacles.at(k).position.y,2)) < m_minimum_distance_to_obstacle)
          {
            m_minimum_distance_to_obstacle = sqrt(pow(m_Obstacles.at(k).position.x ,2) + pow(m_Obstacles.at(k).position.y,2));
          }
          visualization_msgs::Marker obstacle_;
          obstacle_.header.stamp = ros::Time::now();
          obstacle_.header.frame_id = m_base_frame_id;
          obstacle_.type = visualization_msgs::Marker::CUBE;
          obstacle_.action = visualization_msgs::Marker::ADD;
          obstacle_.ns = "obstacles";
          obstacle_.id = marker_id_++;
          obstacle_.pose.position.x = m_Obstacles.at(k).position.x, obstacle_.pose.position.y = m_Obstacles.at(k).position.y, obstacle_.pose.position.z = 0;
          obstacle_.pose.orientation.w = 1;
          obstacle_.scale.x = 0.3, obstacle_.scale.y = 0.3, obstacle_.scale.z = 0.1;
          obstacle_.color.r = 0, obstacle_.color.b = 0, obstacle_.color.g = 1, obstacle_.color.a = 1;
          obstacle_.lifetime = ros::Duration(0.05);
          m_obstacle_markers.markers.push_back(obstacle_);
          once = k;
        } 
      }
    }
  }
  m_current_rollout_index = m_best_rollout_index;
  if (m_ClosestIndex != -1)
    CalculateCurvatureCost(m_CurvatureCost_ptr, RollOut);
  CalculateTrasitionCost(m_TrasitionCost_ptr, RollOut, m_current_rollout_index);
  CalculateConsistancyCost(m_ConsistancyCost_ptr, RollOut, m_best_rollout_index);
  NormalizeCost(m_CollisionCost_ptr, m_CurvatureCost_ptr, m_TrasitionCost_ptr, m_ConsistancyCost_ptr, RollOut);
  pub_obstacles_.publish(m_obstacle_markers);


} 

void CollisionChecker::NormalizeCost(
  double* CollisionCost, double* CurvatureCost, double* TrasitionCost, double* ConsistancyCost,
  const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut)
{
  double collision_cost_;
  double trasition_cost_;
  double curvature_cost_;
  double consistancy_cost_;
  bool all_path_blocked_check_  = true;
  for (unsigned int i=0; i < RollOut.size(); i++)
  {
    curvature_cost_ = *(CurvatureCost + i) / (m_IntegCurvatureCost + 0.0001);
    collision_cost_= *(CollisionCost + i ) ;
    trasition_cost_ = *(TrasitionCost + i) / (m_IntegTrasitionCost + 0.0001);
    consistancy_cost_ = *(ConsistancyCost + i);
    *(m_RollOutCost_ptr + i) = m_WeightCurvature * curvature_cost_ + m_WeightTrasition * trasition_cost_ + 
                               m_WeightCollision * collision_cost_ + m_WeightConsistancy * consistancy_cost_;

    if (*(m_RollOutCost_ptr + i) > m_PathBlockedThres)
    {
      *(m_PathBlocked_ptr + i) = true;
    }
    if (*(m_PathBlocked_ptr + i) && all_path_blocked_check_)
      m_AllPathBlocked = true;
    else
    {
      all_path_blocked_check_ = false;
      m_AllPathBlocked = false;
    }

    // std::cout << "i: " << i <<",  Curvature: " << curvature_cost_ << " ,IntegCurvature: "<< m_IntegCurvatureCost << std::endl;
    // std::cout << "i: " << i <<",  Collision: " << collision_cost_ << std::endl;
    // std::cout << "i: " << i <<",  Trasition: " << trasition_cost_ << " ,IntegTrasition: "<< m_IntegTrasitionCost  << std::endl;
    std::cout << "Cost of Path num[" << i <<"] : " << *(m_RollOutCost_ptr + i) <<  "block : " <<  *(m_PathBlocked_ptr + i) << std::endl;
  }
  std::cout << "minimum distance: " << m_minimum_distance_to_obstacle << ", All Path Blocked? : "<< m_AllPathBlocked <<std::endl;
}

void CollisionChecker::BestPath(const autoware_msgs::LaneArray &GlobalPathCandidates, double* CostArray, bool* BlockedArray,
  std::vector<std::vector<PlannerHNS::WayPoint>> LocalPathCandidates)
{
  for (unsigned int i = 0; i < LocalPathCandidates.size(); i++)
  {
    if (*(CostArray + i) < m_minimum_cost)
    {
      m_minimum_cost = *(CostArray + i);
      m_best_rollout_index = i;
      m_BestPathBlocked = *(CostArray + i); 
    }
  }

  if(LocalPathCandidates.size()>0)
  { 
    autoware_msgs::Lane LaneCost;
    LaneCost.closest_object_distance = m_minimum_distance_to_obstacle;
    // LaneCost.closest_object_velocity = tc.closest_obj_velocity;
    LaneCost.cost = m_minimum_cost;
    LaneCost.is_blocked = m_BestPathBlocked;
    LaneCost.lane_index = m_best_rollout_index;
    pub_TrajectoryCost_.publish(LaneCost);
      
    autoware_msgs::LaneArray local_lanes;
    for(unsigned int i=0; i < LocalPathCandidates.size(); i++)
    {
      autoware_msgs::Lane lane;
      lane = GlobalPathCandidates.lanes.at(i); 
      // lane.closest_object_distance = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_distance;
      // lane.closest_object_velocity = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_velocity;
      lane.cost = *(CostArray + i); 
      lane.is_blocked = *(m_PathBlocked_ptr + i);
      lane.lane_index = i;
      local_lanes.lanes.push_back(lane);
    }

    pub_LocalWeightedTrajectories_.publish(local_lanes);

    visualization_msgs::MarkerArray all_rollOuts;
    TrajectoriesToColoredMarkers(LocalPathCandidates, CostArray, BlockedArray, all_rollOuts);
    pub_LocalWeightedTrajectoriesRviz_.publish(all_rollOuts);

  }
  else
  {
    ROS_ERROR("GlobalLaneArray() Not Equal m_GeneratedRollOuts.size()");
  }

  free(m_RollOutCost_ptr);
  free(m_CollisionCost_ptr);
  free(m_CurvatureCost_ptr);
  free(m_TrasitionCost_ptr);
  free(m_ConsistancyCost_ptr);
  free(m_PathBlocked_ptr);
}

void CollisionChecker::GridMapCheckWithPath_GPU(const nav_msgs::OccupancyGrid& Grid, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut)
{
  int candidates_size = RollOut.size();
  PathCandidates* candidates = (PathCandidates*)malloc(candidates_size * sizeof(PathCandidates));

  for (int i =0; i < candidates_size; i++){
    // PathCandidates device_single_path = candidates[i];
    candidates[i].points_size_ = RollOut.at(i).size();
    candidates[i].points_x_ = (double*)malloc(candidates[i].points_size_ * sizeof(double));
    for (int j = 0; j < candidates[i].points_size_; j++)
    {
      *(candidates[i].points_x_ + j) = RollOut.at(i).at(j).pos.x;
    }

    candidates[i].points_y_ = (double*)malloc(candidates[i].points_size_ * sizeof(double));
    for (int j = 0; j < candidates[i].points_size_; j++)
    {
      *(candidates[i].points_y_ + j) = RollOut.at(i).at(j).pos.y;
    }   
  }
  
  GridMap * grid_map_ = (GridMap*)malloc(m_OccupancyGrid_ptr->info.width * sizeof(GridMap));
  for (unsigned int width = 0; width < m_OccupancyGrid_ptr->info.width; width++){
    grid_map_[width].grid_x_ = (double*)malloc(m_OccupancyGrid_ptr->info.height * sizeof(double));
    for (unsigned int height = 0; height < m_OccupancyGrid_ptr->info.height; height++)
    {
      *(grid_map_[width].grid_x_ + height) = 
        width * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.x;
    }
    grid_map_[width].grid_y_ = (double*)malloc(m_OccupancyGrid_ptr->info.height * sizeof(double));
    for (unsigned int height = 0; height < m_OccupancyGrid_ptr->info.height; height++)
    {
      *(grid_map_[width].grid_y_ + height) = 
        height * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.y;
    }
    grid_map_[width].occupied_intensity_ = (int*)malloc(m_OccupancyGrid_ptr->info.height * sizeof(int));
    for (unsigned int height = 0; height < m_OccupancyGrid_ptr->info.height; height++)
    {
      if(m_OccupancyGrid_ptr->data[height * m_OccupancyGrid_ptr->info.width + width] > 0)
      {
        *(grid_map_[width].occupied_intensity_ + height) = 255;
      }
      else
      {
        *(grid_map_[width].occupied_intensity_ + height) = 0;
      }
    }
  }

  path_candidates_initialize(
    candidates, candidates_size,
    grid_map_, m_OccupancyGrid_ptr->info.width, m_OccupancyGrid_ptr->info.height);

}



/* Utility */
tf::Transform CollisionChecker::getTransform(const std::string& from, const std::string& to)
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

double CollisionChecker::getLookAheadDistance(const nav_msgs::Odometry& CurrentPose,const std::vector<PlannerHNS::WayPoint>& SinglePath, const int& look_ahead_index) 
{
  tf::Transform global2baselink = getTransform(m_base_frame_id, CurrentPose.header.frame_id);
  geometry_msgs::Pose origin_;
  geometry_msgs::Pose look_ahead_;
  origin_ = transformPose(CurrentPose.pose.pose, global2baselink);
  int look_ahead_index_ = look_ahead_index; 
  if ((unsigned int)look_ahead_index >= SinglePath.size())
    look_ahead_index_ = SinglePath.size()  - 1; //m_BaseWaypoints_ptr->waypoints.size() - m_ClosestIndex;
  look_ahead_.position.x = SinglePath.at(look_ahead_index_).pos.x;
  look_ahead_.position.y = SinglePath.at(look_ahead_index_).pos.y;
  tf::Transform origin_tf;
  tf::poseMsgToTF(origin_, origin_tf);

  tf::Transform look_ahead_tf;
  tf::poseMsgToTF(look_ahead_, look_ahead_tf);
  
  return tf::tfDistance(origin_tf.getOrigin(), look_ahead_tf.getOrigin());
}
  
double CollisionChecker::getLookAheadAngleconst(const nav_msgs::Odometry& CurrentPose,const std::vector<PlannerHNS::WayPoint>& SinglePath, const int& look_ahead_index)
{

  tf::Transform global2baselink = getTransform(m_base_frame_id, CurrentPose.header.frame_id);
  geometry_msgs::Pose origin_;
  geometry_msgs::Pose look_ahead_;
  origin_ = transformPose(CurrentPose.pose.pose, global2baselink);
  int look_ahead_index_ = look_ahead_index; 
  if ((unsigned int)look_ahead_index >= SinglePath.size())
    look_ahead_index_ = SinglePath.size() - 1;
  look_ahead_.position.x = SinglePath.at(look_ahead_index_).pos.x;
  look_ahead_.position.y = SinglePath.at(look_ahead_index_).pos.y;

  double dydx = atan2(look_ahead_.position.y - origin_.position.y, look_ahead_.position.x - origin_.position.x);

  return dydx;
}

void CollisionChecker::TrajectoriesToColoredMarkers(
  const std::vector<std::vector<PlannerHNS::WayPoint> >& paths, double* CostArray, bool* BlockedArray,
  visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = m_base_frame_id;
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "local_lane_array_marker_colored";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.1;
  lane_waypoint_marker.scale.y = 0.1;
  //lane_waypoint_marker.scale.z = 0.1;
  lane_waypoint_marker.color.a = 0.9;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.g = 1.0;
  lane_waypoint_marker.color.b = 1.0;
  lane_waypoint_marker.frame_locked = false;

  int count = 0;
  for (unsigned int i = 0; i < paths.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = count;

    for (unsigned int j=0; j < paths.at(i).size(); j++)
    {
      geometry_msgs::Point point;

      point.x = paths.at(i).at(j).pos.x;
      point.y = paths.at(i).at(j).pos.y;
      point.z = paths.at(i).at(j).pos.z;

      lane_waypoint_marker.points.push_back(point);
    }

    lane_waypoint_marker.color.b = 0;

    double norm_cost = *(CostArray + i); 
    if(norm_cost <= 1.0)
    {
      lane_waypoint_marker.color.r = norm_cost;
      lane_waypoint_marker.color.g = 1.0;
    }
    else if(norm_cost > 1.0)
    {
      lane_waypoint_marker.color.r = 1.0;
      lane_waypoint_marker.color.g = 2.0 - norm_cost;
    } 
    else
    {
      lane_waypoint_marker.color.r = 1.0;
      lane_waypoint_marker.color.g = 0.0;
    }

    
    if(*(BlockedArray + i))
    {
      lane_waypoint_marker.color.r = 1.0;
      lane_waypoint_marker.color.g = 0.0;
      lane_waypoint_marker.color.b = 0.0;
    }

    markerArray.markers.push_back(lane_waypoint_marker);
    count++;
  }
}