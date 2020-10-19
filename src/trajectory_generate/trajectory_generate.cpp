#include "trajectory_generate/trajectory_generate.h"
#include <string>
#include <assert.h>

// Constructor
TrajectoryGenerator::TrajectoryGenerator() : nh_("~"), private_nh_("~"),
bGlobalWaypoint(false), bOdometry(false), bClosestWaypoint(false)
{
  pubLocalTrajectories = nh_.advertise<autoware_msgs::LaneArray>("/local_trajectories", 1,true);
  pubLocalTrajectoriesRviz = nh_.advertise<visualization_msgs::MarkerArray>("/local_trajectories_gen_rviz", 1);

  subGlobalWaypoint = nh_.subscribe("/lane_waypoints_array", 1, &TrajectoryGenerator::CallbackGlobalWptThroughAutowareMsg, this);
  subOdom = nh_.subscribe("/Odometry/ekf_slam", 1, &TrajectoryGenerator::CallbackOdometry, this);
  subClosestWaypoint = nh_.subscribe("/closest_waypoint", 1, &TrajectoryGenerator::CallbackClosestIndex, this);

  init();
}

TrajectoryGenerator::~TrajectoryGenerator(){}

void TrajectoryGenerator::init()
{
  ROS_INFO("Trajectory generator init");

  private_nh_.param("enableLaneChange" , m_PlanningParams.enableLaneChange , bool(false));
  private_nh_.param("microPlanDistance" , m_PlanningParams.microPlanDistance , double(35.0));
  private_nh_.param("maxVelocity" , m_PlanningParams.maxSpeed , double(6.0));
  private_nh_.param("minVelocity" , m_PlanningParams.minSpeed , double(0.1));
  private_nh_.param("carTipMargin" , m_PlanningParams.carTipMargin , double(4));
  private_nh_.param("rollInMargin" , m_PlanningParams.rollInMargin , double(16));
  private_nh_.param("rollInSpeedFactor" , m_PlanningParams.rollInSpeedFactor , double(0.25));
  private_nh_.param("pathDensity" , m_PlanningParams.pathDensity , double(0.5));
  private_nh_.param("rollOutDensity" , m_PlanningParams.rollOutDensity , double(0.5));
  private_nh_.param("rollOutsNumber" , m_PlanningParams.rollOutNumber , int(6));
  private_nh_.param("smoothingDataWeight" , m_PlanningParams.smoothingDataWeight , double(0.45));
  private_nh_.param("smoothingSmoothWeight" , m_PlanningParams.smoothingSmoothWeight , double(0.4));
  private_nh_.param("speedProfileFactor" , m_PlanningParams.speedProfileFactor , double(1.2));
  private_nh_.param("enableHeadingSmoothing" , m_PlanningParams.horizonDistance , double(100));
  private_nh_.param("horizonDistance" , m_PlanningParams.enableHeadingSmoothing , bool(false));
}

void TrajectoryGenerator::CallbackOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  m_Odometry_ptr = std::make_shared<nav_msgs::Odometry>(*msg);
  m_CurrentPos = PlannerHNS::WayPoint(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z,
                                      tf::getYaw(msg->pose.pose.orientation));
  bOdometry = true;
}

void TrajectoryGenerator::CallbackGlobalWptThroughAutowareMsg(const autoware_msgs::LaneArrayConstPtr& msg)
{
  m_GlobalWaypointAutowareMsg_ptr = std::make_shared<autoware_msgs::LaneArray>(*msg);

  m_GlobalPaths.clear();
  for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
  {
    std::vector<PlannerHNS::WayPoint> temp_path;
    PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), temp_path);

    PlannerHNS::PlanningHelpers::CalcAngleAndCost(temp_path);
    m_GlobalPaths.push_back(temp_path);
  }

  bGlobalWaypoint = true;
}

void TrajectoryGenerator::CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg)
{
  m_ClosestIndex = msg->data;
  bClosestWaypoint = true;
}

void TrajectoryGenerator::run()
{

  if(bOdometry && bGlobalWaypoint && m_GlobalPaths.size()>0)
  {
    m_GlobalPathSections.clear();

    for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
    {
      t_centerTrajectorySmoothed.clear();
      PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), m_CurrentPos, 
                                                                               m_PlanningParams.horizonDistance,
                                                                               m_PlanningParams.pathDensity ,
                                                                               t_centerTrajectorySmoothed);

      m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
    }
    
    std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
    m_Planner.GenerateRunoffTrajectory(m_GlobalPathSections, m_CurrentPos,
              m_PlanningParams.enableLaneChange,       // enableLaneChange
              m_Odometry_ptr->twist.twist.linear.x,    // velocity [ m/s ]
              m_PlanningParams.microPlanDistance,      // microPlanDistance
              m_PlanningParams.maxSpeed,               // maxSpeed
              m_PlanningParams.minSpeed,               // minSpeed
              m_PlanningParams.carTipMargin,           // carTopMargin
              m_PlanningParams.rollInMargin,           // rollInMargin
              m_PlanningParams.rollInSpeedFactor,      // rollInSpeedFactor
              m_PlanningParams.pathDensity,            // pathDensity
              m_PlanningParams.rollOutDensity,         // rollOutDensity
              m_PlanningParams.rollOutNumber,          // rollOutNumber
              m_PlanningParams.smoothingDataWeight,    // smoothingDataWeight
              m_PlanningParams.smoothingSmoothWeight,  // smoothingSmoothWeight
              m_PlanningParams.smoothingToleranceError,// smoothingToleranceError
              m_PlanningParams.speedProfileFactor,     // speedProfileFactor
              m_PlanningParams.enableHeadingSmoothing, // enableHeadingSmoothing
              -1 , -1,
              m_RollOuts, sampledPoints_debug);

    autoware_msgs::LaneArray local_lanes;
    for(unsigned int i=0; i < m_RollOuts.size(); i++)
    {
      for(unsigned int j=0; j < m_RollOuts.at(i).size(); j++)
      {
        autoware_msgs::Lane lane;
        PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_RollOuts.at(i).at(j), m_CurrentPos, m_PlanningParams.minSpeed, m_PlanningParams.microPlanDistance);
        PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_RollOuts.at(i).at(j), lane);
        lane.closest_object_distance = 0;
        lane.closest_object_velocity = 0;
        lane.cost = 0;
        lane.is_blocked = false;
        lane.lane_index = i;
        local_lanes.lanes.push_back(lane);
      }
    }

    pubLocalTrajectories.publish(local_lanes);
    visualization_msgs::MarkerArray all_rollOuts;
    PlannerHNS::ROSHelpers::TrajectoriesToMarkers(m_RollOuts, all_rollOuts);
    pubLocalTrajectoriesRviz.publish(all_rollOuts);
  }
  else
  {
    ROS_WARN("Check trajectory generator.");
  }
  
}