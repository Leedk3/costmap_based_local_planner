/*
 * voronoi_planner.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */

#include "voronoi_planner/voronoi_planner.h"
#include <string>
#include <assert.h>

// Constructor
VoronoiPlanner::VoronoiPlanner() : nh_(""), private_nh_("~"), bPredictivePath(false)
{

  // private_nh_.param("base_frame_id", m_base_frame_id, std::string("base_link"));
  // private_nh_.param("local_update_horizon", m_local_update_horizon, double(10));

  PubOutlierFilteredPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/OutlierFiltered", 1,true);
  SubPoints = nh_.subscribe("/front/velodyne_points", 1, &VoronoiPlanner::CallbackPoints, this);
  SubPredictivePath = nh_.subscribe("/Marker/System/PredictivePath", 1, &VoronoiPlanner::CallbackPredictivePath, this);

  init();
}

VoronoiPlanner::~VoronoiPlanner(){}

void VoronoiPlanner::init()
{
  ROS_INFO("initialize voronoi planner");

  std::vector<Point> points;
  points.push_back(Point(0, 0));
  points.push_back(Point(1, 6));
  std::vector<Segment> segments;
  segments.push_back(Segment(-4, 5, 5, -1));
  segments.push_back(Segment(3, -11, 13, -1));
  
  // voronoi_diagram::voronoi_diagram<double> vd;
  // construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

}

void VoronoiPlanner::run()
{

}

void VoronoiPlanner::CallbackPredictivePath(const visualization_msgs::MarkerConstPtr &msg)
{
  m_PredictivePath_ptr = std::make_shared<visualization_msgs::Marker>(*msg);
  bPredictivePath = true;
  
  // std::cout << "----------------------" << std::endl;
  // m_Collision = false;
  // if(bCostMap && bVehicleState)
  // {
  //     CalculatePredictivePath();
  //     for (auto obstacle : m_Obstacles)
  //     {
  //         double cost_distance = sqrt(pow(obstacle.position.x,2) + 
  //                                 pow(obstacle.position.y,2));
  //         if (cost_distance > m_NeglectableArea || fabs(obstacle.position.x) < m_tooCloseArea || obstacle.position.x < 0)
  //         {
  //             continue;
  //         }
  //         if (!m_PredictivePathMarker.points.empty())
  //         {
  //             for (auto point : m_PredictivePathMarker.points)
  //             {
  //                 double slope;
  //                 if (point.x - point_prev.x == 0)
  //                 {
  //                     slope = 0;
  //                 }
  //                 else
  //                 {
  //                     slope = (point.y - point_prev.y) / 
  //                             (point.x - point_prev.x);                        
  //                 }
  //                 double bias = point.y - slope * point.x;
  //                 double distance = fabs(slope * obstacle.position.x - obstacle.position.y + bias) /
  //                                   sqrt(pow(slope,2) + 1);

  //                 double squared_distance = sqrt(pow(point.x - obstacle.position.x,2) + 
  //                                                 pow(point.y - obstacle.position.y,2));
                  
  //                 if (distance < m_normalDist && obstacle.position.x > point_prev.x && obstacle.position.x < point.x)
  //                 {
  //                     std::cout << obstacle.position << std::endl;
  //                     std::cout << "collision!, dist: " << distance <<std::endl;
  //                     std::cout << "bias: " << bias <<std::endl;
  //                     std::cout << "slope: " << slope <<std::endl;
                      
  //                     m_Collision = true;
  //                     break;
  //                 }
  //                 point_prev = point;
  //             }
  //         }
  //     }
  // }

  // else
  // {
  //     ROS_WARN("Check costmap generator or control command publisher.");
  // }
  // std_msgs::Bool collision_msg;
  // collision_msg.data = m_Collision;
  // PubCollision.publish(collision_msg);
    
}


void VoronoiPlanner::CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr RawSensorCloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *RawSensorCloud_ptr);  
  pcl::PointIndices::Ptr outlier_indices(new pcl::PointIndices);
  m_OutlierFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  for(unsigned int i=0; i< RawSensorCloud_ptr->points.size(); i++)
  {
    pcl::PointXYZI points_raw;
    points_raw.x= RawSensorCloud_ptr->points[i].x;
    points_raw.y= RawSensorCloud_ptr->points[i].y;
    points_raw.z= RawSensorCloud_ptr->points[i].z;
    points_raw.intensity = RawSensorCloud_ptr->points[i].intensity;

    // if (points_raw.y < (in_left_lane_threshold) && points_raw.y > -1.0*in_right_lane_threshold)
    // {
    //   if (points_raw.x < (in_front_threshold) && points_raw.x > -1.0*in_rear_threshold)
    //   {
    //     outlier_indices->indices.push_back(i);
    //   }
    // }
  }
  
  m_OutlierFilteredCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(RawSensorCloud_ptr);
  extract.setIndices(outlier_indices);
  extract.setNegative(true);//true removes the indices, false leaves only the indices
  extract.filter(*m_OutlierFilteredCloud);

  sensor_msgs::PointCloud2 OutlierFilteredCloudMsg;
  pcl::toROSMsg(*m_OutlierFilteredCloud, OutlierFilteredCloudMsg);
  OutlierFilteredCloudMsg.header = msg->header;
  PubOutlierFilteredPoints.publish(OutlierFilteredCloudMsg);


}