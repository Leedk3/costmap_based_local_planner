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

  private_nh_.param("/voronoi_planner_node/neglectable_area",         m_NeglectableArea, double(50));
  private_nh_.param("/voronoi_planner_node/too_close_area",           m_tooCloseArea, double(0.2));
  private_nh_.param("/voronoi_planner_node/close_to_predictive_path", m_CloseToPredictivePath, double(5));
  private_nh_.param("/voronoi_planner_node/wheel_base",               m_WheelBase, double(1.5));
  private_nh_.param("/voronoi_planner_node/use_cloud_ring",           m_useCloudRing, bool(true));

  PubOutlierFilteredPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/OutlierFiltered", 1,true);
  PubGroundPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/Ground", 1,true);
  PubGroundRemovedPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/GroundRemoved", 1,true);
  SubPoints = nh_.subscribe("/velodyne_points", 1, &VoronoiPlanner::CallbackPoints, this);
  SubVehicleState = nh_.subscribe("/Ackermann/veh_state", 10, &VoronoiPlanner::CallbackAckermann,this);
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


void VoronoiPlanner::CallbackAckermann(const ackermann_msgs::AckermannDriveStampedConstPtr& msg) 
{    
  m_PredictivePoints.clear();
  double range = M_PI / 1.5;
  double increment = 0.01;
  bVehicleState = true;
  double steering_angle_rad = msg->drive.steering_angle * M_PI / 180;
  if (steering_angle_rad == 0)
  {
      m_radius = DBL_MAX; 
  }
  else
  {
      m_radius = m_WheelBase / tan(steering_angle_rad);    
  }

  for (double i = 0; i < range; i += increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    if (m_radius == DBL_MAX)
    {
      p.y = 0;
      p.x = i * 20; //Just for visualization
    }
    else
    {
      if (m_radius > 0)
      {
          p.x = m_radius * sin(i);
          p.y = - m_radius * cos(i) + m_radius;
      }
      else
      {
          p.x = -m_radius * sin(i);
          p.y = - m_radius * cos(i) + m_radius;
      }

    }        
    m_PredictivePoints.push_back(p);
  }
}

void VoronoiPlanner::resetMemory()
{
  //reset variables
  m_OutlierFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_RawSensorCloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());
  m_fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_GroundRemovedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_fullCloud->points.resize(N_SCAN*Horizon_SCAN);
  
}

void VoronoiPlanner::CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  resetMemory();

  // receive pointcloud msg
  pcl::fromROSMsg(*msg, *m_RawSensorCloud_ptr);

  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*m_RawSensorCloud_ptr, *m_RawSensorCloud_ptr, indices);
  
  // have "ring" channel in the cloud
  if (m_useCloudRing == true){
    pcl::fromROSMsg(*msg, *m_laserCloudInRing);
    if (m_laserCloudInRing->is_dense == false) {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }  
  }

  projectPointCloud();
  filteringCloseToPath(msg);
  groundRemoval();
}

void VoronoiPlanner::projectPointCloud()
{
  // range image projection
  float verticalAngle, horizonAngle, range;
  size_t rowIdn, columnIdn, index, cloudSize; 
  pcl::PointXYZI thisPoint;

  cv::Mat rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

  cloudSize = m_RawSensorCloud_ptr->points.size();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    thisPoint.x = m_RawSensorCloud_ptr->points[i].x;
    thisPoint.y = m_RawSensorCloud_ptr->points[i].y;
    thisPoint.z = m_RawSensorCloud_ptr->points[i].z;
    // find the row and column index in the iamge for this point
    if (m_useCloudRing == true){
        rowIdn = m_laserCloudInRing->points[i].ring;
    }
    else{
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
    }
    if (rowIdn < 0 || rowIdn >= N_SCAN)
        continue;

    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
    if (range < sensorMinimumRange)
        continue;
    
    rangeMat.at<float>(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    index = columnIdn  + rowIdn * Horizon_SCAN;
    m_fullCloud->points[index] = thisPoint;
    m_fullCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
  }
}

void VoronoiPlanner::filteringCloseToPath(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointIndices::Ptr outlier_indices(new pcl::PointIndices);
  if(!bVehicleState)
  {
    return;
    ROS_WARN("No vehicle state. Check a '/Ackermann/veh_state' topic.");
  }
    
  for(unsigned int i=0; i< m_fullCloud->points.size(); i++)
  {
    pcl::PointXYZI points_raw;
    points_raw.x= m_fullCloud->points[i].x;
    points_raw.y= m_fullCloud->points[i].y;
    points_raw.z= m_fullCloud->points[i].z;
    points_raw.intensity = m_fullCloud->points[i].intensity;

    double distance2point = sqrt(pow(points_raw.x,2) + 
                                 pow(points_raw.y,2) + 
                                 pow(points_raw.z,2) );
    if (distance2point > m_NeglectableArea || distance2point < m_tooCloseArea || points_raw.x < 0)
    {
      continue;
    }

    point_prev.x = 0, point_prev.y = 0, point_prev.z = 0;

    for (auto point : m_PredictivePoints)
    {
      double slope;
      if (point.x - point_prev.x == 0)
      {
          slope = 0;
      }
      else
      {
          slope = (point.y - point_prev.y) / 
                  (point.x - point_prev.x);                        
      }
      double bias = point.y - slope * point.x;
      double normal_distance = fabs(slope * points_raw.x - points_raw.y + bias) /
                        sqrt(pow(slope,2) + 1);

      //Filtering the points close to the predictive path
      if (normal_distance < m_CloseToPredictivePath && points_raw.x > point_prev.x && points_raw.x < point.x)
      {
        outlier_indices->indices.push_back(i);
      }
      point_prev = point;
    }

  }
  
  m_OutlierFilteredCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_fullCloud);
  extract.setIndices(outlier_indices);
  extract.setNegative(false);//true removes the indices, false leaves only the indices
  extract.filter(*m_OutlierFilteredCloud);

  sensor_msgs::PointCloud2 OutlierFilteredCloudMsg;
  pcl::toROSMsg(*m_OutlierFilteredCloud, OutlierFilteredCloudMsg);
  OutlierFilteredCloudMsg.header = msg->header;
  PubOutlierFilteredPoints.publish(OutlierFilteredCloudMsg);  
}


void VoronoiPlanner::groundRemoval()
{
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  size_t lowerInd, upperInd;
  float diffX, diffY, diffZ, angle;
  cv::Mat groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  // groundMat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < Horizon_SCAN; ++j)
  {
    for (size_t i = 0; i < groundScanInd; ++i)
    {
      lowerInd = j + ( i )*Horizon_SCAN;
      upperInd = j + (i+1)*Horizon_SCAN;

      if (m_fullCloud->points[lowerInd].intensity == -1 ||
          m_fullCloud->points[upperInd].intensity == -1)
      {
        // no info to check, invalid points
        groundMat.at<int8_t>(i,j) = -1;
        continue;
      }
          
      diffX = m_fullCloud->points[upperInd].x - m_fullCloud->points[lowerInd].x;
      diffY = m_fullCloud->points[upperInd].y - m_fullCloud->points[lowerInd].y;
      diffZ = m_fullCloud->points[upperInd].z - m_fullCloud->points[lowerInd].z;

      angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

      if (abs(angle - sensorMountAngle) <= 3)
      {
        groundMat.at<int8_t>(i,j) = 1;
        groundMat.at<int8_t>(i+1,j) = 1;
      }
    }
  }
  // extract ground cloud (groundMat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for segmentation
  // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
  // for (size_t i = 0; i < N_SCAN; ++i)
  // {
  //   for (size_t j = 0; j < Horizon_SCAN; ++j)
  //   {
  //     if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX)
  //     {
  //       labelMat.at<int>(i,j) = -1;
  //     }
  //   }
  // }
  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i <= groundScanInd; ++i)
  {
    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      if (groundMat.at<int8_t>(i,j) == 1)
        groundCloud->push_back(m_fullCloud->points[j + i*Horizon_SCAN]);
        ground_indices->indices.push_back(j + i*Horizon_SCAN);
    }
  }

  m_GroundRemovedCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_fullCloud);
  extract.setIndices(ground_indices);
  extract.setNegative(true);//true removes the indices, false leaves only the indices
  extract.filter(*m_GroundRemovedCloud);

  sensor_msgs::PointCloud2 GroundCloudMsg;
  pcl::toROSMsg(*groundCloud, GroundCloudMsg);
  GroundCloudMsg.header.frame_id = "velodyne";
  GroundCloudMsg.header.stamp = ros::Time::now();
  PubGroundPoints.publish(GroundCloudMsg);  

  sensor_msgs::PointCloud2 GroundRemovedCloudMsg;
  pcl::toROSMsg(*m_GroundRemovedCloud, GroundRemovedCloudMsg);
  GroundRemovedCloudMsg.header.frame_id = "velodyne";
  GroundRemovedCloudMsg.header.stamp = ros::Time::now();
  PubGroundRemovedPoints.publish(GroundRemovedCloudMsg);  

}
