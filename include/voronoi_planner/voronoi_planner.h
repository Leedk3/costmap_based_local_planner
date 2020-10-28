/*
 * voronoi_planner.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */
#ifndef VORONOI_PLANNER_H
#define VORONOI_PLANNER_H

// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>

#include <opencv/cv.h>

// headers in Boost Library
#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

  template <>
  struct geometry_concept<Point> {
    typedef point_concept type;
  };

  template <>
  struct point_traits<Point> {
    typedef int coordinate_type;

    static inline coordinate_type get(
        const Point& point, orientation_2d orient) {
      return (orient == HORIZONTAL) ? point.a : point.b;
    }
  };

  template <>
  struct geometry_concept<Segment> {
    typedef segment_concept type;
  };

  template <>
  struct segment_traits<Segment> {
    typedef int coordinate_type;
    typedef Point point_type;

    static inline point_type get(const Segment& segment, direction_1d dir) {
      return dir.to_int() ? segment.p1 : segment.p0;
    }
  };
}  // polygon
}  // boost


// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double>& vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    if (it->is_primary())
      ++result;
  }
  return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<double> &vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
       it != vd.cells().end(); ++it) {
    const voronoi_diagram<double>::cell_type& cell = *it;
    const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
    // This is convenient way to iterate edges around Voronoi cell.
    do {
      if (edge->is_primary())
        ++result;
      edge = edge->next();
    } while (edge != cell.incident_edge());
  }
  return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
int iterate_primary_edges3(const voronoi_diagram<double> &vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_vertex_iterator it =
       vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    const voronoi_diagram<double>::vertex_type& vertex = *it;
    const voronoi_diagram<double>::edge_type* edge = vertex.incident_edge();
    // This is convenient way to iterate edges around Voronoi vertex.
    do {
      if (edge->is_primary())
        ++result;
      edge = edge->rot_next();
    } while (edge != vertex.incident_edge());
  }
  return result;
}


/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;


class VoronoiPlanner
{
public:
  VoronoiPlanner();
  ~VoronoiPlanner();

  void init();
  void run();
  void CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg);
  void CallbackAckermann(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
  void projectPointCloud();
  void filteringCloseToPath(const sensor_msgs::PointCloud2ConstPtr& msg);
  void resetMemory();
  void groundRemoval();
private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher PubOutlierFilteredPoints;
  ros::Publisher PubGroundPoints, PubGroundRemovedPoints;
  ros::Subscriber SubPoints;
  ros::Subscriber SubVehicleState;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_OutlierFilteredCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_RawSensorCloud_ptr;
  pcl::PointCloud<PointXYZIR>::Ptr m_laserCloudInRing;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_GroundRemovedCloud;


  bool bPredictivePath;
  bool bVehicleState;

  double m_NeglectableArea;  
  double m_tooCloseArea;
  double m_CloseToPredictivePath;
  double m_radius;
  double m_WheelBase;
  bool m_useCloudRing;

  geometry_msgs::Point point_prev;
  std::vector<geometry_msgs::Point> m_PredictivePoints;

  // VLP-16
  const int N_SCAN = 16;
  const int Horizon_SCAN = 1800;
  const float ang_res_x = 0.2;
  const float ang_res_y = 2.0;
  const float ang_bottom = 15.0+0.1;
  const int groundScanInd = 5; //7

  const float sensorMinimumRange = 1.0;
  const float sensorMountAngle = 0.0;
  const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
  const int segmentValidPointNum = 5;
  const int segmentValidLineNum = 3;
  const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
  const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
  
  const int edgeFeatureNum = 2;
  const int surfFeatureNum = 4;
  const int sectionsTotal = 6;
  const float edgeThreshold = 0.1;
  const float surfThreshold = 0.1;
  const float nearestFeatureSearchSqDist = 25;
};


#endif  // VORONOI_PLANNER_H
