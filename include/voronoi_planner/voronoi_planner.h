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

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>

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


class VoronoiPlanner
{
public:
  VoronoiPlanner();
  ~VoronoiPlanner();

  void init();
  void run();
  void CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg);
  void CallbackPredictivePath(const visualization_msgs::MarkerConstPtr &msg);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher PubOutlierFilteredPoints;
  ros::Subscriber SubPoints;
  ros::Subscriber SubPredictivePath;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_OutlierFilteredCloud;
  std::shared_ptr<visualization_msgs::Marker> m_PredictivePath_ptr;
  bool bPredictivePath;
};

#endif  // VORONOI_PLANNER_H
