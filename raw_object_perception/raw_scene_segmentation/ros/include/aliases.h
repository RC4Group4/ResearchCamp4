#ifndef ALIASES_H_
#define ALIASES_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/geometry/planar_polygon.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal PointNT;
typedef pcl::Label PointLT;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<PointLT> PointCloudL;

typedef pcl::PlanarPolygon<PointT> PlanarPolygon;
typedef boost::shared_ptr<PlanarPolygon> PlanarPolygonPtr;
typedef boost::shared_ptr<const PlanarPolygon> PlanarPolygonConstPtr;

#endif
