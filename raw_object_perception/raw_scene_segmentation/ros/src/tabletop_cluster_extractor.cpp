#include <boost/make_shared.hpp>

#include "tabletop_cluster_extractor.h"

TabletopClusterExtractor::TabletopClusterExtractor()
{
  // Setup ExtractPolygonalPrismData for determining the points supported by the plane
  eppd_.setHeightLimits(0.01, 3);
  // Setup EuclideanClusterExtraction
  ece_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  ece_.setClusterTolerance(0.01);
  ece_.setMinClusterSize(15);
  ece_.setMaxClusterSize(5000);
}

TabletopClusterExtractor::TabletopClusterExtractor(double object_min_height,
                                                   double object_max_height,
                                                   double object_cluster_tolerance,
                                                   int object_cluster_min_size,
                                                   int object_cluster_max_size)
{
  // Setup ExtractPolygonalPrismData for determining the points supported by the plane
  eppd_.setHeightLimits(object_min_height, object_max_height);
  // Setup EuclideanClusterExtraction
  ece_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  ece_.setClusterTolerance(object_cluster_tolerance);
  ece_.setMinClusterSize(object_cluster_min_size);
  ece_.setMaxClusterSize(object_cluster_max_size);
}

void TabletopClusterExtractor::extract(std::vector<PointCloud::Ptr>& clusters)
{
  pcl::PointIndices::Ptr tabletop_objects_indices(new pcl::PointIndices);
  PointCloud::Ptr table_polygon_cloud(new PointCloud);
  table_polygon_cloud->points = table_polygon_->getContour();
  // TODO: shrink the region a bit to get rid of weird stuff on the frontiers
  eppd_.setInputCloud(input_);
  eppd_.setInputPlanarHull(table_polygon_cloud);
  eppd_.segment(*tabletop_objects_indices);

  PointCloud::Ptr tabletop_objects(new PointCloud);
  pcl::copyPointCloud(*input_, *tabletop_objects_indices, *tabletop_objects);
  std::vector<pcl::PointIndices> clusters_indices;
  ece_.setInputCloud(tabletop_objects);
  ece_.extract(clusters_indices);

  for (const pcl::PointIndices& cluster_indices : clusters_indices)
  {
    PointCloud::Ptr cluster_cloud(new PointCloud);
    pcl::copyPointCloud(*tabletop_objects, cluster_indices, *cluster_cloud);
    clusters.push_back(cluster_cloud);
  }
}
