#ifndef TABLETOP_CLUSTER_EXTRACTOR_H_
#define TABLETOP_CLUSTER_EXTRACTOR_H_

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include "aliases.h"

class TabletopClusterExtractor
{

public:

  TabletopClusterExtractor();

  TabletopClusterExtractor(double object_min_height,
                           double object_max_height,
                           double object_cluster_tolerance,
                           int object_cluster_min_size,
                           int object_cluster_max_size);

  ~TabletopClusterExtractor()
  {
    input_.reset();
  }

  void extract(std::vector<PointCloud::Ptr>& clusters);

  inline void setInputCloud(const PointCloud::ConstPtr& cloud)
  {
    input_ = cloud;
  }

  inline PointCloud::ConstPtr getInputCloud() const
  {
    return input_;
  }

  inline void setTablePolygon(const PlanarPolygonConstPtr& polygon)
  {
    table_polygon_ = polygon;
  }

  inline PlanarPolygonConstPtr getTablePolygon() const
  {
    return table_polygon_;
  }

private:

  PointCloud::ConstPtr input_;
  PlanarPolygonConstPtr table_polygon_;

  pcl::ExtractPolygonalPrismData<PointT> eppd_;
  pcl::EuclideanClusterExtraction<PointT> ece_;

};

#endif
