#include <pcl/common/centroid.h>

#include "dominant_plane_extractor.h"

void DominantPlaneExtractor::shrinkPointCloud(PointCloud::Ptr& cloud, double shrink_ratio)
{
  if (shrink_ratio == 0.0) return;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    auto& pt = cloud->points[i];
    pt.getVector4fMap() = (pt.getVector4fMap() - centroid) * (1 - shrink_ratio) + centroid;
  }
}
