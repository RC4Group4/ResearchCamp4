#ifndef RANSAC_DOMINANT_PLANE_EXTRACTOR_H_
#define RANSAC_DOMINANT_PLANE_EXTRACTOR_H_

#include "dominant_plane_extractor.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

/** Dominant plane extractor based on the PCL's SACSegmentation class. */
class RansacDominantPlaneExtractor : public DominantPlaneExtractor
{

public:

  RansacDominantPlaneExtractor();

  RansacDominantPlaneExtractor(double downsampling_leaf_size,
                               unsigned int normal_estimation_neighbors,
                               double sac_distance_threshold,
                               unsigned int sac_max_iterations,
                               double sac_normal_distance_weight,
                               bool sac_optimize_coefficiens,
                               double sac_probability);

  virtual void extract(PlanarPolygon& planar_polygon);

  virtual void setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold);

  virtual void removePlaneConstraints();

protected:

  pcl::VoxelGrid<PointT> vg_;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_;
  pcl::SACSegmentationFromNormals<PointT, PointNT> ssfn_;
  pcl::ProjectInliers<PointT> pi_;

};

#endif
