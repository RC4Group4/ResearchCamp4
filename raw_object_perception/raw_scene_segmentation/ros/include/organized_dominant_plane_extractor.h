#ifndef ORGANIZED_DOMINANT_PLANE_EXTRACTOR_H_
#define ORGANIZED_DOMINANT_PLANE_EXTRACTOR_H_

#include "dominant_plane_extractor.h"

#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

/** Dominant plane extractor based on the PCL's OrganizedMultiPlaneSegmentation class. */
class OrganizedDominantPlaneExtractor : public DominantPlaneExtractor
{

public:

  OrganizedDominantPlaneExtractor();

  OrganizedDominantPlaneExtractor(double normal_max_depth_change_factor,
                                  double normal_smoothing_size,
                                  unsigned int min_inliers,
                                  double angular_threshold,
                                  double distance_threshold,
                                  double maximum_curvature,
                                  double refinement_threshold,
                                  double refinement_depth_dependent);

  virtual void extract(PlanarPolygon& planar_polygon);

  virtual void setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold);

  virtual void removePlaneConstraints();

protected:

  static double compute2DPolygonalArea(const PointCloud::VectorType& points, const Eigen::Vector4f& normal);

  pcl::IntegralImageNormalEstimation<PointT, PointNT> ne_;
  pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT> mps_;
  pcl::ProjectInliers<PointT> pi_;

  bool apply_plane_constraints_;
  Eigen::Vector3f plane_normal_;
  double angular_threshold_;

};

#endif
