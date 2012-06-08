#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/console/print.h>

#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>

#include "ransac_dominant_plane_extractor.h"

RansacDominantPlaneExtractor::RansacDominantPlaneExtractor()
: DominantPlaneExtractor()
{
  // Setup VoxelGrid for downsampling
  vg_.setLeafSize(0.01, 0.01, 0.01);
  vg_.setDownsampleAllData(false);
  // Setup NormalEstimation
  ne_.setKSearch(50);
  ne_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  // setup SACSegmentationFromNormals for plane fitting
  ssfn_.setDistanceThreshold(0.01);
  ssfn_.setMaxIterations(2000);
  ssfn_.setNormalDistanceWeight(0.1);
  ssfn_.setOptimizeCoefficients(true);
  ssfn_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  ssfn_.setMethodType(pcl::SAC_RANSAC);
  ssfn_.setProbability(0.99);
  // Setup ProjectInliers
  pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}

RansacDominantPlaneExtractor::RansacDominantPlaneExtractor(double downsampling_leaf_size,
                                                           unsigned int normal_estimation_neighbors,
                                                           double sac_distance_threshold,
                                                           unsigned int sac_max_iterations,
                                                           double sac_normal_distance_weight,
                                                           bool sac_optimize_coefficiens,
                                                           double sac_probability)
: DominantPlaneExtractor()
{
  // Setup VoxelGrid for downsampling
  vg_.setLeafSize(downsampling_leaf_size, downsampling_leaf_size, downsampling_leaf_size);
  vg_.setDownsampleAllData(false);
  // Setup NormalEstimation
  ne_.setKSearch(normal_estimation_neighbors);
  ne_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  // Setup SACSegmentationFromNormals for plane fitting
  ssfn_.setDistanceThreshold(sac_distance_threshold);
  ssfn_.setMaxIterations(sac_max_iterations);
  ssfn_.setNormalDistanceWeight(sac_normal_distance_weight);
  ssfn_.setOptimizeCoefficients(sac_optimize_coefficiens);
  ssfn_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  ssfn_.setMethodType(pcl::SAC_RANSAC);
  ssfn_.setProbability(sac_probability);
  // Setup ProjectInliers
  pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}

void RansacDominantPlaneExtractor::extract(PlanarPolygon& planar_polygon)
{
  // Part 1: find inliers and coefficients for the dominant plane
  pcl::PointIndices::Ptr plane_inliers_(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr plane_coefficients_(new pcl::ModelCoefficients);
  PointCloud::Ptr downsampled_cloud(new PointCloud);
  PointCloudN::Ptr downsampled_normals(new PointCloudN);
  // Downsample input
  vg_.setInputCloud(input_);
  vg_.filter(*downsampled_cloud);
  // Estimate normals for the downsampled cloud
  ne_.setInputCloud(downsampled_cloud);
  ne_.compute(*downsampled_normals);
  // Segment plane
  ssfn_.setInputCloud(downsampled_cloud);
  ssfn_.setInputNormals(downsampled_normals);
  ssfn_.segment(*plane_inliers_, *plane_coefficients_);
  if (plane_inliers_->indices.size() == 0)
  {
    ROS_WARN("[RansacDominantPlaneExtractor::extract] No planar regions found! Aborting.");
    return;
  }

  // Part 2: create corresponding PlanarPolygon
  PointCloud::Ptr plane_cloud(new PointCloud);
  PointCloud::Ptr plane_hull(new PointCloud);
  // Project inliers onto the plane
  pi_.setInputCloud(downsampled_cloud);
  pi_.setIndices(plane_inliers_);
  pi_.setModelCoefficients(plane_coefficients_);
  pi_.filter(*plane_cloud);
  // Calculate convex hull for inliers
  pcl::ConvexHull<PointT> convex_hull;
  convex_hull.setInputCloud(plane_cloud);
  convex_hull.reconstruct(*plane_hull);
  // Create PlanarPolygon
  Eigen::Vector4f coefficients;
  coefficients << plane_coefficients_->values[0],
                  plane_coefficients_->values[1],
                  plane_coefficients_->values[2],
                  plane_coefficients_->values[3];
  shrinkPointCloud(plane_hull, shrink_plane_polygon_ratio_);
  planar_polygon = PlanarPolygon(plane_hull->points, coefficients);
}

void RansacDominantPlaneExtractor::setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold)
{
  ssfn_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  ssfn_.setAxis(normal);
  ssfn_.setEpsAngle(angle_threshold);
}

void RansacDominantPlaneExtractor::removePlaneConstraints()
{
  ssfn_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}
