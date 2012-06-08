#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/console/print.h>
#include <pcl/surface/convex_hull.h>

#include "organized_dominant_plane_extractor.h"

OrganizedDominantPlaneExtractor::OrganizedDominantPlaneExtractor()
: DominantPlaneExtractor()
, apply_plane_constraints_(false)
{
  // Setup IntegralImageNormalEstimation
  ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
  ne_.setMaxDepthChangeFactor(0.02);
  ne_.setNormalSmoothingSize(10.0);
  // Setup OrganizedMultiPlaneSegmentation
  mps_.setMinInliers(1000);
  mps_.setAngularThreshold(pcl::deg2rad(3.0));
  mps_.setDistanceThreshold(0.03);
  mps_.setMaximumCurvature(0.002);
  // Setup ProjectInliers
  pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}

OrganizedDominantPlaneExtractor::OrganizedDominantPlaneExtractor(double normal_max_depth_change_factor,
                                                                 double normal_smoothing_size,
                                                                 unsigned int min_inliers,
                                                                 double angular_threshold,
                                                                 double distance_threshold,
                                                                 double maximum_curvature,
                                                                 double refinement_threshold,
                                                                 double refinement_depth_dependent)
: DominantPlaneExtractor()
, apply_plane_constraints_(false)
{
  // Setup IntegralImageNormalEstimation
  ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
  ne_.setMaxDepthChangeFactor(normal_max_depth_change_factor);
  ne_.setNormalSmoothingSize(normal_smoothing_size);
  // Setup OrganizedMultiPlaneSegmentation
  mps_.setMinInliers(min_inliers);
  mps_.setAngularThreshold(angular_threshold);
  mps_.setDistanceThreshold(distance_threshold);
  mps_.setMaximumCurvature(maximum_curvature);
  pcl::PlaneRefinementComparator<PointT, PointNT, PointLT>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, PointNT, PointLT>());
  refinement_compare->setDistanceThreshold(refinement_threshold, refinement_depth_dependent);
  mps_.setRefinementComparator(refinement_compare);
  // Setup ProjectInliers
  pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
}

void OrganizedDominantPlaneExtractor::extract(PlanarPolygon& planar_polygon)
{
  // Step 1: perform multiplanar segmentation
  PointCloudN::Ptr normals(new PointCloudN);
  std::vector<pcl::PlanarRegion<PointT>> regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  PointCloudL::Ptr labels(new PointCloudL);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  // Estimate normals
  ne_.setInputCloud(input_);
  ne_.compute(*normals);
  // Run multiplane segmentation with refinement
  mps_.setInputCloud(input_);
  mps_.setInputNormals(normals);
  mps_.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  if (regions.size() == 0)
  {
    ROS_WARN("[OrganizedDominantPlaneExtractor::extract] No planar regions found! Aborting.");
    return;
  }

  // Step 2: find the largest region (applying plane normal constraints if needed)
  PointCloud::Ptr largest_region_hull;
  double largest_region_area = -1;
  size_t largest_region_id = 0;
  // Prepare ProjectInliers module
  pi_.setInputCloud(input_);
  for (size_t i = 0; i < regions.size(); i++)
  {
    if (apply_plane_constraints_)
    {
      Eigen::Vector3f region_normal = regions.at(i).getCoefficients().head<3>();
      if (region_normal.dot(plane_normal_) < angular_threshold_)
        continue;
    }
    PointCloud::Ptr region_cloud(new PointCloud);
    PointCloud::Ptr region_hull(new PointCloud);
    // Project boundary points onto the plane
    pi_.setIndices(boost::make_shared<pcl::PointIndices>(boundary_indices.at(i)));
    pi_.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(model_coefficients.at(i)));
    pi_.filter(*region_cloud);
    // Compute convex hull around boundary points
    pcl::ConvexHull<PointT> convex_hull;
    convex_hull.setInputCloud(region_cloud);
    convex_hull.reconstruct(*region_hull);
    double area = compute2DPolygonalArea(region_hull->points, regions.at(i).getCoefficients());
    if (area > largest_region_area)
    {
      largest_region_hull = region_hull;
      largest_region_area = area;
      largest_region_id = i;
    }
  }
  shrinkPointCloud(largest_region_hull, shrink_plane_polygon_ratio_);
  planar_polygon = PlanarPolygon(largest_region_hull->points, regions.at(largest_region_id).getCoefficients());
}

void OrganizedDominantPlaneExtractor::setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold)
{
  apply_plane_constraints_ = true;
  plane_normal_ = normal;
  angular_threshold_ = angle_threshold;
}

void OrganizedDominantPlaneExtractor::removePlaneConstraints()
{
  apply_plane_constraints_ = false;
}

double OrganizedDominantPlaneExtractor::compute2DPolygonalArea(const PointCloud::VectorType& points, const Eigen::Vector4f& normal)
{
  // Find axis with largest normal component and project onto perpendicular plane
  int k0, k1, k2;
  k0 = (std::fabs(normal[0]) > std::fabs(normal[1])) ? 0 : 1;
  k0 = (std::fabs(normal[k0]) > std::fabs(normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  double area = 0;
  for (size_t i = 0; i < points.size(); i++)
  {
    size_t j = (i + 1) % points.size();
    float p1[3] = { points[i].x, points[i].y, points[i].z };
    float p2[3] = { points[j].x, points[j].y, points[j].z };
    area += p1[k1] * p2[k2] - p1[k2] * p2[k1];
  }

  return fabs(area) / (2 * std::fabs(normal[k0]));
}
