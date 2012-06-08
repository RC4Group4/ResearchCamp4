#ifndef DOMINANT_PLANE_EXTRACTOR_H_
#define DOMINANT_PLANE_EXTRACTOR_H_

#include "aliases.h"

/** Interface for the classes that analyze the scene and find the largest planar region. */
class DominantPlaneExtractor
{

public:

  typedef std::shared_ptr<DominantPlaneExtractor> Ptr;
  typedef std::unique_ptr<DominantPlaneExtractor> UPtr;

  DominantPlaneExtractor()
  : shrink_plane_polygon_ratio_(0.0)
  { }

  virtual void extract(PlanarPolygon& planar_polygon) = 0;

  virtual void setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold) = 0;

  virtual void removePlaneConstraints() = 0;

  inline void setInputCloud(const PointCloud::ConstPtr &cloud)
  {
    input_ = cloud;
  }

  inline PointCloud::ConstPtr getInputCloud() const
  {
    return input_;
  }

  inline void setShrinkPlanePolygonRatio(double ratio)
  {
    shrink_plane_polygon_ratio_ = ratio;
  }

  inline double getShrinkPlanePolygonRatio() const
  {
    return shrink_plane_polygon_ratio_;
  }

  virtual ~DominantPlaneExtractor()
  {
    input_.reset();
  }

protected:

  /** Move each point of the point cloud towards its centroid by @arg shrink_ratio percents of its original distance
    * to the centroid.
    * @param shrink_ratio : value in [0..1] range. */
  static void shrinkPointCloud(PointCloud::Ptr& cloud, double shrink_ratio);

  PointCloud::ConstPtr input_;

  double shrink_plane_polygon_ratio_;

};

#endif
