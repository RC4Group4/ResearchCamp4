#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <pcl/filters/passthrough.h>

#include <raw_srvs/GetDominantPlane.h>
#include "dominant_plane_extractor.h"
#include "ransac_dominant_plane_extractor.h"

DominantPlaneExtractor::UPtr dpe;
PlanarPolygon planar_polygon;
bool waiting_for_cloud;
ros::Time stamp;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
{
  // Get filtering bounds from the parameter server.
  ros::NodeHandle pn("~");
  double min_x_bound, max_x_bound;
  double min_y_bound, max_y_bound;
  double min_z_bound, max_z_bound;
  pn.param("min_x", min_x_bound, -1.5);
  pn.param("max_x", max_x_bound,  1.5);
  pn.param("min_y", min_y_bound, -1.5);
  pn.param("max_y", max_y_bound,  1.5);
  pn.param("min_z", min_z_bound,  0.4);
  pn.param("max_z", max_z_bound,  1.5);

  // Prepare point clouds.
  PointCloud::Ptr cloud(new PointCloud);
  PointCloud::Ptr cloud_filtered(new PointCloud);
  pcl::fromROSMsg(*ros_cloud, *cloud);

  // Setup and run pass through filter.
  pcl::PassThrough<PointT> pass_through;
  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(min_x_bound, max_x_bound);
  pass_through.setFilterFieldName("y");
  pass_through.setFilterLimits(min_y_bound, max_y_bound);
  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(min_z_bound, max_z_bound);
  pass_through.setKeepOrganized(true);
  pass_through.setInputCloud(cloud);
  pass_through.filter(*cloud_filtered);

  // Extract dominant plane.
  dpe->setInputCloud(cloud_filtered);
  dpe->setShrinkPlanePolygonRatio(0.07);
  dpe->extract(planar_polygon);

  // Log information.
  ROS_INFO_STREAM("Number of points in plane contour: " << planar_polygon.getContour().size());
  ROS_INFO_STREAM("Plane coefficients:\n" << planar_polygon.getCoefficients());

  // Signal that the processing is done.
  waiting_for_cloud = false;
  stamp = cloud->header.stamp;
}

bool extractPlaneCallback(raw_srvs::GetDominantPlane::Request& request, raw_srvs::GetDominantPlane::Response& response)
{
  ROS_INFO("Received a request to extract dominant plane.");

  // TODO: Configure plane extractor with the parameters from request.

  // Get point cloud topic name from the parameter server.
  ros::NodeHandle pn("~");
  std::string input_cloud_topic;
  pn.param("input_cloud_topic", input_cloud_topic, std::string("/depth_registered/points"));

  // Subscribe to the topic.
  ros::NodeHandle node;
  ros::Subscriber subscriber = node.subscribe(input_cloud_topic, 1, cloudCallback);

  // Wait until a message is received and processed.
  waiting_for_cloud = true;
  while (waiting_for_cloud && ros::ok())
  {
    ros::spinOnce();
  }
  subscriber.shutdown();

  // Pack the response.
  response.stamp = stamp;
  for (int i = 0; i < 4; ++i)
  {
    response.coefficients[i] = planar_polygon.getCoefficients()[i];
  }
  for (const auto& point : planar_polygon.getContour())
  {
    geometry_msgs::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    response.contour.push_back(pt);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dominant_plane_extractor_node");

  dpe = DominantPlaneExtractor::UPtr(new RansacDominantPlaneExtractor);

  ros::NodeHandle node;
  ros::ServiceServer extract_plane_service = node.advertiseService("extract_dominant_plane", extractPlaneCallback);

  ROS_INFO("Dominant plane extractor service started.");

  ros::spin();

  return 0;
}
