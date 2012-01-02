/*
 *  objectCandidateExtraction3D.cpp
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */
#define DO_CANDIDATION 1
#define SHOW_OBJECTS 1
#define SHOW_PLANES 1
#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <ros/ros.h>
#include <ros/publisher.h>
#include "sensor_msgs/Image.h"

#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"
#include "pcl/io/pcd_io.h"

#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_ros/segmentation/extract_clusters.h"
#include "CObjectCandidateExtraction.h"

#include "tf/transform_listener.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "CToolBoxROS.h"
#include "pcl_ros/publisher.h"
#include <stdio.h>
#include <string>
#include <stdlib.h>

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "dynamic_reconfigure/ConfigDescription.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "dynamic_reconfigure/Config.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include "std_msgs/MultiArrayDimension.h"
#include <boost/thread/mutex.hpp>

#include "StructPlanarSurface.h"
#include "unistd.h"

#include "brsu_msgs/ObjectCandidateList3D.h"
#include "brsu_srvs/GetObjectCandidateList3D.h"

pcl::PointCloud<pcl::PointXYZ> cloud_Inliers;
ros::Publisher pmd_pub0, pmd_pub1, pmd_pub2, pmd_pub3, pmd_pub4;

CObjectCandidateExtraction *objectCandidateExtractor;
CToolBoxROS toolBox;

int counter = 0;
boost::mutex mutexExtractedObjects;

int max_augment = 0;
pcl::PointCloud<pcl::PointXYZRGB> augmentPointCloud;

#define SAMPLING_DISTANCE 0.025

#define SPHERICAL_DISTANCE 2.5f

#define X_DISTANCE_MIN -1.5f
#define X_DISTANCE_MAX 1.5f

#define Y_DISTANCE_MIN -1.5f
#define Y_DISTANCE_MAX 1.5f

#define Z_DISTANCE_MIN 0.5f
#define Z_DISTANCE_MAX 1.5f

#define KINECT_ANGLE_INIT -30
#define KINECT_ANGLE_STEP 10

#define MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE 100
#define MIN_DIST_TO_GRASP 1.0

#define OPENNI 0

tf::TransformListener *listenerKinectRGBToKinect;
bool setup_tf = true;
ros::ServiceClient *dynamicReconfigureClientKinectTilt;

std::string nodeName = "objectCandidateExtraction3D_node";
std::string toFrame = std::string("/base_link");
std::string kinectTopicToSubscribe = std::string("/cam3d/rgb/points");
//std::string kinectTopicToSubscribe = std::string("/kinect/rgb/points2");

std::vector<sensor_msgs::PointCloud2> finalClusteredObjectsMsg;
std::vector<geometry_msgs::Point> finalClusteredObjectsCenroidsMsg;
int finalBestObjectsCentroidMsg;

#define KINECT_MAX_TILT (30)
#define KINECT_MIN_TILT (-30)

void setKinectTilt(ros::ServiceClient *service_client, float tiltDegree)
{
	double parameter;

	dynamic_reconfigure::Reconfigure reconfigure;
	dynamic_reconfigure::Config updated_tilt_config;

	if (tiltDegree > KINECT_MAX_TILT)
		tiltDegree = KINECT_MAX_TILT;

	if (tiltDegree < KINECT_MIN_TILT)
		tiltDegree = KINECT_MIN_TILT;

	updated_tilt_config.doubles.resize(1);

	updated_tilt_config.doubles[0].name = "tilt";
	updated_tilt_config.doubles[0].value = tiltDegree;

	reconfigure.request.config = updated_tilt_config;

	if (!service_client->call(reconfigure))
		ROS_ERROR("Unable to set Kinect tilt value!");

	if (ros::param::has("/kinect_driver/tilt"))
	{
		while (1) // wait for kinect
		{

			ros::param::get("/kinect_driver/tilt", parameter);
			if (parameter == tiltDegree)
			{
				ROS_INFO("kinect_tilt set to %f ",tiltDegree);
				break;
			}
			ROS_INFO("kinect_tilt set to %f ",tiltDegree);
		}
	}
	else
	{
		ROS_WARN("No PARAM /kinect_driver/tilt found!");
	}
}

int findBestObject(std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects)
{

	int bestCandidateIdx = 0;
	if (clusteredObjectsCentroidsMsg.size() > 0 && clusteredObjects.size() > 0)
	{
		std::vector<int> candidatesIdx;
		for (unsigned int i = 0; i < clusteredObjects.size(); ++i)
		{
			if (clusteredObjects[i].points.size() > MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE)
			{
				candidatesIdx.push_back(i);
			}
		}

		if (candidatesIdx.size() > 0)
		{
			double minDist = 99999;
			bestCandidateIdx = candidatesIdx[0];
			for (unsigned int i = 0; i < candidatesIdx.size(); ++i)
			{
				double dist = sqrt(
						pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].x, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].y, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].z, 2));
				if (dist < minDist)
				{
					bestCandidateIdx = candidatesIdx[i];
					minDist = dist;
				}

			}
		}
	}
	return bestCandidateIdx;
}

int findBestObject2(std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects)
{

	int bestCandidateIdx = 0;
	if (clusteredObjectsCentroidsMsg.size() > 0 && clusteredObjects.size() > 0)
	{
		std::vector<int> candidatesIdx;
		for (unsigned int i = 0; i < clusteredObjects.size(); ++i)
		{
			if (clusteredObjects[i].points.size() > MIN_POINTS_FOR_BEST_OBJECT_CANDIDATE)
			{
				candidatesIdx.push_back(i);
			}
		}

		if (candidatesIdx.size() > 0)
		{
			double minDistToGrasp = MIN_DIST_TO_GRASP;
			bestCandidateIdx = candidatesIdx[0];
			unsigned int maxPointCloudSize = 0;
			for (unsigned int i = 0; i < candidatesIdx.size(); ++i)
			{
				double dist = sqrt(
						pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].x, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].y, 2) + pow(clusteredObjectsCentroidsMsg[candidatesIdx[i]].z, 2));
				if (dist < minDistToGrasp && clusteredObjects[candidatesIdx[i]].points.size() > maxPointCloudSize)
				{
					bestCandidateIdx = candidatesIdx[i];
					maxPointCloudSize = clusteredObjects[candidatesIdx[i]].points.size();
				}

			}
		}
	}
	return bestCandidateIdx;
}

//TEST OBJECT DETECTION
void objectCandidateExtractionCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	ROS_INFO("[%s/objectCandidateExtractionCallback] started...", nodeName.c_str());
	std::vector<sensor_msgs::PointCloud2> clusteredObjectsMsg;
	std::vector<geometry_msgs::Point> clusteredObjectsCentroidsMsg;
	sensor_msgs::PointCloud2 pointsCloudMsg;
	int bestCandidateIdx = 0;
 
  if(point_cloud_msg->width <=0 && point_cloud_msg->height <=0)
  {
    ROS_INFO("[%s] pointCloud Msg empty",nodeName.c_str());
    return ;
  }


	ros::Time start, finish;
	start = ros::Time::now();

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredPlanes;
	pcl::PointCloud<pcl::PointXYZRGBNormal> planar_point_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_RGB;
	pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_RGB2;
	pcl::PointCloud<pcl::PointXYZRGB> augmentPointCloudCopy;
	std::vector < structPlanarSurface > hierarchyPlanes;
	sensor_msgs::PointCloud2 pointCloud2MsgTransformed;

	std::string fromFrame = std::string(point_cloud_msg->header.frame_id);
  ROS_INFO("[%s] pointCloud tf transform... ",nodeName.c_str());
	if(!toolBox.transformPointCloud(*listenerKinectRGBToKinect, fromFrame, toFrame, *point_cloud_msg, pointCloud2MsgTransformed))
	{
		 ROS_INFO("[%s] pointCloud tf transform...failed",nodeName.c_str());
		 counter = 0;
		 return;
	}
  ROS_INFO("[%s] pointCloud tf transform...done",nodeName.c_str());
	pcl::fromROSMsg(pointCloud2MsgTransformed, point_cloud);

  if(point_cloud.points.size() <=0 )
  {
    ROS_INFO("[%s] pointCloud empty ",nodeName.c_str());
    return;
  }

	if (counter == 0)
	{
		augmentPointCloud = point_cloud;
		augmentPointCloud.header.frame_id = point_cloud.header.frame_id;
		//augmentPointCloud.header.stamp=ros::Time::now();
		ROS_INFO("[%s] augment pointCloud = %d points",nodeName.c_str(),(int)augmentPointCloud.size());
		//		kinectTiltDegree += KINECT_ANGLE_STEP;
	}
	else if (max_augment == (counter - 1))
	{

		//	kinectTiltDegree = KINECT_ANGLE_INIT;
		//setKinectTilt(dynamicReconfigureClientKinectTilt,kinectTiltDegree);
		ROS_INFO("[%s] extract object candidates",nodeName.c_str());

		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, X_DISTANCE_MIN, X_DISTANCE_MAX, "x");
		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, Y_DISTANCE_MIN, Y_DISTANCE_MAX, "y");
		augmentPointCloud = toolBox.filterDistance(augmentPointCloud, Z_DISTANCE_MIN, Z_DISTANCE_MAX, "z");

		//toolBox.subsampling(augmentPointCloud, 0.004); //0.01
		toolBox.subsampling(augmentPointCloud, 0.008); //0.01

		augmentPointCloudCopy = augmentPointCloud;
		
		if (augmentPointCloudCopy.points.size() == 0) {
			ROS_DEBUG("[%s] point cloud empty after filtering",nodeName.c_str());
			counter = 0;
			return;
		}

    ROS_INFO("Point cloud size: %i", augmentPointCloudCopy.points.size());

		objectCandidateExtractor->extractObjectCandidates(augmentPointCloudCopy, planar_point_cloud, hierarchyPlanes);

		if (DO_CANDIDATION || SHOW_OBJECTS || SHOW_PLANES)
		{
			for (unsigned int iterPlanes = 0; iterPlanes < hierarchyPlanes.size(); iterPlanes++)
			{
				for (unsigned int iterObject = 0; iterObject < hierarchyPlanes[iterPlanes].clusteredObjects.size(); iterObject++)
				{
					if (DO_CANDIDATION || SHOW_OBJECTS)
					{
						clusteredObjects.push_back(hierarchyPlanes[iterPlanes].clusteredObjects[iterObject]);
					}
					if (DO_CANDIDATION)
					{
						pcl::toROSMsg(clusteredObjects.back(), pointsCloudMsg);
						pointsCloudMsg.header.frame_id = pointsCloudMsg.header.frame_id;
						pointsCloudMsg.header.stamp = ros::Time::now();
						clusteredObjectsMsg.push_back(pointsCloudMsg);

						pcl::PointXYZ centroid = toolBox.pointCloudCentroid(clusteredObjects.back());
						geometry_msgs::Point pointMsg;
						pointMsg.x = centroid.x;
						pointMsg.y = centroid.y;
						pointMsg.z = centroid.z;
						clusteredObjectsCentroidsMsg.push_back(pointMsg);
					}

				}
				if (SHOW_PLANES)
					clusteredPlanes.push_back(hierarchyPlanes[iterPlanes].pointCloud);
			}
		}

		if (DO_CANDIDATION)
		{
			bestCandidateIdx = findBestObject(clusteredObjectsCentroidsMsg, clusteredObjects);
			mutexExtractedObjects.lock();
			finalClusteredObjectsMsg = clusteredObjectsMsg;
			finalClusteredObjectsCenroidsMsg = clusteredObjectsCentroidsMsg;
			finalBestObjectsCentroidMsg = bestCandidateIdx;

			brsu_msgs::ObjectCandidateList3D pointcloud_3d_msg;
			pointcloud_3d_msg.pointClouds = clusteredObjectsMsg;

			mutexExtractedObjects.unlock();

			pmd_pub4.publish(pointcloud_3d_msg);
		}

		if (SHOW_OBJECTS)
		{
			toolBox.markClusteredPointCloud(clusteredObjects, point_cloud_RGB);
			pcl::toROSMsg(point_cloud_RGB, pointsCloudMsg);
			pointsCloudMsg.header.frame_id = point_cloud.header.frame_id;
			pointsCloudMsg.header.stamp = ros::Time::now();
			pmd_pub1.publish(pointsCloudMsg);
		}

		if (SHOW_PLANES)
		{
			toolBox.markClusteredPointCloud(clusteredPlanes, point_cloud_RGB2);
			pcl::toROSMsg(point_cloud_RGB2, pointsCloudMsg);
			pointsCloudMsg.header.frame_id = point_cloud.header.frame_id;
			pointsCloudMsg.header.stamp = ros::Time::now();
			pmd_pub2.publish(pointsCloudMsg);

		}
		counter = -1;

	}
	else
	{
		augmentPointCloud.header.frame_id = point_cloud.header.frame_id;
		augmentPointCloud += point_cloud;
		ROS_INFO("[%s] augment pointCloud = %d points",nodeName.c_str(),(int)augmentPointCloud.size());
	}

	++counter;
	finish = ros::Time::now();

	ROS_INFO("[%s/objectCandidateExtractionCallback] Execution time =  %lfsec",nodeName.c_str(), (finish.toSec() - start.toSec() ));
}

/*Service getKinect_objectCandidateList*/
bool getKinectObjectCandidates3D(brsu_srvs::GetObjectCandidateList3D::Request &req, brsu_srvs::GetObjectCandidateList3D::Response &res)
{

	mutexExtractedObjects.lock();
	res.pointClouds = finalClusteredObjectsMsg;
	res.pointCloudCentroids = finalClusteredObjectsCenroidsMsg;
	res.bestPointCloudCentroidIndex = finalBestObjectsCentroidMsg;
	finalClusteredObjectsMsg.clear();
	finalClusteredObjectsCenroidsMsg.clear();
	mutexExtractedObjects.unlock();

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, nodeName);

	ros::NodeHandle n;
	ros::ServiceServer srvGetKinectObjectCandidates;

	listenerKinectRGBToKinect = new tf::TransformListener();
	objectCandidateExtractor = new CObjectCandidateExtraction(nodeName, SPHERICAL_DISTANCE);

	pmd_pub1 = n.advertise<sensor_msgs::PointCloud2> ("object_perception/kinect_objects", 1);
	pmd_pub2 = n.advertise<sensor_msgs::PointCloud2> ("object_perception/kinect_objects_surface", 1);
	pmd_pub3 = n.advertise<sensor_msgs::PointCloud2> ("object_perception/kinect_input_point_cloud", 1);
	pmd_pub4 = n.advertise<brsu_msgs::ObjectCandidateList3D> ("object_perception/ObjectCandidateList3D", 1);

	srvGetKinectObjectCandidates = n.advertiseService("GetObjectCandidates3D", getKinectObjectCandidates3D);

	ros::Subscriber sub = n.subscribe(kinectTopicToSubscribe, 1, objectCandidateExtractionCallback);

	ros::spin();

	return 0;
}

