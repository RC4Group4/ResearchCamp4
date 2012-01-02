/*
 *  objectCandidateExtraction2D_node
 *  Requirement: objectCandidateExtraction3D_node
 *
 *  Created on: 21.12.2010
 *      Author: Christian Mueller
 */
#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <istream>
#include <math.h>
#include "sensor_msgs/Image.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <stdio.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "brsu_msgs/ObjectCandidateList3D.h"
#include "brsu_msgs/ObjectCandidateList2D.h"
#include "brsu_srvs/GetObjectCandidateList2D.h"

#include <CToolBoxROS.h>

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

/*FRAMES DEFINITIONS*/
#define WORLD "/base_link"
#define KINECT_RGB "/openni_rgb_frame"

pthread_mutex_t projectedPointsMutex;
boost::mutex mutexExtractedRGBImages;
boost::mutex mutexExtractedRangeImages;
std::vector<sensor_msgs::Image> finalImageRGBMgs;
std::vector<sensor_msgs::Image> finalImageRangeMgs;

/*single object*/
std::vector<cv::Point2f> projectedPoints;

/*all received objects*/
std::vector<std::vector<cv::Point2f> > projectedPointsObjects;

/*projection parameters*/
cv::Mat cameraMatrix;
cv::Mat rotationVector;
cv::Mat rotationMatrix;
cv::Mat distortion;
cv::Mat translationVector;

ros::ServiceClient client;
ros::Publisher pubImages;
ros::Publisher pubImages2; //rgb images list
ros::Publisher pubImages3; //distance images list
tf::TransformListener *listenerWorldToKinectRGB;

std::string nodeName = "objectCandidateExtraction2D_node";
bool setup_tf = true;
int imageNumber = 0;

CvScalar cvGetColor(IplImage* source, int x, int y) {
	int step = source->widthStep / sizeof(uchar);
	int channels = source->nChannels;
	CvScalar color;

	uchar* data = (uchar *) source->imageData;

	color.val[0] = data[x * step + y * channels + 0];
	color.val[1] = data[x * step + y * channels + 1];
	color.val[2] = data[x * step + y * channels + 2];

	return color;
}

void cvSetColor(IplImage* target, int x, int y, CvScalar color) {
	int step = target->widthStep / sizeof(uchar);
	int channels = target->nChannels;

	uchar* data = (uchar *) target->imageData;

	data[x * step + y * channels + 0] = color.val[0];
	data[x * step + y * channels + 1] = color.val[1];
	data[x * step + y * channels + 2] = color.val[2];
}

void computeImagesHistogramEqualization(cv::Mat &imageGray, cv::Mat &norm,
		cv::Mat &cannyImage) {

	int apertureSize = 7;
	//cv::namedWindow("Original");
	//cv::namedWindow("norm. Original");
	//cv::namedWindow("lap. Original");
	//cv::namedWindow("canny. Original");

	// Perform histogram equalization
	//	cv::equalizeHist(imageGray, norm);
	norm = imageGray;

	cv::Mat laplacianImage = cv::Mat(norm.size(), CV_8UC1);

	cv::Laplacian(norm, laplacianImage, CV_8UC1);

	//25 40
	cv::Canny(norm, cannyImage, 25 * apertureSize * apertureSize, 40
			* apertureSize * apertureSize, apertureSize); //3 less

	//cv::imshow("Original", imageGray);
	//cv::imshow("norm. Original", norm);
	//cv::imshow("lap. Original", laplacianImage);
	//cv::imshow("canny. Original", cannyImage);
}

std::vector<float> rangeImageObject(pcl::PointCloud<pcl::PointXYZ> point_cloud) {

	CToolBoxROS toolBox;
	unsigned int maxValue = 255;//255; //GrayScale


	//cv::Mat rangeImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);

	unsigned int numberPoints = point_cloud.points.size();
	std::vector<float> point_cloude_distances;

	point_cloude_distances.resize(numberPoints);

	float minDistance = std::numeric_limits<double>::infinity();
	float maxDistance = std::numeric_limits<double>::epsilon();

	pcl::PointXYZ minPoint;
	pcl::PointXYZ maxPoint;

	/* */
	sensor_msgs::PointCloud2 pcl2Msg, pcl2MsgTrans;
	pcl::toROSMsg(point_cloud, pcl2Msg); //th
	std::string worldFrameID = WORLD;
	std::string kinectNormFrameID = KINECT_RGB;
	toolBox.transformPointCloud(*listenerWorldToKinectRGB, worldFrameID,
			kinectNormFrameID, pcl2Msg, pcl2MsgTrans);
	pcl::fromROSMsg(pcl2MsgTrans, point_cloud); //th

	//here we expect points in kinect frame!!! then Z axis is important ->  if in world then x is important!!
	/*	double dTilt = 0;
	 if (ros::param::has("/kinect_driver/tilt")) {
	 ros::param::get("/kinect_driver/tilt", dTilt); //degree
	 //dTilt = dTilt / 180 * M_PI; // rad
	 //	std::cout<<dTilt<<" ";
	 }
	 //	tf::Transform transformRGB;
	 //	transformRGB.setOrigin(tf::Vector3(0, 0, 0)); //  -0.03
	 //	transformRGB.setRotation(tf::Quaternion(0.0, 0.0, -dTilt));

	 for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
	 tf::Vector3 point(point_cloud.points[iterPoints].x,
	 point_cloud.points[iterPoints].y,
	 point_cloud.points[iterPoints].z);

	 point = point.rotate(tf::Vector3(0, 0, 1), -dTilt);

	 point_cloud.points[iterPoints].x = point.getX();
	 point_cloud.points[iterPoints].y = point.getY();
	 point_cloud.points[iterPoints].z = point.getZ();
	 }
	 */

	//This is the perspective view (from kinect) //means always the closest point to kinect is chosen
	/*	for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
	 double distance = sqrt(pow(fabs(point_cloud.points[iterPoints].x), 2)
	 + pow(fabs(point_cloud.points[iterPoints].y), 2) + pow(fabs(
	 point_cloud.points[iterPoints].z), 2));

	 if (distance < minDistance) {
	 minDistance = distance;
	 minPoint = point_cloud.points[iterPoints];
	 }
	 if (distance > maxDistance) {
	 maxDistance = distance;
	 maxPoint = point_cloud.points[iterPoints];
	 }
	 }

	 //This is on axis
	 // minDistance = toolBox.minValuePointCloud3d(point_cloud, 2,minPoint);
	 // maxDistance = toolBox.maxValuePointCloud3d(point_cloud, 2,maxPoint);

	 //transform points to relative distance poincloud
	 for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
	 point_cloud.points[iterPoints].x -= minPoint.x;
	 point_cloud.points[iterPoints].y -= minPoint.y;
	 point_cloud.points[iterPoints].z -= minPoint.z;
	 }
	 */

	//we look just on the X axis so the distance not the perspective distance, the paralle one the points
	for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
		double distance = sqrt(pow(fabs(point_cloud.points[iterPoints].z), 2)
		//	+ pow(fabs(point_cloud.points[iterPoints].y), 2) + pow(fabs(
				//	point_cloud.points[iterPoints].z), 2)
				);

		if (distance < minDistance) {
			minDistance = distance;
			minPoint = point_cloud.points[iterPoints];
		}
		if (distance > maxDistance) {
			maxDistance = distance;
			maxPoint = point_cloud.points[iterPoints];
		}
	}

	//This is on axis
	// minDistance = toolBox.minValuePointCloud3d(point_cloud, 2,minPoint);
	// maxDistance = toolBox.maxValuePointCloud3d(point_cloud, 2,maxPoint);

	//transform points to relative distance poincloud
	//Again, the intensity value is just based on the parallel front view so X axis y and z not required
	for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
		point_cloud.points[iterPoints].x = 0; //minPoint.x;//minPoint.x;
		point_cloud.points[iterPoints].y = 0;//minPoint.y;
		point_cloud.points[iterPoints].z -= minPoint.z;//
	}

	for (unsigned int iterPoints = 0; iterPoints < numberPoints; iterPoints++) {
		double distance = sqrt(pow(fabs(point_cloud.points[iterPoints].x), 2)
				+ pow(fabs(point_cloud.points[iterPoints].y), 2) + pow(fabs(
				point_cloud.points[iterPoints].z), 2));
		//	std::cout << "distance <<" << distance << "\n";
		if (distance <= maxDistance) {
			//distance = (maxValue - (sqrt((distance / maxDistance)) * maxValue));
			distance = (maxValue
					- (sqrt(((distance) / maxDistance)) * maxValue));
			//	distance = ((exp((-1)*sqrt((distance / maxDistance)/)) * maxValue));
			//	distance = (maxValue - ((distance / maxDistance) * maxValue));
			point_cloude_distances[iterPoints] = distance;
		} else {
			point_cloude_distances[iterPoints] = 0;
		}
	}
	return point_cloude_distances;
}

cv::Mat rangeImageProjection(
		std::vector<cv::Point2f> vecPoint2f_projectedClusteredObjects,
		std::vector<float> rangeDistances) {
	int pointSize = 1;// 3;
	cv::Mat cv_imageFinal;
	cv_imageFinal = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
	cv_imageFinal.zeros(cv_imageFinal.size(), CV_8UC1);

	cv::Mat cv_imageTotal;
	cv_imageTotal = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
	cv_imageTotal.zeros(cv_imageTotal.size(), CV_8UC1);

	cv::Mat cv_image, cv_imageDown;

	cv_image = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_64FC1);
	cv_image.zeros(cv_image.size(), CV_64FC1);

	cv_imageDown = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
	cv_imageDown.zeros(cv_imageDown.size(), CV_8UC1);

	cv::Mat cv_imageFinalTemp;
	cv_imageFinalTemp = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
	cv_imageFinalTemp.zeros(cv_imageFinalTemp.size(), CV_8UC1);

	//	cvSetZero(cv_image);
	for (unsigned int iterPoint = 0; iterPoint
			< vecPoint2f_projectedClusteredObjects.size(); iterPoint++) {
		//cvCircle(cv_image,cvPoint(	(unsigned int) vecPoint2f_projectedClusteredObjects[iterObjects][iterPoint].x,
		//		(unsigned int) vecPoint2f_projectedClusteredObjects[iterObjects][iterPoint].y),1,cvScalar(
		//			vecFloat_distanceClusteredObjects[iterObjects][iterPoint]),5);

		cv::rectangle(
				cv_image,
				cv::Point(
						(unsigned int) vecPoint2f_projectedClusteredObjects[iterPoint].x
								- pointSize,
						(unsigned int) vecPoint2f_projectedClusteredObjects[iterPoint].y
								+ pointSize),
				cv::Point(
						(unsigned int) vecPoint2f_projectedClusteredObjects[iterPoint].x
								+ pointSize,
						(unsigned int) vecPoint2f_projectedClusteredObjects[iterPoint].y
								- pointSize), cv::Scalar(
						rangeDistances[iterPoint]), CV_FILLED);
	}

	cv_image.convertTo(cv_image, CV_8UC1);

	//cv::pyrDown(cv_image, cv_imageDown);
	cv_imageDown = cv_image;

	cv::medianBlur(cv_imageDown, cv_imageDown, 7); //now preserve real edgees
	//	cv::GaussianBlur(cv_imageDown, cv_imageDown, cv::Size(5, 5), 2, 2);

	cv::Mat normalizedImage = cv::Mat(cv_imageDown.size(), CV_8UC1);
	cv::Mat cannyImage = cv::Mat(normalizedImage.size(), CV_8UC1);

	computeImagesHistogramEqualization(cv_imageDown, normalizedImage,
			cannyImage);
	//With canny
	//cv::add(normalizedImage, cannyImage, cv_imageFinalTemp);
	//W/O canny
	cv_imageFinalTemp = normalizedImage;

	//cv::add(normalizedImage, cannyImage, cv_imageFinal);
	//cv::imshow("Range Image", cv_imageFinal);


	//cv_imageFinal = cv_imageDown;

	//cv::GaussianBlur(cv_imageFinal, cv_imageFinal, cv::Size(3, 3), 2, 2);
	//cv::waitKey(200);

	return cv_imageFinalTemp;
}

std::vector<cv::Point2f> pointCloudProjection(
		pcl::PointCloud<pcl::PointXYZ> pointCloud) {
	ROS_DEBUG("[%s]/pointCloudProjection...",nodeName.c_str());

	std::vector<cv::Point2f> projectedPoints;

	std::vector<cv::Point3f> modelPoints;
	cv::Mat matModelPoints;

	modelPoints.resize(pointCloud.points.size());

	projectedPoints.resize(pointCloud.points.size());

	for (unsigned int p = 0; p < pointCloud.points.size(); p++) {

		modelPoints.at(p).x = pointCloud.points[p].x;
		modelPoints.at(p).y = pointCloud.points[p].y;
		modelPoints.at(p).z = pointCloud.points[p].z;
	}

	matModelPoints = cv::Mat(modelPoints);

	cv::projectPoints(matModelPoints, //3d points
			rotationVector, //rotation vector
			translationVector, //translation vector
			cameraMatrix, //intrinsic matrix
			distortion, //distortion coeffs
			projectedPoints); //2d projected points

	return projectedPoints;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cvNamedWindow("Image window");
	cvNamedWindow("Image window projected");
	cvNamedWindow("Image window mask");
	ROS_INFO("[%s]imageCallback...",nodeName.c_str());
	sensor_msgs::CvBridge bridge;

	IplImage *cv_image = NULL;
	try {
		cv_image = bridge.imgMsgToCv(msg, "bgr8");
		cvShowImage("Image window", cv_image);
		cvWaitKey(10);
	}

	catch (sensor_msgs::CvBridgeException error) {
		ROS_ERROR("error");
	}

	pthread_mutex_lock(&projectedPointsMutex);
	ROS_DEBUG("[%s]imageCallback...locked",nodeName.c_str());

	//imageContainer = cv_image;

	IplImage *projectedImage = cvCreateImage(cvGetSize(cv_image),
			cv_image->depth, cv_image->nChannels);
	IplImage *maskImage = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);

	//IplImage *mask = cvCreateImage(cvGetSize(cv_image),1,1);

	cvZero(projectedImage);
	cvZero(maskImage);

	cv::Mat matMask = cv::Mat(cv_image->height, cv_image->width, CV_64FC1);
	std::vector<cv::Point2f> hullPoints;
	cv::Mat matProjectedPoints;

	projectedImage->origin = cv_image->origin;
	maskImage->origin = cv_image->origin;
	if (projectedPoints.size() > 0) {
		matProjectedPoints = cv::Mat(projectedPoints);

		ROS_INFO("[%s]imageCallback...projecting",nodeName.c_str());
		int x = 0;
		int y = 0;
		for (int i = 0; i < projectedPoints.size(); i++) {
			x = projectedPoints.at(i).x;
			y = projectedPoints.at(i).y;
			cvCircle(maskImage, cvPoint(x, y), 1, cvScalar(255, 255, 255), 5);
			//----//---------
			//	color = cvGetColor(cv_image,y,x);
			//	cvSetColor(projectedImage,y,x,color);
			//---------------
		}

		//	 cv::convexHull(matProjectedPoints,hullPoints,true);
		//	 cv::Point *pointHullPoints;
		// pointHullPoints = (cv::Point*)malloc(hullPoints.size()*sizeof(cv::Point));

		//		 cv::fillConvexPoly(matMask, pointHullPoints, hullPoints.size(), cvScalar(255,255,255));
	}

	ROS_DEBUG("[%s]imageCallback...unlocked",nodeName.c_str());
	pthread_mutex_unlock(&projectedPointsMutex);

	if (projectedImage != NULL) {

		if (projectedPoints.size() > 0 && maskImage != NULL) {
			cvCopy(cv_image, projectedImage, maskImage);

			cvShowImage("Image window mask", maskImage);
			cvShowImage("Image window projected", projectedImage);
			imageNumber++;
			char filename[50];
			sprintf(filename, "trainImage%d.png", imageNumber);
			cvSaveImage(filename, projectedImage);
			cvWaitKey(10);

			sensor_msgs::ImagePtr imageMsg = sensor_msgs::CvBridge::cvToImgMsg(
					projectedImage, "bgr8");//mono8

			imageMsg->header.frame_id = WORLD; //"/kinect_rgb";//"world";
			imageMsg->header.stamp = ros::Time::now();

			pubImages.publish(imageMsg);

			projectedPoints.clear();
		}

		cvReleaseImage(&projectedImage);
		cvReleaseImage(&maskImage);
	}
}

void pointCloudCallback(const sensor_msgs::PointCloud2Ptr& pointCloudMsg) {

	ROS_INFO("[%s]pointCloudCallback...locked",nodeName.c_str());
	tf::StampedTransform transform;

	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	sensor_msgs::PointCloud pointCloudMsgTransformed;
	sensor_msgs::PointCloud2 pointCloud2MsgTransformed;

	sensor_msgs::PointCloud pointCloudMsgConvert;

	sensor_msgs::convertPointCloud2ToPointCloud(*pointCloudMsg,
			pointCloudMsgConvert);

	while (setup_tf) {
		setup_tf = false;
		try {
			listenerWorldToKinectRGB->waitForTransform(KINECT_RGB, WORLD,
					ros::Time(0), ros::Duration(1.0));
			listenerWorldToKinectRGB->transformPointCloud(std::string(
					"/kinect_rgb"), pointCloudMsgConvert,
					pointCloudMsgTransformed);
		} catch (tf::TransformException ex) {
			setup_tf = true;
		}
	}
	setup_tf = true;

	sensor_msgs::convertPointCloudToPointCloud2(pointCloudMsgTransformed,
			pointCloud2MsgTransformed);

	pcl::fromROSMsg(pointCloud2MsgTransformed, pointCloud);

	//	sensor_msgs::PointCloud2 pointsCloud2MSG;
	//	pcl::toROSMsg(pointCloud,pointsCloud2MSG);
	//	pointsCloud2MSG.header.frame_id= "/kinect_rgb";
	//	pointsCloud2MSG.header.stamp=ros::Time::now();
	//	pubImages2PCL.publish(pointsCloud2MSG);

	std::vector<cv::Point3f> modelPoints;
	cv::Mat matModelPoints;

	modelPoints.resize(pointCloud.points.size());

	pthread_mutex_lock(&projectedPointsMutex);

	if (projectedPoints.size() > 0)
		projectedPoints.clear();

	projectedPoints.resize(pointCloud.points.size());

	for (unsigned int p = 0; p < pointCloud.points.size(); p++) {

		modelPoints.at(p).x = pointCloud.points[p].x;
		modelPoints.at(p).y = pointCloud.points[p].y;
		modelPoints.at(p).z = pointCloud.points[p].z;
	}

	matModelPoints = cv::Mat(modelPoints);

	cv::projectPoints(matModelPoints, //3d points
			rotationVector, //rotation vector
			translationVector, //translation vector
			cameraMatrix, //intrinsic matrix
			distortion, //distortion coeffs
			projectedPoints); //2d projected points


	ROS_INFO("[%s]imageCallback...unlocked projectedPoints",nodeName.c_str());
	pthread_mutex_unlock(&projectedPointsMutex);
}
//#######################################################################
void imageVectorCallback(const sensor_msgs::ImageConstPtr& msg) {

	//cvNamedWindow("Image window");
	//cvNamedWindow("Image window projected");
	//cvNamedWindow("Image window mask");
	int pointSize = 7;// 3;
	ROS_INFO("[%s]imageCallback...",nodeName.c_str());
	std::vector<sensor_msgs::Image> vecImageMgs;
	brsu_msgs::ObjectCandidateList2D
			kinect_objectCandidateList2DMsg;
	sensor_msgs::CvBridge bridge;

	IplImage *cv_image = NULL;
	try {
		cv_image = bridge.imgMsgToCv(msg, "bgr8");
		cvShowImage("Image window", cv_image);
	}

	catch (sensor_msgs::CvBridgeException error) {
		ROS_ERROR("error");
	}

	pthread_mutex_lock(&projectedPointsMutex);
	ROS_DEBUG("[%s]imageCallback...locked",nodeName.c_str());

	if (projectedPointsObjects.size() > 0) {
		for (unsigned int iterObject = 0; iterObject
				< projectedPointsObjects.size(); ++iterObject) {
			ROS_INFO("[%s]imageCallback...projecting",nodeName.c_str());

			IplImage *projectedImage = cvCreateImage(cvGetSize(cv_image),
					cv_image->depth, cv_image->nChannels);
			IplImage *maskImage = cvCreateImage(cvGetSize(cv_image),
					IPL_DEPTH_8U, 1);
			cvZero(maskImage);
			cvZero(projectedImage);
			maskImage->origin = cv_image->origin;
			projectedImage->origin = cv_image->origin;

			int x = 0;
			int y = 0;
			for (int i = 0; i < projectedPointsObjects[iterObject].size(); i++) {
				x = projectedPointsObjects[iterObject][i].x;
				y = projectedPointsObjects[iterObject][i].y;
				cvCircle(maskImage, cvPoint(x, y), 1, cvScalar(255, 255, 255),
						pointSize);
				/*cvRectangle(
				 maskImage,
				 cvPoint(
				 (unsigned int) projectedPointsObjects[iterObject][i].x
				 - pointSize,
				 (unsigned int) projectedPointsObjects[iterObject][i].y
				 + pointSize),
				 cvPoint(
				 (unsigned int) projectedPointsObjects[iterObject][i].x
				 + pointSize,
				 (unsigned int) projectedPointsObjects[iterObject][i].y
				 - pointSize),
				 cvScalar(CvScalar(255,255,255)));*/
			}

			cvCopy(cv_image, projectedImage, maskImage);

			sensor_msgs::ImagePtr imageMsg = sensor_msgs::CvBridge::cvToImgMsg(
					projectedImage, "bgr8");

			//		cvShowImage("Image window projected", maskImage);
			//		cvWaitKey(10);
			vecImageMgs.push_back(*imageMsg);
			cvReleaseImage(&projectedImage);
			cvReleaseImage(&maskImage);
		}

	}

	ROS_DEBUG("[%s]imageCallback...unlocked",nodeName.c_str());

	if (vecImageMgs.size() > 0) {
		mutexExtractedRGBImages.lock();
		finalImageRGBMgs = vecImageMgs;
		mutexExtractedRGBImages.unlock();

		kinect_objectCandidateList2DMsg.images = vecImageMgs;
		pubImages2.publish(kinect_objectCandidateList2DMsg);
		//pubImages2.publish(vecImageMgs);
	}
	projectedPointsObjects.clear();

	pthread_mutex_unlock(&projectedPointsMutex);
}

void pointCloudVectorCallback(
		const brsu_msgs::ObjectCandidateList3DPtr& pointCloudVectorMsg) {
	pthread_mutex_lock(&projectedPointsMutex);
	ROS_INFO("[%s/pointCloudVectorCallback]...",nodeName.c_str());

	std::string toFrame = std::string(KINECT_RGB);
	std::string fromFrame = std::string(WORLD);
	bool setup_tf = true;

	std::vector<pcl::PointCloud<pcl::PointXYZ> > vecPcl2_clusteredObjects;
	std::vector<sensor_msgs::PointCloud2> vecPcl2Msg_clusteredObjects;
	std::vector < std::vector<float> > vecFloat_distanceClusteredObjects;

	sensor_msgs::PointCloud pclMsgTransformed;
	sensor_msgs::PointCloud pclMsgSource;
	sensor_msgs::PointCloud2 pcl2MsgTransformed;

	vecPcl2Msg_clusteredObjects = pointCloudVectorMsg->pointClouds;
	vecPcl2_clusteredObjects.resize(vecPcl2Msg_clusteredObjects.size());

	projectedPointsObjects.clear();
	projectedPointsObjects.resize(vecPcl2Msg_clusteredObjects.size());
	vecFloat_distanceClusteredObjects.resize(vecPcl2Msg_clusteredObjects.size());

	//Distances for the RANGE IMAGE are CREATED HERE!!!!!!!!!!!!!!!!!!!!!!!!
	// we do now intensity image to get the intesity value based on the worldsframe for each point!!!
	for (unsigned int iterObjects = 0; iterObjects
			< vecPcl2Msg_clusteredObjects.size(); iterObjects++) {

		pcl::fromROSMsg(vecPcl2Msg_clusteredObjects[iterObjects],
				vecPcl2_clusteredObjects[iterObjects]);

		vecFloat_distanceClusteredObjects[iterObjects] = rangeImageObject(
				vecPcl2_clusteredObjects[iterObjects]);
	}
	vecPcl2_clusteredObjects.clear();
	vecPcl2_clusteredObjects.resize(vecPcl2Msg_clusteredObjects.size());
	//_----------------------------------------

	//Now project to 2D via kinect frame!, however the points will have the parallel front view intenstity value, since world frame is assumed to be parallel to ground
	for (unsigned int iterPclMsg = 0; iterPclMsg
			< vecPcl2Msg_clusteredObjects.size(); iterPclMsg++) {
		//		toolBox.transformPointCloud(*listenerKinectRGBToKinect, fromFrame,
		//			toFrame, vecPcl2Msg_clusteredObjects[iterPclMsg], pcl2MsgTransformed);
		sensor_msgs::convertPointCloud2ToPointCloud(
				vecPcl2Msg_clusteredObjects[iterPclMsg], pclMsgSource);
		while (setup_tf) {
			setup_tf = false;
			try {
				listenerWorldToKinectRGB->waitForTransform(KINECT_RGB,
						WORLD, ros::Time(0), ros::Duration(1.0));

				listenerWorldToKinectRGB->transformPointCloud(std::string(
						KINECT_RGB), pclMsgSource, pclMsgTransformed);
			} catch (tf::TransformException ex) {
				setup_tf = true;
			}
		}
		setup_tf = true;
		sensor_msgs::convertPointCloudToPointCloud2(pclMsgTransformed,
				pcl2MsgTransformed);
		pcl::fromROSMsg(pcl2MsgTransformed,
				vecPcl2_clusteredObjects[iterPclMsg]);

		//	vecFloat_distanceClusteredObjects[iterPclMsg] = rangeImageObject(
		//				vecPcl2_clusteredObjects[iterPclMsg]);

		projectedPointsObjects[iterPclMsg] = pointCloudProjection(
				vecPcl2_clusteredObjects[iterPclMsg]);
	}

	//Now do range image vector of IPLIMAGE
	//##################################################################

	sensor_msgs::CvBridge bridge;
	std::vector<sensor_msgs::Image> imageRangeMgs;
	for (unsigned int iterObjects = 0; iterObjects
			< projectedPointsObjects.size(); iterObjects++) {
		cv::Mat rangeImageTempMat = rangeImageProjection(
				projectedPointsObjects[iterObjects],
				vecFloat_distanceClusteredObjects[iterObjects]);
		IplImage rangeImageTempIpl = rangeImageTempMat;
		//std::cout<<"sdfsdfs"<<rangeImageTempMat.channels();
		//std::cout.flush();
		//	cvConvert(rangeImageTemp, range);
		sensor_msgs::ImagePtr imageMsg = bridge.cvToImgMsg(&rangeImageTempIpl,
				"mono8");
		imageRangeMgs.push_back(*imageMsg);
	}

	if (imageRangeMgs.size() > 0) {

		mutexExtractedRangeImages.lock();
		finalImageRangeMgs = imageRangeMgs;
		mutexExtractedRangeImages.unlock();

		brsu_msgs::ObjectCandidateList2D image_msg;
		image_msg.images = imageRangeMgs;
		pubImages3.publish(image_msg);
	}
	pthread_mutex_unlock(&projectedPointsMutex);
}

//FOR TEST
void objectCandidateCallback1(
		const brsu_msgs::ObjectCandidateList2DPtr& pointCloudVectorMsg) {
	sensor_msgs::CvBridge bridge;
	std::vector<sensor_msgs::Image> vecImages;
	vecImages = pointCloudVectorMsg->images;
	cvNamedWindow("RGB");

	for (unsigned int i = 0; i < vecImages.size(); i++) {
		IplImage *cv_imageTest = NULL;
		try {
			bridge.fromImage(vecImages[i]);
			cv_imageTest = bridge.toIpl();
			cvShowImage("RGB", cv_imageTest);
			cvWaitKey(200);
		} catch (sensor_msgs::CvBridgeException error) {
			ROS_ERROR("error");
		}
	};
}

//FOR TEST
void objectCandidateCallback2(
		const brsu_msgs::ObjectCandidateList2DPtr& pointCloudVectorMsg) {
	sensor_msgs::CvBridge bridge;
	std::vector<sensor_msgs::Image> vecImages;
	vecImages = pointCloudVectorMsg->images;
	cvNamedWindow("DistanceIntensity");

	for (unsigned int i = 0; i < vecImages.size(); i++) {
		IplImage *cv_imageTest = NULL;
		try {
			bridge.fromImage(vecImages[i]);
			cv_imageTest = bridge.toIpl();
			cvShowImage("DistanceIntensity", cv_imageTest);
			cvWaitKey(200);
		} catch (sensor_msgs::CvBridgeException error) {
			ROS_ERROR("error");
		}
	};
}

/*Service getKinect_objectCandidateList*/
bool getKinectObjectCandidates2D(
		brsu_srvs::GetObjectCandidateList2D::Request &req,
		brsu_srvs::GetObjectCandidateList2D::Response &res) {

	mutexExtractedRGBImages.lock();
	res.imagesRGB = finalImageRGBMgs;
	finalImageRGBMgs.clear();
	mutexExtractedRGBImages.unlock();

	mutexExtractedRangeImages.lock();
	res.imagesDistanceIntensity = finalImageRangeMgs;
	finalImageRangeMgs.clear();
	mutexExtractedRangeImages.unlock();

	return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, nodeName);

	ros::NodeHandle n;

	listenerWorldToKinectRGB = new tf::TransformListener();

	pthread_mutex_init(&projectedPointsMutex, NULL);

	//from calibration_rgb.yaml
	double dCamera[3][3] = { { 526.37013657, 0.00000000, 313.68782938 }, {
			0.00000000, 526.37013657, 259.01834898 }, { 0.00000000, 0.00000000,
			1.00000000 } };

	double dDistortion[5] = { 0.18126525, -0.39866885, 0.00000000, 0.00000000,
			0.00000000 };

	double dRotationVector[3] = { 0.0, 0.0, 0.0 };

	double dTranslations[3] = { 0.0, 0.0, 0.0 };

	rotationVector = cv::Mat(1, 3, CV_64F, dRotationVector);
	cameraMatrix = cv::Mat(3, 3, CV_64F, dCamera);
	distortion = cv::Mat(1, 5, CV_64F, dDistortion);
	translationVector = cv::Mat(1, 3, CV_64F, dTranslations);

	//Single object singe pointcloud
	//	ros::Subscriber subImages = n.subscribe("/kinect/rgb/image_rect_color", 1,
	//			imageCallback);
	//	ros::Subscriber subPointCloud = n.subscribe(
	//			"object_perception/kinect_objects", 1, pointCloudCallback);
	//pubImages = n.advertise<sensor_msgs::Image> (
	//		"object_perception/image_projected", 1);


	//Multiple objects vector of points clouds
	ros::Subscriber subImages2 = n.subscribe("/camera/rgb/image_color", 1,
			imageVectorCallback);


	ros::Subscriber subPointCloud2 = n.subscribe(
			"/object_perception/ObjectCandidateList3D", 1,
			pointCloudVectorCallback);

	//ros::Subscriber subPointCloud2 = n.subscribe(
	//		"object_perception/kinect_objectCandidateList3D", 1,
	//		pointCloudVectorCallback);

	pubImages2 = n.advertise<
			brsu_msgs::ObjectCandidateList2D> (
			"object_perception/kinect_objectCandidateRGBList2D", 1);

	pubImages3 = n.advertise<
			brsu_msgs::ObjectCandidateList2D> (
			"object_perception/kinect_objectCandidateDistanceIntensityList2D",
			1);

	ros::ServiceServer service = n.advertiseService("kinect_objectCandidate2D",
		//	brsu_srvs::GetObjectCandidates2D);
			getKinectObjectCandidates2D);
	//for test purposes to check whether extraction is
	ros::Subscriber subPointCloud2TEST = n.subscribe(
			"object_perception/kinect_objectCandidateRGBList2D", 1,
			objectCandidateCallback1);

	ros::Subscriber subPointCloud2TEST3 = n.subscribe(
			"object_perception/kinect_objectCandidateDistanceIntensityList2D",
			1, objectCandidateCallback2);

	/*
	client = n.serviceClient<
				object_categorization_node::getKinect_objectCandidateList2D> (
				"kinect_objectCandidate2D");

				std::cout<<"DOOOOOOO"<<std::endl;

	object_categorization_node::getKinect_objectCandidateList2D srv;
	if (client.call(srv)) {
		std::cout << "NUMBER OF RGB " << srv.response.imagesRGB.size() << "\n";
		std::cout << "NUMBER OF INTEN"
				<< srv.response.imagesDistanceIntensity.size() << "\n";
	}
*/

	ros::spin();

	return 0;
}
