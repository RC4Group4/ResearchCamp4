#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

// cvBlobsLib Includes.
#include <cvblobs/BlobResult.h>

class ImageConverter 
{

public:

  ImageConverter(ros::NodeHandle &n) : n_(n), it_(n_)
  {
    image_pub_ = it_.advertise("image_topic_2",1);

    //cvNamedWindow( "Image window" );
    image_sub_ = it_.subscribe( "/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this );
  }

  ~ImageConverter()
  {
    cvDestroyWindow("Image window");
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    int master_image_width = 0; 
    int master_image_height = 0; 
    int master_image_center_x = 0;
    int master_image_center_y = 0;  

    double x_offset = 0; 
    double y_offset = 0; 
    double rot_offset = 0; 

    IplImage* cv_image = NULL;
    IplImage* blob_image = NULL;
    IplImage* display_image = NULL; 

    CBlob* currentBlob; 

    try
    {
      cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }

    //  Obtain image properties that we require. 
    master_image_width = cv_image->width; 
    master_image_height = cv_image->height; 
    master_image_center_x = ( master_image_width / 2 );
    master_image_center_y = ( master_image_height / 2 );

    IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1 );
    cvCvtColor( cv_image, gray, CV_BGR2GRAY );

    blob_image = cvCreateImage( cvGetSize( cv_image ), 8, 3 ); 

    cvThreshold( gray, gray, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

    // Find any blobs that are not white. 
    CBlobResult blobs = CBlobResult( gray, NULL, 0 );

    //  Make sure they are big enough to really be considered.
    //  In this case we will use an area of AT LEAST 200 px. 
    int minimum_blob_area = 200; 
    int maximum_blob_area = 1000;
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minimum_blob_area ); 
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, maximum_blob_area ); 

    int blob_number = blobs.GetNumBlobs(); 
    std::cout << "\nNumber of Blobs Present: " << blob_number << std::endl; 
    //  Add the found blobs to the blob_image.
    for ( int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
      CBlobGetOrientation get_orientation; 
      currentBlob = blobs.GetBlob(i);
      currentBlob->FillBlob( blob_image, CV_RGB( 0, 0, ( rand() % 255 ) ) );

      double maxx = currentBlob->MaxX(); 
      double minx = currentBlob->MinX(); 
      double maxy = currentBlob->MaxY(); 
      double miny = currentBlob->MinY(); 

      double blob_x = ( ( minx + maxx ) / 2 );
      double blob_y = ( ( miny + maxy ) / 2 ); 

      double dist_x = ( blob_x - master_image_center_x ); 
      double dist_y = ( blob_y - master_image_center_y ); 

      double distance = sqrt( ( dist_x * dist_x ) + ( dist_y * dist_y ) ); 

      double rotation = 0.0; 
      //double rotation = tan( get_orientation( currentBlob ) * ( 3.1415926535 / 180 ) ); 

      //  DEBUGGING
      std::cout << "Blob #:\t\t\t" << i << std::endl; 
      std::cout << "Blob Center x:\t\t" << blob_x << std::endl; 
      std::cout << "Blob Center y:\t\t" << blob_y << std::endl; 
      std::cout << "Distance to Center:\t" << distance << std::endl; 
      std::cout << "Horizontal Offset:\t" << abs( dist_x ) << std::endl; 
      std::cout << "Vertical Offset:\t" << abs( dist_y ) << std::endl; 
      std::cout << "Rotational Offset:\t" << rotation << std::endl; 
      std::cout << "\n" << std::endl; 

      if( i == 0 )
      {
        x_offset = abs( dist_x ); 
        y_offset = abs( dist_y ); 
        rot_offset = rotation; 
      }

      cvCircle( blob_image, cvPoint( blob_x, blob_y ), 10, CV_RGB( 255, 0, 0 ), 2 ); 
    }

    // display filtered blobs
    //cvMerge( blob_image, NULL, NULL, NULL, display_image );

    // Setting up fonts for overlay information.
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);


    cvLine( blob_image,   cvPoint( 0, (master_image_height/2) ), cvPoint( master_image_width, (master_image_height / 2) ), CV_RGB( 255, 0, 0 ), 2, 0 ); 
    cvLine( blob_image,   cvPoint( (master_image_width/2), 0 ), cvPoint( (master_image_width/2), master_image_height ), CV_RGB( 255, 0, 0 ), 2, 0 );
    //cvPutText( gray, "Hello World!", cvPoint( 10, gray->height - 10 ), &font, cvScalar( 255, 1, 1 ) );
    cvRectangle( blob_image, cvPoint( 0, blob_image->height-40 ), cvPoint( blob_image->width, blob_image->height ), CV_RGB( 0, 0, 0 ), -1 );


    std::string x_str = "X: "; 
    x_str += boost::lexical_cast<std::string>( x_offset ); 

    std::string y_str = "Y: "; 
    y_str += boost::lexical_cast<std::string>( y_offset ); 

    std::string rot_str = "Rotation: "; 
    rot_str += boost::lexical_cast<std::string>( rot_offset ); 

    cvPutText( blob_image, x_str.c_str(), cvPoint( 75, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, y_str.c_str(),  cvPoint( 225, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, rot_str.c_str(), cvPoint( 375, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );

    //cvShowImage( "Original", cv_image ); 
    //cvShowImage( "Thresholding", gray ); 
    cvShowImage( "Found Blobs", blob_image ); 
    //cvShowImage( "Blob Detection", display_image ); 

    //  Wait for user interaction.
    cvWaitKey(3);
  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::spin();
  return 0;
}