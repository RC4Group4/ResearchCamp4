#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

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
    IplImage* cv_image = NULL;
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

    IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1);
    cvCvtColor( cv_image, gray, CV_BGR2GRAY );

    display_image = cvCreateImage( cvGetSize( cv_image ), cv_img->depth, cv_img->nChannels ); 

    cvThreshold( gray, gray, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

    // Find any blobs that are not white. 
    CBlobResult blobs = CBlobResult( gray, NULL, 255 );

    // get mean gray color of biggest blob
    CBlob biggestBlob;
    CBlobGetMean getMeanColor( cv_image );
    double meanGray;

    blobs.GetNthBlob( CBlobGetArea(), 0, biggestBlob );
    meanGray = getMeanColor( biggestBlob );

    // display filtered blobs
    cvMerge( gray, gray, gray, NULL, display_image );

    for ( int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
      currentBlob = blobs.GetBlob(i);
      currentBlob->FillBlob( display_image, CV_RGB( 0, 0, 255 ) );
    }

    // Setting up fonts for overlay information.
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

    //cvPutText( gray, "Hello World!", cvPoint( 10, gray->height - 10 ), &font, cvScalar( 255, 1, 1 ) );
    cvRectangle( gray, cvPoint( 0, gray->height-50 ), cvPoint( gray->width, gray->height ), CV_RGB( 0, 255, 0 ), -1 );

    cvPutText( gray, "X: ", cvPoint( 10, gray->height - 10 ), &font, CV_RGB( 0, 255, 255 ) );
    cvPutText( gray, "Y: ", cvPoint( 110, gray->height - 10 ), &font, CV_RGB( 0, 255, 255 ) );
    cvPutText( gray, "Rotation: ", cvPoint( 210, gray->height - 10 ), &font, CV_RGB( 0, 255, 255 ) );

    cvShowImage(  "Thresholding", gray ); 

    cvShowImage( "Found Blobs", display_image ); 

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