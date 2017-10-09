/*
Modified by: Kehinde Aina Sept. 9, 2017
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CompressedImage.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

volatile int j =0;
geometry_msgs::Point32 ballPoint;

void FindDirection(ros::Publisher _ballDirection_pub)
{
  ROS_INFO("detectObject publish: %d",j);  
  _ballDirection_pub.publish(ballPoint);
}

void getImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) //ImageConstPtr
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {    
    int x_1 = 0;
    int y_1 =0;
    double x_angle = 0.0;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  
    Mat src, src_gray, hsv_image, lower_red_hue_range, upper_red_hue_range;    

    src = cv_ptr->image;
    resize(src,src,Size(320,240));

    medianBlur(src, src, 3);
    cvtColor(src, hsv_image, COLOR_BGR2HSV);  // Convert input image to HSV
  	// Threshold the HSV image, keep only the red pixels
  	inRange(hsv_image, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
    inRange(hsv_image, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);

    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, src_gray);      
    GaussianBlur(src_gray, src_gray, Size(7, 7), 9, 9); //Size(9, 9), 2, 2);

    vector<Vec3f> circles;    
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, 2000, 100, 20, 0, 0);
    //HoughCircles(src_gray,circles,CV_HOUGH_GRADIENT,1,10,30,90,10,200);  
    int im_size = circles.size();    
    for(int i=0; i<im_size; i++)
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);

      circle( src_gray, center, 3, Scalar(0,255,0), -1, 8, 0 );
      circle( src_gray, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
    
    if(im_size>0) 
    {
      x_1 = circles[0][0];
      y_1 = circles[0][0];
    
      int x1 = 160-x_1;
      x_angle = x1*62.2/320;
      ballPoint.x = x_1;
      ballPoint.y = y_1;
      ballPoint.z = x_angle; //radians;

    }

    imshow("Hough Circle Transform Demo", src_gray);    
    ROS_INFO("image:[%d], size:%d, x:%d, angle:%f",++j,im_size,x_1,x_angle);

    waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "detectObject");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/raspicam_node/image/compressed", 1, getImageCallback);  
  
  ros::Publisher ballDirection_pub = nh.advertise<geometry_msgs::Point32>("ball_direction", 1);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    FindDirection(ballDirection_pub);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::spin();  
      
  return 0;
}


