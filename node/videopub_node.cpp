#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  std::string filename;
  int framerate;

  nh.param<std::string>("filename",filename,"/home/mag/catkin_ws/src/mag_tracking/data/gripper.ogv");
  nh.param<int>("framerate",framerate,30);

  cout << filename;

  cv::VideoCapture cap(filename.c_str()); // open the default camera
  ROS_INFO("Video publisher starts!");
 
  if(!cap.isOpened())  // check if we succeeded
  {
    ROS_INFO("Cannot open video!");
    return -1;
  }
  cv::Mat image;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/image", 1);

  ros::Rate loop_rate(framerate);
  while (nh.ok())
  {

      cap >> image; // get a new frame from camera
      cv::waitKey(30);
      
      if(!image.empty())
      {
        try{
              sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
              image_pub.publish(image_msg);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from image to 'bgr8'.");
        }
      }
      
      ros::spinOnce();
      loop_rate.sleep();
  }

}
