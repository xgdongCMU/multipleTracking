#include <ros/ros.h>
#include <iostream>
#include <vector>


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "mag_tracking/detectorBase.h"
#include "mag_tracking/detection.h"
#include "mag_tracking/multipleTracking.h"


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>

#include <mag_tracking/TrackingConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

// model information
double robotLength; // mm
double robotRatio;
double scale; // mm/pixel
double goalRatio; // for object handle position


// messages 
Eigen::Vector2f goalPosition;
double goalOrientation;
bool goalSet = false;

Eigen::VectorXf goalRobotPosition;
bool goalRobotSet = false;

Eigen::Vector2f robotOrientation;
bool orientationSet = false;

char strStatemachine[200];
bool stateSet = false;

double frequency;
char strFrequency[200];
bool commandSet = false;



/// image sources 
cv::Mat image, image_mask, bwRobot, bwObjects, src_grayR,src_grayO;

// multiple tracking
multipleTracking * multipleTrackerRobot;
multipleTracking * multipleTrackerObjects;

// control flags
bool isFirst        = true;
bool isTracking     = false;
bool drawRadius     = true;

image_transport::Publisher image_pub;
image_transport::Publisher image_mask_pub;

sensor_msgs::ImagePtr image_msg;

/// mouse click callback function
vector<cv::Point2f> initPositions;
vector<cv::Rect> initBboxes;

// Parameters for robot
int threshold_valueR = 0;
int threshold_typeR = 3;;
int RLowH = 0;
int RHighH = 179;
int RLowS = 0; 
int RHighS = 198;
int RLowV = 0;
int RHighV = 115;
// contour detection parameters
int minAreaR = 100;
int maxAreaR = 2200;
int minThresholdR = 100;
int ratioR = 3;
int kernelSizeR = 0;
int blursizeR = 2;
// morpology operations
int morph_elemR = 0;
int morph_sizeR = 0;
int morph_operatorR = 0;

paramBase paramsRobot;


// callback functions for image processing
void DetectionRobot(int, void*){}
void DetectionObjects(int, void*){}

// intitialization function
static void on_mouse( int event, int x, int y, int d, void *ptr);

// draw function
void drawOnImage(vector<track> &tracks);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// utilities function
cv::Point2f image2world(cv::Point2f imagePoint, int width, int height, double scale);
cv::Point2f world2image(cv::Point2f worldPoint, int width, int height, double scale);

// dynamic reconfigure

typedef mag_tracking::TrackingConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
void configureImage(Config& newconfig, uint32_t level)
{
  try{
    threshold_valueR = newconfig.threshold_valueR;
    threshold_typeR = newconfig.threshold_typeR;
    
    RLowH = newconfig.RLowH;
    RHighH = newconfig.RHighH;
    RLowS = newconfig.RLowS; 
    RHighS = newconfig.RHighS;
    RLowV = newconfig.RLowV;
    RHighV = newconfig.RHighV;
    // contour detection parameters
    minAreaR = newconfig.minAreaR;
    maxAreaR = newconfig.maxAreaR;
    minThresholdR = 100;
    ratioR = newconfig.ratioR;
    kernelSizeR = newconfig.kernelSizeR;
    blursizeR = newconfig.blursizeR;
    // morpology operations
    morph_elemR = 0;
    morph_sizeR = 0;
    morph_operatorR = 0;

    isTracking = newconfig.isTracking;

  } 
  catch(const std::exception& e){
        ROS_ERROR_STREAM("Cannot update configuration!" << e.what());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh("~");

  // mm/pixel
  nh.param<double>("scale",scale,0.035);
  nh.param<double>("robotLength",robotLength,1.75);
  nh.param<double>("robotRatio",robotRatio,0.4);
  nh.param<double>("goalRatio",goalRatio,1.1);

  ReconfigureServer reconfigure_server_;
  reconfigure_server_.setCallback(boost::bind(&configureImage, _1, _2));


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub       = it.subscribe("/image", 1, imageCallback);

  image_transport::Publisher image_pub  = it.advertise("/image_draw", 1);
  image_transport::Publisher image_mask_pub  = it.advertise("/image_mask", 1);


  ros::Rate loop_rate(10);
  while(nh.ok()) {
      if(!image.empty())
      {
        try{
              image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
              image_pub.publish(image_msg);

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from image to 'bgr8'.");
        }
      }


      if(!image_mask.empty())
      {
        try{
              image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_mask).toImageMsg();
              image_mask_pub.publish(image_msg);

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


static void on_mouse( int event, int x, int y, int d, void *ptr )
{
    if( event != cv::EVENT_LBUTTONDOWN )
        return;

    cv::Point2f temp0 = cv::Point2f(x,y);
    cv::Rect temp1 = cv::Rect(x,y,10,10);
    
    initPositions.push_back(temp0); 
    initBboxes.push_back(temp1);    
    cout << "Initialized points number: "<<initPositions.size() << endl;  
    cout << "Point: x = " << temp0.x << ", y = "<<temp0.y << endl; 
}

void drawObjectsOnImage(vector<track> &tracks)
{
  
  int width = image.cols;
  int height = image.rows; 

  // draw origin
  cv::Point2f origin = cv::Point2f(width/2,height/2);
  cv::circle(image, origin, 2,cv::Scalar( 0, 0, 255),2, 8);

  for(int i = 0; i < tracks.size(); i++)
  {
      // draw predictions of objects
      cv::Point2f pointA = tracks[i].predictedPosition;
      cv::circle(image, pointA, 2,cv::Scalar( 0, 0, 255),2, 8);

      // draw radius of robots and objects
      if(drawRadius)
      {
        cv::Point2f pointB,pointC; 
        // pointB and pointC determine the orientations of the object drawing on the plane
        pointB = pointA + robotRatio*robotLength/scale*cv::Point2f(cos(tracks[i].heading),sin(tracks[i].heading));
        pointC = pointA - (1-robotRatio)*robotLength/scale*cv::Point2f(cos(tracks[i].heading),sin(tracks[i].heading));
        cv::circle(image, pointC, 2,cv::Scalar( 255, 0, 0 ),2, 8);
        cv::circle(image, pointC, robotLength/scale,cv::Scalar( 255, 0, 0 ),2, 8);
        cv::line(image, pointB, pointC, cv::Scalar( 0, 255, 0 ),2,8);
      }

      // draw trajectories of robots and objects
      if(tracks[i].trajectory.size() > 2)
      {
          for(int j = 1; j < tracks[i].trajectory.size() -1;j++)
              cv::line(image,tracks[i].trajectory[j],tracks[i].trajectory[j+1],cv::Scalar(0,min(j*5,255),min(j*255/100,255)),4);
      }
      
      // draw labels of objects
      char strID[200]; 
      char strCat[200];
      char str_x[100];
      char str_y[100];  
      char str_heading[100]; 

      sprintf(strID,"ID: %d",tracks[i].id);
      sprintf(strCat,"CAT: %d",tracks[i].category);
      sprintf(str_x,"x: %2.2f",tracks[i].predictedPositionmm.x);
      sprintf(str_y,"y: %2.2f",tracks[i].predictedPositionmm.y);   
      sprintf(str_heading,"h: %2.2f",-tracks[i].heading);

      cv::putText(image, strID, pointA+cv::Point2f(25,0), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));  
      //cv::putText(image, strCat, pointA+cv::Point2f(25,25),cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));           
      cv::putText(image, str_x, pointA+cv::Point2f(25,35), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));  
      cv::putText(image, str_y, pointA+cv::Point2f(25,45), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));          
      cv::putText(image, str_heading, pointA+cv::Point2f(25,55), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));   
  }

}

void drawRobotOnImage(vector<track> &tracks)
{
  
  int width = image.cols;
  int height = image.rows; 

  // draw origin
  cv::Point2f origin = cv::Point2f(width/2,height/2);
  cv::circle(image, origin, 2,cv::Scalar( 0, 0, 255),2, 8);


  // draw robots position and their trajectories
  for(int i = 0; i < tracks.size(); i++)
  {
      // draw predictions of robots
      cv::Point2f pointA = tracks[i].predictedPosition;
      cv::circle(image, pointA, 2,cv::Scalar( 0, 0, 255),2, 8);

      // draw radius of robots and objects
      if(drawRadius)
      {
        cv::Point2f pointB,pointC,displacement; 
        if(orientationSet)
          displacement = robotLength/scale*
              cv::Point2f(cos(robotOrientation[1])*cos(robotOrientation[0]),-cos(robotOrientation[1])*sin(robotOrientation[0]));
        else
          displacement = robotLength/scale*
              cv::Point2f(cos(tracks[i].heading),sin(tracks[i].heading));
        pointB = pointA + robotRatio* displacement;
        pointC = pointA - (1-robotRatio)* displacement;

        cv::circle(image, pointC, 2,cv::Scalar( 255, 0, 0 ),2, 8);
        cv::circle(image, pointC, robotLength/scale,cv::Scalar( 255, 0, 0 ),2, 8);
        cv::line(image, pointB, pointC, cv::Scalar( 0, 255, 0 ),2,8);
      }

      // draw trajectories of robots and objects
      if(tracks[i].trajectory.size() > 2)
      {
          for(int j = 1; j < tracks[i].trajectory.size() -1;j++)
              cv::line(image,tracks[i].trajectory[j],tracks[i].trajectory[j+1],cv::Scalar(0,min(j*5,255),min(j*255/100,255)),4);
      }
      
      // draw labels of robots
      char strID[200]; 
      //char strCat[200];
      char str_x[100];
      char str_y[100];  
      char str_heading[100]; 
      char str_vx[100];
      char str_vy[100];  

      sprintf(strID,"ID: %d",tracks[i].id);
      //sprintf(strCat,"CAT: %d",tracks[i].category);
      sprintf(str_x,"x: %2.2f",tracks[i].predictedPositionmm.x);
      sprintf(str_y,"y: %2.2f",tracks[i].predictedPositionmm.y);   
      sprintf(str_heading,"h: %2.2f",robotOrientation[0]);
      sprintf(str_vx,"vx: %2.2f",tracks[i].predictedVelocitymm.x);
      sprintf(str_vy,"vy: %2.2f",tracks[i].predictedVelocitymm.y);  
      

	
      cv::putText(image, strID, pointA+cv::Point2f(25,0), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));  
      //cv::putText(image, strCat, pointA+cv::Point2f(25,25),cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));           
      //cv::putText(image, str_x, pointA+cv::Point2f(25,35), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));  
      //cv::putText(image, str_y, pointA+cv::Point2f(25,45), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));          
      //cv::putText(image, str_heading, pointA+cv::Point2f(25,55), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));          
      //cv::putText(image, str_vx, pointA+cv::Point2f(25,65), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255));  
      //cv::putText(image, str_vy, pointA+cv::Point2f(25,75), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(0,0,255,255)); 

 
  }


  // draw scale bar
  char scalebar[200];
  sprintf(scalebar,"1 mm");
  cv::Point2f scalebarA = cv::Point2f(width-25,height-50);
  cv::Point2f scalebarB = scalebarA - cv::Point2f(1/scale,0);
  cv::line(image, scalebarA, scalebarB, cv::Scalar(0, 0, 0),2,8);
  cv::putText(image, scalebar, cv::Point2f(width-65,height-70), cv::FONT_HERSHEY_PLAIN, 1.0,  cv::Scalar(0,0,0,255));  

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;   

      if(image.empty())
      {
        ROS_ERROR("Cannot grab image");
        return;
      }

      //cv::cvtColor(image, src_grayR, CV_BGR2GRAY );
      //cv::cvtColor(image, src_grayR, COLOR_BGR2HSV);
      src_grayO = image.clone();
      src_grayR = image.clone();

      //// Robot
      //cv::threshold(src_grayR, bwRobot, threshold_valueR, 255,threshold_typeR);
      //cv::threshold(src_grayR, bwRobot, threshold_valueR, 255,threshold_typeR);
      cv::inRange(src_grayR, Scalar(RLowH, RLowS, RLowV), Scalar(RHighH, RHighS, RHighV), bwRobot);

      paramsRobot.mode = 1;
      // Filter by Threshold
      paramsRobot.minThreshold = minThresholdR;
      paramsRobot.maxThreshold = (ratioR >= 2?ratioR:2)*minThresholdR;
      // Filter by Area
      paramsRobot.minArea = minAreaR;
      paramsRobot.maxArea = maxAreaR;
      paramsRobot.kernelSize = kernelSizeR >= 1? kernelSizeR:1;
      // Blur kernel size
      paramsRobot.blursize = blursizeR >= 1? blursizeR:1;
      
      
      cv::Mat elementR = cv::getStructuringElement(morph_elemR, 
                                  cv::Size(2*morph_sizeR + 1, 2*morph_sizeR+1),
                                  cv::Point(morph_sizeR, morph_sizeR));
      cv::morphologyEx(bwRobot, bwRobot, morph_operatorR + 2, elementR);

      if(!isTracking)
      {
          detectorBase* detectorRobot = new detectorBase(paramsRobot);
          vector<detection> detectionsRobot = detectorRobot->detect(bwRobot);            
          image_mask = detectorRobot->mask;

      }
      
      char k = (char)cv::waitKey(30);
      if( k == 27 ) isTracking = true;  

      if(isTracking)
      {
          if(isFirst)
          {

            cv::Point2f temp0 = cv::Point2f(0,0);
            cv::Rect temp1 = cv::Rect(0,0,10,10);

            initPositions.push_back(temp0); 
            initBboxes.push_back(temp1);    
            cout << "Initialized points number: "<<initPositions.size() << endl;  
            cout << "Point: x = " << temp0.x << ", y = "<<temp0.y << endl; 


            multipleTrackerRobot = new multipleTracking(scale,image.size().width, image.size().height,paramsRobot);
            multipleTrackerRobot->initializeTracks(initPositions, initBboxes);


          }
          isFirst = false;

          multipleTrackerRobot->trackingOneStep(bwRobot);
          //multipleTrackerObjects->trackingOneStep(bwObjects);

          if(multipleTrackerRobot->mask.empty())
          {  
              ROS_ERROR("mask is empty");
              return;
          }
          // Get the tracking data and draw on the image
          vector<track> tracksRobot = multipleTrackerRobot->getTracks();
          drawRobotOnImage(tracksRobot);

          // Show in a window
          image_mask = multipleTrackerRobot->mask;
      }

      char k1 = (char)cv::waitKey(30);
      if( k1 == 32 && isTracking) 
      {
        isTracking  = false;
        isFirst     = true;
        delete multipleTrackerRobot;
        //delete multipleTrackerObjects;
      }
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
}

// coordinates transformations
cv::Point2f image2world(cv::Point2f imagePoint, int width, int height, double scale)
{
  cv::Point2f worldPoint;
  worldPoint = cv::Point2f(scale*(imagePoint.y - width/2),scale*(-imagePoint.x + height/2));
  return worldPoint;
}

cv::Point2f world2image( cv::Point2f worldPoint,int width, int height, double scale)
{
  cv::Point2f imagePoint;
  imagePoint = cv::Point2f(-worldPoint.y/scale + height/2,worldPoint.x/scale + width/2);
  return imagePoint;

}
