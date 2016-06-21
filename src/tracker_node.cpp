#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <queue>


#include "mag_tracking/detectorBlob.h"
#include "mag_tracking/multipleTracking.h"
#include <mag_tracking/states_stickslip.h>
using namespace std;

cv::Mat image, bw;
cv::SimpleBlobDetector::Params params;
cv::Mat element;
      
// params initialize
int area = 100;
int minThreshold = 10;
int maxThreshold = 200;
void BlobParams(int, void*){}

// morpology operations
int morph_elem = 0;
int morph_size = 5;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;
void Morphology_Operations( int, void* ){}

// mouse click callback function
vector<cv::Point2f> initPositions;
vector<cv::Rect> initBboxes;
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

cv::Mat drawing;
multipleTracking * multipleTracker;
bool isFirst = true;
bool isTracking = false;
double scale = 0.1;

ros::Publisher states_pub;

void MyEllipse( cv::Mat img, cv::Point center )
{
    int thickness = -1;
    int lineType = 8;

    cv::circle( img,
         center,
         5,
         cv::Scalar( 0, 0, 255 ),
         thickness,
         lineType );
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;    
    cv::cvtColor(image, bw, CV_BGR2GRAY );

    if(!isTracking)
    {
        params.minThreshold = minThreshold;
        params.maxThreshold = maxThreshold;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = area;

        // Filter by Circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;      
        detectorBlob* detector = new detectorBlob(params);

        // morphology operations start
        // Since MORPH_X : 2,3,4,5 and 6
        element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), 
            cv::Point( morph_size, morph_size ) );
        /// Apply the specified morphology operation
        cv::morphologyEx( bw, bw, morph_operator + 2, element );
        // morphology operations done
        detector->detect(bw);
        cv::imshow("view",detector->mask);
    }

    char k = (char)cv::waitKey(30);
    if( k == 27 ) isTracking = true;  

    if(isTracking)
    {
        if(isFirst)
        {
          multipleTracker = new multipleTracking(scale,params);
          multipleTracker->initializeTracks(initPositions, initBboxes);
          drawing = cv::Mat::zeros( bw.size(), CV_8UC3 );
        }
        isFirst = false;
        //loop
        morphologyEx( bw, bw, morph_operator + 2, element );
        clock_t begin = clock();

        multipleTracker->trackingOneStep(bw);
        clock_t end1 = clock();
        double elapsed_secs1 = double(end1 - begin) / CLOCKS_PER_SEC;

        ROS_INFO("mutiple tracking framerate: %f", 1/elapsed_secs1);

        if (multipleTracker->mask.empty())
        {  
            ROS_ERROR("mask is empty");
            return;
          }
        // display
        vector<track> tracks = multipleTracker->getTracks();

        mag_tracking::states_stickslip state_msg;
        state_msg.robotNumber = tracks.size();
        
        //vector< vector<cv::Point2f> > trajectories;
        //trajectories.resize(tracks.size());
        // draw prediction on the image

        for(int i = 0; i < tracks.size(); i++)
        {
            state_msg.positions.push_back(tracks[i].predictedPositionmm.x);
            state_msg.positions.push_back(tracks[i].predictedPositionmm.y);
            state_msg.velocities.push_back(tracks[i].predictedVelocitymm.x);
            state_msg.velocities.push_back(tracks[i].predictedVelocitymm.y);
            //cout << "access" <<endl;
            // draw predictions
            MyEllipse(multipleTracker->mask, tracks[i].predictedPosition);
           
           /*
            // draw trajectories
            trajectories[i].push_back(tracks[i].predictedPosition);
            if(trajectories[i].size() > 100)
                trajectories[i].erase(trajectories[i].begin());

            if (trajectories[i].size() > 2)
            {
                for(int j = 1; j < trajectories[i].size() -1;j++)
                    cv::line(multipleTracker->mask,trajectories[i][j],trajectories[i][j+1],
                        cv::Scalar(0,min(j,255),min(j*100/255,255)),4);
            }
            */
            // label the objects
            char str[200];
            sprintf(str,"%d",tracks[i].id);
            cv::putText(multipleTracker->mask, str, tracks[i].predictedPosition, 
                        cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));           
                         
        }

        states_pub.publish(state_msg);
        // Show in a window
        cv::imshow("view", multipleTracker->mask);
      }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void detection(cv::Mat & img)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  // parameters for tracking
  cv::namedWindow( "BlobParams");
  cv::createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", 
  "BlobParams", &morph_operator, max_operator, Morphology_Operations );
  cv::createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", "BlobParams",
               &morph_elem, max_elem,
               Morphology_Operations );
  cv::createTrackbar( "Kernel size:\n 2n +1", "BlobParams",
               &morph_size, max_kernel_size,
               Morphology_Operations);
  cv::createTrackbar( "Blob min threshold:\n ", "BlobParams",
               &minThreshold, 100,
               BlobParams );
  cv::createTrackbar( "Blob max threshold:\n ", "BlobParams",
               &maxThreshold, 100,
               BlobParams );
  cv::createTrackbar( "Blob min area threshold:\n ", "BlobParams",
               &area, 200,
               BlobParams);  

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);

  cv::setMouseCallback("view",on_mouse, NULL); 

  states_pub = nh.advertise<mag_tracking::states_stickslip>("/robotStates", 10);


  ros::spin();
  cv::destroyWindow("view");

}
