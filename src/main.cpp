#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "multipleTracking.h"
#include "detectorBlob.h"
#include <ctime>

using namespace std;
using namespace cv;

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
// params initialize
int area = 50;
int minThreshold = 10;
int maxThreshold = 100;
void BlobParams(int, void*){}

// morpology operations
int morph_elem = 0;
int morph_size = 5;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;
void Morphology_Operations( int, void* ){}


// drawing functions
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

int main(int argc, char *argv[])
{
	// read in from video
	cv::VideoCapture cap;
    cap.open("../multiple-8.mp4");
    if( !cap.isOpened() )
    {
        printf("can not open camera or video file\n");
        return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    /// initialize by mouse click
    cv::Mat img;
    cap >> img;
    cv::cvtColor(img, img, CV_BGR2GRAY );  
    if( img.empty() )
        return -1;
    
    cv::namedWindow( "BlobParams", CV_WINDOW_AUTOSIZE );
    cv::createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", 
    "BlobParams", &morph_operator, max_operator, Morphology_Operations );
    /// Create Trackbar to select kernel type
    cv::createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", "BlobParams",
                 &morph_elem, max_elem,
                 Morphology_Operations );
    /// Create Trackbar to choose kernel size
    cv::createTrackbar( "Kernel size:\n 2n +1", "BlobParams",
                 &morph_size, max_kernel_size,
                 Morphology_Operations);
    cv::Mat element;
    /// parameters tuning for blob tracking
    //cv::namedWindow( "BlobParams", cv::WINDOW_NORMAL );
    cv::createTrackbar( "Blob min threshold:\n ", "BlobParams",
                 &minThreshold, 100,
                 BlobParams );
    cv::createTrackbar( "Blob max threshold:\n ", "BlobParams",
                 &maxThreshold, 100,
                 BlobParams );
    cv::createTrackbar( "Blob min area threshold:\n ", "BlobParams",
                 &area, 100,
                 BlobParams);

    cv::SimpleBlobDetector::Params params;
    // wait in a loop until a signal
    cv::namedWindow("initialize");
    cv::setMouseCallback("initialize",on_mouse, NULL); 

    for(;;)
    {
        //cap >> img;
        // Change thresholds
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
        element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), 
            Point( morph_size, morph_size ) );
        /// Apply the specified morphology operation
        morphologyEx( img, img, morph_operator + 2, element );
        // morphology operations done
        detector->detect(img);
        cv::imshow("initialize",detector->mask);
        //cv::imshow("initialize",img);
        char k = (char)cv::waitKey(30);
        if( k == ' ' ) break;
    }    


    // initialize
    double scale = 10/100;
    multipleTracking multipleTracker(scale,params);
	multipleTracker.initializeTracks(initPositions, initBboxes);
    cv::namedWindow( "Tracking", cv::WINDOW_AUTOSIZE );

    // container for predicted positions
    // containner for measured positions
    vector< vector<cv::Point2f> > trajectories;
    trajectories.resize(2);

    cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 );
    // loop
	for(int i = 0; i < 2000; i++)
    {
    	// read in frame
        cap >> img;
        cv::cvtColor( img, img, CV_BGR2GRAY );      
        if( img.empty() )
            break;

        morphologyEx( img, img, morph_operator + 2, element );

        clock_t begin = clock();
		// tracking
		multipleTracker.trackingOneStep(img);
        clock_t end1 = clock();
        double elapsed_secs1 = double(end1 - begin) / CLOCKS_PER_SEC;

        cout << "mutiple tracking framerate: " << 1/elapsed_secs1<<endl;

        if (multipleTracker.mask.empty())
            break;

		// display
		vector<track> tracks = multipleTracker.getTracks();

		// draw prediction on the image
        for(int i = 0; i < tracks.size(); i++)
        {
            MyEllipse(multipleTracker.mask, tracks[i].predictedPosition);
            trajectories[i].push_back(tracks[i].predictedPosition);
            if (trajectories[i].size() > 2)
            {
                for(int j = 1; j < trajectories[i].size() -1;j++)
                    cv::line(multipleTracker.mask,trajectories[i][j],trajectories[i][j+1],
                        cv::Scalar(0,min(j,255),min(j*100/255,255)),4);
            }
        }
        // Show in a window
        cv::imshow("Tracking", multipleTracker.mask );


        //cout << "ID " << multipleTracker.getMaximumID()<<endl;
        char k = (char)cv::waitKey(30);
        if( k == 27 ) break;
    }

    return 0;


}