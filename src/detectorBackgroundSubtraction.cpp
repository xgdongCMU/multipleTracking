#include "detectorBackgroundSubtraction.h"

detectorBackgroundSubtraction::detectorBackgroundSubtraction()
{
	bg_model = cv::BackgroundSubtractorMOG2(5, 16, false);

}

detectorBackgroundSubtraction::~detectorBackgroundSubtraction()
{


}

vector<detection> detectorBackgroundSubtraction::detect(cv::Mat frame)
{
	cv::Mat fgmask;

    bg_model(frame, fgmask, -1);

	// Morphology Operations
	int morph_elem = 0;
	int morph_size = 0;
	int morph_operator = 0;	
	cv::Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

	/// Apply the specified morphology operation
	morphologyEx( fgmask, fgmask, MORPH_OPEN, element );
	morphologyEx( fgmask, fgmask, MORPH_CLOSE, element );

	// update mask
	mask = fgmask;

    // find contour 
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( fgmask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // bounding box
	vector<Point>  contours_poly;
	vector<Rect>   boundRect;
	double areaThreshold = 50;

	for( int i = 0; i < contours.size(); i++ )
	 { 
	 	if(contourArea(contours[i],false) > areaThreshold)
	 	{
	 		approxPolyDP( Mat(contours[i]), contours_poly, 3, true );
	   		boundRect.push_back(boundingRect( Mat(contours_poly) ));
	   	}
	 }
    
    vector<struct detection> detections;
    detection temp;
    for( int i = 0; i < boundRect.size(); i ++)
    {
        temp.centroid   = cv::Point2f(boundRect[i].x + boundRect[i].width/2,boundRect[i].y + boundRect[i].height/2);
        temp.bbox       = boundRect[i];                        
        detections.push_back(temp);
	}

    return detections;

}