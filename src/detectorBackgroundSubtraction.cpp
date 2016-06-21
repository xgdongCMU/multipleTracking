/*
 * Copyright [2016] [Xiaoguang Dong xgdong2013@gmail.com]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mag_tracking/detectorBackgroundSubtraction.h"

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