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

#ifndef detection_H
#define detection_H

//#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"

// detections of a single frame for multiple blobs

class detection{
public:
	cv::Point2f centroid; 		// centroids of detections in pixel
	cv::Rect  bbox;				// bounding box of detections in pixel
	double heading;				// heading of the object in rad
	int category;				// category of the object
};

class paramContour{
public:
	int minThreshold;
	int maxThreshold;
	int blursize;
	int minArea;
	int maxArea;
	int kernelSize;
};

class paramBase{
public:
	int mode;

	// mode 1
	int blursize;
	int kernelSize;
	int maxArea;

	// mode 2
	// Filter by Threshold
	int minThreshold;
	int maxThreshold;

	// Filter by Area
	bool filterByArea;
	int minArea;

	// Filter by Circularity
	bool filterByCircularity;
	float minCircularity;

	// Filter by Convexity
	bool filterByConvexity;
	float minConvexity;

	// Filter by Inertia
	bool filterByInertia;
	float minInertiaRatio; 

};
#endif
