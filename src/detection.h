#ifndef detection_H
#define detection_H
#include "opencv2/core/core.hpp"

// detections of a single frame for multiple blobs

class detection{
public:
	cv::Point2f centroid; 			// centroids of detections in pixel
	cv::Rect  bbox;					// bounding box of detections in pixel
};


#endif