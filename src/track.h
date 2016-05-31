#ifndef track_H
#define track_H
#include "opencv2/core/core.hpp"

// tracks for maintaining different observed objects
class track{

public:
	cv::Point2f predictedPosition;		// predicted position in pixel
	cv::Point2f predictedPositionmm; 	// predicted position in mm	
	cv::Point2f predictedVelocitymm;		// predicted velocity in mm/s
    	cv::KalmanFilter KF;				// filter for this track

	cv::Rect bbox;						// bounding box for new detection in mm
	
    int id;
    int age;							// number of frames since starting
    int totalVisibleCount;				// total frames of exsisting
    int consecutiveInvisibleCount; 		// frames number of being consecutive invisible 

};

#endif
