#ifndef assignmentInfo_H
#define assignmentInfo_H
#include "opencv2/core/core.hpp"


// assignmentInfo
class assignmentInfo{
public:
	cv::Mat assignments; 				// Mx2 matrix
	vector<int> unassignedTracks;		// vector of unassigned tracks
	vector<int> unassignedDetections; 	// vector of unassigned detections 
};

#endif
