#ifndef multipleTracking_H
#define multipleTracking_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ctime>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "munkres.h"
#include "detectorBackgroundSubtraction.h"
#include "detectorBlob.h"

#include "track.h"
#include "detection.h"
#include "assignmentInfo.h"

using namespace std;

class multipleTracking{

public:
	// detectors: need to maintain
	//detectorBackgroundSubtraction detector;
	detectorBlob* detector;

	// scale mm/pixel
	double scale;						

	// mask of frame: no need to maintain
	cv::Mat mask;

	// detections: no need to maintain
	vector<detection> detections;

	// tracks: need to maintain
	vector<track> tracks;
	
	// assignments: no need to maintain
	assignmentInfo assigns;

public:
	multipleTracking(double scale, cv::SimpleBlobDetector::Params params);
	~multipleTracking();

	// manually initialize the objects to be tracked
	void initializeTracks(vector<cv::Point2f> initPositions, vector<cv::Rect> bboxes);


	// tracking one step
	void trackingOneStep(cv::Mat frame);

	// automatic detection by detector
	void detectObjects(cv::Mat frame);

	// use filter to predict current tracks' next postions
	void predictNewLocationsOfTracks();

	// update tracks by assigning new detection to tracks
	void detectionToTrackAssignment();

	void updateAssignedTracks();
	void updateUnassignedTracks();
	void deleteLostTracks();
	void createNewTracks();
	vector <track> getTracks();


	// utils
	double distance(cv::Point2f point1, cv::Point2f point2);
	int getMaximumID();



};

#endif