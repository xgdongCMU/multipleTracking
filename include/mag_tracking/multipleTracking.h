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

#ifndef multipleTracking_H
#define multipleTracking_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ctime>
/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
*/

#include "mag_tracking/munkres.h"
#include "mag_tracking/detectorBackgroundSubtraction.h"
#include "mag_tracking/detectorBlob.h"
#include "mag_tracking/detectorBase.h"
#include "mag_tracking/track.h"
#include "mag_tracking/detection.h"
#include "mag_tracking/assignmentInfo.h"

using namespace std;

class multipleTracking{

public:
	// detectors: need to maintain
	//detectorBackgroundSubtraction detector;
	//detectorBlob* detector;
	detectorBase* detector;

	// scale mm/pixel
	double scale;
	// image size						
	double width;
	double height;
	
	// mask of frame: no need to maintain
	cv::Mat mask;

	// detections: no need to maintain
	vector<detection> detections;

	// Filter parameters
	double processNoiseCov;
	double measurementNoiseCov;
	double errorCovPost;
	
	// tracks: need to maintain
	vector<track> tracks;
	
	// assignments: no need to maintain
	assignmentInfo assigns;

public:
	multipleTracking(double scale, double width, double height, paramBase params);
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
