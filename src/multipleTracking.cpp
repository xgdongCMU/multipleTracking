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
#include "multipleTracking.h"

	multipleTracking::multipleTracking(double scale, cv::SimpleBlobDetector::Params params)
	{
		//detector = detectorBackgroundSubtraction();

		// params: pre-calibrate
		detector = new detectorBlob(params);
		this->scale = scale;

	}

	multipleTracking::~multipleTracking()
	{



	}


	void multipleTracking::trackingOneStep(cv::Mat frame)
	{

		detectObjects(frame);


		predictNewLocationsOfTracks();


    	detectionToTrackAssignment();


		updateAssignedTracks();
		updateUnassignedTracks();
		deleteLostTracks();
		createNewTracks();


	}
	// manually initialize the objects to be tracked
	void multipleTracking::initializeTracks(vector<cv::Point2f> initPositions, vector<cv::Rect> bboxes)
	{

		// specify initial positions of tracks manually 
		for(int i = 0; i < initPositions.size(); i ++)
		{
			// initialize a track
			track temp;

			// initialize bounding box
			temp.bbox = bboxes[i];

			// initialize Kalman Filter
			temp.KF 	= KalmanFilter(4,2,0,CV_32F);
			double dt = 0.1;
			temp.KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,dt,0,   0,1,0,dt,  0,0,1,0,  0,0,0,1);		 
			temp.KF.statePre.at<float>(0) = initPositions[i].x;
			temp.KF.statePre.at<float>(1) = initPositions[i].y;
			temp.KF.statePre.at<float>(2) = 0;
			temp.KF.statePre.at<float>(3) = 0;
			setIdentity(temp.KF.measurementMatrix);
			setIdentity(temp.KF.processNoiseCov, Scalar::all(1e-2));
			setIdentity(temp.KF.measurementNoiseCov, Scalar::all(0.001));
			setIdentity(temp.KF.errorCovPost, Scalar::all(.001));

			temp.id  	= i;
			temp.age = 0;
			temp.totalVisibleCount = 1;
			temp.consecutiveInvisibleCount = 0;

			// push track
			tracks.push_back(temp);
		}

		// debug
		cout << "Tracks number: " << tracks.size()<<endl;

	}

	// automatic detection by detector
	void multipleTracking::detectObjects(cv::Mat frame)
	{
		detections = detector->detect(frame);
		mask = detector->mask;

	}

	// use filter to predict current tracks' next postions
	void multipleTracking::predictNewLocationsOfTracks()
	{
		for(int i = 0; i < tracks.size(); i ++)
		{
			// predict using Kalman Filter
			cv::Mat prediction = tracks[i].KF.predict();
			tracks[i].predictedPosition = cv::Point2f(prediction.at<float>(0),prediction.at<float>(1));
			tracks[i].predictedPositionmm = cv::Point2f(prediction.at<float>(0),prediction.at<float>(1))*scale;
			tracks[i].predictedVelocitymm= cv::Point2f(prediction.at<float>(2),prediction.at<float>(3))*scale;

			// shift bounding box
			cv::Rect temp = tracks[i].bbox;
			tracks[i].bbox = cv::Rect(tracks[i].predictedPosition.x-temp.x - temp.width/2, 
				tracks[i].predictedPosition.y-temp.y - temp.height/2,temp.width,temp.height);

		}

	}

	// update tracks by assigning new detection to tracks
	void multipleTracking::detectionToTrackAssignment()
	{
		int trackNum 		= tracks.size();
		int detectionNum 	= detections.size();

		/// calculate cost matrix
		cv::Mat_<int> costMatrix(trackNum,detectionNum);

		//cv::Mat costMatrix(trackNum,detectionNum,CV_32F);
		for(int i = 0; i < trackNum; i ++)
		{
			for(int j = 0; j < detectionNum; j ++)
			{
				//cout << "prediction" << tracks[i].predictedPosition<<endl;
				//cout << "detection" << detections[j].centroid<<endl;

				costMatrix.at<int>(i,j) = distance(tracks[i].predictedPosition, detections[j].centroid);
			}
		}

		// debug
		//cout << "cost " <<costMatrix << endl;

		/// solve assignment using Hungarian algorithm
		// cost of not assigning to any tracks needs to be tuned
		Munkres m;
	    m.diag(false);
		m.solve(costMatrix);

		// debug
		//cout << "cost 2 " <<costMatrix << endl;

		/// assigned tracks
		vector <int> temp1, temp2, temp3, temp4;

		for(int i = 0; i < trackNum; i ++)
			temp3.push_back(i);

		for(int j = 0; j < detectionNum; j ++)
			temp4.push_back(j);


		for(int i = 0; i < trackNum; i ++)
		{
			for(int j = 0; j < detectionNum; j ++)
			{

				if (costMatrix.at<int>(i,j) == 0)
				{
					// add i to assigned track list and j to assigned detection list
					temp1.push_back(i);
					temp2.push_back(j);

					// remove i from unassigned track's list
					std::vector<int>::iterator position1 = std::find(temp3.begin(), temp3.end(), i);
					if (position1 != temp3.end())
						temp3.erase(position1); 

					// remove j from unassigned detection's list
					std::vector<int>::iterator position2 = std::find(temp4.begin(), temp4.end(), j);
					if (position2 != temp4.end())
						temp4.erase(position2); 

					break;
				}
			}
		}



		/// assigned tracks and detections
		assigns.assignments = cv::Mat(temp1.size(),2, CV_32F);
		for(int i = 0; i < temp1.size(); i ++)
		{
			assigns.assignments.at<int>(i,0) = temp1[i];
			assigns.assignments.at<int>(i,1) = temp2[i];
		}

		/// unassigned tracks
		assigns.unassignedTracks = temp3;

		/// unassigned detections
		assigns.unassignedDetections = temp4;


	}

	void multipleTracking::updateAssignedTracks()
	{
		// use assigns to update tracks
		for(int i = 0; i < assigns.assignments.rows; i ++)
		{   

			int trackIdx 		= assigns.assignments.at<int>(i,0);
            int detectionIdx 	= assigns.assignments.at<int>(i,1);
            // Correct the estimate of the object's location
            // using the new detection.
			cv::Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));            
			measurement(0) = detections[detectionIdx].centroid.x;
            measurement(1) = detections[detectionIdx].centroid.y;
			
            //cout << "good"<< trackIdx << assigns.assignments<<endl;

			tracks[trackIdx].KF.correct(measurement);


            // Replace predicted bounding box with detected
            // bounding box.
            tracks[trackIdx].bbox = detections[detectionIdx].bbox;

            // Update track's age.
            tracks[trackIdx].age += 1;

            // Update visibility.
			tracks[trackIdx].totalVisibleCount  += 1;
            tracks[trackIdx].consecutiveInvisibleCount = 0;
        }

	}

	void multipleTracking::updateUnassignedTracks()
	{
        for (int i = 0; i < assigns.unassignedTracks.size(); i++)
        {
            int ind = assigns.unassignedTracks[i];
            tracks[ind].age += 1;
            tracks[ind].consecutiveInvisibleCount += 1;
        }

	}

	void multipleTracking::deleteLostTracks()
	{
        if(tracks.empty())
            return;

        // threshold for deleting
        int invisibleForTooLong = 20;
        int ageThreshold = 80;

        for(int i = 0; i < tracks.size(); i++)
        {
        	double visibility = tracks[i].totalVisibleCount / (double)tracks[i].age;
        	if (tracks[i].age < ageThreshold && visibility < 0.6)
        		tracks.erase(tracks.begin() + i);
		}

	}

	void multipleTracking::createNewTracks()
	{
		// create new tracks using unassigned detections
		for (int i = 0; i < assigns.unassignedDetections.size(); i ++)
		{

		    track temp;

		    // bounding box
		    temp.bbox 		= detections[assigns.unassignedDetections[i]].bbox;
			
			// Kalman Filter
			temp.KF 		= KalmanFilter(4,2,0,CV_32F);
			temp.KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);		 
			temp.KF.statePre.at<float>(0) = detections[assigns.unassignedDetections[i]].centroid.x;
			temp.KF.statePre.at<float>(1) = detections[assigns.unassignedDetections[i]].centroid.y;
			temp.KF.statePre.at<float>(2) = 0;
			temp.KF.statePre.at<float>(3) = 0;
			setIdentity(temp.KF.measurementMatrix);
			setIdentity(temp.KF.processNoiseCov, Scalar::all(1e-4));
			setIdentity(temp.KF.measurementNoiseCov, Scalar::all(10));
			setIdentity(temp.KF.errorCovPost, Scalar::all(.1));

		    temp.id  		= getMaximumID()+1;			
			temp.age 		= 0;
			temp.totalVisibleCount = 1;
			temp.consecutiveInvisibleCount = 0;

			// push track
			tracks.push_back(temp);
		}

	}

	double multipleTracking::distance(cv::Point2f point1, cv::Point2f point2)
	{
		cv::Point2f diff = point1 - point2;
		double dist = std::sqrt( diff.x*diff.x + diff.y*diff.y );
		return dist;
	}

	int multipleTracking::getMaximumID()
	{
		int maxID = tracks[0].id;
		for(int i = 0; i < tracks.size(); i++)
			if(tracks[i].id > maxID)
				maxID = tracks[i].id;
		return maxID;
	}

	vector <track> multipleTracking::getTracks()
	{
		return tracks;

	} 