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

#ifndef track_H
#define track_H
#include "opencv2/core/core.hpp"
#include <vector>

using namespace std;

// tracks for maintaining different observed objects
class track{

public:
	cv::Point2f predictedPosition;		// predicted position in pixel
	cv::Point2f predictedPositionmm; 	// predicted position in mm	
	cv::Point2f predictedVelocitymm;	// predicted velocity in mm/s
    cv::KalmanFilter KF;				// filter for this track

	cv::Rect bbox;						// bounding box for new detection in mm
    double heading;                     // heading in rad
    int category;                       // category

    int id;
    int age;							// number of frames since starting
    int totalVisibleCount;				// total frames of exsisting
    int consecutiveInvisibleCount; 		// frames number of being consecutive invisible 

    vector<cv::Point2f> trajectory; 	// record the positions back for maximum N frames
    int Nm;
};

#endif
