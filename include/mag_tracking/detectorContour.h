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
#ifndef detectorContour_H
#define detectorContour_H

/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
*/

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
 
#include "mag_tracking/detection.h"

using namespace std;
using namespace cv;

static bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );

class detectorContour{
public:
	detectorContour(paramContour params);
	~detectorContour();
	void detect(cv::Mat &frame, vector<cv::Rect> &boundRects, vector<double> &angles, vector<cv::Point> &positions, vector<int> &categories);
private:
	// image processing functions
	paramContour params;
	void preprocessing(cv::Mat &frame, cv::Mat &bw);
	void detectOuterContour(cv::Mat &bw, vector< vector<Point> > &contours);
	bool detectOrientation(vector<Point> &pts, cv::Rect &boundRect, double &angle, cv::Point &pos, int &category);

};

#endif

