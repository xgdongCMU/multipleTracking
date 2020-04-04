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
#ifndef detectorBlob_H
#define detectorBlob_H

#include "mag_tracking/detection.h"

/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
*/

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace cv;

class detectorBlob{

public:
	cv::Mat mask;
	cv::SimpleBlobDetector blobDetector;
	detectorBlob(paramBase params);
	~detectorBlob();
	vector<detection> detect(cv::Mat frame);


};

#endif
