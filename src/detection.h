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
#include "opencv2/core/core.hpp"

// detections of a single frame for multiple blobs

class detection{
public:
	cv::Point2f centroid; 			// centroids of detections in pixel
	cv::Rect  bbox;					// bounding box of detections in pixel
};


#endif
