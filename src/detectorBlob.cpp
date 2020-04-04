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
#include "mag_tracking/detectorBlob.h"

detectorBlob::detectorBlob(paramBase paramsB)
{
        
        cv::SimpleBlobDetector::Params params;
        // Change thresholds
        params.minThreshold = paramsB.minThreshold;
        params.maxThreshold = paramsB.maxThreshold;

        // Filter by Area.
        params.filterByArea = paramsB.filterByArea;
        params.minArea = paramsB.minArea;

        // Filter by Circularity
        params.filterByCircularity = paramsB.filterByCircularity;
        params.minCircularity = paramsB.minCircularity;

        // Filter by Convexity
        params.filterByConvexity = paramsB.filterByConvexity;
        params.minConvexity = paramsB.minConvexity;

        // Filter by Inertia
        params.filterByInertia = paramsB.filterByInertia;
        params.minInertiaRatio = paramsB.minInertiaRatio;
        
	blobDetector = cv::SimpleBlobDetector(params);

}

detectorBlob::~detectorBlob()
{


}

vector <detection> detectorBlob::detect(cv::Mat frame)
{

        vector<cv::KeyPoint> keypoints;
        blobDetector.detect(frame, keypoints);

        vector<struct detection> detections;
        detection temp;
        
        // drawKeypoints
        drawKeypoints( frame, keypoints, mask, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // extract centroid and bbox
        for(int i =0; i < keypoints.size();i ++)
        {
                temp.centroid   = keypoints[i].pt;
                temp.bbox       = cv::Rect(temp.centroid.x,temp.centroid.y,keypoints[i].size,keypoints[i].size);                        
                detections.push_back(temp);

        }


        return detections;
}