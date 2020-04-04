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
#include "mag_tracking/detectorBase.h"

detectorBase::detectorBase(paramBase paramsB/* param */)
{
        mode  = paramsB.mode;
        if(paramsB.mode == 1)
        {
                paramContour params;
                params.minThreshold = paramsB.minThreshold;
                params.maxThreshold = paramsB.maxThreshold;
                params.blursize = paramsB.blursize;
                params.minArea = paramsB.minArea;
                params.maxArea = paramsB.maxArea;
                params.kernelSize = paramsB.kernelSize;

                detector_contour = new detectorContour(params);

        }
        else
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
        
                detectorBlob = cv::SimpleBlobDetector(params);

        }
}

detectorBase::~detectorBase()
{


}

vector <detection> detectorBase::detect(cv::Mat frame)
{
        vector<detection> detections;
        detection temp;

        if(mode == 1)
        {
                vector<cv::Rect> boundRect;
                vector<double> angles;
                vector<cv::Point> positions;
                vector<int> categories;
                mask = frame.clone();
                detector_contour->detect(mask, boundRect, angles, positions, categories);
                
                for(int i =0; i < angles.size(); i++)
                {
                        cv::rectangle(mask, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 2, 8, 0 );
                        temp.centroid   = positions[i];
                        temp.bbox       = boundRect[i];
                        temp.heading    = 3.14159+angles[i]; 
                        temp.category   = categories[i];                       
                        detections.push_back(temp);

                }
        }
        else
        {

                vector<cv::KeyPoint> keypoints;
                detectorBlob.detect(frame, keypoints);
                
                // drawKeypoints
                drawKeypoints(frame, keypoints, mask, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

                // extract centroid and bbox
                for(int i =0; i < keypoints.size();i ++)
                {
                        temp.centroid   = keypoints[i].pt;
                        temp.bbox       = cv::Rect(temp.centroid.x,temp.centroid.y,keypoints[i].size,keypoints[i].size);  
                        temp.heading    = 0;
                        temp.category   = 0;                       
                        detections.push_back(temp);

                }               
        }

        return detections;
}