#include "detectorBlob.h"

detectorBlob::detectorBlob(cv::SimpleBlobDetector::Params params)
{
        /*
        cv::SimpleBlobDetector::Params params;
        // Change thresholds
        params.minThreshold = 50;
        params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 100;

        // Filter by Circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;
        */
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