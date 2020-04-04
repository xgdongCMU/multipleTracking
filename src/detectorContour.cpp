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

 #include "mag_tracking/detectorContour.h"


detectorContour::detectorContour(paramContour params)
{
	this->params = params;
}

detectorContour::~detectorContour()
{

}

void detectorContour::detect(cv::Mat &frame, vector<cv::Rect> &boundRects, vector<double> &angles, vector<cv::Point> &positions, vector<int> &categories)
{
	// preprocessing
    cv::Mat bw;
    vector<vector<Point> > contours;
    int N;
    preprocessing(frame, bw);
	  
    // find contour
    detectOuterContour(bw,contours);
	  
    // find position and orientation uisng PCA
    for(int i =0; i < contours.size(); i ++)
    {
	    double angle;
    	cv::Rect boundRect;
    	cv::Point position;
      int category;
  		if(detectOrientation(contours[i], boundRect, angle, position, category))
  		{	
  			boundRects.push_back(boundRect);
  			angles.push_back(angle);
  			positions.push_back(position);
        categories.push_back(category);
  		}
  	}

	// find category


}

void detectorContour::preprocessing(cv::Mat &frame, cv::Mat &bw)
{
    cv::blur(frame, bw, cv::Size(params.blursize, params.blursize));
    cv::Canny(bw, bw, params.minThreshold, params.maxThreshold, 3);
}

void detectorContour::detectOuterContour(cv::Mat &bw, vector< vector<Point> > &detectedContours)
{
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    
    findContours( bw, contours, hierarchy, RETR_EXTERNAL , CV_CHAIN_APPROX_NONE); 

    // filter contour by area
    for(int i =0; i < contours.size(); i++)
    {
      double area = cv::contourArea(contours[i]);
        if( area >  params.minArea && area < params.maxArea)
        {
          detectedContours.push_back(contours[i]);
        }   
    }    

    if(!detectedContours.empty())
    {
      std::sort(detectedContours.begin(), detectedContours.end(), compareContourAreas);
      //cout << "N = " << N << endl;
    }
}


bool detectorContour::detectOrientation(vector<Point> &pts, cv::Rect &boundRect, double &angle, cv::Point &pos, int &vertices)
{
	if (pts.size() == 0) return false;

	/// Construct a buffer used by the pca analysis
	Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
	for (int i = 0; i < data_pts.rows; i++)
	{
		data_pts.at<double>(i, 0) = pts[i].x;
		data_pts.at<double>(i, 1) = pts[i].y;
	}

	/// Perform PCA analysis
	PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
	pos = Point(pca_analysis.mean.at<double>(0, 0),pca_analysis.mean.at<double>(0, 1));

	/// Store the eigenvalues and eigenvectors
	vector<Point2d> eigen_vecs(2);
	vector<double> eigen_val(2);
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
	}

	//
	vector<cv::Point> contours_poly;
	//cv::Rect boundRect;
	//cv::Point2f center;
	//float radius;
	cv::approxPolyDP( Mat(pts), contours_poly, 3, true );
	boundRect = boundingRect( Mat(contours_poly) );
	//minEnclosingCircle( (Mat)contours_poly, center, radius);  

  vertices = contours_poly.size();
	angle = atan2(-eigen_vecs[0].y, -eigen_vecs[0].x);
	return true;

}

static bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 )
{
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}
