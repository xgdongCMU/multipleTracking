#ifndef detectorBackgroundSubtraction_H
#define detectorBackgroundSubtraction_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "detection.h"


using namespace std;
using namespace cv;
class detectorBackgroundSubtraction{

public:
	cv::Mat mask;
	cv::BackgroundSubtractorMOG2 bg_model;
	detectorBackgroundSubtraction();
	~detectorBackgroundSubtraction();
	vector<detection> detect(cv::Mat frame);


};

#endif