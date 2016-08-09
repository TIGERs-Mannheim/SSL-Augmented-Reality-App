/*
 * StaticDetector.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "StaticDetector.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace tigers
{

StaticDetector::StaticDetector(cv::Mat& H)
{
	this->H = H;
}

StaticDetector::~StaticDetector()
{
}

void StaticDetector::findTransformation(cv::Mat& src, cv::Mat& imgDst,
			std::vector<cv::Point2f>& modelBots, cv::Mat& H)
{
	H = this->H;
}

void StaticDetector::showControls(cv::TrackbarCallback callBack, void* ref)
{
}

} /* namespace tigers */
