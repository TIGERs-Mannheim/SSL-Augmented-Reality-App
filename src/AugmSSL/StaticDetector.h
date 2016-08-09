/*
 * StaticDetector.h
 *
 *  Created on: Aug 4, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef STATICDETECTOR_H_
#define STATICDETECTOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "IHDetector.h"

namespace tigers
{

class StaticDetector : public IHDetector
{
public:
	cv::Mat H;

	StaticDetector(cv::Mat& H);
	virtual ~StaticDetector();

	void findTransformation(cv::Mat& src, cv::Mat& imgDst,
			std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	void showControls(cv::TrackbarCallback callBack, void* ref);
};

} /* namespace tigers */

#endif /* STATICDETECTOR_H_ */
