/*
 * IHDetector.h
 *
 *  Created on: Aug 4, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef IHDETECTOR_H_
#define IHDETECTOR_H_

namespace tigers
{

class IHDetector
{
public:
	virtual void showControls(cv::TrackbarCallback callBack, void* ref) = 0;
	virtual void findTransformation(cv::Mat& src, cv::Mat& imgDst,
			std::vector<cv::Point2f>& modelBots, cv::Mat& H) = 0;
};

} /* namespace tigers */

#endif /* IHDETECTOR_H_ */
