/*
 * LineSampler.h
 *
 *  Created on: Aug 9, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef LINESAMPLER_H_
#define LINESAMPLER_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "IHDetector.h"
#include "FieldModel.h"
#include <map>

namespace tigers
{

enum SampleType
{
	GOOD_FEATURES,
	HOUGH_LINE,
	THRESHOLD
};

class LineSampler : public IHDetector
{
public:
	LineSampler(FieldModel* fieldModel);
	virtual ~LineSampler();
	SampleType sampleType = THRESHOLD;

	void findTransformation(cv::Mat& src, cv::Mat& imgDst,
			std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	void showControls(cv::TrackbarCallback callBack, void* ref);

private:
	FieldModel* fieldModel = NULL;
	// possible border line intersections (all 4 corners + 2 points on left/right lines)
	std::vector<cv::Point2f> fieldIntersections;
	// combinations for the intersections to be used for calc intersection points
	std::vector<cv::Point2i> comps;
	std::map<int, std::vector<int>> intersecMapping;
	std::vector<cv::Scalar> meanColors;
	cv::Scalar meanColor;
	std::vector<std::vector<cv::Point2f>> intersectionBuffer;
	std::vector<cv::Point2f> lastIntersections;

	void sample(cv::Mat& src, cv::Point lStart, cv::Point lEnd, std::vector<cv::Point2f>& samples, cv::Mat* imgDst = NULL);
	void getIntersections(std::vector<cv::Vec4f> &lines, cv::Rect fieldRect, std::vector<cv::Point2f> &intersections);
};

} /* namespace tigers */

#endif /* LINESAMPLER_H_ */
