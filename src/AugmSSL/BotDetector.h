/*
 * BotDetector.h
 *
 *  Created on: Jul 8, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef BOTDETECTOR_H_
#define BOTDETECTOR_H_

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "IHDetector.h"

namespace tigers
{

class BotDetector : public IHDetector
{
public:
	BotDetector();
	virtual ~BotDetector();

	int iLowH = 0;
	int iHighH = 179;
	int iLowS = 11;
	int iHighS = 255;
	int iLowV = 91;
	int iHighV = 255;

	int blobThreshMin = 24;
	int blobThreshMax = 200;
	int areaEqTol = 700;

	void showControls(cv::TrackbarCallback callBack, void* ref);
	float recFindTransform(std::vector<cv::Point2f>& bots, std::vector<cv::Point2f>& transBots, cv::Mat& H);
	float findTransTryRotations(std::vector<cv::Point2f>& bots, std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	float findTransBruteForce(std::vector<cv::Point2f>& bots, std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	std::vector<cv::Point2f> detect(cv::Mat& img, cv::Mat& drawing);
	std::vector<cv::Point2f> contours(cv::Mat& imgThresholded, cv::Mat& drawing);
	cv::Mat threshold(cv::Mat& imgOriginal);
	void findTransformation(cv::Mat& src, cv::Mat& imgDst, std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	float calcTransformationError(cv::Mat& H, std::vector<cv::Point2f>& bots, std::vector<cv::Point2f>& modelBots);

};

} /* namespace tigers */

#endif /* BOTDETECTOR_H_ */
