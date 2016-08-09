/*
 * TransformationCalibration.h
 *
 *  Created on: Aug 6, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef TRANSFORMATIONCALIBRATION_H_
#define TRANSFORMATIONCALIBRATION_H_

#include <string>
#include <opencv2/imgproc/imgproc.hpp>

namespace tigers
{

class TransformationCalibration
{
public:
	TransformationCalibration(float fieldWidth, float fieldLength);
	virtual ~TransformationCalibration();
	void start(std::string window_image);
	cv::Mat* H = NULL;

	int xId = 1;
	int yId = 1;
	float fieldWidth;
	float fieldLength;
	std::string window_image;
	std::vector<cv::Point2f> imgPoints;
	std::vector<cv::Point2f> modelPoints;

	float getCurModelX();
	float getCurModelY();
};

} /* namespace tigers */

#endif /* TRANSFORMATIONCALIBRATION_H_ */
