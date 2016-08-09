/*
 * Transformer.h
 *
 *  Created on: Sep 15, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef TRANSFORMER_H_
#define TRANSFORMER_H_

#include <opencv2/imgproc/imgproc.hpp>
#include "FieldModel.h"

namespace tigers
{

class Transformer
{
public:
	Transformer();
	virtual ~Transformer();
	cv::Mat H;

	void calibrate(cv::Mat &img, cv::Mat &H,
			std::vector<cv::Point2f>& modelBots, FieldModel& fieldModel);
	void transform(std::vector<cv::Point3f>& in, std::vector<cv::Point2f>& out);

private:
	bool first = true;
	cv::Mat cameraMatrix;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::Mat distCoeffs;
};

} /* namespace tigers */

#endif /* TRANSFORMER_H_ */
