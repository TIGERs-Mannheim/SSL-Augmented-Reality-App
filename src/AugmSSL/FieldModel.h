/*
 * FieldModel.h
 *
 *  Created on: Aug 4, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef FIELDMODEL_H_
#define FIELDMODEL_H_

#include "FieldCorner.h"

namespace tigers
{

class FieldModel
{
public:
	std::vector<FieldCorner*> corners;
	cv::Rect field;
	std::vector<cv::Point2f> fieldLines;
	float width, length;

	FieldModel(float width, float length);
	virtual ~FieldModel();

	void draw(cv::Mat& H, cv::Mat& imgDst);
	std::vector<cv::Vec4f> getBorderLines();
	std::vector<cv::Vec4f> getBorderLinesTransf(cv::Mat& H);
	void getCorners(std::vector<cv::Point2f>& points);
};

} /* namespace tigers */

#endif /* FIELDMODEL_H_ */
