/*
 * GeoUtil.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef GEOUTIL_H_
#define GEOUTIL_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace tigers
{

class GeoUtil
{
public:
	GeoUtil();
	virtual ~GeoUtil();
	static float computeAngle(cv::Vec4i a, cv::Vec4i b);
	static float computeAngleVV(cv::Point2f v1, cv::Point2f v2);
	static bool comparePoints(cv::Point2d p1, cv::Point2d p2);
	static bool pointBetweenPoints(std::vector<cv::Point2f> corners, int pId, double tolerance);
	static bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r);
	static void intersectionRectLine(cv::Rect rect, cv::Vec4f line,
			std::vector<cv::Point2f>& intersections);
	static double distance_to_Line(cv::Point2f line_start, cv::Point2f line_end,
			cv::Point2f point);
	static void drawLine(cv::Mat& imgDst, cv::Vec4f l, const cv::Scalar& color, int thickness);
};

} /* namespace tigers */

#endif /* GEOUTIL_H_ */
