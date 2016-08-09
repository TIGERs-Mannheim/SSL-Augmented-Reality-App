/*
 * GeoUtil.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "GeoUtil.h"

using namespace std;
using namespace cv;

namespace tigers
{

GeoUtil::GeoUtil()
{
}

GeoUtil::~GeoUtil()
{
}

float GeoUtil::computeAngle(cv::Vec4i a, cv::Vec4i b)
{
	// delta_y / delta_x
	cv::Vec4i va((a[0] - a[2]), (a[1] - a[3]));
	cv::Vec4i vb((b[0] - b[2]), (b[1] - b[3]));
	return acosf(
			(float) (va[0] * vb[0] + va[1] * vb[1])
					/ (sqrt(va[0] * va[0] + va[1] * va[1])
							* sqrt(vb[0] * vb[0] + vb[1] * vb[1])));
}

float GeoUtil::computeAngleVV(cv::Point2f v1, cv::Point2f v2)
{
	float a = (v1.x * v2.x + v1.y * v2.y);
	float v1l = sqrt(v1.x * v1.x + v1.y * v1.y);
	float v2l = sqrt(v2.x * v2.x + v2.y * v2.y);
	float b = v1l * v2l;
	float c = a / b;
	return acosf(c);
}

bool GeoUtil::comparePoints(cv::Point2d p1, cv::Point2d p2)
{
	if ((int) p1.x == (int) p2.x)
		return p1.y < p2.y;
	return p1.x < p2.x;
}

bool GeoUtil::pointBetweenPoints(std::vector<cv::Point2f> corners, int pId,
		double tolerance)
{
	for (int j = 0; j < corners.size(); j++)
	{
		if (pId == j)
		{
			continue;
		}
		for (int k = j + 1; k < corners.size(); k++)
		{
			if (pId == k)
			{
				continue;
			}
			cv::Rect rect(corners[j], corners[k]);
			if (rect.contains(corners[pId]))
			{
				double dist = distance_to_Line(corners[j], corners[k],
						corners[pId]);
				if (fabsf(dist) < tolerance)
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool GeoUtil::intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2,
		cv::Point2f p2, cv::Point2f &r)
{
	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = p1 - o1;
	cv::Point2f d2 = p2 - o2;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

void GeoUtil::intersectionRectLine(cv::Rect rect, cv::Vec4f line,
		std::vector<cv::Point2f>& intersections)
{
	Point2f edges[4];
	edges[0] = Point2f(rect.x + 1, rect.y + 1);
	edges[1] = Point2f(rect.x + rect.width - 1, rect.y + 1);
	edges[2] = Point2f(rect.x + rect.width - 1, rect.y + rect.height - 1);
	edges[3] = Point2f(rect.x + 1, rect.y + rect.height - 1);
	Point2f lineP1(line[2], line[3]);
	Point2f lineP2 = lineP1 + 10 * Point2f(line[0], line[1]);
	for (int i = 0; i < 4; i++)
	{
		Point2f intersec;
		bool intersect = GeoUtil::intersection(edges[i], edges[(i + 1) % 4],
				lineP1, lineP2, intersec);
		if (intersect && rect.contains(intersec))
		{
			intersections.push_back(intersec);
		}
	}
}

double GeoUtil::distance_to_Line(cv::Point2f line_start, cv::Point2f line_end,
		cv::Point2f point)
{
	double normalLength = hypot(line_end.x - line_start.x,
			line_end.y - line_start.y);
	double distance = (double) ((point.x - line_start.x)
			* (line_end.y - line_start.y)
			- (point.y - line_start.y) * (line_end.x - line_start.x))
			/ normalLength;
	return fabsf(distance);
}

void GeoUtil::drawLine(cv::Mat& imgDst, cv::Vec4f l, const cv::Scalar& color,
		int thickness = 1)
{
	float m = (float) l[1] / (l[0] + 1e-10f);
	float b = l[3] - m * l[2];
	float x0 = 0;
	float x1 = (imgDst.cols - 1);
	float y0 = b;
	float y1 = m * x1 + b;
	cv::line(imgDst, cv::Point(x0, y0), cv::Point(x1, y1), color, thickness);
}

} /* namespace tigers */
