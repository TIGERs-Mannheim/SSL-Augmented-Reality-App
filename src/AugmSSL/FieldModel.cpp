/*
 * FieldModel.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "FieldModel.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

namespace tigers
{

FieldModel::FieldModel(float width, float length)
{
	this->width = width;
	this->length = length;
	const int numCorners = 11;
	FieldCorner* corners[numCorners];
	for (int i = 0; i < numCorners; i++)
	{
		corners[i] = new FieldCorner;
	}
	// only interested in half the sizes (from zero point to outside)
	width /= 2;
	length /= 2;

	field = Rect(Point2f(-length, -width), Point2f(length, width));

	// even: first point on line, odd: second pt on line
	fieldLines.push_back(Point2f(-length, -width));
	fieldLines.push_back(Point2f(-length, width));

	fieldLines.push_back(Point2f(0, -width));
	fieldLines.push_back(Point2f(0, width));

	fieldLines.push_back(Point2f(length, -width));
	fieldLines.push_back(Point2f(length, width));

	fieldLines.push_back(Point2f(-length, -width));
	fieldLines.push_back(Point2f(length, -width));

	fieldLines.push_back(Point2f(-length, width));
	fieldLines.push_back(Point2f(length, width));

	fieldLines.push_back(Point2f(-length, 0));
	fieldLines.push_back(Point2f(length, 0));

	corners[0]->point = Point2f(-length, -width);
	corners[1]->point = Point2f(-length, 0.0f);
	corners[2]->point = Point2f(-length, width);
	corners[3]->point = Point2f(0.0f, -width);
	corners[4]->point = Point2f(0.0f, 0.0f);
	corners[5]->point = Point2f(0.0f, width);
	corners[6]->point = Point2f(length, -width);
	corners[7]->point = Point2f(length, 0.0f);
	corners[8]->point = Point2f(length, width);
	corners[9]->point = Point2f(2200.0f, 0.0f);
	corners[10]->point = Point2f(-2200.0f, 0.0f);

	corners[0]->insert_Neighbor(corners[1]);
	//	corners[0]->insert_Neighbor(corners[2]);
	corners[0]->insert_Neighbor(corners[3]);
	//	corners[0]->insert_Neighbor(corners[6]);

	corners[1]->insert_Neighbor(corners[0]);
	corners[1]->insert_Neighbor(corners[2]);
	corners[1]->insert_Neighbor(corners[4]);
	//	corners[1]->insert_Neighbor(corners[7]);

	//	corners[2]->insert_Neighbor(corners[0]);
	corners[2]->insert_Neighbor(corners[1]);
	corners[2]->insert_Neighbor(corners[5]);
	//	corners[2]->insert_Neighbor(corners[8]);

	corners[3]->insert_Neighbor(corners[0]);
	corners[3]->insert_Neighbor(corners[4]);
	//	corners[3]->insert_Neighbor(corners[5]);
	corners[3]->insert_Neighbor(corners[6]);

	corners[4]->insert_Neighbor(corners[1]);
	corners[4]->insert_Neighbor(corners[3]);
	corners[4]->insert_Neighbor(corners[5]);
	corners[4]->insert_Neighbor(corners[7]);

	corners[5]->insert_Neighbor(corners[2]);
	//	corners[5]->insert_Neighbor(corners[3]);
	corners[5]->insert_Neighbor(corners[4]);
	corners[5]->insert_Neighbor(corners[8]);

	//	corners[6]->insert_Neighbor(corners[0]);
	corners[6]->insert_Neighbor(corners[3]);
	corners[6]->insert_Neighbor(corners[7]);
	//	corners[6]->insert_Neighbor(corners[8]);

	//	corners[7]->insert_Neighbor(corners[1]);
	corners[7]->insert_Neighbor(corners[4]);
	corners[7]->insert_Neighbor(corners[6]);
	corners[7]->insert_Neighbor(corners[8]);

	//	corners[8]->insert_Neighbor(corners[2]);
	corners[8]->insert_Neighbor(corners[5]);
	//	corners[8]->insert_Neighbor(corners[6]);
	corners[8]->insert_Neighbor(corners[7]);

	// penalty line crossing
	corners[9]->insert_Neighbor(corners[1]);
	corners[9]->insert_Neighbor(corners[4]);
	corners[10]->insert_Neighbor(corners[4]);
	corners[10]->insert_Neighbor(corners[7]);
	corners[1]->insert_Neighbor(corners[9]);
	corners[4]->insert_Neighbor(corners[9]);
	corners[4]->insert_Neighbor(corners[10]);
	corners[7]->insert_Neighbor(corners[10]);

	this->corners = vector<FieldCorner*>(corners,
			corners + sizeof corners / sizeof corners[0]);
}

FieldModel::~FieldModel()
{
}

void FieldModel::draw(cv::Mat& H, cv::Mat& imgDst)
{
	// draw center circle
	vector<Point2f> centerCircle;
	for (float alpha = 0; alpha < 2 * CV_PI; alpha += .2)
	{
		centerCircle.push_back(Point2f(500 * cos(alpha), 500 * sin(alpha)));
	}
	vector<Point2f> centerCircleTrans;
	perspectiveTransform(centerCircle, centerCircleTrans, H);
	for (int i = 0; i < centerCircleTrans.size(); i++)
	{
		line(imgDst, centerCircleTrans[i],
				centerCircleTrans[(i + 1) % centerCircleTrans.size()],
				Scalar(255, 255, 0), 1, 8);
	}

	// draw model coordinates
//	vector<Point2f> edgesPoints;
//	vector<Point2f> edgesTrans;
//	for (int i = 0; i < corners.size(); i++)
//	{
//		edgesPoints.push_back(corners[i]->point);
//	}
//	perspectiveTransform(edgesPoints, edgesTrans, H);
//	for (int i = 0; i < edgesTrans.size(); i++)
//	{
//		std::stringstream ss;
//		ss << corners[i]->point;
//		putText(imgDst, ss.str(), edgesTrans[i], FONT_HERSHEY_PLAIN, 1,
//				Scalar(0, 0, 250));
//	}

	// draw field lines
	if (!fieldLines.empty())
	{
		vector<Point2f> linesTrans;
		perspectiveTransform(fieldLines, linesTrans, H);
		for (int i = 0; i < linesTrans.size(); i += 2)
		{
			line(imgDst, linesTrans.at(i), linesTrans.at(i + 1),
					Scalar(255, 255, 0), 2, 8);
		}
	}
}

vector<cv::Vec4f> FieldModel::getBorderLines()
{
	vector<cv::Vec4f> borderLines(5);
	borderLines[0][0] = -length / 2.0f;
	borderLines[0][1] = -width / 2.0f;
	borderLines[0][2] = length / 2.0f;
	borderLines[0][3] = -width / 2.0f;
	borderLines[1][0] = length / 2.0f;
	borderLines[1][1] = -width / 2.0f;
	borderLines[1][2] = length / 2.0f;
	borderLines[1][3] = width / 2.0f;
	borderLines[2][0] = length / 2.0f;
	borderLines[2][1] = width / 2.0f;
	borderLines[2][2] = -length / 2.0f;
	borderLines[2][3] = width / 2.0f;
	borderLines[3][0] = -length / 2.0f;
	borderLines[3][1] = width / 2.0f;
	borderLines[3][2] = -length / 2.0f;
	borderLines[3][3] = -width / 2.0f;
	borderLines[4][0] = 0;
	borderLines[4][1] = width / 2.0f;
	borderLines[4][2] = 0;
	borderLines[4][3] = -width / 2.0f;

//	vector<cv::Vec4f> borderLines(7);
//	borderLines[0][0] = -length / 2.0f;
//	borderLines[0][1] = -width / 2.0f;
//	borderLines[0][2] = 0;
//	borderLines[0][3] = -width / 2.0f;
//	borderLines[1][0] = length / 2.0f;
//	borderLines[1][1] = -width / 2.0f;
//	borderLines[1][2] = length / 2.0f;
//	borderLines[1][3] = width / 2.0f;
//	borderLines[2][0] = length / 2.0f;
//	borderLines[2][1] = width / 2.0f;
//	borderLines[2][2] = 0;
//	borderLines[2][3] = width / 2.0f;
//	borderLines[3][0] = -length / 2.0f;
//	borderLines[3][1] = width / 2.0f;
//	borderLines[3][2] = -length / 2.0f;
//	borderLines[3][3] = -width / 2.0f;
//	borderLines[4][0] = 0;
//	borderLines[4][1] = width / 2.0f;
//	borderLines[4][2] = 0;
//	borderLines[4][3] = -width / 2.0f;
//	borderLines[5][0] = 0;
//	borderLines[5][1] = -width / 2.0f;
//	borderLines[5][2] = length / 2.0f;
//	borderLines[5][3] = -width / 2.0f;
//	borderLines[6][0] = 0;
//	borderLines[6][1] = width / 2.0f;
//	borderLines[6][2] = -length / 2.0f;
//	borderLines[6][3] = width / 2.0f;
	return borderLines;
}

vector<cv::Vec4f> FieldModel::getBorderLinesTransf(cv::Mat& H)
{
	vector<cv::Vec4f> borderLines = getBorderLines();
	vector<cv::Vec4f> borderLinesTrans;
	vector<Point2f> points, pointsTrans;
	for (int i = 0; i < borderLines.size(); i++)
	{
		points.push_back(Point2f(borderLines[i][0], borderLines[i][1]));
		points.push_back(Point2f(borderLines[i][2], borderLines[i][3]));
	}
	perspectiveTransform(points, pointsTrans, H);
	for (int i = 0; i < pointsTrans.size(); i += 2)
	{
		borderLinesTrans.push_back(
				Vec4f(pointsTrans[i].x, pointsTrans[i].y, pointsTrans[i + 1].x,
						pointsTrans[i + 1].y));
	}
	return borderLinesTrans;
}

void FieldModel::getCorners(vector<cv::Point2f>& points)
{
	for(int i=0;i<corners.size();i++)
	{
		points.push_back(corners.at(i)->point);
	}
}

} /* namespace tigers */
