/*
 * Transformer.cpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "Transformer.h"
#include "FieldModel.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "opencv_compat.h"

using namespace std;
using namespace cv;

namespace tigers
{

Transformer::Transformer()
{
	cameraMatrix = cv::Mat::eye(Size(3, 3), CV_32F);
}

Transformer::~Transformer()
{
}

void Transformer::calibrate(cv::Mat& src, cv::Mat& H,
		std::vector<cv::Point2f>& modelBots, FieldModel& fieldModel)
{
	this->H = H;

	vector<cv::Point2f> corners;
	fieldModel.getCorners(corners);
	modelBots = corners;

	if(modelBots.size() < 4)
	{
		return;
	}

	// generate 3d points from model
	vector<vector<Point3f>> objectPoints(1);
	for (int i = 0; i < modelBots.size(); i++)
	{
		objectPoints[0].push_back(Point3f(modelBots[i].x, modelBots[i].y, 0));
	}

	// transform points to image with known H matrix
	vector<vector<Point2f>> imagePointsTrans2d(1);
	perspectiveTransform(modelBots, imagePointsTrans2d[0], H);

	// get intrinsic and extrinsic params
	if (first)
	{
		calibrateCamera(objectPoints, imagePointsTrans2d,
				Size(src.cols, src.rows), cameraMatrix, distCoeffs, rvecs,
				tvecs);
		first = false;
	}
	else
	{
		try
		{
			calibrateCamera(objectPoints, imagePointsTrans2d,
					Size(src.cols, src.rows), cameraMatrix, distCoeffs, rvecs,
					tvecs,
					CV_CALIB_USE_INTRINSIC_GUESS);
		} catch (cv::Exception& e)
		{
			cerr << "Exception during calibrateCamera()" << e.what() << endl;
			calibrateCamera(objectPoints, imagePointsTrans2d,
					Size(src.cols, src.rows), cameraMatrix, distCoeffs, rvecs,
					tvecs);
		}
	}
	// adapt object points in height
	for (int i = 0; i < modelBots.size(); i++)
	{
		objectPoints[0][i].z = 150;
	}

	// transform 3d points into image points
	vector<Point2f> imagePointsTrans3d;
	projectPoints(objectPoints[0], rvecs[0], tvecs[0], cameraMatrix, distCoeffs,
			imagePointsTrans3d);
}

void Transformer::transform(std::vector<cv::Point3f>& in,
		std::vector<cv::Point2f>& out)
{
	projectPoints(in, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, out);
}

} /* namespace tigers */
