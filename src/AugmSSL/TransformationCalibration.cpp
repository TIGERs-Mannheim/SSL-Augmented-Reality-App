/*
 * TransformationCalibration.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "TransformationCalibration.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <string>
#include <iostream>

#include "FieldModel.h"

using namespace std;
using namespace cv;

namespace tigers
{

TransformationCalibration::TransformationCalibration(float fieldWidth,
		float fieldLength)
{
	this->fieldLength = fieldLength;
	this->fieldWidth = fieldWidth;
}

TransformationCalibration::~TransformationCalibration()
{
}

float TransformationCalibration::getCurModelX()
{
	switch (xId)
	{
	case 0:
		return -fieldLength;
	case 1:
		return 0;
	case 2:
		return fieldLength;
	}
}

float TransformationCalibration::getCurModelY()
{
	switch (yId)
	{
	case 0:
		return -fieldWidth;
	case 1:
		return 0;
	case 2:
		return fieldWidth;
	}
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	TransformationCalibration* tc;
	tc = (TransformationCalibration*) userdata;
	if (event == EVENT_LBUTTONDOWN)
	{
		int idx = -1;
		for (int i = 0; i < tc->modelPoints.size(); i++)
		{
			if (fabsf(tc->modelPoints[i].x - tc->getCurModelX()) < 1e-6
					&& fabsf(tc->modelPoints[i].y - tc->getCurModelY()) < 1e-6)
			{
				idx = i;
				break;
			}
		}
		if (idx == -1)
		{
			tc->modelPoints.push_back(
					Point2f(tc->getCurModelX(), tc->getCurModelY()));
			tc->imgPoints.push_back(Point2f());
			idx = tc->modelPoints.size() - 1;
		}
		tc->imgPoints[idx].x = x;
		tc->imgPoints[idx].y = y;

		cout << "img " << tc->imgPoints << endl;
		cout << "mod " << tc->modelPoints << endl;

		if (tc->imgPoints.size() == 4)
		{
			if (tc->H == NULL)
			{
				tc->H = new Mat;
			}
			*tc->H = findHomography(tc->modelPoints, tc->imgPoints, 0);

			cout << "H=" << endl << *(tc->H) << endl;

			setMouseCallback(tc->window_image, NULL, NULL);
		}
	}
}

void TransformationCalibration::start(string window_image)
{
	modelPoints.clear();
	imgPoints.clear();
	this->window_image = window_image;
	string window_params = "transformation calibration";
	namedWindow(window_params, WINDOW_NORMAL);
	createTrackbar("x - Negative, Zero, Positive", window_params, &xId, 2);
	createTrackbar("y - Negative, Zero, Positive", window_params, &yId, 2);

	setMouseCallback(window_image, &CallBackFunc, this);
}

} /* namespace tigers */

using namespace tigers;

int main(int argc, char **argv)
{
	VideoCapture cap("data/Parsian_p2_20140723_113034.cut.mp4");
	Mat img;
	cap.read(img);
	string window_image = "image";
	namedWindow(window_image, WINDOW_NORMAL);
	cv::imshow(window_image, img);

	float fixedDefH[9] =
	{ .2, 0, 600, 0, .2, 400, 0, 0, 1 };
	Mat H = Mat(Size(3, 3), CV_32F, fixedDefH);

	FieldModel fieldModel(4450, 6550);
	TransformationCalibration tc(fieldModel.width / 2, fieldModel.length / 2);
	tc.H = &H;
	tc.start(window_image);
	int key;
	while ((key = waitKey()))
	{
		Mat imgDest = img.clone();
		fieldModel.draw(H, imgDest);
		cv::imshow(window_image, imgDest);
		if (key == 27) //wait for 'esc' key press
			exit(0);
	}
}
