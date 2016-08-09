/*
 * BotDetector.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "BotDetector.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <assert.h>
#include "opencv_compat.h"

using namespace cv;
using namespace std;

namespace tigers
{

BotDetector::BotDetector()
{
}

BotDetector::~BotDetector()
{
}

bool pointComparator(Point2f p1, Point2f p2)
{
	if (fabsf(p1.y - p2.y) < 1)
	{
		return p1.x < p2.x;
	}
	return (p1.y < p2.y);
}

void BotDetector::showControls(TrackbarCallback callBack, void* ref)
{
	string window_params = "Parameters BotDetector";
	namedWindow(window_params, WINDOW_NORMAL);
	createTrackbar("LowH", window_params, &iLowH, 179, callBack, ref); //Hue (0 - 179)
	createTrackbar("HighH", window_params, &iHighH, 179, callBack, ref);
	createTrackbar("LowS", window_params, &iLowS, 255, callBack, ref); //Saturation (0 - 255)
	createTrackbar("HighS", window_params, &iHighS, 255, callBack, ref);
	createTrackbar("LowV", window_params, &iLowV, 255, callBack, ref); //Value (0 - 255)
	createTrackbar("HighV", window_params, &iHighV, 255, callBack, ref);

	createTrackbar("blobThreshMin", window_params, &blobThreshMin, 300,
			callBack, ref);
	createTrackbar("blobThreshMax", window_params, &blobThreshMax, 300,
			callBack, ref);
	createTrackbar("areaEqTol", window_params, &areaEqTol, 1500, callBack, ref);
}

float BotDetector::recFindTransform(vector<Point2f>& bots,
		vector<Point2f>& transBots, Mat& H)
{
	if (bots.size() == transBots.size())
	{
//		H = findHomography(transBots, bots, RANSAC);
		H = findHomography(transBots, bots, 0);
		return calcTransformationError(H, bots, transBots);
	}
	Mat bestH(Size(3, 3), CV_32F);
	float bestError = INFINITY;
	for (int i = 0; i < transBots.size(); i++)
	{
		vector<Point2f> nTransBots;
		for (int j = 0; j < transBots.size(); j++)
		{
			if (j != i)
			{
				nTransBots.push_back(transBots.at(j));
			}
		}
		float error = recFindTransform(bots, nTransBots, H);
		if (error < 10)
		{
			return error;
		}
		if (bestError > error)
		{
			bestError = error;
			bestH = H;
		}
	}
	H = bestH;
	return bestError;
}

float BotDetector::findTransTryRotations(vector<Point2f>& bots,
		vector<Point2f>& modelBots, Mat& H)
{
	vector<Point2f> transBots;
	float minError = INFINITY;
	for (float r = 0; r < M_PI; r += 0.5)
	{
		float fixedH[9] =
		{ cos(r), -sin(r), 0, sin(r), cos(r), 0, 0, 0, 1 };
		Mat nH(Size(3, 3), CV_32F, fixedH);
		perspectiveTransform(modelBots, transBots, nH);
		sort(transBots.begin(), transBots.end(), pointComparator);
		float error = recFindTransform(bots, transBots, nH);
		if (error < minError)
		{
			minError = error;
			H = nH;
		}
	}
	return minError;
}

float BotDetector::findTransBruteForce(vector<Point2f>& bots,
		vector<Point2f>& modelBots, Mat& H)
{
	sort(modelBots.begin(), modelBots.end(), pointComparator);

	int numAddBotsModel = modelBots.size() - bots.size();
	float error = 0;
	float minError = INFINITY;
	std::vector<Point2f>::iterator lastEnd = modelBots.end();
	vector<Point2f> lastBots;
	do
	{
		// TODO we assume there are more bots in model than detected atm...
		vector<Point2f> bots2(modelBots.begin(),
				modelBots.end() - numAddBotsModel);
//		for(int i=0;i<modelBots.size();i++)
//		{
//			cout << modelBots.at(i) << " ";
//		}
//		cout << endl;

		if (lastBots != bots2)
		{
			//			Mat nH = findHomography(bots, modelBots, RANSAC);
			Mat nH = findHomography(bots2, bots, 0);
			error = calcTransformationError(nH, bots, bots2);
			if (error < minError)
			{
				minError = error;
				H = nH;
				cout << "minError=" << minError << endl;
//				for (int i = 0; i < bots2.size(); i++)
//				{
//					cout << bots2.at(i) << " ";
//				}
//				cout << endl;
//				for (int i = 0; i < bots.size(); i++)
//				{
//					cout << bots.at(i) << " ";
//				}
//				cout << endl;
			}

//			cout << error << endl;
			if (error < 30)
			{
				break;
			}
		}
		lastBots = bots2;

	} while (next_permutation(modelBots.begin(), modelBots.end(),
			pointComparator));
	return minError;
}

void BotDetector::findTransformation(Mat& src, Mat& imgDst,
		vector<Point2f>& modelBots, Mat& H)
{
	vector<Point2f> bots = detect(src, imgDst);
	if (bots.size() < 3)
	{
		cout << "Not enough bots detected! (" << bots.size() << ")" << endl;
		return;
	}

	vector<Point2f> bots2;
	bots2.push_back(bots[0]);
	bots2.push_back(bots[4]);
	bots2.push_back(bots[5]);
	bots2.push_back(bots[6]);
//	bots2.push_back(bots[4]);
	bots = bots2;
//	while (bots.size() > 6)
//	{
//		bots.pop_back();
//	}

	float error = findTransBruteForce(bots, modelBots, H);
//	float error = findTransTryRotations(bots, modelBots, H);
	cout << "error=" << error << endl << H << endl;
}

float BotDetector::calcTransformationError(Mat& H, vector<Point2f>& bots,
		vector<Point2f>& modelBots)
{
	assert(bots.size() == modelBots.size());
	vector<Point2f> transBots;
	perspectiveTransform(modelBots, transBots, H);
	float sum = 0;
	for (int i = 0; i < bots.size(); i++)
	{
		float dist = norm(bots.at(i) - transBots.at(i));
		sum += dist;
	}
	return sum;
}

std::vector<Point2f> BotDetector::detect(Mat& img, Mat& drawing)
{
	std::vector<Point2f> scenePoints;
	Mat imgThresholded = threshold(img);
	scenePoints = contours(imgThresholded, drawing);
	return scenePoints;
}

Mat BotDetector::threshold(Mat& imgOriginal)
{
	Mat imgHSV, imgThresholded;
	//Convert the captured frame from BGR to HSV
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

	//Threshold the image
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),
			imgThresholded);

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

//	imshow("thres", imgThresholded);

	return imgThresholded;
}

std::vector<Point2f> BotDetector::contours(Mat& imgThresholded, Mat& drawing)
{
	std::vector<Point2f> scenePoints;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_CCOMP,
			CHAIN_APPROX_SIMPLE, Point(0, 0));

	RotatedRect rotRecs[contours.size()];

	double maxArea = 0;
	int maxAreaContour = 0;
	vector<int> filteredContours;
	for (size_t i = 0; i < contours.size(); i++)
	{
		Scalar color = Scalar(250, 250, 0);
		if (hierarchy[i][3] < 0)
		{
			color = Scalar(0, 250, 250);
		}
		if (drawing.cols > 0)
		{
			drawContours(drawing, contours, (int) i, color, 1, 8, hierarchy, 0,
					Point());
		}

		if (contours[i].size() > 4)
		{
			rotRecs[i] = minAreaRect(contours[i]);
			Rect brect = rotRecs[i].boundingRect();
			double area = contourArea(contours[i]);
			double diff = (brect.area() - area);
			double areaEq = diff / ((area + brect.area()) / 2.0);
			if (brect.width < blobThreshMax && brect.height < blobThreshMax
					&& brect.width > blobThreshMin
					&& brect.height > blobThreshMin
					&& areaEq < (areaEqTol / 1000.0) && hierarchy[i][3] >= 0)
			{
				filteredContours.push_back(i);
			}

			if (area > maxArea)
			{
				maxArea = area;
				maxAreaContour = i;
			}
		}
	}

	for (int j = 0; j < filteredContours.size(); j++)
	{
		int i = filteredContours[j];
		// contour must be child of contour with max area
		if (hierarchy[i][3] != maxAreaContour)
		{
			continue;
		}
		Rect brect = rotRecs[i].boundingRect();
		scenePoints.push_back(
				Point2d(brect.x + brect.width / 2.0,
						brect.y + 3.0 * brect.height / 4.0));

		if (drawing.cols > 0)
		{
			rectangle(drawing, brect, Scalar(250, 0, 250));
			stringstream ss;
			string s;
			ss << scenePoints.size();
			ss >> s;
			putText(drawing, s,
					Point(scenePoints.back().x, scenePoints.back().y),
					FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 250));
		}
	}
	return scenePoints;
}

} /* namespace tigers */
