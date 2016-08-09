#include "FieldInit.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <math.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "opencv_compat.h"

using namespace tigers;
using namespace cv;
using namespace std;

static void pullCorner(int event, int x, int y, int flags, void* fieldInit)
{

	FieldInit* fi = (FieldInit*) fieldInit;
	if (event == EVENT_LBUTTONDOWN)
	{
		// std::cout << "maxDist: " << pow(fi->maxDist, 2) << std::endl;
		for (int i = 0; i < fi->corners.size(); i++)
		{
			if (i % 3 == 1)
			{
				continue;
			}
			// std::cout << "corner " << i << " -- Distance^2: " << pow(x-fi->corners.at(i).x, 2) + pow(y-fi->corners.at(i).y,2) << std::endl;
			if (pow(fi->maxDist, 2)
					>= pow(x - fi->corners.at(i).x, 2)
							+ pow(y - fi->corners.at(i).y, 2))
			{
				fi->selectedCorner = i;
				fi->vValidCorners.at(fi->selectedCorner) =
						!fi->vValidCorners.at(fi->selectedCorner);
				fi->drawRawField();
				break;
			}
		}
	}
	else if (fi->selectedCorner >= 0)
	{
		if (event == EVENT_LBUTTONUP)
		{
			fi->selectedCorner = -1;
		}
		else if (event == EVENT_MOUSEMOVE)
		{
			// std::cout << fi->selectedCorner << std::endl;
			fi->corners[fi->selectedCorner].x = x;
			fi->corners[fi->selectedCorner].y = y;
			fi->drawRawField();
		}
	}
}
;

FieldInit::FieldInit(FieldModel* fieldmodel)
{
	this->fieldModel = fieldmodel;
	float l = fieldModel->length / 30;
	float w = fieldModel->width / 30;
	maxDist = 12;

	vValidCorners.push_back(false);
	vValidCorners.push_back(false);
	vValidCorners.push_back(false);

	vValidCorners.push_back(false);
	vValidCorners.push_back(false);
	vValidCorners.push_back(false);

	vValidCorners.push_back(false);
	vValidCorners.push_back(false);
	vValidCorners.push_back(false);

	isInitialized = false;
	selectedCorner = -1;

	std::cout << "Initialized startup" << std::endl;
}
;
FieldInit::~FieldInit()
{
}

void FieldInit::initCorners(int width, int length)
{
	float px = width / 2;
	float py = length / 2;
	float l = fieldModel->length / 30;
	float w = fieldModel->width / 30;

	corners.clear();
	corners.push_back(Point2f(-l, -w) + Point2f(px, py));
	corners.push_back(Point2f(-l, +0) + Point2f(px, py)); //1
	corners.push_back(Point2f(-l, +w) + Point2f(px, py));

	corners.push_back(Point2f(+0, -w) + Point2f(px, py));
	corners.push_back(Point2f(+0, +0) + Point2f(px, py)); //4
	corners.push_back(Point2f(+0, w) + Point2f(px, py));

	corners.push_back(Point2f(+l, -w) + Point2f(px, py));
	corners.push_back(Point2f(+l, +0) + Point2f(px, py)); //7
	corners.push_back(Point2f(+l, +w) + Point2f(px, py));

	initFieldLines();
}

void FieldInit::initCornersH(Mat H)
{
	vector<Point2f> edgesTrans;
	fieldModel->getCorners(edgesTrans);
	perspectiveTransform(edgesTrans, corners, H);

	initFieldLines();
}

void FieldInit::initFieldLines()
{
	vFieldLines.clear();
	vFieldLines.push_back(&corners[0]);
	vFieldLines.push_back(&corners[2]);

	vFieldLines.push_back(&corners[3]);
	vFieldLines.push_back(&corners[5]);

	vFieldLines.push_back(&corners[6]);
	vFieldLines.push_back(&corners[8]);

	vFieldLines.push_back(&corners[0]);
	vFieldLines.push_back(&corners[3]);

	vFieldLines.push_back(&corners[3]);
	vFieldLines.push_back(&corners[6]);

	vFieldLines.push_back(&corners[2]);
	vFieldLines.push_back(&corners[5]);

	vFieldLines.push_back(&corners[5]);
	vFieldLines.push_back(&corners[8]);
}

bool FieldInit::initializeField(cv::Mat& H, cv::Mat imgDst)
{
	imgSrc = imgDst;
	cv::imshow("init_Window", imgDst);
	isInitialized = true;

	setMouseCallback("init_Window", pullCorner, this);
	initCorners(imgDst.cols, imgDst.rows);
//	initCornersH(H);
	drawRawField();
	while (true)
	{
		char key = waitKey(100);
		void* wh = cvGetWindowHandle("init_Window");
		if (!active || key == 27 || wh == NULL) //wait for 'esc' key press
		{
			isInitialized = false;
			break;
		} else if((key == ' ' || key == 10) // space or enter
				&& (std::count(vValidCorners.begin(), vValidCorners.end(), true) >= 4))
		{
			break;
		} else if(key != -1) {
			std::cout << key << std::endl;
			std::cout << "Not enough Points. Need at least 4. Got "
				<< std::count(vValidCorners.begin(), vValidCorners.end(), true)
				<< std::endl;
		}
	}

	if(isInitialized)
	{
		std::vector<Point2f> vModelPoints, vImagePoints;
		for (int i = 0; i < vValidCorners.size(); i++)
		{
			if (vValidCorners.at(i))
			{
				vModelPoints.push_back(fieldModel->corners.at(i)->point);
				vImagePoints.push_back(corners.at(i));
			}
		}
		H = findHomography(vModelPoints, vImagePoints, 0);
	}

	setMouseCallback("init_Window", NULL, NULL);
//	fieldModel->draw(H, imgSrc);
//	imshow("init_Window", imgSrc);
//    waitKey();
	destroyWindow("init_Window");
	return isInitialized;
}
;

void FieldInit::drawRawField()
{
	cv::Mat imgDst = imgSrc.clone();
	for (int i = 0; i < vFieldLines.size(); i += 2)
	{
		line(imgDst, *vFieldLines[i], *vFieldLines[i + 1], Scalar(0, 0, 255), 2,
				8);
	}
	for (int i = 0; i < corners.size(); i++)
	{
		if (vValidCorners.at(i))
		{
			circle(imgDst, corners.at(i), 8, Scalar(0, 255, 0), 3);
		}
	}
	putText(imgDst, "Left", corners.at(5), CV_FONT_HERSHEY_PLAIN, 2,
			Scalar(0, 0, 0), 2);
	putText(imgDst, "Right", corners.at(3), CV_FONT_HERSHEY_PLAIN, 2,
			Scalar(0, 0, 0), 2);
	Point2f tpoint = corners.at(0) + corners.at(1) + corners.at(2)
			+ corners.at(3);
	putText(imgDst, "Tigers", Point2f(tpoint.x / 4, tpoint.y / 4),
	CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
	cv::imshow("init_Window", imgDst);
}
;

