/*
 * FieldLineDetector.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "FieldLineDetector.h"
#include "GeoUtil.h"
#include "FieldCorner.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "augm_wrapper.pb.h"
#include <assert.h>
#include <cmath>
using namespace std;
using namespace cv;


#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY COLOR_BGR2GRAY
#endif

namespace tigers
{

FieldLineDetector::FieldLineDetector(FieldModel* fieldModel)
{
	this->fieldModel = fieldModel;
}

FieldLineDetector::~FieldLineDetector()
{
}

void FieldLineDetector::showControls(TrackbarCallback callBack, void* ref)
{
	string window_params = "Parameters";
	namedWindow(window_params, WINDOW_NORMAL);
	createTrackbar("LowH", window_params, &iLowH, 179, callBack, ref); //Hue (0 - 179)
	createTrackbar("HighH", window_params, &iHighH, 179, callBack, ref);
	createTrackbar("LowS", window_params, &iLowS, 255, callBack, ref); //Saturation (0 - 255)
	createTrackbar("HighS", window_params, &iHighS, 255, callBack, ref);
	createTrackbar("LowV", window_params, &iLowV, 255, callBack, ref); //Value (0 - 255)
	createTrackbar("HighV", window_params, &iHighV, 255, callBack, ref);

	createTrackbar("threshold", window_params, &p_trackbar, max_trackbar,
			callBack, ref);
	createTrackbar("minLineLength", window_params, &minLineLength, max_trackbar,
			callBack, ref);
	createTrackbar("maxLineGap", window_params, &maxLineGap, max_trackbar,
			callBack, ref);

	createTrackbar("scaleFactor*10", window_params, &scaleFactor, 50, callBack, ref);
}

void FieldLineDetector::findTransformation(cv::Mat& src, cv::Mat& imgDst,
		std::vector<cv::Point2f>& modelBots, cv::Mat& H)
{
	this->botPosField = modelBots;

	Mat imgBw;
	blur(src, imgBw, Size(5, 5));
	cvtColor(imgBw, imgBw, CV_BGR2GRAY);

	Mat imgEdges;
	Canny(imgBw, imgEdges, 50, 100, 3);

//	imshow("bw", imgBw);
//	imshow("edges", imgEdges);

	std::vector<cv::Vec4i> lines;
	HoughLinesP(imgEdges, lines, 1, CV_PI / 180, min_threshold + p_trackbar,
			minLineLength, maxLineGap);

	// Expand the lines little bit (by scaleFactor)
	for (int i = 0; i < lines.size(); i++)
	{
		cv::Vec4i v = lines[i];
		cv::Point2f p1 = Point2f(v[0], v[1]);
		cv::Point2f p2 = Point2f(v[2], v[3]);
		cv::Point2f p1p2 = p2 - p1;
		float length = norm(p1p2);

		cv::Point2f scaleP2 = p2 + p1p2 * (scaleFactor / 10.0f);
		cv::Point2f scaleP1 = p1 - p1p2 * (scaleFactor / 10.0f);

		lines[i][0] = scaleP1.x;
		lines[i][1] = scaleP1.y;
		lines[i][2] = scaleP2.x;
		lines[i][3] = scaleP2.y;
	}

	createThresholdedImg(src);

	// do line detection!
	detectCorners(lines);
	filterCorners();
	findNeighbors(lines);
	findCornerMapping(mappedEdges);
	for (int i = 0; i < mappedEdges.size(); i++)
	{
		cout << (*mappedEdges[i]) << endl;
	}
	findFieldMatch(mappedEdges, H);

	if (imgDst.cols > 0)
	{
		// Draw lines
		for (int i = 0; i < lines.size(); i++)
		{
			cv::Vec4i v = lines[i];
			cv::line(imgDst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]),
					cv::Scalar(0, 255, 0), 2);
		}

		// draw corners
		for (int i = 0; i < cornerBuffer.size(); i++)
		{
			cv::circle(imgDst, cornerBuffer.at(i), 1, cv::Scalar(255, 0, 0), 2);
		}

		// draw filtered corners
		for (int i = 0; i < detectedCorners.size(); i++)
		{
			circle(imgDst, detectedCorners[i]->point, (int) 20,
					Scalar(0, 255, 255), 1);
		}

		// draw detected corner coordinates
		for (int i = 0; i < detectedCorners.size(); i++)
		{
			stringstream ss;
			ss << detectedCorners[i]->point;
			putText(imgDst, ss.str(),
					detectedCorners[i]->point + Point2f(0, 10),
					FONT_HERSHEY_PLAIN, 1, Scalar(250, 0, 0));
		}
	}
}

Mat FieldLineDetector::threshold(Mat& imgOriginal)
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

	return imgThresholded;
}

void FieldLineDetector::createThresholdedImg(Mat& src)
{
	vector<vector<cv::Point> > cont;
	vector<vector<cv::Point> > contPoly;
	vector<Vec4i> hierarchy;
	Mat imgBwInv;
	imgThres = threshold(src);

	// find large contours and remove them from thresholded image
	bitwise_not(imgThres, imgBwInv);
	findContours(imgBwInv, cont, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE,
			cv::Point(0, 0));
	for (int i = 0; i < cont.size(); i++)
	{
		double area = contourArea(cont[i]);
		if (area > 100000)
		{
			contPoly.push_back(cont[i]);
		}
	}
	fillPoly(imgThres, contPoly, Scalar(255, 255, 255));
	imshow("thres", imgThres);
}

void FieldLineDetector::sortCorners(std::vector<cv::Point2f>& corners)
{
	// 1. remove middle points (no corners)
	vector<Point2f> cornersTmp(corners);
	corners.clear();
	for (int i = 0; i < cornersTmp.size(); i++)
	{
		if (!GeoUtil::pointBetweenPoints(cornersTmp, i, 30))
		{
			corners.push_back(cornersTmp[i]);
		}
	}

	// 2. sort top to bottom
	sort(corners.begin(), corners.end(), [](Point2d p1, Point2d p2)
	{
		return (p1.y < p2.y);
	});

	// 3. choose left or right
	int N = corners.size() - 1;
	cv::Point2f tl = corners[0].x > corners[1].x ? corners[1] : corners[0];
	cv::Point2f tr = corners[0].x > corners[1].x ? corners[0] : corners[1];
	cv::Point2f bl =
			corners[N].x > corners[N - 1].x ? corners[N - 1] : corners[N];
	cv::Point2f br =
			corners[N].x > corners[N - 1].x ? corners[N] : corners[N - 1];

	// 4. put all 4 corners back in list
	corners.clear();
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
}

double FieldLineDetector::calcMatchError(vector<FieldCorner*> &fieldCrossings,
		vector<Point2f> &cornersPoints, Mat &H)
{
	assert(fieldCrossings.size() == cornersPoints.size());
	vector<Point2f> cornersPointsTrans;
	perspectiveTransform(cornersPoints, cornersPointsTrans, H);
	double errorSum = 0;
	for (int i = 0; i < fieldCrossings.size(); i++)
	{
		double error = norm(
				fieldCrossings.at(i)->point - cornersPointsTrans.at(i));
		error *= error;
		errorSum += error;
	}

	return errorSum;
}

double FieldLineDetector::findFieldMatchRec(
		vector<FieldCorner*> &fieldCrossings, vector<FieldCorner*> &corners,
		vector<FieldCorner*> &bestcorners)
{
	if (fieldCrossings.size() == corners.size())
	{
		vector<Point2f> cornersPoints;
		vector<Point2f> fieldPoints;
		for (int i = 0; i < corners.size(); i++)
		{
			cornersPoints.push_back(corners.at(i)->point);
			fieldPoints.push_back(fieldCrossings.at(i)->point);
		}
		Mat H = findHomography(cornersPoints, fieldPoints, 0);
		bestcorners.clear();
		bestcorners.insert(bestcorners.end(), corners.begin(), corners.end());
		return calcMatchError(fieldCrossings, cornersPoints, H);
	}

	double bestError = INFINITY;
	for (int i = 0; i < corners.back()->getNumNeighbors(); i++)
	{
		FieldCorner* neighbor = corners.back()->getNeighbor(i);
		bool neighborUsed = false;
		for (int j = 0; j < corners.size() - 1; j++)
		{
			if (neighbor == corners.at(j))
			{
				neighborUsed = true;
				break;
			}
		}
		if (neighborUsed)
		{
			continue;
		}
		vector<FieldCorner*> newBestcorners;
		vector<FieldCorner*> nextcorners(corners);
		nextcorners.push_back(neighbor);
		double error = findFieldMatchRec(fieldCrossings, nextcorners,
				newBestcorners);
		if (error < bestError)
		{
			bestError = error;
			bestcorners.clear();
			bestcorners.insert(bestcorners.end(), newBestcorners.begin(),
					newBestcorners.end());
		}
	}
	return bestError;
}

void FieldLineDetector::findFieldMatch(std::vector<FieldCorner*> fieldCrossings,
		cv::Mat& H)
{
	if (fieldCrossings.size() < 4)
	{
		return;
	}
	vector<FieldCorner*> bestcorners;
	double smallestError = INFINITY;
	for (int i = 0; i < fieldModel->corners.size(); i++)
	{
		clock_t start = clock();
		vector<FieldCorner*> corners;
		vector<FieldCorner*> newBestCorners;
		corners.push_back(this->fieldModel->corners.at(i));
		double error = findFieldMatchRec(fieldCrossings, corners,
				newBestCorners);
		double time = (double) (clock() - start) / CLOCKS_PER_SEC;
		cout << i << ") error=" << error << " time=" << time << "s" << endl;
		if (error < 100)
		{
			bestcorners = newBestCorners;
			break;
		}
		if (error < smallestError)
		{
			smallestError = error;
			bestcorners = newBestCorners;
		}
	}
	if (bestcorners.size() > 3)
	{
		vector<Point2f> cornersPoints;
		vector<Point2f> fieldPoints;
		for (int i = 0; i < bestcorners.size(); i++)
		{
			cornersPoints.push_back(bestcorners.at(i)->point);
			fieldPoints.push_back(fieldCrossings.at(i)->point);
		}
		H = findHomography(cornersPoints, fieldPoints, RANSAC);

		if (botPosField.size() > 3)
		{
			// compare with robots on field and mirrored H
			float error = calcMatchErrorBots(botPosField, H);

			H.at<double>(0,0) *= -1;
			H.at<double>(0,1) *= -1;
			H.at<double>(1,0) *= -1;
			H.at<double>(1,1) *= -1;
			H.at<double>(2,0) *= -1;
			H.at<double>(2,1) *= -1;

			float error2 = calcMatchErrorBots(botPosField, H);

			if(error2 > error)
			{
				H.at<double>(0,0) *= -1;
				H.at<double>(0,1) *= -1;
				H.at<double>(1,0) *= -1;
				H.at<double>(1,1) *= -1;
				H.at<double>(2,0) *= -1;
				H.at<double>(2,1) *= -1;
			}
		}
	}
}

float FieldLineDetector::calcMatchErrorBots(vector<Point2f>& botPosField, Mat& H)
{
	vector<Point2f> botPosFieldTrans;
	perspectiveTransform(botPosField, botPosFieldTrans, H);

//	Mat img;
//	cvtColor(imgThres, img, CV_GRAY2BGR);

	int offset = 30;
	float sumRatio = 0;
	for (int j = 0; j < botPosFieldTrans.size(); j++)
	{
		Point2f p(botPosFieldTrans.at(j));
		if (p.x - offset < 0 || p.y - offset < 0
				|| p.x + offset > imgThres.cols
				|| p.y + offset > imgThres.rows)
		{
			continue;
		}
		if (!fieldModel->field.contains(botPosField.at(j)))
		{
			continue;
		}
		Rect rect(Point2f(p.x - offset, p.y - offset),
				Point2f(p.x + offset, p.y + offset));
		Mat sub = imgThres(rect);
		int numNonBlack = countNonZero(sub);
		int total = sub.rows * sub.cols;
		float ratio = (float) numNonBlack / total;
		sumRatio += ratio;
//		rectangle(img, rect, Scalar(250, 0, 250));
//		stringstream ss;
//		ss << ratio;
//		putText(img, ss.str(), cv::Point(rect.x, rect.y - 15),
//				CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 250), 1);
	}
	return sumRatio / botPosFieldTrans.size();
}

bool FieldLineDetector::findCornerMappingRec(vector<FieldCorner*> &allcorners,
		vector<FieldCorner*> &corners)
{
	if (allcorners.size() == corners.size())
	{
		return true;
	}
	for (int i = 0; i < corners.back()->getNumNeighbors(); i++)
	{
		FieldCorner* neighbor = corners.back()->getNeighbor(i);
		bool neighborUsed = false;
		for (int j = 0; j < corners.size() - 1; j++)
		{
			if (neighbor == corners.at(j))
			{
				neighborUsed = true;
				break;
			}
		}
		if (neighborUsed)
		{
			continue;
		}
		corners.push_back(neighbor);
		if (findCornerMappingRec(allcorners, corners))
		{
			return true;
		}
		corners.pop_back();
	}
	return false;
}

bool FieldLineDetector::findCornerMapping(vector<FieldCorner*> &cornerMapping)
{
	vector<FieldCorner*> corners(detectedCorners);
	sort(corners.begin(), corners.end(), [](FieldCorner* p1, FieldCorner* p2)
	{
		return (p1->point.y < p2->point.y);
	});
	// check current mapping, if available
	if (!cornerMapping.empty())
	{
		if (findCornerMappingRec(corners, cornerMapping))
		{
			return true;
		}
	}
	for (int i = 0; i < corners.size(); i++)
	{
		cornerMapping.clear();
		cornerMapping.push_back(corners.at(i));
		if (findCornerMappingRec(corners, cornerMapping))
		{
			return true;
		}
	}
	cornerMapping.clear();
	return false;
}

void FieldLineDetector::detectCorners(vector<Vec4i> &lines)
{
	for (int i = 0; i < lines.size(); i++)
	{
		for (int j = i + 1; j < lines.size(); j++)
		{
			Point2f o1(lines[i][0], lines[i][1]);
			Point2f p1(lines[i][2], lines[i][3]);
			Point2f o2(lines[j][0], lines[j][1]);
			Point2f p2(lines[j][2], lines[j][3]);
			Point2f pt;
			GeoUtil::intersection(o1, p1, o2, p2, pt);

			Rect rect1(o1, p1);
			Rect rect2(o2, p2);
			if (!rect1.contains(pt) || !rect2.contains(pt))
			{
				continue;
			}

			float angle = fabsf(GeoUtil::computeAngle(lines[i], lines[j]));
			if (pt.x >= 0 && pt.y >= 0 && angle > 0.2f)
			{
				cornerBuffer.push_back(pt);
			}
		}
	}

	cout << "cornerBuffer.size=" << cornerBuffer.size() << endl;

	// remove old corners
	if (cornerBuffer.size() > cornerBufferSize)
	{
		cornerBuffer = vector<Point2f>(cornerBuffer.end() - cornerBufferSize,
				cornerBuffer.end());
	}
}

void FieldLineDetector::filterCorners()
{
	if (cornerBuffer.size() < knn_N)
	{
		cerr << "Warn: Not enough corners detected: " << cornerBuffer.size()
				<< endl;
		return;
	}
	Mat means;
	Mat trainData = (Mat(cornerBuffer, CV_64F)).reshape(1);
	Mat labels;
	// TODO performance improvement with KMEANS_USE_INITIAL_LABELS
	double compactness = kmeans(trainData, knn_N, labels,
			TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0), 3,
			KMEANS_PP_CENTERS, means);
	// check for duplicates
	for (int i = 0; i < means.rows; i++)
	{
		for (int j = i + 1; j < means.rows; j++)
		{
			Point2f pt1 = Point2f(means.at<float>(i, 0), means.at<float>(i, 1));
			Point2f pt2 = Point2f(means.at<float>(j, 0), means.at<float>(j, 1));
			float dist = norm(pt1 - pt2);
			if (dist < 100)
			{
				knn_N--;
				break;
			}
		}
	}
	if (compactness > 100000)
	{
		knn_N++;
	}
	// always have some as backup
//	knn_N += 2;

	detectedCorners.clear();
	for (int i = 0; i < means.rows; i++)
	{
		Point2f pt = Point2f(means.at<float>(i, 0), means.at<float>(i, 1));
		// TODO delete edge somewhere
		FieldCorner* edge = new FieldCorner;
		edge->point = pt;
		bool duplicate = false;
		for (int j = 0; j < detectedCorners.size(); j++)
		{
			double diff = norm(detectedCorners[j]->point - pt);
			if (diff < 200)
			{
				duplicate = true;
				break;
			}
		}
		if (!duplicate)
		{
			detectedCorners.push_back(edge);
		}
	}
}

void FieldLineDetector::findNeighbors(std::vector<Vec4i> &lines)
{
	for (int k = 0; k < detectedCorners.size(); k++)
	{
		for (int i = 0; i < lines.size(); i++)
		{
			double dist = GeoUtil::distance_to_Line(
					Point2f(lines[i][0], lines[i][1]),
					Point2f(lines[i][2], lines[i][3]),
					detectedCorners[k]->point);
			if (fabsf(dist) > 50)
			{
				continue;
			}
			int lastDetection = -1;
			for (int j = 0; j < detectedCorners.size(); j++)
			{
				if (j == k)
					continue;
				dist = GeoUtil::distance_to_Line(
						Point2f(lines[i][0], lines[i][1]),
						Point2f(lines[i][2], lines[i][3]),
						detectedCorners[j]->point);
				if (fabsf(dist) > 50)
				{
					continue;
				}
				if (lastDetection != -1)
				{
					// detected a 3rd point on the same line.
					// use the point that is nearer to k
					// but only if k is not in the middle
					double distNew = norm(
							detectedCorners[k]->point
									- detectedCorners[j]->point);
					double distLast = norm(
							detectedCorners[k]->point
									- detectedCorners[lastDetection]->point);
					double distNewLast = norm(
							detectedCorners[j]->point
									- detectedCorners[lastDetection]->point);
					// is k outside of k,j,lastDetection?
					if ((distNew > distNewLast || distLast > distNewLast))
					{
						// is new corner nearer to k?
						if (distNew < distLast)
						{
							// new point is nearer, so remove the last one, new is added below
							detectedCorners[k]->pop_Neighbor();
							detectedCorners[lastDetection]->pop_Neighbor();
						}
						else
						{
							// new point is farer away, so stop here.
							continue;
						}
					}
				}
				detectedCorners[k]->insert_Neighbor(detectedCorners[j]);
				detectedCorners[j]->insert_Neighbor(detectedCorners[k]);
				lastDetection = j;
			}
		}
	}
}

} /* namespace tigers */
