/*
 * LineSampler.cpp
 *
 *  Created on: Aug 9, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "LineSampler.h"
#include <assert.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "GeoUtil.h"
#include "opencv_compat.h"


namespace tigers
{

using namespace std;
using namespace cv;

LineSampler::LineSampler(FieldModel* fieldModel)
{
	this->fieldModel = fieldModel;
	float l = fieldModel->length / 2;
	float w = fieldModel->width / 2;

	// define the 6 intersection points
	fieldIntersections.push_back(Point2f(l, -w));
	fieldIntersections.push_back(Point2f(-l, -w));
	fieldIntersections.push_back(Point2f(0, -w));
	fieldIntersections.push_back(Point2f(l, w));
	fieldIntersections.push_back(Point2f(-l, w));
	fieldIntersections.push_back(Point2f(0, w));

	// comps must map to fieldIntersections
	// Point2i is a mapping line -> line of two lines that intersect.
	// This intersection point maps to the fieldIntersections above in same order
	// line0=negative field line,
	// line1=positive goal line,
	// line2=positive field line,
	// line3=negative goal line
	// line4=center line
	comps.push_back(Point2i(0, 1));
	comps.push_back(Point2i(0, 3));
	comps.push_back(Point2i(0, 4));
	comps.push_back(Point2i(1, 2));
	comps.push_back(Point2i(2, 3));
	comps.push_back(Point2i(2, 4));

	for (int i = 0; i < 5; i++)
		intersecMapping.insert(pair<int, vector<int>>(i, vector<int>()));
	intersecMapping[0].push_back(1);
	intersecMapping[0].push_back(3);
	intersecMapping[0].push_back(4);
	intersecMapping[1].push_back(0);
	intersecMapping[1].push_back(2);
	intersecMapping[2].push_back(1);
	intersecMapping[2].push_back(3);
	intersecMapping[2].push_back(4);
	intersecMapping[3].push_back(0);
	intersecMapping[3].push_back(2);

	intersectionBuffer = vector<vector<Point2f>>(6);
}

LineSampler::~LineSampler()
{
}

void LineSampler::findTransformation(cv::Mat& src, cv::Mat& imgDst,
		std::vector<cv::Point2f>& modelBots, cv::Mat& H)
{
	Mat imgBw;
	cvtColor(src, imgBw, CV_RGB2GRAY);

	// initialize mean color
	if (meanColor[0] == 0)
	{
		meanColor = mean(imgBw);
	}

	// 1. sample on lines
	vector<Vec4f> fieldBorders = fieldModel->getBorderLinesTransf(H);
	vector<vector<Point2f>> samples(fieldBorders.size());
	for (int i = 0; i < fieldBorders.size(); i++)
	{
		// draw samples
		if (imgDst.cols > 0)
		{
			sample(imgBw, Point2f(fieldBorders[i][0], fieldBorders[i][1]),
					Point2f(fieldBorders[i][2], fieldBorders[i][3]), samples[i],
					&imgDst);
			for (int j = 0; j < samples[i].size(); j++)
			{
				circle(imgDst, samples[i][j], 5, Scalar(255, 0, 255), 2);
			}
		}
		else
		{
			sample(imgBw, Point2f(fieldBorders[i][0], fieldBorders[i][1]),
					Point2f(fieldBorders[i][2], fieldBorders[i][3]),
					samples[i]);
		}
	}
	meanColor = mean(meanColors);
	meanColors.clear();

	// 2. If no samples were detected on a line segment, check if there are useful samples on crossing lines
	vector<vector<Point>> newSamples(samples.size());
//	for (int i = 0; i < samples.size(); i++)
//	{
//		if (samples[i].size() == 0)
//		{
//			Point lStart(fieldBorders[i][0], fieldBorders[i][1]);
//			Point lEnd(fieldBorders[i][2], fieldBorders[i][3]);
//			for (int j = 0; j < intersecMapping[i].size(); j++)
//			{
//				int k = intersecMapping[i][j];
//				if (samples[k].empty())
//				{
//					continue;
//				}
//				float distFirst = GeoUtil::distance_to_Line(lStart, lEnd,
//						samples[k][0]);
//				float distLast = GeoUtil::distance_to_Line(lStart, lEnd,
//						samples[k][samples[k].size() - 1]);
//				Point2f newSample;
//				Point2f dir;
//				if (distFirst < 100 || distLast < 100)
//				{
//					if (distFirst < distLast)
//					{
//						newSample = samples[k][0];
//						dir = samples[k][0] - samples[k][samples[k].size() - 1];
//						dir = Point2f(dir.x / norm(dir), dir.y / norm(dir));
//					}
//					else
//					{
//						newSample = samples[k][samples[k].size() - 1];
//						dir = samples[k][samples[k].size() - 1] - samples[k][0];
//						dir = Point2f(dir.x / norm(dir), dir.y / norm(dir));
//					}
//					float offset = 100;
//					newSamples[i].push_back(newSample + Point2f(dir.x * offset, dir.y * offset));
//				}
//			}
//		}
//	}

	// 3. add new samples and fill with corners, if there are less than two samples for a segment
	vector<float> numSamples(fieldBorders.size());
	for (int i = 0; i < newSamples.size(); i++)
	{
		for (int j = 0; j < newSamples[i].size(); j++)
		{
			samples[i].push_back(newSamples[i][j]);
		}
		numSamples[i] = samples[i].size();
		if (samples[i].size() < 2)
		{
			samples[i].push_back(
					Point2f(fieldBorders[i][0], fieldBorders[i][1]));
			samples[i].push_back(
					Point2f(fieldBorders[i][2], fieldBorders[i][3]));
		}
	}

	// 4. fit line through samples per segment
	vector<Vec4f> approxLines(fieldBorders.size());
	for (int i = 0; i < samples.size(); i++)
	{

		int distType = CV_DIST_L2;
		double param = 0;
		double reps = 0.01;
		double aeps = 0.01;
		fitLine(samples[i], approxLines[i], distType, param, reps, aeps);

		if (imgDst.cols > 0)
		{
			GeoUtil::drawLine(imgDst, approxLines[i], Scalar(0, 0, 0), 2);
		}
	}

	// 5. Calc intersections
	vector<Point2f> intersections, intersecFiltered, intersecFiltered2,
			fieldIntersect;
	getIntersections(approxLines, Rect(0, 0, src.cols, src.rows),
			intersections);

	if (lastIntersections.empty())
	{
		for (int i = 0; i < intersections.size(); i++)
		{
			lastIntersections.push_back(intersections[i]);
		}
//		lastIntersections = vector<Point2f>(intersections);
	}

	// 6. Filter useful intersections
	Rect fieldRect(0, 0, src.cols, src.rows);
	for (int i = 0; i < intersections.size(); i++)
	{
//		intersectionBuffer[i].push_back(intersections[i]);
//		if (intersectionBuffer[i].size() > 10)
//		{
//			intersectionBuffer[i] = vector<Point2f>(
//					intersectionBuffer[i].begin() + 1,
//					intersectionBuffer[i].end());
//		}
//		Scalar meanScalar = mean(intersectionBuffer[i]);
//		Point2f meanIntersection(meanScalar[0], meanScalar[1]);

		Point2f diffIntersecs = intersections[i] - lastIntersections[i];
		float c = 0.5;
		Point2f meanIntersection = lastIntersections[i]
				+ Point2f(diffIntersecs.x * c, diffIntersecs.y * c);
		lastIntersections[i] = meanIntersection;

//		Point2f meanIntersection = intersections[i];

//		if (fieldRect.contains(meanIntersection))
		{
			intersecFiltered2.push_back(meanIntersection);
//			fieldIntersect.push_back(fieldIntersections[i]);
			if (imgDst.cols > 0)
			{
				circle(imgDst, meanIntersection, 2 + i * 2, Scalar(100, 255, 0),
						1);
			}
		}
	}

	// only use the intersections from line segments with most samples
	if (numSamples[1] < numSamples[4] && numSamples[1] < numSamples[3])
	{
		// no positive goal line
		intersecFiltered.push_back(intersecFiltered2[1]);
		fieldIntersect.push_back(fieldIntersections[1]);
		intersecFiltered.push_back(intersecFiltered2[2]);
		fieldIntersect.push_back(fieldIntersections[2]);
		intersecFiltered.push_back(intersecFiltered2[4]);
		fieldIntersect.push_back(fieldIntersections[4]);
		intersecFiltered.push_back(intersecFiltered2[5]);
		fieldIntersect.push_back(fieldIntersections[5]);
	}
	else if (numSamples[4] < numSamples[1] && numSamples[4] < numSamples[3])
	{
		// no middle line
		intersecFiltered.push_back(intersecFiltered2[1]);
		fieldIntersect.push_back(fieldIntersections[1]);
		intersecFiltered.push_back(intersecFiltered2[0]);
		fieldIntersect.push_back(fieldIntersections[0]);
		intersecFiltered.push_back(intersecFiltered2[4]);
		fieldIntersect.push_back(fieldIntersections[4]);
		intersecFiltered.push_back(intersecFiltered2[3]);
		fieldIntersect.push_back(fieldIntersections[3]);
	}
	else if (numSamples[3] < numSamples[1] && numSamples[3] < numSamples[4])
	{
		// no negative goal line
		intersecFiltered.push_back(intersecFiltered2[2]);
		fieldIntersect.push_back(fieldIntersections[2]);
		intersecFiltered.push_back(intersecFiltered2[0]);
		fieldIntersect.push_back(fieldIntersections[0]);
		intersecFiltered.push_back(intersecFiltered2[5]);
		fieldIntersect.push_back(fieldIntersections[5]);
		intersecFiltered.push_back(intersecFiltered2[3]);
		fieldIntersect.push_back(fieldIntersections[3]);
	}
	else
	{
		// all have the same number of samples... choose all?...
		intersecFiltered.push_back(intersecFiltered2[0]);
		fieldIntersect.push_back(fieldIntersections[0]);
		intersecFiltered.push_back(intersecFiltered2[1]);
		fieldIntersect.push_back(fieldIntersections[1]);
		intersecFiltered.push_back(intersecFiltered2[2]);
		fieldIntersect.push_back(fieldIntersections[2]);
		intersecFiltered.push_back(intersecFiltered2[3]);
		fieldIntersect.push_back(fieldIntersections[3]);
		intersecFiltered.push_back(intersecFiltered2[4]);
		fieldIntersect.push_back(fieldIntersections[4]);
		intersecFiltered.push_back(intersecFiltered2[5]);
		fieldIntersect.push_back(fieldIntersections[5]);
//		cout << "All the same samples: " << numSamples[1] << numSamples[3]
//				<< numSamples[4] << endl;
	}

	// Finally: find homography if possible
	if (intersecFiltered.size() == fieldIntersect.size()
			&& fieldIntersect.size() > 3)
	{
//		cout << fieldIntersect << endl;
//		cout << intersecFiltered << endl;
		H = findHomography(fieldIntersect, intersecFiltered, RANSAC);
	}
	else
	{
		cout << "Detected " << intersecFiltered.size()
				<< " intersections, but should be " << fieldIntersect.size()
				<< " > 3" << endl;
		cout << intersecFiltered << endl;
	}
}

void LineSampler::showControls(cv::TrackbarCallback callBack, void* ref)
{
}

void LineSampler::sample(cv::Mat& src, cv::Point lStart, cv::Point lEnd,
		std::vector<cv::Point2f>& samples, cv::Mat* imgDst)
{
	assert(src.channels() == 1);

	// size of sub image
	int scanWidth = 60;
	// dist between sub images
	float stepSize = 60;

	Vec4f mainLine(lStart.x, lStart.y, lEnd.x, lEnd.y);
	Point2f vLine = lEnd - lStart;
	Point2f vDir = vLine * (stepSize / norm(vLine));
	Point vOffset = vLine * (scanWidth / 2 / norm(vLine));

	// start before lStart and stop after lEnd
	int iters = (norm(vLine) / stepSize) + 1;
	for (int i = 0; i < iters; i++)
	{
		Point p = lStart + vOffset + Point2i(i * vDir.x, i * vDir.y);
		if (p.x < 0 || p.y < 0 || p.x >= src.cols || p.y >= src.rows)
		{
			// point out of image
			continue;
		}

		int x = p.x - scanWidth / 2;
		int y = p.y - scanWidth / 2;
		int width = scanWidth;
		int height = scanWidth;

		// adapt subRect dimensions to fit into image
		if (x < 0) {
			width += x;
			x = 0;
		}
		if (y < 0) {
			height += y;
			y = 0;
		}
		if (x + width >= src.cols)
			width = src.cols - x - 1;
		if (y + height >= src.rows)
			height = src.rows - y - 1;

		Rect subRect(x, y, width, height);
		Point2f subTranslate(x, y);
		Mat sub = Mat(src, subRect).clone();

		Scalar mColor = cv::mean(sub);
		meanColors.push_back(mColor);

		// only accept sub images that match the mean color
		if (mColor[0] > meanColor[0] - 20)
		{

			// threshold
			sub = sub > 170;
			Mat nonZero;
			findNonZero(sub, nonZero);

			Mat subColor;
			if (imgDst != NULL)
			{
				subColor = Mat(*imgDst, subRect);
				cvtColor(sub, subColor, CV_GRAY2BGR);
			}

			if (nonZero.total() > 10)
			{
				int inc = ceil(nonZero.total() / 50.0f);
				vector<Point> nonZeroReduced;
				for (int i = 0; i < nonZero.total(); i += inc)
				{
					nonZeroReduced.push_back(nonZero.at<Point>(i));
				}

				Vec4f appLine;
				fitLine(nonZeroReduced, appLine, CV_DIST_L2, 0, 0.01, 0.01);
				if (imgDst != NULL)
				{
					GeoUtil::drawLine(subColor, appLine, Scalar(250, 0, 0), 2);
				}
				float angle = fabsf(
						GeoUtil::computeAngleVV(Point2f(appLine[0], appLine[1]),
								vLine));
				if (angle > CV_PI / 2)
				{
					// we do not care for the direction of the vectors, so we also have to check if they nearly around 180deg.
					angle = fabsf(angle - CV_PI);
				}
				// only interested in rather parallel lines
				if (angle < 0.3f)
				{
					int inc = ceil(nonZero.total() / 20.0f);
					float distSum = 0;
					int numSum = 0;
					for (int i = 0; i < nonZero.total(); i += inc)
					{
						double dist = GeoUtil::distance_to_Line(
								Point2f(appLine[2] + 10 * appLine[0],
										appLine[3] + 10 * appLine[1]),
								Point2f(appLine[2], appLine[3]),
								nonZero.at<Point>(i));
						distSum += dist;
						numSum++;
					}
					float meanDist = distSum / numSum;
					if (meanDist < 4.0f)
					{
//						Rect rect(0, 0, subRect.width, subRect.height);
//						vector<Point2f> intersecs;
//						GeoUtil::intersectionRectLine(rect, appLine, intersecs);
//						for (int i = 0; i < intersecs.size(); i++)
//						{
//							samples.push_back(
//									Point2f(subRect.x, subRect.y)
//											+ intersecs[i]);
//						}
						samples.push_back(
								Point2f(subRect.x + appLine[2],
										subRect.y + appLine[3]));
//						for(int i=0;i<nonZero.total();i++)
//						{
//							samples.push_back(Point(subRect.x, subRect.y) + nonZero.at<Point>(i));
//						}
						if (imgDst != NULL)
						{
							// signal that we use this sub rect
							Mat greenRect(subColor(Rect(0, 0, 8, 8)));
							greenRect = Scalar(0, 250, 0);
						}
					}
					if (imgDst != NULL)
					{
						std::stringstream ss;
						ss << meanDist;
						putText(*imgDst, ss.str(),
								Point(subRect.x, subRect.y + scanWidth + 15),
								CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
					}
				}
			}
		}

		if ((imgDst != NULL) && (imgDst->cols > 0))
		{
			rectangle(*imgDst, subRect, Scalar(255, 255, 255), 1);
		}
	}
}

void LineSampler::getIntersections(vector<Vec4f> &lines, Rect fieldRect,
		vector<Point2f> &intersections)
{
	for (int c = 0; c < comps.size(); c++)
	{
		int i = comps[c].x;
		int j = comps[c].y;
		Point2f o1(lines[i][2], lines[i][3]);
		Point2f p1(lines[i][2] + 1000 * lines[i][0],
				lines[i][3] + 1000 * lines[i][1]);
		Point2f o2(lines[j][2], lines[j][3]);
		Point2f p2(lines[j][2] + 1000 * lines[j][0],
				lines[j][3] + 1000 * lines[j][1]);
		Point2f pt;
		GeoUtil::intersection(o1, p1, o2, p2, pt);

		intersections.push_back(pt);
	}
}

} /* namespace tigers */
