/*
 * FieldLineDetector.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef FIELDLINEDETECTOR_H_
#define FIELDLINEDETECTOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "FieldCorner.h"
#include "augm_wrapper.pb.h"
#include "IHDetector.h"
#include "FieldModel.h"

namespace tigers
{
class FieldLineDetector: public IHDetector
{
public:
	FieldLineDetector(FieldModel* fieldModel);
	virtual ~FieldLineDetector();
	FieldModel* fieldModel;
	std::vector<cv::Point2f> cornerBuffer;
	std::vector<FieldCorner*> detectedCorners;
	std::vector<cv::Point2f> botPosField;

	void findTransformation(cv::Mat& src, cv::Mat& imgDst,
			std::vector<cv::Point2f>& modelBots, cv::Mat& H);
	cv::Mat threshold(cv::Mat& imgOriginal);
	bool findCornerMapping(std::vector<FieldCorner*> &cornerMapping);
	void findFieldMatch(std::vector<FieldCorner*> fieldCrossings, cv::Mat& H);
	void detectCorners(std::vector<cv::Vec4i> &lines);
	void filterCorners();
	void findNeighbors(std::vector<cv::Vec4i> &lines);
	void createThresholdedImg(cv::Mat& src);
	void showControls(cv::TrackbarCallback callBack, void* ref);

private:
	std::vector<FieldCorner*> mappedEdges;
	int cornerBufferSize = 200;
	int knn_N = 9;

	int min_threshold = 50;
	int max_trackbar = 150;
	int p_trackbar = 150;
	int minLineLength = 60;
	int maxLineGap = 10;
	int scaleFactor = 7;

	int iLowH = 0;
	int iHighH = 179;
	int iLowS = 27;
	int iHighS = 255;
	int iLowV = 91;
	int iHighV = 255;
	cv::Mat imgThres;

	bool findCornerMappingRec(std::vector<FieldCorner*> &allCorners,
			std::vector<FieldCorner*> &corners);
	double findFieldMatchRec(std::vector<FieldCorner*> &fieldCrossings,
			std::vector<FieldCorner*> &corners,
			std::vector<FieldCorner*> &bestCorners);
	double calcMatchError(std::vector<FieldCorner*> &fieldCrossings,
			std::vector<cv::Point2f> &cornerPoints, cv::Mat &H);
	void sortCorners(std::vector<cv::Point2f>& corners);
	float calcMatchErrorBots(std::vector<cv::Point2f>& botPosField, cv::Mat& H);
};

} /* namespace tigers */

#endif /* FIELDLINEDETECTOR_H_ */
