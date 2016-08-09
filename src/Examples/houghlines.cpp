#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <cstdio>

#include "LSWMS.h"

using namespace cv;
using namespace std;

#ifndef LINE_AA
#define LINE_AA CV_AA
#endif

/** General variables */
Mat src, edges;
Mat src_gray;
Mat probabilistic_hough;
Mat lineSegments;
int min_threshold = 50;
int max_trackbar = 150;

const char* window_name_hough = "Hough Lines";
const char* window_name_lineSeg = "Line Segments";

int p_trackbar = max_trackbar;
int minLineLength = 30;
int maxLineGap = 10;

/**
 * @function Probabilistic_Hough
 */
void Probabilistic_Hough(int, void*)
{
	vector<Vec4i> p_lines;
	cvtColor(edges, probabilistic_hough, COLOR_GRAY2BGR);

	/// 2. Use Probabilistic Hough Transform
	HoughLinesP(edges, p_lines, 1, CV_PI / 180, min_threshold + p_trackbar,
			minLineLength, maxLineGap);

	/// Show the result
	for (size_t i = 0; i < p_lines.size(); i++)
	{
		Vec4i l = p_lines[i];
		line(probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]),
				Scalar(255, 0, 0), 3, LINE_AA);
	}

	imshow(window_name_hough, probabilistic_hough);
}

void LineSegmentDetect(int, void*)
{
	cvtColor(src_gray, lineSegments, COLOR_GRAY2BGR);

	// Line segments (LSWMS and PPHT)
	std::vector<LSEG> lSegs, lSegsPPHT;
	std::vector<double> errors;

	int width = src.cols;
	int height = src.rows;
	Size procSize = cv::Size(width, height);
	int numMaxLSegs = 300;
	bool verbose = false;

	// Create and init LSWMS
	int R = 3;
	LSWMS lswms(procSize, R, numMaxLSegs, verbose);

	lswms.run(src_gray, lSegs, errors);
	lswms.drawLSegs(lineSegments, lSegs, errors);

	imshow(window_name_lineSeg, lineSegments);
}

int main(int argc, char** argv)
{
	/// Read the image
	src = imread(argv[1], 1);

	if (src.empty())
	{
		return -1;
	}

	/// Pass the image to gray
	cvtColor(src, src_gray, COLOR_RGB2GRAY);

	/// Apply Canny edge detector
	Canny(src_gray, edges, 50, 200, 3);

	/// Create Trackbars for Thresholds
	char thresh_label[50];
	sprintf(thresh_label, "Thres: %d + input", min_threshold);

	namedWindow(window_name_hough, WINDOW_AUTOSIZE);
	createTrackbar(thresh_label, window_name_hough, &p_trackbar, max_trackbar,
			Probabilistic_Hough);
	createTrackbar("minLineLength", window_name_hough, &minLineLength,
			max_trackbar, Probabilistic_Hough);
	createTrackbar("maxLineGap", window_name_hough, &maxLineGap, max_trackbar,
			Probabilistic_Hough);

	/// Initialize
	Probabilistic_Hough(0, 0);
	LineSegmentDetect(0, 0);
	waitKey(0);
	return 0;
}
