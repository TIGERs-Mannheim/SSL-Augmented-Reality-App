// http://opencv-srf.blogspot.de/2010/09/object-detection-using-color-seperation.html

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "augm_wrapper.pb.h"
#include <ctime>

using namespace cv;
using namespace std;

void featureDetection(Mat imgIn_1, Mat imgIn_2);

string window_threshold = "Threshold";
string window_contours = "Contours";
string window_templMatch = "Template Match";

Scalar color1 = Scalar(255, 0, 0); // red
Scalar color2 = Scalar(0, 255, 0); // green
Scalar color3 = Scalar(0, 0, 255); // blue
Scalar color4 = Scalar(255, 0, 255); // magenta?

int iLowH = 52;
int iHighH = 96;
int iLowS = 27;
int iHighS = 255;
int iLowV = 91;
int iHighV = 255;

int blobThreshMin = 24;
int blobThreshMax = 150;
int areaEqTol = 700;
int numFrames = 0;

int theta = 0;

int match_method = 0;

VideoCapture cap;
int sliderPos = 0;

void onTrackbarSlide(int current_frame, void*)
{
	cap.set(CV_CAP_PROP_POS_FRAMES, sliderPos);
}

bool comparePointsBottom2Top(Point2d p1, Point2d p2)
{
	return (p1.x < p2.x);
}

void drawFieldPos(Mat img, Point2f p, float fieldLength, float fieldWidth,
		Scalar color)
{
	float scale = img.rows / fieldLength / 4.0f;
	Point2d pTrans((p.y + fieldWidth / 2) * scale,
			(p.x + fieldLength / 2) * scale);
	circle(img, pTrans, 90 * scale, color, -1, 8);
}

Mat drawBWField(vector<Point2f> points, float fieldLength, float fieldWidth)
{
	float scale = 0.2f;
	Mat out = Mat((int) (fieldLength * scale), (int) (fieldWidth * scale),
	CV_8U, Scalar(255, 255, 255));
	for (int i = 0; i < points.size(); i++)
	{
		Point2f p = points.at(i);
		Point2d pTrans((p.y + fieldWidth / 2) * scale,
				(p.x + fieldLength / 2) * scale);
		circle(out, pTrans, 90 * scale, Scalar(0, 0, 0), -1, 8);
	}
	return out;
}

Mat threshold(Mat imgOriginal)
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

	//show the thresholded image
	imshow(window_threshold, imgThresholded);

	return imgThresholded;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
Mat MatchingMethod(Mat img, Mat templ)
{
	Mat result;
	/// Source image to display
	Mat img_display;
	img.copyTo(img_display);

	/// Create the result matrix
	int result_cols = img.cols - templ.cols + 1;
	int result_rows = img.rows - templ.rows + 1;

	result.create(result_cols, result_rows, CV_32FC1);

	/// Do the Matching and Normalize
	matchTemplate(img, templ, result, match_method);
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

	/// Localizing the best match with minMaxLoc
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
	{
		matchLoc = minLoc;
	}
	else
	{
		matchLoc = maxLoc;
	}

	/// Show me what you got
	rectangle(img_display, matchLoc,
			Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
			Scalar::all(0), 2, 8, 0);
	rectangle(result, matchLoc,
			Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
			Scalar::all(0), 2, 8, 0);

	imshow(window_templMatch, img_display);

	return img_display;
}

std::vector<Point2f> contours(Mat imgThresholded, Mat drawing,
		std::vector<Point2f> objPoints)
{
	std::vector<Point2f> scenePoints;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_TREE,
			CHAIN_APPROX_SIMPLE, Point(0, 0));

	RotatedRect rotRecs[contours.size()];

	for (size_t i = 0; i < contours.size(); i++)
	{
		Scalar color = color1;
		if (hierarchy[i][3] < 0)
		{
			color = color3;
		}
		drawContours(drawing, contours, (int) i, color, 1, 8, hierarchy, 0,
				Point());

		if (contours[i].size() > 4)
		{
			rotRecs[i] = minAreaRect(contours[i]);
			Rect brect = rotRecs[i].boundingRect();
			double area = contourArea(contours[i]);
			double diff = (brect.area() - area);
			double areaEq = diff / ((area + brect.area()) / 2.0);
			//				cout << brect.area() << " " << area << " " << diff << " " << areaEq << endl;
			if (brect.width < blobThreshMax && brect.height < blobThreshMax
					&& brect.width > blobThreshMin
					&& brect.height > blobThreshMin
					&& areaEq < (areaEqTol / 1000.0) && hierarchy[i][3] >= 0)
			{
				rectangle(drawing, brect, color2);
				//			          ellipse( drawing, rotRecs[i], color, 2, 8 );
				scenePoints.push_back(
						Point2d(brect.x + brect.width / 2.0,
								brect.y + 3.0 * brect.height / 4.0));
				stringstream ss;
				string s;
				ss << scenePoints.size();
				ss >> s;
				putText(drawing, s,
						Point(scenePoints.back().x, scenePoints.back().y),
						FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 250));
//				if (scenePoints.size() == 1)
//				{
//					Mat subImg = drawing(brect);
//					Mat templ = imread("./data/pattern_0y_2.png");
////					MatchingMethod(subImg, templ);
//
//					featureDetection(subImg, templ);
//				}
			}
		}
	}
	return scenePoints;
}

int loadAiDataFromFile(string fileName, tigers::AugmWrapper * wrapper)
{
	fstream input(fileName, ios::in | ios::binary);
	if (!input)
	{
		cout << fileName << ": File not found." << endl;
		return -2;
	}
	else if (!(*wrapper).ParseFromIstream(&input))
	{
		cerr << "Failed to parse." << endl;
		return -1;
	}
	return 0;
}

void featureDetection(Mat imgIn_1, Mat imgIn_2)
{
	Mat img_1, img_2;

	cvtColor(imgIn_1, img_1, COLOR_BGR2GRAY);
	cvtColor(imgIn_2, img_2, COLOR_BGR2GRAY);

	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match(descriptors_1, descriptors_2, matches);

	double max_dist = 0;
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector<DMatch> good_matches;

	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches,
			img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(),
			DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	imshow("Good Matches", img_matches);
}

void fitBotHomography(Mat drawing, std::vector<Point2f> objPoints,
		std::vector<Point2f> scenePoints, tigers::AugmWrapper wrapper)
{
	if (objPoints.size() > 3)
	{
		Mat H = findHomography(objPoints, scenePoints, RANSAC);
		if (H.rows == 3)
		{
			std::vector<Point2f> scenePointsH;
			perspectiveTransform(objPoints, scenePointsH, H);

			double error = 0;
			for (int i = 0; i < scenePoints.size(); i++)
			{
				Point2f sub = (scenePoints.at(i) - scenePointsH.at(i));
				error += sqrt(sub.x * sub.x + sub.y * sub.y);
			}
			cout << "error=" << error << endl;

			for (Point2f vec : scenePointsH)
			{
				circle(drawing, vec, 10, Scalar(0, 255, 0), 1);
			}

			//-- Get the corners from the image_1 ( the object to be "detected" )
			std::vector<Point2f> obj_corners(4);
			obj_corners[0] = Point2f(wrapper.field().length() / 2.0f,
					wrapper.field().width() / 2.0f);
			obj_corners[1] = Point2f(-wrapper.field().length() / 2.0f,
					wrapper.field().width() / 2.0f);
			obj_corners[2] = Point2f(-wrapper.field().length() / 2.0f,
					-wrapper.field().width() / 2.0f);
			obj_corners[3] = Point2f(wrapper.field().length() / 2.0f,
					-wrapper.field().width() / 2.0f);
			std::vector<Point2f> scene_corners(4);

			perspectiveTransform(obj_corners, scene_corners, H);

			//			cout << obj_corners << endl << scene_corners << endl;

			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
			line(drawing, scene_corners[0] + Point2f(0, 0),
					scene_corners[1] + Point2f(0, 0), Scalar(0, 255, 0), 1);
			line(drawing, scene_corners[1] + Point2f(0, 0),
					scene_corners[2] + Point2f(0, 0), Scalar(0, 255, 0), 1);
			line(drawing, scene_corners[2] + Point2f(0, 0),
					scene_corners[3] + Point2f(0, 0), Scalar(0, 255, 0), 1);
			line(drawing, scene_corners[3] + Point2f(0, 0),
					scene_corners[0] + Point2f(0, 0), Scalar(0, 255, 0), 1);
		}
	}
}

std::vector<Point2f> sortBots(std::vector<Point2f> objPoints,
		std::vector<Point2f> scenePoints)
{
	float thetaRad = theta / 360.0f * 2 * CV_PI;
	float initValue[] =
			{ cos(thetaRad), -sin(thetaRad), 0, sin(thetaRad), cos(thetaRad), 0,
					0, 0, 1 };
	Mat R(3, 3, CV_32F, initValue);
	std::vector<Point2f> objPoints2;
	perspectiveTransform(objPoints, objPoints2, R);

	std::sort(objPoints2.begin(), objPoints2.end(), comparePointsBottom2Top);
	cout << "sorted" << endl << objPoints2 << endl;

	return objPoints2;
}

int main(int argc, char** argv)
{
	cap = VideoCapture(argv[1]);

	namedWindow(window_threshold, WINDOW_AUTOSIZE);

	createTrackbar("LowH", window_threshold, &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", window_threshold, &iHighH, 179);
	createTrackbar("LowS", window_threshold, &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", window_threshold, &iHighS, 255);
	createTrackbar("LowV", window_threshold, &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", window_threshold, &iHighV, 255);

	namedWindow(window_contours, WINDOW_AUTOSIZE);
	int numFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	createTrackbar("blobThreshold min", window_contours, &blobThreshMin, 1000);
	createTrackbar("blobThreshold max", window_contours, &blobThreshMax, 1000);
	createTrackbar("areaEqTol", window_contours, &areaEqTol, 2000);
	createTrackbar("theta", window_contours, &theta, 360);
	createTrackbar("Video", window_contours, &sliderPos, numFrames,
			onTrackbarSlide);

	namedWindow(window_templMatch, WINDOW_NORMAL);
	const char* trackbar_label =
			"Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
	createTrackbar(trackbar_label, window_templMatch, &match_method, 5);

	Mat imgOriginal;
	Mat newImg;
	Mat imgThresholded;
	Mat drawing;
	clock_t lastTime = clock();

	while (true)
	{
		bool bSuccess = cap.read(newImg);
		setTrackbarPos("Video", window_contours, ++sliderPos);

		if (bSuccess)
		{
			imgOriginal = newImg;
		}

		if (imgOriginal.cols == 0)
		{
			cerr << "Could not read resource." << endl;
			exit(1);
		}

		imgThresholded = threshold(imgOriginal);
		imshow("thresholded", imgThresholded);
		imgOriginal.copyTo(drawing);

		std::vector<Point2f> objPoints;
		std::vector<Point2f> scenePoints;
		scenePoints = contours(imgThresholded, drawing, objPoints);

		tigers::AugmWrapper wrapper;
		if (loadAiDataFromFile("./data/frame_1400758110714.bin", &wrapper) != 0)
		{
			exit(2);
		}

		float fieldLength = wrapper.field().length();
		float fieldWidth = wrapper.field().width();

		// draw bot positions into drawing
		for (int i = 0; i < wrapper.bots().size(); i++)
		{
			tigers::Vector pos = wrapper.bots().Get(i).pos();
			Point2f pos2d = Point2f(pos.x(), pos.y());
			drawFieldPos(drawing, pos2d, fieldLength, fieldWidth,
					Scalar(255, 0, 0));
			objPoints.push_back(Point2f(pos.x(), pos.y()));
		}

//		Mat field = drawBWField(objPoints, fieldLength, fieldWidth);
//		imshow("field", field);

		// use feature detection algorithm on thresholded image with artificial bw field
//		featureDetection(imgThresholded, field);

//		Mat templ = imread("./data/pattern_0y_2.png");
//		featureDetection(imgOriginal, templ);

		// hard coded points
//		objPoints.clear();
//		objPoints.push_back(Point2f(-2423, -36));
//		objPoints.push_back(Point2f(-2131, -294));
//		objPoints.push_back(Point2f(-2125, 217));
//		objPoints.push_back(Point2f(-563, 1576));
//		objPoints.push_back(Point2f(-376, -60));
//		objPoints.push_back(Point2f(-336.72208, -920.67688));
//		objPoints.push_back(Point2f(-293, 1301));
////	  objPoints.push_back(Point2f(12.763503, -183.98402));
//		objPoints.push_back(Point2f(175.13538, 803.29797));
//		objPoints.push_back(Point2f(377.94601, 98.685684));
////		objPoints.push_back(Point2f(856.61829, -53.061306));
////		objPoints.push_back(Point2f(2585.5442, -206.7235));

		if (scenePoints.size() > 3)
		{
			objPoints = sortBots(objPoints, scenePoints);
		}

		cout << "objPoints: " << objPoints.size() << endl;
		cout << "scenePoints: " << scenePoints.size() << endl;

		while (objPoints.size() > scenePoints.size())
		{
			objPoints.pop_back();
		}
		while (objPoints.size() < scenePoints.size())
		{
			scenePoints.pop_back();
		}
		cout << objPoints << endl << scenePoints << endl << endl;

		// draw transformed bot positions
		for (int i = 0; i < objPoints.size(); i++)
		{
			drawFieldPos(drawing, objPoints.at(i), fieldLength, fieldWidth,
					Scalar(0, 255, 0));
		}

		// try to find homography
		fitBotHomography(drawing, objPoints, scenePoints, wrapper);

		clock_t curTime = clock();
		double secs = double(curTime - lastTime) / CLOCKS_PER_SEC;
		lastTime = curTime;
		stringstream ss;
		string s;
		ss << int(1 / secs);
		ss >> s;
		putText(drawing, s, Point(drawing.cols - 80, 40), FONT_HERSHEY_PLAIN, 2,
				Scalar(0, 0, 250));

		imshow(window_contours, drawing);

		if (waitKey() == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}
