// http://opencv-srf.blogspot.de/2010/09/object-detection-using-color-seperation.html

#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>

#ifndef CAP_PROP_POS_FRAMES
#define CAP_PROP_POS_FRAMES CV_CAP_PROP_POS_FRAMES
#endif
#ifndef CAP_PROP_FRAME_COUNT
#define CAP_PROP_FRAME_COUNT CV_CAP_PROP_FRAME_COUNT
#endif

using namespace cv;
using namespace std;

//RNG rng(12345);
//Scalar color1 = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
//      rng.uniform(0, 255));
//Scalar color2 = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
//      rng.uniform(0, 255));
//Scalar color3 = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
//      rng.uniform(0, 255));

Scalar color1 = Scalar(255,0,0); // red
Scalar color2 = Scalar(0,255,0); // green
Scalar color3 = Scalar(0,0,255); // blue
Scalar color4 = Scalar(255,0,255); // magenta?
Scalar white = Scalar(1);


int blobThreshMin = 40;
int blobThreshMax = 200;
int areaEqTol = 600;

VideoCapture cap;
int sliderPos = 0;

void onTrackbarSlide(int current_frame, void*)
{
    cap.set(CAP_PROP_POS_FRAMES,sliderPos);
}

void reduceImageMask(Mat& src, Mat& dst)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_TREE,
                 CHAIN_APPROX_SIMPLE, Point(0, 0));


    double largest_area = 0;
    int largest_contour_index = -1;
    for(int i = 0; i<contours.size() ; i++){
         double a=contourArea( contours[i],false);  //  Find the area of contour
         if(a>largest_area)
         {
             largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }
    drawContours( dst, contours,largest_contour_index, white, -1, 8); // Draw the largest contour using previously stored index.
}

void optimizeChannel(Mat& src, Mat& dst)
{
    Mat temp;
    src.copyTo(temp);
    equalizeHist(src,dst);
    threshold(temp, dst, 230, 255, THRESH_BINARY);
}

void optimizeImage(Mat& src, Mat& dst)
{
    std::vector<Mat> channels(3), outChannels(3);

    split(src, channels);
    for(int i=0; i<3;i++)
    {
        optimizeChannel(channels[i], outChannels[i]);
    }
    merge(outChannels, dst);


}


void findCircles(Mat& src, Mat& dst, vector<KeyPoint>& points){
    src.copyTo(dst);
    Mat grey;
    cvtColor(src, grey, COLOR_BGR2GRAY);

// set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 1.0f;
    params.filterByInertia = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = false;
    params.minArea = blobThreshMin;
    params.maxArea = blobThreshMax;

// ... any other params you don't want default value

// set up and create the detector using the parameters
   SimpleBlobDetector blob_detector(params);

            //blob_detector->read(fn);
// detect
    vector<KeyPoint> keypoints;
    blob_detector.detect(grey, keypoints);

    for (int i=0; i<keypoints.size(); i++){
        // float X=keypoints[i].pt.x;
        // float Y=keypoints[i].pt.y;
        // float sz = keypoints[i].size;

        circle(dst, keypoints[i].pt, keypoints[i].size, Scalar(255, 255, 255),-1);

    }
}

void findBlobs(Mat& src, Mat& dst, vector<KeyPoint>& points)
{
	Mat canny_output;
	Mat src_gray;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cvtColor( src, src_gray, COLOR_BGR2GRAY );
	int thresh = 10;

	/// Detect edges using canny
	Canny( src_gray, canny_output, thresh, thresh*2, 5 );

	canny_output.copyTo(dst);

	/// Find contours
	findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

	std::vector<RotatedRect> rotRecs;
	std::vector<Point2d> blobCenters;

	for (size_t i = 0; i < contours.size(); i++)
	{
		RotatedRect rotRect = minAreaRect(contours[i]);
		rotRecs.push_back(rotRect);
		Rect brect = rotRect.boundingRect();
		blobCenters.push_back(Point2d(brect.x + brect.width / 2.0,
				brect.y + brect.height / 2.0));
		double area = contourArea(contours[i]);

		Scalar color = color1;
		if (hierarchy[i][3] < 0)
		{
			color = color3;
		}
		if(i==0)
		{
			color = color2;
		}
		drawContours(dst, contours, (int) i, color, 1, 8, hierarchy, 0,
				Point());
	}

	std::sort(blobCenters.begin(), blobCenters.end(), [](Point2d p1, Point2d p2)
		{
			return (p1.x < p2.x);
		});
}


int main(int argc, char** argv) {
    //~ VideoCapture cap(0); //capture the video from web cam
    //~
    //~ if ( !cap.isOpened() )  // if not success, exit program
    //~ {
    //~ cout << "Cannot open the web cam" << endl;
    //~ return -1;
    //~ }

    string window_threshold = "Threshold";
    string window_original = "Contours";
    string window_combined = "Combined";
    //  string window_houghLines = "HoughLines";
    namedWindow(window_threshold, WINDOW_AUTOSIZE);
    namedWindow(window_original, WINDOW_AUTOSIZE);
    namedWindow(window_combined, WINDOW_AUTOSIZE);
    //  namedWindow(window_houghLines, WINDOW_AUTOSIZE);

    int iLowH = 52;
    int iHighH = 96;

    int iLowS = 27;
    int iHighS = 255;

    int iLowV = 91;
    int iHighV = 255;

    createTrackbar("LowH", window_threshold, &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", window_threshold, &iHighH, 179);

    createTrackbar("LowS", window_threshold, &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", window_threshold, &iHighS, 255);

    createTrackbar("LowV", window_threshold, &iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", window_threshold, &iHighV, 255);


    createTrackbar("blobThreshold min", window_original, &blobThreshMin, 1000);
    createTrackbar("blobThreshold max", window_original, &blobThreshMax, 1000);
    createTrackbar("areaEqTol", window_original, &areaEqTol, 2000);

//  int cannyThreshold1 = 20;
//  int cannyThreshold2 = 100;
//  int houghThreshold = 150;
//  int minLineLength = 10;
//  int maxLineGap = 0;
//  int rho = 1;
//  createTrackbar("cannyThreshold1", window_houghLines, &cannyThreshold1, 255);
//  createTrackbar("cannyThreshold2", window_houghLines, &cannyThreshold2, 255);
//  createTrackbar("houghThreshold", window_houghLines, &houghThreshold, 255);
//  createTrackbar("minLineLength", window_houghLines, &minLineLength, 255);
//  createTrackbar("maxLineGap", window_houghLines, &maxLineGap, 255);
//  createTrackbar("rho", window_houghLines, &rho, 255);

    Mat imgOriginal;
    Mat imgHSV;
    Mat imgThresholded;
    Mat probabilistic_hough;
    Mat edges;
    Mat combined, mask;
    Mat imgThresholdedBGR;
//  VideoCapture cap(argv[1]);
    cap = VideoCapture(argv[1]);
    int numFrames = cap.get(CAP_PROP_FRAME_COUNT);
    createTrackbar("Video", window_original, &sliderPos, numFrames, onTrackbarSlide);

    while (true) {
//      imgOriginal = imread(argv[1], 1);


       bool bSuccess = cap.read(imgOriginal);
       setTrackbarPos("Video", window_original, ++sliderPos);

       if (!bSuccess)
       {
        cout << "Cannot read a frame from video stream" << endl;
        break;
       }

        //Convert the captured frame from BGR to HSV
    imshow(window_original, imgOriginal);
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        //Threshold the image
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
            Scalar(iHighH, iHighS, iHighV), imgThresholded);

        //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded,
          getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded,
           getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        //morphological closing (removes small holes from the foreground)
    dilate(imgThresholded, imgThresholded,
           getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    erode(imgThresholded, imgThresholded,
          getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));

        //show the thresholded image





    // Mask original image to remove noisy background

    // cvtColor(imgThresholded,imgThresholdedBGR, COLOR_HSV2BGR);
    mask = Mat::zeros(imgOriginal.size(), CV_8UC1);
    reduceImageMask(imgThresholded, mask);
    imgOriginal.copyTo(combined, mask);
    imshow(window_threshold, combined);

    imgThresholdedBGR = Mat::zeros(imgOriginal.size(), CV_8UC3);
    optimizeImage(combined, imgThresholdedBGR);
    vector<KeyPoint> points;
//    findCircles(imgThresholdedBGR, imgThresholdedBGR, points);
    Mat blobs = Mat::zeros(imgThresholdedBGR.size(), CV_8UC1);
    findBlobs(imgThresholdedBGR, blobs, points);
    imshow("Blobs", blobs);


//          polyContours.resize(contours.size());
//          approxPolyDP(Mat(contours[i]), polyContours[i], 3, true);
//          drawContours(drawing, polyContours, (int) i, color4, 1, 8, hierarchy, 0,
//                  Point());

    imshow(window_combined, imgThresholdedBGR);


//      vector<Vec4i> p_lines;
//
//      /// Apply Canny edge detector
//      Canny(imgThresholded, edges, cannyThreshold1, cannyThreshold2, 3);
//
//      cvtColor(edges, probabilistic_hough, COLOR_GRAY2BGR);
//
//      /// 2. Use Probabilistic Hough Transform
//      HoughLinesP(edges, p_lines, rho, CV_PI / 180, houghThreshold, minLineLength, maxLineGap);
//
//      /// Show the result
//      for (size_t i = 0; i < p_lines.size(); i++) {
//          Vec4i l = p_lines[i];
//          line(probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]),
//                  color3, 3, LINE_AA);
//      }
//      imshow(window_houghLines, probabilistic_hough);

        //~ imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;
}
