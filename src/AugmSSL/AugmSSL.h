#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <time.h>

#include "GeoUtil.h"
#include "AiDataLoader.h"
#include "augm_wrapper.pb.h"
#include "FieldModel.h"
#include "Transformer.h"

#include "IHDetector.h"
#include "StaticDetector.h"
#include "FieldLineDetector.h"
#include "BotDetector.h"
#include "LineSampler.h"

#include "PointRenderer.h"
#include "LineRenderer.h"
#include "CircleRenderer.h"
#include "FieldInit.h"
#include "TextRenderer.h"

#include "opencv_compat.h"

enum EDetectorType {
	BOT = 0, LINES = 1, STATIC = 2, MANUEL = 3
};

struct Source {
	std::string capIn;
	std::string aiIn;
	cv::Mat H;
	float fps;
};

enum EInputType {
	INPUT_TYPE_CAM, INPUT_TYPE_FILE
};

struct AugmSSL_params {
	std::string videoFile = "data/Parsian_p2_20140723_113034.cut.mp4";
	std::string aiFilesDir = "data/frames_parsian/";
	int camId = 0;
	int fps = 30;
	int resWidth = 1280;
	int resHeight = 720;
	EInputType inputType = INPUT_TYPE_CAM;
	EDetectorType detectorType = STATIC;
	int port = 42001;
	std::string netAddress = "0.0.0.0";
	std::string mcastAddress = "224.5.23.3";bool continuousTracking = false;bool showDebug =
			false;bool frameByFrame = false;bool handMovement = false;bool fullscreen =
			false;
	double fixedDefH[9] = { .2, 0, 600, 0, .2, 400, 0, 0, 1 };
};

namespace tigers {
class AugmSSL {
private:
	AugmSSL_params params;
	cv::Mat src;
	IHDetector* hDetector = NULL;
	LineSampler* lineSampler = NULL;
	FieldModel* fieldModel = NULL;
	AiDataLoader* aiDataLoader = NULL;
	cv::VideoWriter* outputVideo = NULL;
	AugmWrapper aiDataWrapper;
	cv::Mat mask;
	cv::Size outputSize;

	FieldInit* fi = NULL;
	Transformer transformer;

	PointRenderer pointRenderer;
	LineRenderer lineRenderer;
	CircleRenderer circleRenderer;
	TextRenderer textRenderer;

	cv::Point2f rndTrans = cv::Point2f(0, 0);

	std::vector<std::string> visibleShapeContainers;

	void drawAiData(AugmWrapper aiDataWrapper, cv::Mat &drawing,
			Transformer &transformer);
	void drawReferee(AugmWrapper aiDataWrapper, cv::Mat &drawing, cv::Mat H);
	void drawAllBotDests(AugmWrapper aiDataWrapper, cv::Mat &drawing,
			cv::Mat H);
	void drawAllBots(AugmWrapper aiDataWrapper, cv::Mat &drawing, cv::Mat H);
	void botOcclusion(AugmWrapper aiDataWrapper, cv::Mat &drawing, cv::Mat H,
			cv::Mat &src, cv::Mat &mask);
	void loadAiBots(AugmWrapper aiDataWrapper,
			std::vector<cv::Point_<float>> &modelBots);

public:
	AugmSSL(AugmSSL_params params);
	AugmSSL();
	void run();
	void stop();
	void processFrame();

	cv::Mat H = cv::Mat(cv::Size(3, 3), CV_64F);
	cv::Mat initH = cv::Mat(cv::Size(3, 3), CV_64F);
	bool active = true;
	int frameNumber = 0;
	cv::VideoCapture* cap = NULL;
};

} /* namespace tigers */
