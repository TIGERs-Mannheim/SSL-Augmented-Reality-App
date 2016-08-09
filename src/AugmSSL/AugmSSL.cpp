/*
 * LineDetection.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "AugmSSL.h"

using namespace cv;
using namespace std;
using namespace tigers;

static int handleError(int status, const char* func_name, const char* err_msg,
		const char* file_name, int line, void* userdata) {
	//Do nothing -- will suppress console output
	return 0;   //Return value is not used
}

void AugmSSL::botOcclusion(AugmWrapper aiDataWrapper, Mat& drawing, Mat H,
		Mat& src, Mat& mask) {
	vector<Point2f> botPosReal;
	for (int i = 0; i < aiDataWrapper.bots_size(); i++) {
		Point2f botPos(aiDataWrapper.bots(i).pos().x(),
				aiDataWrapper.bots(i).pos().y());
		botPosReal.push_back(botPos);
	}

	Size s = src.size();
	mask = Mat::zeros(s, CV_8U);
	Point2f minmin(-70, -70);
	Point2f maxmax(70, 70);

	Rect roi;
	Mat roiImg, roiThresh, roiImgGray;

	float tVal;
	redirectError(handleError);
	if (!botPosReal.empty()) {
		vector<Point2f> botPosOut;
		perspectiveTransform(botPosReal, botPosOut, H);
		for (int i = 0; i < botPosOut.size(); i++) {
			// rectangle(mask, botPosOut.at(i)+ll, botPosOut.at(i)+ru, Scalar(125, 125, 125), -1);
			Point2f p1, p2;
			p1 = botPosOut.at(i) + minmin;
			p2 = botPosOut.at(i) + maxmax;
			p1.x = p1.x > 0 ? p1.x : 0;
			p1.y = p1.y > 0 ? p1.y : 0;

			p2.x = p2.x < s.width ? p2.x : s.width - 1;
			p2.y = p2.y < s.height ? p2.y : s.height - 1;

			roi = Rect(p1, p2);
			try {
				roiImg = drawing(roi);
			} catch (cv::Exception& e) {
				// bot is out of image
				continue;
			}
			cvtColor(roiImg, roiImgGray, COLOR_BGR2GRAY);
			tVal = mean(roiImg)[0];
			threshold(roiImgGray, roiThresh, tVal, 255, 1);
			// cvtColor(roiThresh, roiThresh, COLOR_GRAY2BGR);
			GaussianBlur(roiThresh, roiThresh, Size(9, 9), 12);
			roiThresh.copyTo(mask(roi));
		}
	}
	redirectError(NULL);
}

void AugmSSL::drawAllBots(AugmWrapper aiDataWrapper, Mat& drawing, Mat H) {
	vector<Point2f> botPosReal;
	for (int i = 0; i < aiDataWrapper.bots_size(); i++) {
		Point2f botPos(aiDataWrapper.bots(i).pos().x(),
				aiDataWrapper.bots(i).pos().y());
		botPosReal.push_back(botPos);
	}
	if (!botPosReal.empty()) {
		vector<Point2f> botPosOut;
		perspectiveTransform(botPosReal, botPosOut, H);
		for (int i = 0; i < botPosOut.size(); i++) {
			circle(drawing, botPosOut.at(i), 10, Scalar(125, 125, 125), 2);
		}
	}
}

void AugmSSL::drawReferee(AugmWrapper aiDataWrapper, Mat& drawing, Mat H) {
	int us = aiDataWrapper.referee().timeleft();
	int minutes = us / 6e7;
	us -= minutes * 6e7;
	int seconds = us / 1e6;
	us -= seconds * 1e6;
	int ms = us / 1000;

	stringstream ss;
	ss << "State: " << aiDataWrapper.referee().gamestate() << "   Stage: "
			<< aiDataWrapper.referee().stage() << "   Time left: " << minutes
			<< ":" << seconds << "," << ms << "   Score: "
			<< aiDataWrapper.referee().teamyellow().score() << " : "
			<< aiDataWrapper.referee().teamblue().score();
	putText(drawing, ss.str(), cv::Point(30, drawing.rows - 50),
	CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);
}

void AugmSSL::drawAiData(AugmWrapper aiDataWrapper, Mat& drawing,
		Transformer& transformer) {
//	drawAllBots(aiDataWrapper, drawing, H);
	for(int i=0;i<aiDataWrapper.shapecollections_size();i++)
	{
		ShapeCollection shapeColl = aiDataWrapper.shapecollections(i);
		circleRenderer.render(&shapeColl, drawing, transformer);
		lineRenderer.render(&shapeColl, drawing, transformer);
		pointRenderer.render(&shapeColl, drawing, transformer);
		textRenderer.render(&shapeColl, drawing, transformer);
	}
	drawReferee(aiDataWrapper, drawing, transformer.H);
}

void AugmSSL::loadAiBots(AugmWrapper aiDataWrapper,
		vector<Point2f>& modelBots) {
	for (int i = 0; i < aiDataWrapper.bots_size(); i++) {
		modelBots.push_back(
				Point2f(aiDataWrapper.bots(i).pos().x(),
						aiDataWrapper.bots(i).pos().y()));
	}
}

static void newFramePos(int, void* userdata) {
	AugmSSL *as = (AugmSSL *) userdata;
	if (as->cap->get(CV_CAP_PROP_POS_FRAMES) != as->frameNumber) {
		as->cap->set(CV_CAP_PROP_POS_FRAMES, as->frameNumber);
	}
}

static void trackbarCallback(int, void* userdata) {
	AugmSSL *as = (AugmSSL *) userdata;
	as->processFrame();
}

void AugmSSL::processFrame() {
	// catch invalid src image
	if (src.rows == 0) {
		return;
	}

	if (params.handMovement) {
		float x = rand() % 11 - 5;
		float y = rand() % 11 - 5;
		rndTrans += Point2f(x, y);
		Mat M(Size(3, 3), CV_32F, new float[9] { 1, 0, rndTrans.x, 0, 1,
				rndTrans.y, 0, 0, 1 });
		warpPerspective(src, src, M, Size(src.cols, src.rows), INTER_CUBIC);
	}

	clock_t frameStartTime = clock();
	cv::Mat imgDst = src.clone();
	vector<Point2f> modelBots;
	loadAiBots(aiDataWrapper, modelBots);

	if (params.showDebug) {
		hDetector->findTransformation(src, imgDst, modelBots, H);
	} else {
		Mat dummy = Mat();
		hDetector->findTransformation(src, dummy, modelBots, H);
	}

	transformer.calibrate(src, H, modelBots, *fieldModel);
	botOcclusion(aiDataWrapper, imgDst, H, src, mask);
	src.copyTo(imgDst, mask);
	fieldModel->draw(H, imgDst);
	drawAiData(aiDataWrapper, imgDst, transformer);

	if (outputSize.height != 0) {
		resize(imgDst, imgDst, outputSize);
	}

	float time = ((float) (clock() - frameStartTime) / CLOCKS_PER_SEC);
	float fps = 1 / time;
	char charString[8];
	sprintf(charString, "%.1fms", time * 1000);
	putText(imgDst, string(charString), cv::Point(imgDst.cols - 120, 30),
	CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0), 2);

	cv::imshow("image", imgDst);
}

AugmSSL::AugmSSL(AugmSSL_params params) {
	// Verify that the version of the library that we linked against is
	// compatible with the version of the headers we compiled against.
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	this->params = params;

	visibleShapeContainers.push_back("PATHS");
	visibleShapeContainers.push_back("ROLE_NAMES");
}

AugmSSL::AugmSSL() {
	// Verify that the version of the library that we linked against is
	// compatible with the version of the headers we compiled against.
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	active = false;
}

void AugmSSL::run() {
	try {
		H = Mat(Size(3, 3), CV_64F, params.fixedDefH);

		outputSize = Size(0, 0);

		if (params.inputType == INPUT_TYPE_CAM) {
			AiDataLoaderNetwork* aiDataLoaderNetwork = new AiDataLoaderNetwork(
					params.netAddress, params.mcastAddress, params.port);
			aiDataLoader = aiDataLoaderNetwork;

			cout << "Waiting for first data package..." << endl;
			while (!aiDataLoaderNetwork->receivedData) {
				waitKey(100);
				if (!active) {
					return;
				}
			}
			cout << "First data package arrived." << endl;
			cap = new VideoCapture(); // camera
			bool resSet = cap->set(CV_CAP_PROP_FRAME_WIDTH,params.resWidth);
			resSet = resSet && cap->set(CV_CAP_PROP_FRAME_HEIGHT,params.resHeight);
			if(!resSet) cerr << "Could not set resolution " << params.resWidth << "x" << params.resHeight << endl;
			cap->open(params.camId);
		} else {
			cap = new VideoCapture(params.videoFile);
			params.fps = cap->get(CV_CAP_PROP_FPS);
			aiDataLoader = new AiDataLoaderFile(params.aiFilesDir, params.fps);
		}
		if (!cap->isOpened()) {
			cerr << "Could not open VideoCapture. " << endl;
			delete cap;
			return;
		}
		AugmWrapper augmData = aiDataLoader->getCurrent();
		fieldModel = new FieldModel(augmData.field().width(),
				augmData.field().length());
		lineSampler = new LineSampler(fieldModel);
		switch (params.detectorType) {
		case BOT:
			hDetector = new BotDetector();
			break;
		case LINES:
			hDetector = new FieldLineDetector(fieldModel);
			break;
		case STATIC:
			hDetector = new StaticDetector(H);
			break;
		case MANUEL:
			fi = new FieldInit(fieldModel);
			hDetector = lineSampler;
			break;
		default:
			cerr << "Invalid detection type" << endl;
			exit(1);
		}

		if (params.showDebug) {
			hDetector->showControls(&trackbarCallback, this);
		}

		cap->read(src);
		if (fi != NULL && !fi->isInitialized) {
//			std::cout << "H_before=" << std::endl << H << std::endl;
			bool init = fi->initializeField(H, src.clone());
			if (!init) {
				stop();
			} else {
				initH = H.clone();
				cout << "Change initH to " << initH << endl;
//				std::cout << "H_after=" << std::endl << H << std::endl;
				if (params.continuousTracking)
					hDetector = lineSampler;
				else
					hDetector = new StaticDetector(H);
			}
		}

		namedWindow("image", WINDOW_NORMAL);
//		if (params.fullscreen)
//			cvSetWindowProperty("image", CV_WND_PROP_FULLSCREEN,
//					CV_WINDOW_FULLSCREEN);
		int maxFrameCount = 1;
		if (params.inputType == INPUT_TYPE_FILE) {
			maxFrameCount = (int) cap->get(CV_CAP_PROP_FRAME_COUNT);
			createTrackbar("", "image", &frameNumber, maxFrameCount,
					&newFramePos, this);
		}

		clock_t clockPostFrame = clock();
		while (active) {
			float time = ((float) (clock() - clockPostFrame) / CLOCKS_PER_SEC);
			int tWait = (int) ((1000 / (params.fps)) - (time * 1000));
			if (tWait < 1)
				tWait = 1;
			char key = waitKey(tWait);
			if (key == 27) //wait for 'esc' key press 27
				stop();
			if (params.inputType != INPUT_TYPE_CAM
					&& frameNumber >= maxFrameCount - 2)
				stop();
			if (params.frameByFrame && key != 32) { // space? 32
				continue;
			}

			cap->read(src);

			void* wh = cvGetWindowHandle("image");
			if (wh == NULL) {
				stop();
				break;
			}

			if (params.inputType == INPUT_TYPE_FILE) {
				frameNumber = (int) cap->get(CV_CAP_PROP_POS_FRAMES);
				setTrackbarPos("", "image", frameNumber);
				aiDataWrapper = aiDataLoader->get(frameNumber - 1);
			} else {
				frameNumber = -2;
				aiDataWrapper = aiDataLoader->getNext();
			}
			try {
				processFrame();
			} catch (std::exception& err) {
				std::cerr << "Exception in processFrame(): " << err.what()
						<< std::endl;
			}

			// go over to line sampler after first frame
			if (params.continuousTracking)
				hDetector = lineSampler;
			clockPostFrame = clock();
		}
		destroyAllWindows();
		delete cap;
		cap = NULL;
	} catch (std::exception& err) {
		std::cerr << "Exception in augm run(): " << err.what() << std::endl;
	}
}

void AugmSSL::stop() {
	active = false;
	if (fi != NULL)
		fi->active = false;
}
