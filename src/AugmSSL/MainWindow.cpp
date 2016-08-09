#include <QLineEdit>
#include <QFileDialog>
#include "MainWindow.h"

#include <iostream>

using namespace std;

MainWindow::MainWindow() {
	gridLayout = new QGridLayout;
	gridLayout->addWidget(createSourceTypeSelection(), 0, 0);

	fileWidget = createFileInputs();
	liveWidget = createLiveInputs();
	gridLayout->addWidget(fileWidget, 1, 0);
	gridLayout->addWidget(liveWidget, 2, 0);

	gridLayout->addWidget(createFieldDetectorSelection(), 3, 0);

	continuesTracking = new QCheckBox("Continues Tracking");
	gridLayout->addWidget(continuesTracking, 4, 0);
	fullscreen = new QCheckBox(tr("&Fullscreen"));
	gridLayout->addWidget(fullscreen, 5, 0);

	startstop = new QPushButton(tr("&Start"));
	gridLayout->addWidget(startstop, 6, 0);
	connect(startstop, SIGNAL(clicked(bool)), this, SLOT(togglePlayback()) );

	// Set layout in QWidget
	QWidget *window = new QWidget();
	window->setLayout(gridLayout);
	setCentralWidget(window);

	setWindowTitle(tr("Augmented SSL"));

	readSettings();
	memcpy(augm.initH.data, defH, 9 * sizeof(double));

	inputChanged();

	setUnifiedTitleAndToolBarOnMac(true);
}

MainWindow::~MainWindow() {
}

QWidget* MainWindow::createFileInputs() {
	QWidget *widget = new QWidget();
	QGridLayout *grid = new QGridLayout;
	widget->setLayout(grid);

	QLabel *lblInput = new QLabel("Input file: ");
	grid->addWidget(lblInput, 0, 0);
	inputFile = new QLineEdit();
	grid->addWidget(inputFile, 0, 1);
	btnFileInput = new QPushButton("Browse");
	grid->addWidget(btnFileInput, 0, 2);
	connect(btnFileInput, SIGNAL(clicked()), this, SLOT(openVideoFile()));

	QLabel *lblAiFile = new QLabel("AI files folder: ");
	grid->addWidget(lblAiFile, 1, 0);
	aiFile = new QLineEdit();
	grid->addWidget(aiFile, 1, 1);
	btnAiFileInput = new QPushButton("Browse");
	grid->addWidget(btnAiFileInput, 1, 2);
	connect(btnAiFileInput, SIGNAL(clicked()), this, SLOT(openAiFile()));

	frameByFrame = new QCheckBox("Frame by frame");
	grid->addWidget(frameByFrame, 2, 0);

	handMovement = new QCheckBox("Simulate hand movement");
	grid->addWidget(handMovement, 3, 0);

	showDebug = new QCheckBox("Show Debug");
	grid->addWidget(showDebug, 4, 0);

	return widget;
}

QWidget* MainWindow::createLiveInputs() {
	QWidget *widget = new QWidget();
	QGridLayout *grid = new QGridLayout;
	widget->setLayout(grid);

	QLabel *lblCamera = new QLabel("Camera: ");
	grid->addWidget(lblCamera, 0, 0);
	camera = new QLineEdit();
	grid->addWidget(camera, 0, 1);

	QLabel *lblMcast = new QLabel("Multicast address: ");
	grid->addWidget(lblMcast, 1, 0);
	mcastAddress = new QLineEdit();
	grid->addWidget(mcastAddress, 1, 1);

	QLabel *lblNetAddress = new QLabel("Network address: ");
	grid->addWidget(lblNetAddress, 2, 0);
	netAddress = new QLineEdit();
	grid->addWidget(netAddress, 2, 1);

	QLabel *lblPort = new QLabel("Port: ");
	grid->addWidget(lblPort, 3, 0);
	port = new QLineEdit();
	grid->addWidget(port, 3, 1);

	QLabel *lblResWidth = new QLabel("Resolution width: ");
	grid->addWidget(lblResWidth, 4, 0);
	resWidth = new QLineEdit();
	grid->addWidget(resWidth, 4, 1);

	QLabel *lblResHeight = new QLabel("Resolution height: ");
	grid->addWidget(lblResHeight, 5, 0);
	resHeight = new QLineEdit();
	grid->addWidget(resHeight, 5, 1);

	return widget;
}

QGroupBox* MainWindow::createSourceTypeSelection() {
	QGroupBox *groupBox = new QGroupBox(tr("Input source type: "));

	radioVideoFile = new QRadioButton(tr("&Video file"));
	radioCamera = new QRadioButton(tr("&Camera"));

	connect(radioCamera, SIGNAL(toggled(bool)), this, SLOT(inputChanged()));
	connect(radioVideoFile, SIGNAL(toggled(bool)), this, SLOT(inputChanged()));

	QHBoxLayout *box = new QHBoxLayout;
	box->addWidget(radioVideoFile);
	box->addWidget(radioCamera);
	box->addStretch(1);
	groupBox->setLayout(box);

	return groupBox;
}

QGroupBox* MainWindow::createFieldDetectorSelection() {
	QGroupBox *groupBox = new QGroupBox(tr("Initial field detector: "));

	radioInitLast = new QRadioButton(tr("&Last homography"));
	radioInitManual = new QRadioButton(tr("&Manual calibration"));
	radioInitBot = new QRadioButton(tr("&Bot detector"));
	radioInitLine = new QRadioButton(tr("&Line detector"));

	radioInitLast->setChecked(true);

	QHBoxLayout *box = new QHBoxLayout;
	box->addWidget(radioInitLast);
	box->addWidget(radioInitManual);
	box->addWidget(radioInitBot);
	box->addWidget(radioInitLine);
	box->addStretch(1);
	groupBox->setLayout(box);

	return groupBox;
}

void MainWindow::openVideoFile() {
	QFileInfo fileInfo(inputFile->text());
	QString fileName = QFileDialog::getOpenFileName(this, "Select file",
			fileInfo.absoluteDir().absolutePath());
	if (!fileName.isEmpty())
		inputFile->setText(fileName);
}

void MainWindow::openAiFile() {
	QString fileName = QFileDialog::getExistingDirectory(this, "Select dir",
			aiFile->text());
	if (!fileName.isEmpty())
		aiFile->setText(fileName);
}

void MainWindow::inputChanged() {
	if (radioCamera->isChecked()) {
		fileWidget->setDisabled(true);
		liveWidget->setDisabled(false);
	} else {
		fileWidget->setDisabled(false);
		liveWidget->setDisabled(true);
	}
	this->update();
}

void MainWindow::stopPlayback() {
	startstop->setText(tr("&Start"));
	memcpy(defH, augm.initH.data, 9 * sizeof(double));
	augm.stop();
}

void MainWindow::startPlayback() {
	startstop->setText(tr("&Stop"));
	if (augm.active)
		return;
	AugmSSL_params params;
	params.frameByFrame = frameByFrame->isChecked();
	params.showDebug = showDebug->isChecked();
	params.videoFile = inputFile->text().toStdString();
	params.aiFilesDir = aiFile->text().toStdString();
	params.continuousTracking = continuesTracking->isChecked();
	params.detectorType = getDetectorType();
	params.camId = camera->text().toInt();
	params.fullscreen = fullscreen->isChecked();
	params.handMovement = handMovement->isChecked();
	params.inputType =
			radioCamera->isChecked() ? INPUT_TYPE_CAM : INPUT_TYPE_FILE;
	params.mcastAddress = mcastAddress->text().toStdString();
	params.netAddress = netAddress->text().toStdString();
	params.port = port->text().toInt();
	params.resWidth = resWidth->text().toInt();
	params.resHeight = resHeight->text().toInt();
	memcpy(params.fixedDefH, defH, 9 * sizeof(double));

	augm = tigers::AugmSSL(params);
	augm.run();
	stopPlayback();
}

void MainWindow::togglePlayback() {
	if (augm.active) {
		stopPlayback();
	} else {
		startPlayback();
	}
}

EDetectorType MainWindow::getDetectorType() {
	if (radioInitLast->isChecked())
		return STATIC;
	if (radioInitBot->isChecked())
		return BOT;
	if (radioInitLine->isChecked())
		return LINES;
	if (radioInitManual->isChecked())
		return MANUEL;
	throw "No detector type selected?!";
}

void MainWindow::readSettings() {
	QSettings settings("Tigers", "AugmSSL");
	QPoint pos = settings.value("pos", QPoint(200, 200)).toPoint();
	QSize size = settings.value("size", QSize(400, 400)).toSize();
	resize(size);
	move(pos);

	continuesTracking->setChecked(
			settings.value("continuesTracking", "false").toString() == "true");
	fullscreen->setChecked(
			settings.value("fullscreen", "false").toString() == "true");

	inputFile->setText(settings.value("inputVideo", "").toString());
	aiFile->setText(settings.value("aiFile", "").toString());
	frameByFrame->setChecked(
			settings.value("frameByFrame", "false").toString() == "true");
	handMovement->setChecked(
			settings.value("handMovement", "false").toString() == "true");

	radioCamera->setChecked(
			settings.value("useCamera", "true").toString() == "true");
	radioVideoFile->setChecked(
			settings.value("useVideoFile", "false").toString() == "true");

	camera->setText(settings.value("camera", "").toString());
	mcastAddress->setText(
			settings.value("mcastAddress", "224.5.23.3").toString());
	netAddress->setText(settings.value("netAddress", "0.0.0.0").toString());
	port->setText(settings.value("port", "42001").toString());
	resWidth->setText(settings.value("resWidth", "1280").toString());
	resHeight->setText(settings.value("resHeight", "720").toString());

	radioInitLast->setChecked(
			settings.value("initLast", "true").toString() == "true");
	radioInitManual->setChecked(
			settings.value("initManual", "false").toString() == "true");
	radioInitLine->setChecked(
			settings.value("initLine", "false").toString() == "true");
	radioInitBot->setChecked(
			settings.value("initBot", "false").toString() == "true");

	int defHsize = settings.beginReadArray("defH");
	if (defHsize == 9) {
		for (int i = 0; i < 9; i++) {
			settings.setArrayIndex(i);
			defH[i] = settings.value("defH", defH[i]).toDouble();
		}
	} else {
		std::cerr << "Invalid defH size: " << defHsize << std::endl;
	}
	settings.endArray();
}

void MainWindow::writeSettings() {
	QSettings settings("Tigers", "AugmSSL");
	settings.setValue("pos", pos());
	settings.setValue("size", size());

	settings.setValue("continuesTracking", continuesTracking->isChecked());
	settings.setValue("fullscreen", fullscreen->isChecked());

	settings.setValue("inputVideo", inputFile->text());
	settings.setValue("aiFile", aiFile->text());
	settings.setValue("frameByFrame", frameByFrame->isChecked());
	settings.setValue("handMovement", handMovement->isChecked());

	settings.setValue("useCamera", radioCamera->isChecked());
	settings.setValue("useVideoFile", radioVideoFile->isChecked());

	settings.setValue("camera", camera->text());
	settings.setValue("mcastAddress", mcastAddress->text());
	settings.setValue("netAddress", netAddress->text());
	settings.setValue("port", port->text());
	settings.setValue("resWidth", resWidth->text());
	settings.setValue("resHeight", resHeight->text());

	settings.setValue("initLast", radioInitLast->isChecked());
	settings.setValue("initManual", radioInitManual->isChecked());
	settings.setValue("initLine", radioInitLine->isChecked());
	settings.setValue("initBot", radioInitBot->isChecked());

	settings.beginWriteArray("defH");
	for (int i = 0; i < 9; i++) {
		settings.setArrayIndex(i);
		settings.setValue("defH", defH[i]);
	}
	settings.endArray();
}

void MainWindow::closing() {
	stopPlayback();
	writeSettings();
}

int main(int argc, char** argv) {
//    Q_INIT_RESOURCE(application);

	SafeQApplication app(argc, argv);
	app.setOrganizationName("Tigers");
	app.setApplicationName("AugmSSL");
	MainWindow mainWin;
#if defined(Q_OS_SYMBIAN)
	mainWin.showMaximized();
#else
	mainWin.show();
#endif
	QObject::connect(&app, SIGNAL(aboutToQuit()), &mainWin, SLOT(closing()));
	return app.exec();
}
