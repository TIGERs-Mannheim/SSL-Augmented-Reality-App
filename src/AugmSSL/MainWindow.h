#pragma once

#include <QtGui>
#include <QMainWindow>
#include <QApplication>
#include "AugmSSL.h"
#include <iostream>

class SafeQApplication: public QApplication {
public:
	SafeQApplication(int &argc, char* argv[]) :
			QApplication(argc, argv) {
	}
	virtual ~SafeQApplication() {
	}
	virtual bool notify(QObject * receiver, QEvent * e) {
		try {
			return QApplication::notify(receiver, e);
		} catch (std::exception& err) {
			std::cerr << "Exception caught: " << err.what() << std::endl;
			return true;
		}
	}
};

class MainWindow: public QMainWindow {
Q_OBJECT

public:
	MainWindow();
	virtual ~MainWindow();

protected:

private slots:
	void closing();
	void inputChanged();
	void openVideoFile();
	void openAiFile();
	void startPlayback();
	void stopPlayback();
	void togglePlayback();

private:
	void readSettings();
	void writeSettings();
	EDetectorType getDetectorType();

	tigers::AugmSSL augm;
	double defH[9] = { .2, 0, 600, 0, .2, 400, 0, 0, 1 };

	QGroupBox* createSourceTypeSelection();
	QGroupBox* createFieldDetectorSelection();
	QWidget* createFileInputs();
	QWidget* createLiveInputs();

	QGridLayout *gridLayout;
	QWidget* fileWidget;
	QWidget* liveWidget;

	QCheckBox *continuesTracking;
	QCheckBox *fullscreen;
	QCheckBox *showDebug;
	QPushButton *startstop;

	QPushButton* btnFileInput;
	QPushButton* btnAiFileInput;
	QLineEdit *inputFile;
	QLineEdit *aiFile;
	QCheckBox *frameByFrame;
	QCheckBox *handMovement;

	QRadioButton* radioCamera;
	QRadioButton *radioVideoFile;
	QLineEdit *camera;
	QLineEdit *mcastAddress;
	QLineEdit *netAddress;
	QLineEdit *port;
	QLineEdit *resWidth;
	QLineEdit *resHeight;

	QRadioButton *radioInitLast;
	QRadioButton *radioInitManual;
	QRadioButton *radioInitBot;
	QRadioButton *radioInitLine;

	QAction *newAct;
};
