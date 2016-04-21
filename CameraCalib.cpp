#include "CameraCalib.h"

//#include <QPainter>
#include <QFormLayout>
#include <QRegExpValidator>
#include <QFileDialog>
#include <biotracker/Registry.h>
#include <biotracker/settings/Settings.h>

using namespace BioTracker::Core;
using namespace std;
using namespace cv;

extern "C" {
    void registerTracker() {
        BioTracker::Core::Registry::getInstance().registerTrackerType<CameraCalib>("CameraCalibration");
    }
}

CameraCalib::CameraCalib(Settings & settings)
    : TrackingAlgorithm(settings) {

    m_curView = "";
    initToolsFrame();
}

CameraCalib::~CameraCalib(){}

void CameraCalib::postConnect() {
    Q_EMIT registerViews( { { "Chessboard Corners"}, {"Rectified Image"} });
}

void CameraCalib::track(size_t , const cv::Mat & frame) {
    m_curFrame = frame;

    if (m_curView == "Chessboard Corners" && !m_curFrame.empty()) {
        try {
            shownChessboardCorners = getChessboardCorners();
        } catch (int e) {
            if (e == 1) {
                shownChessboardCorners.clear();
            } else {
                Q_EMIT notifyGUI("Unknown exception while searching for chessboard corner: "+ QString::number(e).toStdString(), MessageType::WARNING);
            }
        }
    }
}

void CameraCalib::paint(size_t frameNumber, ProxyMat &m, View const &view) {
    m_frameNumber = frameNumber;

    if (view.name != m_curView) {
        m_curView = view.name;
        Q_EMIT forceTracking();
    }

    if (!m_curFrame.empty() && m_curView == "Chessboard Corners" && !shownChessboardCorners.empty()) {
        Size2i boardSize(m_boardWidthEdit->text().toInt(), m_boardHeightEdit->text().toInt());
        m_curFrame.copyTo(m.getMat());
        drawChessboardCorners(m.getMat(), boardSize, shownChessboardCorners, true);
    } else if (m_curView == "Rectified Image" && !m_curFrame.empty() && !m_cameraMatrix.empty() && !m_distCoeffs.empty()) {
        undistort(m_curFrame, m.getMat(), m_cameraMatrix, m_distCoeffs);
    }
}

void CameraCalib::initToolsFrame() {
    QFormLayout *layout = new QFormLayout(getToolsWidget());

    // Square size settings
    m_squareSizeEdit = new QLineEdit(getToolsWidget());
    QRegExpValidator* squareSizeValidator = new QRegExpValidator(QRegExp("[0-9]+(.[0-9]+)?")); // Just accept whole numbers for square size
    m_squareSizeEdit->setValidator(squareSizeValidator);
    m_squareSizeEdit->setText("2.0");
    layout->addRow("&Square size (in cm):", m_squareSizeEdit);

    // Board size settings
    QRegExpValidator* boardSizeValidator = new QRegExpValidator(QRegExp("[1-9][0-9]*")); // Just accept whole numbers for number of squares
    m_boardWidthEdit = new QLineEdit(getToolsWidget());
    m_boardWidthEdit->setValidator(boardSizeValidator);
    m_boardWidthEdit->setText("9");
    layout->addRow("&Board width (inner corners):", m_boardWidthEdit);
    m_boardHeightEdit = new QLineEdit(getToolsWidget());
    m_boardHeightEdit->setValidator(boardSizeValidator);
    m_boardHeightEdit->setText("6");
    layout->addRow("&Board height (inner corners):", m_boardHeightEdit);

    // Calibration flags for OpenCV
    m_zeroTangentDistCB = new QCheckBox("ZERO_TANGENT_DIST", getToolsWidget());
    layout->addRow(m_zeroTangentDistCB);
    m_rationalModelCB = new QCheckBox("RATIONAL_MODEL", getToolsWidget());
    layout->addRow(m_rationalModelCB);
    m_fixPrincipalPointCB = new QCheckBox("CALIB_FIX_PRINCIPAL_POINT", getToolsWidget());
    layout->addRow(m_fixPrincipalPointCB);
    m_highPolynomeDegree = new QCheckBox("High Polynome Degree", getToolsWidget());
    layout->addRow(m_highPolynomeDegree);

    // Buttons
    m_addFrameBut = new QPushButton("Add frame", getToolsWidget());
    QObject::connect(m_addFrameBut, SIGNAL(clicked()), this, SLOT(addFrame()));
    layout->addRow(m_addFrameBut);

    m_addAllFramesBut = new QPushButton("Add all Frames", getToolsWidget());
    QObject::connect(m_addAllFramesBut, SIGNAL(clicked()), this, SLOT(addAllFrames()));
    layout->addRow(m_addAllFramesBut);
    boost::optional<uint8_t> mediaTypeOpt = m_settings.maybeGetValueOfParam<uint8_t>(GuiParam::MEDIA_TYPE);
    if (!mediaTypeOpt || GuiParam::MediaType::NoMedia == static_cast<GuiParam::MediaType>(*mediaTypeOpt) ||
            GuiParam::MediaType::Camera == static_cast<GuiParam::MediaType>(*mediaTypeOpt)) {
        m_addAllFramesBut->setDisabled(true);
    }

    m_resetSelectedFramesBut = new QPushButton("Reset selected frames", getToolsWidget());
    QObject::connect(m_resetSelectedFramesBut, SIGNAL(clicked()), this, SLOT(resetSelectedFrames()));
    layout->addRow(m_resetSelectedFramesBut);
    m_resetSelectedFramesBut->setDisabled(true);

    m_calibrateBut = new QPushButton("Calibrate", getToolsWidget());
    QObject::connect(m_calibrateBut, SIGNAL(clicked()), this, SLOT(calibrate()));
    layout->addRow(m_calibrateBut);
    m_calibrateBut->setDisabled(true);

    m_saveCalibrationBut = new QPushButton("Save Calibration", getToolsWidget());
    QObject::connect(m_saveCalibrationBut, SIGNAL(clicked()), this, SLOT(saveCalibration()));
    layout->addRow(m_saveCalibrationBut);
    m_saveCalibrationBut->setDisabled(true);

    getToolsWidget()->setLayout(layout);
}

void CameraCalib::inputChanged() {
    resetSelectedFrames();
    boost::optional<uint8_t> mediaTypeOpt = m_settings.maybeGetValueOfParam<uint8_t>(GuiParam::MEDIA_TYPE);
    if (!mediaTypeOpt || GuiParam::MediaType::NoMedia == static_cast<GuiParam::MediaType>(*mediaTypeOpt) ||
            GuiParam::MediaType::Camera == static_cast<GuiParam::MediaType>(*mediaTypeOpt)) {
        m_addAllFramesBut->setDisabled(true);
    } else {
        m_addAllFramesBut->setDisabled(false);
    }
}

// ========================
// === Button Callbacks ===
// ========================

void CameraCalib::addFrame() {
    vector<Point2f> corners;

    try {
        corners = getChessboardCorners();
    } catch (int e) {
        if (e == 1) {
            Q_EMIT notifyGUI("No chessboard found!", MessageType::FAIL);
        } else if (e == 2) {
            Q_EMIT notifyGUI("No frame loaded!", MessageType::FAIL);
        } else {
            Q_EMIT notifyGUI("Unknown exception while searching for chessboard corner: "+ QString::number(e).toStdString(), MessageType::FAIL);
        }
        return;
    }

    boardCorners.push_back(corners);
    // Enable disable buttons related to the number of frames
    m_resetSelectedFramesBut->setDisabled(false);
    if (boardCorners.size() >= 10) {
        m_calibrateBut->setDisabled(false);
        m_saveCalibrationBut->setDisabled(false);
    }
    Q_EMIT notifyGUI("Frame " + QString::number(m_frameNumber).toStdString() + " added!", MessageType::NOTIFICATION);
}

void CameraCalib::addAllFrames() {

    for (int i = 0; i < getMaxFrameNumber(); i++) {
        Q_EMIT jumpToFrame(i);
        Q_EMIT forceTracking();
        addFrame();
    }

    Q_EMIT jumpToFrame(m_frameNumber);
    Q_EMIT forceTracking();
}

void CameraCalib::resetSelectedFrames() {
    m_calibrateBut->setDisabled(true);
    m_saveCalibrationBut->setDisabled(true);
    m_resetSelectedFramesBut->setDisabled(true);
    boardCorners.clear();
    Q_EMIT notifyGUI("Framelist cleared!", MessageType::NOTIFICATION);
}

void CameraCalib::calibrate() {
    if (boardCorners.size() < 10) {
        Q_EMIT notifyGUI("At least 10 frames need to be chosen!", MessageType::FAIL);
        return;
    }

    Size2i boardSize(m_boardWidthEdit->text().toInt(), m_boardHeightEdit->text().toInt());
    float squareSize = m_squareSizeEdit->text().toFloat();

    // Generating object points
    vector<Point3f> planarObjectPoints;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            planarObjectPoints.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
    }
    // Creating a copy of objectpoints for each set of corners
    vector<vector<Point3f > > objectPoints;
    for (uint32_t i = 0; i < boardCorners.size(); i++) {
        objectPoints.push_back(planarObjectPoints);
    }

    // Flags
    int flagsCalib = 0;
    if (m_zeroTangentDistCB->isChecked()) {
        flagsCalib  |= CV_CALIB_ZERO_TANGENT_DIST;
    }
    if (m_rationalModelCB->isChecked()) {
        flagsCalib  |= CV_CALIB_RATIONAL_MODEL;
    }
    if (m_fixPrincipalPointCB->isChecked()) {
        flagsCalib  |= CV_CALIB_FIX_PRINCIPAL_POINT;
    }

    if (!m_highPolynomeDegree->isChecked()) {
        flagsCalib |= CV_CALIB_FIX_K4;
        flagsCalib |= CV_CALIB_FIX_K5;
        flagsCalib |= CV_CALIB_FIX_K6;
    }

    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, boardCorners, m_curFrame.size(), m_cameraMatrix, m_distCoeffs, rvecs, tvecs, flagsCalib);
    Q_EMIT notifyGUI("Calibration finished. Remaining error: " + QString::number(rms).toStdString(), MessageType::NOTIFICATION);
    Q_EMIT update();
}

void CameraCalib::saveCalibration() {
	// Only save calibration, if calibration is already done
    if (m_cameraMatrix.empty() || m_distCoeffs.empty()) {
        calibrate();
        if (m_cameraMatrix.empty() || m_distCoeffs.empty()) {
            return;
        }
    }

    QString filename = QFileDialog::getSaveFileName();
    if (filename.isEmpty()) {
        return;
    }

    // Write file
    FileStorage fh(filename.toStdString(), FileStorage::WRITE);
    if (!fh.isOpened()) {
        Q_EMIT notifyGUI("Cannot open "+ filename.toStdString() +"!", MessageType::FAIL);
        return;
    }
    fh << "cameraMatrix" << m_cameraMatrix;
    fh << "distCoeffs" << m_distCoeffs;
    fh.release();

    Q_EMIT notifyGUI("File "+ filename.toStdString() +" saved.", MessageType::NOTIFICATION);
}

// ==============
// === Helper ===
// ==============

vector<Point2f> CameraCalib::getChessboardCorners() throw (int) {
    // Prevent empty frames, e.g. when no image is loaded
    if (m_curFrame.empty()) {
        throw 2;
    }
    Size2i boardSize(m_boardWidthEdit->text().toInt(), m_boardHeightEdit->text().toInt());

    vector<Point2f> corners;
    Mat grayFrame;
    cvtColor(m_curFrame, grayFrame, COLOR_BGR2GRAY);
    bool found = findChessboardCorners(grayFrame, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    // Throw exception if no chessboard is found
    if (!found) {
        throw 1;
    }

    return corners;
}
