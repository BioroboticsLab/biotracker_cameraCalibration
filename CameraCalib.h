#ifndef CAMERACALIB_H_H
#define CAMERACALIB_H_H

#include <QFrame>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>

#include <biotracker/TrackingAlgorithm.h>

#include <opencv2/highgui/highgui.hpp>


class CameraCalib : public BioTracker::Core::TrackingAlgorithm {
    Q_OBJECT
public:
    //constructor
    CameraCalib(BioTracker::Core::Settings & settings);

    //destructor
    ~CameraCalib();

    //implementations of pure virtual functions:
    void track(size_t frameNumber, const cv::Mat & frame) override;
    void paint(size_t frameNumber, BioTracker::Core::ProxyMat &m, View const &view = OriginalView) override;
    void postConnect() override;

private:

    // GUI-Elements
    QLineEdit *m_squareSizeEdit;
    QLineEdit *m_boardWidthEdit;
    QLineEdit *m_boardHeightEdit;
    QPushButton *m_addFrameBut;
    QPushButton *m_resetSelectedFramesBut;
    QPushButton *m_calibrateBut;
    QPushButton *m_saveCalibrationBut;
    QPushButton *m_addAllFramesBut;
    QCheckBox *m_zeroTangentDistCB;
    QCheckBox *m_rationalModelCB;
    QCheckBox *m_fixPrincipalPointCB;
    QCheckBox *m_highPolynomeDegree;

    QPointer<QFrame> m_toolsFrame;
    void initToolsFrame();

    std::string m_curView;
    cv::Mat m_curFrame;
    cv::Mat m_modifiedFrame;
    size_t m_frameNumber;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    // Chessboard corners, that should be shown live
    std::vector<cv::Point2f> shownChessboardCorners;

    // Corner positions in several frames
    std::vector<std::vector <cv::Point2f> > boardCorners;

    /**
     * Returns a vector with the positions of chessboard corners within the current frame.
     *
     * @return vector of positions of chessboard corners
     */
    std::vector<cv::Point2f> getChessboardCorners() throw (int);

public Q_SLOTS:
	/**
	 * Adds more chessboard corners to the calibration
	 */
	void addFrame();

	/**
	 * Adds all frames where a chessboad corner could be detected
	 */
	void addAllFrames();

	/**
	 * Resets alls chosen points.
	 */
	void resetSelectedFrames();

	/**
	 * Performs calibration
	 */
	void calibrate();

	/**
	 * Saves camera calibration
	 */
	void saveCalibration();
};

#endif // CAMERACALIB_H
