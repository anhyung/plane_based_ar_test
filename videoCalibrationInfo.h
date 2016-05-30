#pragma once

#include <stdio.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

/** @증강현실을 위한 비디오 캘리브레이션 클래스 */
class VideoCalibrationInfo {
	public:
		vector<vector<Point2f> > ImagePoints;
		vector<vector<Point3f> > ObjectPoints;
		vector<Mat> Rvect;
		vector<Mat> Tvect;
		vector<int> notFound;

		Size BoardSize;
		Size ImageSize;
		float squareSize;	//체스보드 사각형 mm단위 수
		float aspectRatio;
		Mat CameraMatrix;
		Mat DistCoeffs;
		String cameraName;
		String outputFilename;
		String inputFilename;

		VideoCapture InputVideo;
		float fps;
		int cols;
		int rows;
		int nframe;
		int caliFrame; //캘리브레이션에 사용된 총 프레임 수
		int flags;

		VideoCalibrationInfo();
		VideoCalibrationInfo(int w, int h, String inputFilename, String outputCameraName, float squareSize = 1.f, float aspectRatio = 1.f);
		~VideoCalibrationInfo();
		void setBoardSize(int w, int h, float squareSize = 1.f, float aspectRatio = 1.f);
		void setOutputXml(String cameraName);
		void setInputVideo(String inputFilename);

		void cameraCalibration();
		void undistort();
		void undistortAndDrawingChessCube();
		void undistortAndDrawingMarkerVideo(String MarkerImage, String MarkerVideo, String ArVideo);
		void undistortAndDrawingMarkerCube(String MarkerImage, String MarkerVideo, int solvePnpRansacFlag = SOLVEPNP_EPNP);
		void world2imageCoord(vector<Point3f> &PointWorld, vector<Point2f> &PointImage, Mat &Rmat, Mat &T);
		void KeyPoint2Point2f(vector<KeyPoint> &InputKeypoint, vector<Point2f> &OutputKeypoint);
		void Point2f2KeyPoint(vector<Point2f> &InputKeypoint, vector<KeyPoint> &OutputKeypoint);

		double computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints, vector<float> &perViewErrors);
		void calcChessboardCorners(int frameN);
		bool runCalibration(vector<float> &reprojErrs, double &totalAvgErr);
		void saveCameraParams(const vector<float> &reprojErrs, double totalAvgErr);
		bool runAndSave();
};