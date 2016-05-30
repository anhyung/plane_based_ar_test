#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "videoCalibrationInfo.h"
using namespace std;
using namespace cv;

#define GRID_MM 28	//ī�޶� Ķ���극�̼� ü�������� ���� ����(mm)
#define CAMERA_CHESS_W 9 //ī�޶� Ķ���극�̼� ü������ �ڳ� ������
#define CAMERA_CHESS_H 6
#define STEREO_FRAME_UNIT 30 //�������� SFM ���׷��� �̹��� ����

int main(void)
{
	//########### ������ ī�޶� Ķ���극�̼� ################
	//ī�޶� ���� Ķ���극�̼� ����
	VideoCalibrationInfo SFMvideo(CAMERA_CHESS_W, CAMERA_CHESS_H, "src_video/G3_Cali01.mp4", "LG_G3", GRID_MM);
		
	SFMvideo.cameraCalibration();
	SFMvideo.undistort();

	SFMvideo.undistortAndDrawingChessCube();
	SFMvideo.undistortAndDrawingMarkerVideo("src_img/query.jpg", "src_video/G3_Model01.mp4", "src_img/arvideo.mp4");
	SFMvideo.undistortAndDrawingMarkerCube("src_img/query.jpg", "src_video/G3_Model01.mp4", SOLVEPNP_UPNP);
	/*	 enum {
	SOLVEPNP_ITERATIVE = 0,
	SOLVEPNP_EPNP      = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
	SOLVEPNP_P3P       = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
	SOLVEPNP_DLS       = 3, // Joel A. Hesch and Stergios I. Roumeliotis. "A Direct Least-Squares (DLS) Method for PnP"
	SOLVEPNP_UPNP      = 4  // A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. "Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation"
	}; */
	destroyAllWindows();
	
	return 0;
}