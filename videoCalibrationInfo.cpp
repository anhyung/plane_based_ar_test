#include "videoCalibrationInfo.h"

#define FRAME_UNIT 5 //ó�� �ӵ��� Ķ���극�̼� �����Ӽ� ���Ѱɱ� : ���� ���� �� ���徿 ����
#define CUBE 5*28. //ü�� ���� ���� �׸� ������ü �Ѻ� ������(mm����)
#define CHESS_AXISDRAW_ONOFF true //ü�� ���� ���� �������� �׸��� ���� ���� On/Off
#define CHESS_CUBEDRAW_ONOFF true //ü�� ���� ���� ť�긦 �׸��� ���� ���� On/Off

#define TRANSLATE_X 210 //���� �̹��� ��ġ ����
#define TRANSLATE_Y 0 //���� �̹��� ��ġ ����

#define RESIZE_QUERY 210 //���� ��Ŀ �̹��� ������¡�� ���� ���� ���� ������
#define REALSIZE_WIDTH 210. //���� ��Ŀ �̹��� ���� ���� ������(mm����)
#define REALSIZE_HEIGHT 297. //���� ��Ŀ �̹��� ���� ���� ������(mm����)
#define KNN_MATCH_RATIO 0.5f //�������� KNN ��Ī ��Ȯ�� ���(��, ��Ī���� �پ��)
#define HOMOGRAPHY_ONOFF false //������ ȣ��׷��Ǹ� ������ �׸��� ���� ���� On/Off : ��Ī ������� �׻� ǥ��
#define MARKER_AXISDRAW_ONOFF true //��Ŀ �̹��� ���� �������� �׸��� ���� ���� On/Off
#define MARKER_CUBEDRAW_ONOFF true //��Ŀ �̹��� ���� ť�긦 �׸��� ���� ���� On/Off

#define MARKER_CUBE 100.f //��Ŀ �̹��� ���� �׸� ������ü �Ѻ� ������(mm����)
#define CUBE_X REALSIZE_WIDTH/2 //��Ŀ �̹��� ���� �׸� ������ü ��ġ ����
#define CUBE_Y REALSIZE_HEIGHT/2 //��Ŀ �̹��� ���� �׸� ������ü ��ġ ����


/** @�⺻ ������ */
VideoCalibrationInfo::VideoCalibrationInfo() {
	setBoardSize(15, 23, 1.f);
	setInputVideo("video_cali_01.mp4");
	outputFilename = "calibration_data.xml";
	cameraName = "NullCamera";
	setOutputXml("TestVideo");
	flags = 0;
}

/** @����ڿ� ���� �ʱ�ȭ ������ */
VideoCalibrationInfo::VideoCalibrationInfo(int w, int h, String inputFilename, String outputCameraName, float squareSize, float aspectRatio) {
	setBoardSize(w, h, squareSize, aspectRatio);
	this->inputFilename = inputFilename;
	setInputVideo(inputFilename);
	outputFilename = "calibration_data.xml";
	cameraName = outputCameraName;
	setOutputXml(outputCameraName);
	flags = 0;
}

/** @�Ҹ��� */
VideoCalibrationInfo::~VideoCalibrationInfo() {
}

/** @ü������ �԰� �Է�(�����ڳ�, �����ڳ�, �簢�� ���� ����, �簢�� ����) */
void VideoCalibrationInfo::setBoardSize(int w, int h, float squareSize, float aspectRatio) {
	BoardSize.width = w;
	BoardSize.height = h;
	this->squareSize = squareSize;
	this->aspectRatio = aspectRatio;
}

/** @��� ������ ���ϸ� �Է� */
void VideoCalibrationInfo::setOutputXml(String cameraName) {
	outputFilename = format("%s_calibration_data.xml", cameraName);
}

/** @���������� ������ ��� */
double VideoCalibrationInfo::computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints, vector<float> &perViewErrors) {
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++) {
		projectPoints(Mat(objectPoints[i]), Rvect[i], Tvect[i], CameraMatrix, DistCoeffs, imagePoints2);
		err = norm(Mat(ImagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

/** @ü������ �ڳʷκ��� ������ǥ(������ǥ) �迭 ���� */
void VideoCalibrationInfo::calcChessboardCorners(int frameN) {
	for (int k = 0; k < frameN; k++) {
		vector<Point3f> Array;
		for (int i = 0; i < BoardSize.height; i++) {
			for (int j = 0; j < BoardSize.width; j++) {
				Array.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
			}
		}
		ObjectPoints.push_back(Array);
	}
}

/** @Ķ���극�̼� ��� �� RMS ���� ��� */
bool VideoCalibrationInfo::runCalibration(vector<float> &reprojErrs, double &totalAvgErr) {
	CameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO) CameraMatrix.at<double>(0, 0) = aspectRatio;
	DistCoeffs = Mat::zeros(8, 1, CV_64F);
	
	//K3�̻��� �ְ� ������ 0���� ����(��ȷ�� �ƴϹǷ� ���ٰ� ����, ����X)
	double rms = calibrateCamera(ObjectPoints, ImagePoints, ImageSize, CameraMatrix, DistCoeffs, Rvect, Tvect, flags | CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5);
	printf("\nRMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(CameraMatrix) && checkRange(DistCoeffs);
	totalAvgErr = computeReprojectionErrors(ObjectPoints, reprojErrs);

	return ok;
}

/** @���� ������ xml���Ϸ� ��� */
void VideoCalibrationInfo::saveCameraParams(const vector<float> &reprojErrs, double totalAvgErr) {
	FileStorage fs(outputFilename, FileStorage::WRITE);
	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf)-1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!Rvect.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(Rvect.size(), reprojErrs.size());
	fs << "image_width" << ImageSize.width;
	fs << "image_height" << ImageSize.height;
	fs << "board_width" << BoardSize.width;
	fs << "board_height" << BoardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)	{
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;
	fs << "camera_matrix" << CameraMatrix;
	fs << "distortion_coefficients" << DistCoeffs;
	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!Rvect.empty() && !Tvect.empty()) {
		CV_Assert(Rvect[0].type() == Tvect[0].type());
		Mat bigmat((int)Rvect.size(), 6, Rvect[0].type());
		for (int i = 0; i < (int)Rvect.size(); i++)	{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(Rvect[i].rows == 3 && Rvect[i].cols == 1);
			CV_Assert(Tvect[i].rows == 3 && Tvect[i].cols == 1);
			r = Rvect[i].t();
			t = Tvect[i].t();
		}
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!ImagePoints.empty()) {
		Mat imagePtMat((int)ImagePoints.size(), (int)ImagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)ImagePoints.size(); i++) {
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(ImagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}

/** @����Լ��� ����, Ķ���극�̼� ����ϰ� ������ */
bool VideoCalibrationInfo::runAndSave() {
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n\n", (ok ? "Calibration succeeded" : "Calibration failed"), totalAvgErr);

	if (ok) saveCameraParams(reprojErrs, totalAvgErr);

	return ok;
}

/** @�Է� ���� ���� �ʱ�ȭ */
void VideoCalibrationInfo::setInputVideo(String inputFilename) {
	//�׽�Ʈ �Է� ������ �ҷ�����
	InputVideo.open(inputFilename);
	if (!InputVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

	//�Է� ������ ���� 
	fps = InputVideo.get(CAP_PROP_FPS);
	cols = InputVideo.get(CAP_PROP_FRAME_WIDTH);
	rows = InputVideo.get(CAP_PROP_FRAME_HEIGHT);
	nframe = InputVideo.get(CAP_PROP_FRAME_COUNT);
	ImageSize = Size(cols, rows);

	cout << endl;
	cout << "Input Video File Name : " << inputFilename << endl;
	cout << "Frame Per Seconds : " << fps << endl;
	cout << "Frame Size : " << cols << " x " << rows << endl;
	cout << "Frame Count : " << nframe << endl;
}

/** @�Է¿��󿡼� ü���� �ڳ� �����ϰ� ������ ����, Ķ���극�̼� ���� �� ��� ��� */
void VideoCalibrationInfo::cameraCalibration() {
	//������ ����� ���ڴ� ����
	String OutputVideoName = format("%s_ChessboardDetection.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view;	//�̹��� ������ �ӽ� ������
	Mat viewGray;	//�����ȼ��� ������ �̹��� ������
	int success = 0;
	
	caliFrame = nframe / FRAME_UNIT + 1; //Ķ���극�̼ǿ� ���� �� ������ ��(0-based�̹Ƿ� +1)
	calcChessboardCorners(caliFrame); //ü�������� ������ǥ ���
		
	for (int i = 0; i < caliFrame; i++) { //ó�� �ӵ��� �����Ӽ� ���Ѱɱ�
		InputVideo.set(CAP_PROP_POS_FRAMES, i*FRAME_UNIT); //��ü ������ �� FRAME_UNIT������ �ǳʶ� i��° ���������� �̵�
		InputVideo >> view;	//�̹��� �ҷ�����

		vector<Point2f> ImageCorners;	//���� ��� ���� ��ǥ
		cvtColor(view, viewGray, CV_BGR2GRAY);	//�����ȼ� �Լ��� �׷��̽������� ���ڷ� ����

		//ü���� �ڳ� ����
		bool found = findChessboardCorners(view, BoardSize, ImageCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			cornerSubPix(viewGray, ImageCorners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			drawChessboardCorners(view, BoardSize, ImageCorners, found); //�ڳ� ���� �� ���� ���� ǥ��
			ImagePoints.push_back(ImageCorners);
			success++;
		}
		else {
			notFound.push_back(InputVideo.get(CAP_PROP_POS_FRAMES)); //ü������ ���� ���� �̹��� �ε��� �߰�(���� ���ܽ�Ű�� ����)

			//�̹��� ���� �޽��� ǥ��
			String msg = "Detection Failed";
			putText(view, msg, Point(view.cols / 2 - 130, view.rows / 2), 1, 5, Scalar(0, 0, 255), 4);
		}
		//�̹��� ������ ��ȣ ǥ��
		String msg = format("%03d/%d", i + 1, caliFrame);
		putText(view, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);

		//ü������ ���� ������ ����
		if (OutputVideo.isOpened()) OutputVideo << view;

		//���� �̹����� �ʹ� Ŭ��� ��� ��¸� �۰� ����
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(view, view, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//���� ��� ���
		imshow("ChessBoard Corners Detection", view);

		if (waitKey(100 / fps) == 27 || i == caliFrame - 1) { //ESC�� �����ų� �����Ӽ���ŭ üũ�ϸ� ���� : 1000/fps�� ���� ��� �ӵ�
			destroyWindow("ChessBoard Corners Detection");
			break;
		}
	}
	cout << endl << success << " Images Chesssboard Pattern Detection Success!!" << endl << "Computing Calibration..." << endl;

	//K3 �ְ������� ���ܽ�Ű�� ����
	DistCoeffs = Mat::zeros(5, 1, CV_64F); //K3 �ְ����� 0���� �ʱ�ȭ ������Ű�� ����
	
	runAndSave();	//Ķ���극�̼� ����, xml ������ �ۼ�, ������ ���

	cout << endl << "Camera Matrix :" << endl << CameraMatrix << endl;
	cout << endl << "Distortion Coefficients :" << endl << DistCoeffs << endl << endl;
}

/** @Ķ���극�̼� �� ���� �ְ����� �Է¿��� �ְ� ���� */
void VideoCalibrationInfo::undistort() {
	//������ ����� ���ڴ� ����
	String OutputVideoName = format("%s_Undistortion.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view, rview, map1, map2;
	//Mat newCameraMatrix = getOptimalNewCameraMatrix(CameraMatrix, DistCoeffs, ImageSize, 0, ImageSize, 0);
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, ImageSize, CV_16SC2, map1, map2);

	//�Է� ���� �ٽ� ������������ ������(Ķ���극�̼� �� ��������Ƿ�)
	if (InputVideo.get(CAP_PROP_POS_FRAMES) != 0) InputVideo.set(CAP_PROP_POS_FRAMES, 0);
			
	for (int i = 0; i < nframe; i++) {
		InputVideo.set(CAP_PROP_POS_FRAMES, i);
		InputVideo >> view;	//�̹��� �ҷ�����

		//undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
		remap(view, rview, map1, map2, INTER_LINEAR);

		//�̹��� ������ ��ȣ ǥ��
		String msg = format("%03d/%d", i + 1, nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);

		//���� ��� ������ ����
		if (OutputVideo.isOpened()) OutputVideo << rview;

		//���� �̹����� �ʹ� Ŭ��� ��� ��¸� �۰� ����
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//���� ��� ���
		imshow("Undistort Image", rview);

		if (waitKey(100 / fps) == 27 || i == nframe - 1) { //ESC�� �����ų� �����Ӽ���ŭ üũ�ϸ� ����
			destroyWindow("Undistort Image");
			break;
		}
	}
}

/** @������ R/T�� ī�޶� ��Ʈ������ ������ǥ�� ���� ��� ��ǥ�� ��ȯ(���� Ȯ�ο�) */
void VideoCalibrationInfo::world2imageCoord(vector<Point3f> &PointWorld, vector<Point2f> &PointImage, Mat &Rmat, Mat &T) {
	CameraMatrix.convertTo(CameraMatrix, CV_64F); //���Ϸ��� 32F->64F�� ����ȯ
	for (int i = 0; i < PointWorld.size(); i++) {
		double worldXYZ[3] = { PointWorld[i].x, PointWorld[i].y, PointWorld[i].z };
		Mat Xobj(3, 1, CV_64FC1, worldXYZ); //������ǥ ������
		Mat Xc = Rmat*Xobj + T;	//������ǥ�� ī�޶� ��ǥ�� ��ȯ
		Mat_<double> xy = CameraMatrix*Xc;
		xy /= xy(2);	//ȣ�����Ͼ ������ ����ȭ

		PointImage.push_back(Point2f(xy(0), xy(1)));
	}
	CameraMatrix.convertTo(CameraMatrix, CV_32F); //���� ������ 32F�� ����ȯ
}

/** @Ư¡�� Ű����Ʈ�� �����ǥ�� ��ȯ */
void VideoCalibrationInfo::KeyPoint2Point2f(vector<KeyPoint> &InputKeypoint, vector<Point2f> &OutputKeypoint) {
	if (!OutputKeypoint.empty()) OutputKeypoint.clear();

	for (int i = 0; i < InputKeypoint.size(); i++) {
		OutputKeypoint.push_back(InputKeypoint[i].pt);
	}
}

/** @�����ǥ�� Ư¡�� Ű����Ʈ�� ��ȯ */
void VideoCalibrationInfo::Point2f2KeyPoint(vector<Point2f> &InputKeypoint, vector<KeyPoint> &OutputKeypoint) {
	if (!OutputKeypoint.empty()) OutputKeypoint.clear();

	for (int i = 0; i < InputKeypoint.size(); i++) {
		OutputKeypoint.push_back(KeyPoint(InputKeypoint[i], 1.f));
	}
}

/** @Ķ���극�̼� �� �ְ� ���� �� üũ ���� ���� ���������� ť�� ��ü �׸��� */
void VideoCalibrationInfo::undistortAndDrawingChessCube() {
	//������ ����� ���ڴ� ����
	String OutputVideoName = format("%s_CubeDrawingChessPattern.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view, rview, map1, map2;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, ImageSize, CV_16SC2, map1, map2);
	
	//���� ������ �ʿ��� �ӽú�����
	Mat Rmat, R, T;
	vector<Point2f> ImageCorners;
	vector<Point3f> ObjectCorners;
	for (int i = 0; i < BoardSize.height; i++) {
		for (int j = 0; j < BoardSize.width; j++) {
			ObjectCorners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		}
	}
	
	//ü������ ���� �׸� 3���� ť�� �������� ������ǥ ����
	vector<Point3f> CubePoints;
	CubePoints.push_back(Point3f(0., 0., 0.));
	CubePoints.push_back(Point3f(CUBE, 0., 0.));
	CubePoints.push_back(Point3f(CUBE, CUBE, 0.));
	CubePoints.push_back(Point3f(0., CUBE, 0.));
	CubePoints.push_back(Point3f(0., 0., -CUBE));
	CubePoints.push_back(Point3f(CUBE, 0., -CUBE));
	CubePoints.push_back(Point3f(CUBE, CUBE, -CUBE));
	CubePoints.push_back(Point3f(0., CUBE, -CUBE));
	
	//�Է� ���� �ٽ� ������������ ������(Ķ���극�̼� �� ��������Ƿ�)
	if (InputVideo.get(CAP_PROP_POS_FRAMES) != 0) InputVideo.set(CAP_PROP_POS_FRAMES, 0);

	for (int i = 0; i < nframe; i++) {
		InputVideo.set(CAP_PROP_POS_FRAMES, i);
		InputVideo >> view;	//�̹��� �ҷ�����
		
		remap(view, rview, map1, map2, INTER_LINEAR);

		//ü������ ��Ŀ �ν� : �̹� �ְ� ������ rview�� �ƴ� �ְ�� view�� �ؾ���(�ְ� ���� �ߺ� ����)
		bool found = findChessboardCorners(view, BoardSize, ImageCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {	//��Ŀ �ν� ������ ����(ī�޶��� Rvec/Tvec) ���� �ǽ�
			solvePnP(ObjectCorners, ImageCorners, CameraMatrix, DistCoeffs, R, T);
			
			//ü������ ���� ť�� �׸���
			Rodrigues(R, Rmat); //ȸ������� �ε帮���� ��ȯ
			vector<Point2f> DstRect; //3���� ������ǥ�� 2���� ��� ������
			world2imageCoord(CubePoints, DstRect, Rmat, T); //3 -> 2���� ��ǥ ��ȯ

			if (CHESS_AXISDRAW_ONOFF) { //ü������ ���� ���� ������ �׸���
				arrowedLine(rview, DstRect[0], DstRect[1], Scalar(0, 0, 255), 3); //������ǥ X�༱ ǥ��
				arrowedLine(rview, DstRect[0], DstRect[3], Scalar(0, 255, 0), 3); //������ǥ Y�༱ ǥ��
				arrowedLine(rview, DstRect[0], DstRect[4], Scalar(255, 0, 0), 3); //������ǥ Z�༱ ǥ��
				circle(rview, DstRect[0], 10, Scalar(255, 255, 255), 3);	//�� �׸�
			}
			if (CHESS_CUBEDRAW_ONOFF) { //ü������ ���� ���� ť�� �׸���
				//line(rview, DstRect[0], DstRect[1], Scalar(0, 255, 255), 2);
				line(rview, DstRect[1], DstRect[2], Scalar(0, 255, 255), 2);
				line(rview, DstRect[2], DstRect[3], Scalar(0, 255, 255), 2);
				//line(rview, DstRect[3], DstRect[0], Scalar(0, 255, 255), 2);
				//line(rview, DstRect[0], DstRect[4], Scalar(0, 255, 255), 2);
				line(rview, DstRect[1], DstRect[5], Scalar(0, 255, 255), 2);
				line(rview, DstRect[2], DstRect[6], Scalar(0, 255, 255), 2);
				line(rview, DstRect[3], DstRect[7], Scalar(0, 255, 255), 2);
				line(rview, DstRect[4], DstRect[5], Scalar(0, 255, 255), 2);
				line(rview, DstRect[5], DstRect[6], Scalar(0, 255, 255), 2);
				line(rview, DstRect[6], DstRect[7], Scalar(0, 255, 255), 2);
				line(rview, DstRect[7], DstRect[4], Scalar(0, 255, 255), 2);
			}
		}

		//�̹��� ������ ��ȣ ǥ��
		String msg = format("%03d/%d", i + 1, nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		
		//���� ��� ������ ����
		if (OutputVideo.isOpened()) OutputVideo << rview;

		//���� �̹����� �ʹ� Ŭ��� ��� ��¸� �۰� ����
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}		

		//���� ��� ���
		imshow("Undistort & drawing Cube Image", rview);

		if (waitKey(100 / fps) == 27 || i == nframe - 1) { //ESC�� �����ų� �����Ӽ���ŭ üũ�ϸ� ����
			destroyWindow("Undistort & drawing Cube Image");
			break;
		}
	}
}

/** @��� AR : Ķ���극�̼� �� �ְ� ���� �� ��Ŀ �̹��� ���� ������ �׸��� */
void VideoCalibrationInfo::undistortAndDrawingMarkerVideo(String MarkerImage, String MarkerVideo, String ArVideo) {
	//���� ��Ŀ�� ���� ���� �̹���
	Mat QueryImg = imread(MarkerImage, IMREAD_COLOR);
	if (QueryImg.empty()) { 
		cerr << "Image File Open Error" << endl; 
		exit(1); 
	}

	//���� �̹����� ���� Ȥ�� ���ΰ� 1000�̻��� ��� ������ ������� ��� ����
	resize(QueryImg, QueryImg, Size(RESIZE_QUERY, (QueryImg.rows * RESIZE_QUERY) / QueryImg.cols));

	//Ư¡ ���� ����� ���� : ���� �̹��� Ư¡ ������ ����
	Ptr<FeatureDetector> Feature = AKAZE::create();
	vector<KeyPoint> KeyPtQuery, KeyPtTrain;
	Feature->detect(QueryImg, KeyPtQuery);
	Mat DescriptQuery, DescriptTrain;
	Feature->compute(QueryImg, KeyPtQuery, DescriptQuery);

	//������ ��Ŀ �Կ� ������ �ҷ�����
	VideoCapture InputMarkerVideo(MarkerVideo);
	if (!InputMarkerVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

	float _fps = InputMarkerVideo.get(CAP_PROP_FPS);
	int _cols = InputMarkerVideo.get(CAP_PROP_FRAME_WIDTH);
	int _rows = InputMarkerVideo.get(CAP_PROP_FRAME_HEIGHT);
	int _nframe = InputMarkerVideo.get(CAP_PROP_FRAME_COUNT);
	Size _ImageSize = Size(_cols, _rows);

	cout << endl;
	cout << "Input Marker Video File Name : " << MarkerVideo << endl;
	cout << "Frame Per Seconds : " << _fps << endl;
	cout << "Frame Size : " << _cols << " x " << _rows << endl;
	cout << "Frame Count : " << _nframe << endl;

	//������1 ����� ���ڴ� ���� : ť�� �׸� ������
	String OutputVideoName01 = format("%s_VideoDrawingMarkerImageHomo.avi", cameraName);
	VideoWriter OutputVideo01(OutputVideoName01, CV_FOURCC('P', 'I', 'M', '1'), _fps, _ImageSize);
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo01.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << endl << "Save Video File : " << OutputVideoName01 << endl;

	//������2 ����� ���ڴ� ���� : ��Ī ��� ������
	String OutputVideoName02 = format("%s_FeatureMatchMarkerImageHomo.avi", cameraName);
	VideoWriter OutputVideo02(OutputVideoName02, CV_FOURCC('P', 'I', 'M', '1'), _fps, Size(_cols + QueryImg.cols, (_rows > QueryImg.rows) ? _rows : QueryImg.rows));
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo02.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << "Save Video File : " << OutputVideoName02 << endl;

	Mat view, rview, map1, map2, viewAR, rviewAR;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, _ImageSize, CV_16SC2, map1, map2);


	//####### ������ ������ �ҷ����� #######
	VideoCapture InputArVideo(ArVideo);
	if (!InputArVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

		
	//�˰��� �ð�üũ�� ����
	clock_t start, finish;

	for (int i = 0; i < _nframe; i++) {
		InputMarkerVideo.set(CAP_PROP_POS_FRAMES, i);
		InputMarkerVideo >> view;	//�̹��� �ҷ�����

		//�ڵ� �ݺ� ���(���� ������ ��ġ�� �������̸� ó������ �ǵ���)
		if (InputArVideo.get(CAP_PROP_POS_FRAMES) == (InputArVideo.get(CAP_PROP_FRAME_COUNT) - 1)) InputArVideo.set(CAP_PROP_POS_FRAMES, 0);
		InputArVideo >> viewAR;	//���� ���� �ҷ�����
		resize(viewAR, viewAR, Size(RESIZE_QUERY, (viewAR.rows * RESIZE_QUERY) / viewAR.cols));

		start = clock();
		remap(view, rview, map1, map2, INTER_LINEAR);

		//###### �����̹��� ��Ŀ �ν� ######
		//������ Ư¡ ���� �� ��� ���
		Feature->detect(rview, KeyPtTrain);
		Feature->compute(rview, KeyPtTrain, DescriptTrain);

		//��� ��Ī : BruteForce - KNN Matching(2-NN)
		Ptr<DescriptorMatcher> Matcher = DescriptorMatcher::create("BruteForce-Hamming");
		vector<vector<DMatch> > MatchListKNN;
		Matcher->knnMatch(DescriptQuery, DescriptTrain, MatchListKNN, 2);
		
		vector<KeyPoint> KeyPtQueryBest, KeyPtTrainBest;
		vector<DMatch> MatchListBest;
		for (size_t i = 0; i < MatchListKNN.size(); i++) {
			DMatch first = MatchListKNN[i][0];
			float dist1 = MatchListKNN[i][0].distance; //�ֱٹ� ��Ī�� �Ÿ�
			float dist2 = MatchListKNN[i][1].distance; //���ٹ� ��Ī�� �Ÿ�

			if (dist1 < KNN_MATCH_RATIO*dist2) { //"�ֱٹ�Ÿ�/���ٹ�Ÿ� < 2-NN_MATCH_RATIO"�� �����ϴ� �ֱٹ��� ����
				KeyPtQueryBest.push_back(KeyPtQuery[first.queryIdx]);
				KeyPtTrainBest.push_back(KeyPtTrain[first.trainIdx]);
				MatchListBest.push_back(DMatch(KeyPtQueryBest.size() - 1, KeyPtTrainBest.size() - 1, 0));
			}
		}
		
		//���� ����� ��� Ư¡ ǥ��
		//for (int i = 0; i<KeyPtTrain.size(); i++) {
		//	circle(rview, KeyPtTrain[i].pt, 3, Scalar(0, 0, 255));
		//}

		Mat MatchingRview;
		drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview, Scalar(0, 0, 255), Scalar(0, 0, 255));
		//drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview);

		vector<Point2f> PointQuery, PointTrain;
		KeyPoint2Point2f(KeyPtQueryBest, PointQuery);
		KeyPoint2Point2f(KeyPtTrainBest, PointTrain);
		Mat inliersVec; //knn ��Ī �� �ƿ����̾� �ε��� ����Ʈ ������

		if (MatchListBest.size() > 4) {
			//ȣ��׷��� ���� == ��Ŀ �ν�
			Mat Homography = findHomography(PointQuery, PointTrain, RANSAC, 2.f, inliersVec);

			vector<Point2f> QueryCorners(4);
			vector<Point2f> ImageCorners(4);
			QueryCorners[0] = Point2f(0, 0);
			QueryCorners[1] = Point2f(QueryImg.cols, 0);
			QueryCorners[2] = Point2f(QueryImg.cols, QueryImg.rows);
			QueryCorners[3] = Point2f(0, QueryImg.rows);
			perspectiveTransform(QueryCorners, ImageCorners, Homography);

			//������ ȣ��׷��� Ȯ�� == ��Ŀ �ν� Ȯ��
			if (HOMOGRAPHY_ONOFF) { //���� ��� ���� ���� ȣ��׷��� ����
				line(rview, ImageCorners[0], ImageCorners[1], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[1], ImageCorners[2], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[2], ImageCorners[3], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[3], ImageCorners[0], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[0], ImageCorners[2], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[1], ImageCorners[3], Scalar(0, 255, 0), 2);
			}
			//��Ī ����� ȣ��׷��� ����
			line(MatchingRview, Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Scalar(255, 0, 255), 2);
			
			Mat Mask(viewAR.rows + TRANSLATE_Y, viewAR.cols + TRANSLATE_X, viewAR.type(), Scalar::all(0));	//���� �� ���� ������ ����ũ
			Mat viewAR2(viewAR.rows + TRANSLATE_Y, viewAR.cols + TRANSLATE_X, viewAR.type(), Scalar::all(0));			
			for (int r = 0; r < viewAR.rows; r++) {
				Vec3b *value = viewAR.ptr<Vec3b>(r);
				for (int c = 0; c < viewAR.cols; c++) {
					viewAR2.at<Vec3b>(r + TRANSLATE_Y, c + TRANSLATE_X) = value[c];
					Mask.at<Vec3b>(r + TRANSLATE_Y, c + TRANSLATE_X) = Vec3b::all(255);
				}
			}
			warpPerspective(viewAR2, rviewAR, Homography, rview.size(), INTER_LINEAR);
			warpPerspective(Mask, Mask, Homography, rview.size(), INTER_NEAREST);
			rviewAR.copyTo(rview, Mask);
		}
		finish = clock();

		//����ð� ��� �� ǥ��
		double implementTime = (double)(finish - start) / CLOCKS_PER_SEC;
		String timeMsg = format("%0.3f sec/frame", implementTime);
		putText(rview, timeMsg, Point(60, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		
		//�̹��� ������ ��ȣ ǥ��
		String msg = format("%03d/%d", i + 1, _nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		putText(MatchingRview, msg, Point(_cols*0.65 + QueryImg.cols, 60), 1, 2, Scalar(0, 0, 255), 2);

		//��Ī �ζ��̾� ǥ��
		vector<int> inliers;
		for (int i = 0; i < inliersVec.rows; i++) {
			if (inliersVec.at<uchar>(i, 0) == 1) inliers.push_back(i);
		}
		for (int i = 0; i < inliers.size(); i++) {
			//���� ����� ǥ��
			//circle(rview, Point(PointTrain[inliers[i]].x, PointTrain[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
			//��Ī ����� ǥ��
			circle(MatchingRview, Point(PointQuery[inliers[i]].x, PointQuery[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
			circle(MatchingRview, Point(PointTrain[inliers[i]].x + QueryImg.cols, PointTrain[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
		}
		String knnMatchMsg = format("KNN Ratio %0.1f", KNN_MATCH_RATIO);
		putText(MatchingRview, knnMatchMsg, Point(_cols*0.65 + QueryImg.cols, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String matchMsg = format("Inliers %d/%d", inliers.size(), MatchListBest.size());
		putText(MatchingRview, matchMsg, Point(_cols*0.65 + QueryImg.cols, 110), 1, 1.5, Scalar(0, 0, 255), 2);
		putText(rview, matchMsg, Point(60, 110), 1, 1.5, Scalar(0, 0, 255), 2);

		//��� ������ ����
		//if (OutputVideo01.isOpened()) OutputVideo01 << rview;
		//if (OutputVideo02.isOpened()) OutputVideo02 << MatchingRview;

		//���� �̹����� �ʹ� Ŭ��� ��� ��¸� �۰� ����
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}		

		//��� ���
		imshow("Matching Test Result", MatchingRview); //��Ī ��� Ȯ�� ���
		imshow("Undistort & drawing Video Image", rview); //���� ��� ���

		if (waitKey(100 / _fps) == 27 || i == _nframe - 1) { //ESC�� �����ų� �����Ӽ���ŭ üũ�ϸ� ����
			destroyAllWindows();
			break;
		}
	}
}

/** @Ķ���극�̼� �� �ְ� ���� �� ��Ŀ �̹��� ���� ���������� ť�� ��ü �׸��� */
void VideoCalibrationInfo::undistortAndDrawingMarkerCube(String MarkerImage, String MarkerVideo, int solvePnpRansacFlag) {
	//���� ��Ŀ�� ���� ���� �̹���
	Mat QueryImg = imread(MarkerImage, IMREAD_COLOR);
	if (QueryImg.empty()) {
		cerr << "Image File Open Error" << endl;
		exit(1);
	}

	//���� �̹����� ���� Ȥ�� ���ΰ� 1000�̻��� ��� ������ ������� ��� ����
	resize(QueryImg, QueryImg, Size(RESIZE_QUERY, (QueryImg.rows * RESIZE_QUERY) / QueryImg.cols));

	//Ư¡ ���� ����� ���� : ���� �̹��� Ư¡ ������ ����
	Ptr<FeatureDetector> Feature = AKAZE::create();
	vector<KeyPoint> KeyPtQuery, KeyPtTrain;
	Feature->detect(QueryImg, KeyPtQuery);
	Mat DescriptQuery, DescriptTrain;
	Feature->compute(QueryImg, KeyPtQuery, DescriptQuery);

	//������ ��Ŀ �Կ� ������ �ҷ�����
	VideoCapture InputMarkerVideo(MarkerVideo);
	if (!InputMarkerVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

	float _fps = InputMarkerVideo.get(CAP_PROP_FPS);
	int _cols = InputMarkerVideo.get(CAP_PROP_FRAME_WIDTH);
	int _rows = InputMarkerVideo.get(CAP_PROP_FRAME_HEIGHT);
	int _nframe = InputMarkerVideo.get(CAP_PROP_FRAME_COUNT);
	Size _ImageSize = Size(_cols, _rows);

	cout << endl;
	cout << "Input Marker Video File Name : " << MarkerVideo << endl;
	cout << "Frame Per Seconds : " << _fps << endl;
	cout << "Frame Size : " << _cols << " x " << _rows << endl;
	cout << "Frame Count : " << _nframe << endl;

	String flagOption;
	if (solvePnpRansacFlag == 0) { flagOption = "ITERATIVE"; }
	else if (solvePnpRansacFlag == 1) { flagOption = "EPNP"; }
	else if (solvePnpRansacFlag == 2) {	flagOption = "P3P"; }
	else if (solvePnpRansacFlag == 3) {	flagOption = "DLS"; }
	else if (solvePnpRansacFlag == 4) {	flagOption = "UPNP"; }
	cout << endl << "solvePnP Flag : RANSAC + " << flagOption << endl;

	//������1 ����� ���ڴ� ���� : ť�� �׸� ������
	String OutputVideoName01 = format("%s_CubeDrawingMarkerImage_%d.avi", cameraName, solvePnpRansacFlag);
	VideoWriter OutputVideo01(OutputVideoName01, CV_FOURCC('P', 'I', 'M', '1'), _fps, _ImageSize);
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo01.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << endl << "Save Video File : " << OutputVideoName01 << endl;

	//������2 ����� ���ڴ� ���� : ��Ī ��� ������
	String OutputVideoName02 = format("%s_FeatureMatchMarkerImage_KNN%0.1f.avi", cameraName, KNN_MATCH_RATIO);
	VideoWriter OutputVideo02(OutputVideoName02, CV_FOURCC('P', 'I', 'M', '1'), _fps, Size(_cols + QueryImg.cols, (_rows > QueryImg.rows) ? _rows : QueryImg.rows));
	//�ڵ� : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo02.isOpened()) cerr << "Video Write Error" << endl; //���� ���н� ����ʰ� �׳� ���� : exit(1) ����
	cout << "Save Video File : " << OutputVideoName02 << endl;

	Mat view, rview, map1, map2;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, _ImageSize, CV_16SC2, map1, map2);
	
	//��Ŀ�̹��� ���� �׸� 3���� ť�� �������� ������ǥ ���� : ť�� ���� ����
	vector<Point3f> CubePoints;
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y + MARKER_CUBE, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y + MARKER_CUBE, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y + MARKER_CUBE, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y + MARKER_CUBE, -MARKER_CUBE));

	//�˰��� �ð�üũ�� ����
	clock_t start, finish;

	for (int i = 0; i < _nframe; i++) {
		InputMarkerVideo.set(CAP_PROP_POS_FRAMES, i);
		InputMarkerVideo >> view;	//�̹��� �ҷ�����
		
		start = clock();
		remap(view, rview, map1, map2, INTER_LINEAR);

		//###### �����̹��� ��Ŀ �ν� ######
		//������ Ư¡ ���� �� ��� ���
		Feature->detect(rview, KeyPtTrain);
		Feature->compute(rview, KeyPtTrain, DescriptTrain);

		//��� ��Ī : BruteForce - KNN Matching(2-NN)
		Ptr<DescriptorMatcher> Matcher = DescriptorMatcher::create("BruteForce-Hamming");
		vector<vector<DMatch> > MatchListKNN;
		Matcher->knnMatch(DescriptQuery, DescriptTrain, MatchListKNN, 2);

		vector<KeyPoint> KeyPtQueryBest, KeyPtTrainBest;
		vector<DMatch> MatchListBest;
		for (size_t i = 0; i < MatchListKNN.size(); i++) {
			DMatch first = MatchListKNN[i][0];
			float dist1 = MatchListKNN[i][0].distance; //�ֱٹ� ��Ī�� �Ÿ�
			float dist2 = MatchListKNN[i][1].distance; //���ٹ� ��Ī�� �Ÿ�

			if (dist1 < KNN_MATCH_RATIO*dist2) { //"�ֱٹ�Ÿ�/���ٹ�Ÿ� < 2-NN_MATCH_RATIO"�� �����ϴ� �ֱٹ��� ����
				KeyPtQueryBest.push_back(KeyPtQuery[first.queryIdx]);
				KeyPtTrainBest.push_back(KeyPtTrain[first.trainIdx]);
				MatchListBest.push_back(DMatch(KeyPtQueryBest.size() - 1, KeyPtTrainBest.size() - 1, 0));
			}
		}

		Mat MatchingRview;
		drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview, Scalar(0, 0, 255), Scalar(0, 0, 255)); //��Ī ����� ���� ������ ǥ��

		vector<Point3f> ObjectCorners;
		vector<Point2f> ImageCorners;
		for (int i = 0; i < KeyPtQueryBest.size(); i++) {
			ObjectCorners.push_back(Point3f(KeyPtQueryBest[i].pt.x, KeyPtQueryBest[i].pt.y, 0.));
		}
		KeyPoint2Point2f(KeyPtTrainBest, ImageCorners);

		//��Ŀ �ν� ��ġ Ȯ�� ������ ����(ī�޶��� Rvec/Tvec) ���� �ǽ�
		Mat Rmat, R, T; //���� ������ �ʿ��� �ӽú�����
		vector<int> inliers; //knn ��Ī �� �ƿ����̾� �ε��� ����Ʈ
		cv::solvePnPRansac(ObjectCorners, ImageCorners, CameraMatrix, Mat(), R, T, false, 100, 1.3f, 0.989999999999999, inliers, solvePnpRansacFlag);
		//�̹� �ְ�� �� ������ ��������Ƿ�, �ְ����� �Է����� ����
		/*	 enum { 
		SOLVEPNP_ITERATIVE = 0,
		SOLVEPNP_EPNP      = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
		SOLVEPNP_P3P       = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
		SOLVEPNP_DLS       = 3, // Joel A. Hesch and Stergios I. Roumeliotis. "A Direct Least-Squares (DLS) Method for PnP"
		SOLVEPNP_UPNP      = 4  // A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. "Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation" 
			}; */

		Rodrigues(R, Rmat); //ȸ������� �ε帮���� ��ȯ
		vector<Point2f> DstRect; //3���� ������ǥ�� 2���� ��� ������
		world2imageCoord(CubePoints, DstRect, Rmat, T); //3 -> 2���� ��ǥ ��ȯ

		if (MARKER_AXISDRAW_ONOFF) { //��Ŀ�̹��� ���� ������ �׸���
			arrowedLine(rview, DstRect[0], DstRect[1], Scalar(0, 0, 255), 3); //������ǥ X�༱ ǥ��
			arrowedLine(rview, DstRect[0], DstRect[3], Scalar(0, 255, 0), 3); //������ǥ Y�༱ ǥ��
			arrowedLine(rview, DstRect[0], DstRect[4], Scalar(255, 0, 0), 3); //������ǥ Z�༱ ǥ��
			circle(rview, DstRect[0], 10, Scalar(255, 255, 255), 3);	//�� �׸�
		}
		if (MARKER_CUBEDRAW_ONOFF) { //��Ŀ�̹��� ���� ť�� �׸���
			//line(rview, DstRect[0], DstRect[1], Scalar(0, 255, 255), 2);
			line(rview, DstRect[1], DstRect[2], Scalar(0, 255, 255), 2);
			line(rview, DstRect[2], DstRect[3], Scalar(0, 255, 255), 2);
			//line(rview, DstRect[3], DstRect[0], Scalar(0, 255, 255), 2);
			//line(rview, DstRect[0], DstRect[4], Scalar(0, 255, 255), 2);
			line(rview, DstRect[1], DstRect[5], Scalar(0, 255, 255), 2);
			line(rview, DstRect[2], DstRect[6], Scalar(0, 255, 255), 2);
			line(rview, DstRect[3], DstRect[7], Scalar(0, 255, 255), 2);
			line(rview, DstRect[4], DstRect[5], Scalar(0, 255, 255), 2);
			line(rview, DstRect[5], DstRect[6], Scalar(0, 255, 255), 2);
			line(rview, DstRect[6], DstRect[7], Scalar(0, 255, 255), 2);
			line(rview, DstRect[7], DstRect[4], Scalar(0, 255, 255), 2);
		}
		finish = clock();

		//����ð� ��� �� ǥ��
		double implementTime = (double)(finish - start) / CLOCKS_PER_SEC;
		String timeMsg = format("%0.3f sec/frame", implementTime);
		putText(rview, timeMsg, Point(60, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String solvePnpMsg = format("%s", flagOption);
		putText(rview, solvePnpMsg, Point(60, 135), 1, 1.5, Scalar(0, 0, 255), 2);

		//�̹��� ������ ��ȣ ǥ��
		String msg = format("%03d/%d", i + 1, _nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		putText(MatchingRview, msg, Point(_cols*0.65 + QueryImg.cols, 60), 1, 2, Scalar(0, 0, 255), 2);

		//��Ī �ζ��̾� ǥ��
		for (int i = 0; i < inliers.size(); i++) { //��Ī ȭ�鿡 ǥ�� : ������� �ζ��̾�
			circle(MatchingRview, Point(ObjectCorners[inliers[i]].x, ObjectCorners[inliers[i]].y), 3, Scalar(30, 255, 30), 2);
			circle(MatchingRview, Point(ImageCorners[inliers[i]].x + QueryImg.cols, ImageCorners[inliers[i]].y), 3, Scalar(30, 255, 30), 2);
			//line(MatchingRview, Point(ObjectCorners[inliers[i]].x, ObjectCorners[inliers[i]].y), Point(ImageCorners[inliers[i]].x + QueryImg.cols, ImageCorners[inliers[i]].y), Scalar(255, 30, 30), 2);
		}
		for (int i = 0; i < inliers.size(); i++) { //���� ȭ�鿡 ǥ�� : ������� �ζ��̾�
			circle(rview, ImageCorners[inliers[i]], 3, Scalar(30, 255, 30), 2);
		}
		String knnMatchMsg = format("KNN Ratio %0.1f", KNN_MATCH_RATIO);
		putText(MatchingRview, knnMatchMsg, Point(_cols*0.65 + QueryImg.cols, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String matchMsg = format("Inliers %d/%d", inliers.size(), MatchListBest.size());
		putText(MatchingRview, matchMsg, Point(_cols*0.65 + QueryImg.cols, 110), 1, 1.5, Scalar(0, 0, 255), 2);
		putText(rview, matchMsg, Point(60, 110), 1, 1.5, Scalar(0, 0, 255), 2);

		//��� ������ ����
		if (OutputVideo01.isOpened()) OutputVideo01 << rview;
		if (OutputVideo02.isOpened()) OutputVideo02 << MatchingRview;

		//���� �̹����� �ʹ� Ŭ��� ��� ��¸� �۰� ����
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//��� ���
		imshow("Matching Test Result", MatchingRview); //��Ī ��� Ȯ�� ���
		imshow("Undistort & drawing Cube Image", rview); //���� ��� ���

		if (waitKey(100 / _fps) == 27 || i == _nframe - 1) { //ESC�� �����ų� �����Ӽ���ŭ üũ�ϸ� ����
			destroyAllWindows();
			break;
		}
	}
}