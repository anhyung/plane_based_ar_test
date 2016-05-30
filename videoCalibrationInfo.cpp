#include "videoCalibrationInfo.h"

#define FRAME_UNIT 5 //처리 속도상 캘리브레이션 프레임수 제한걸기 : 몇장 단위 당 한장씩 선별
#define CUBE 5*28. //체스 패턴 위에 그릴 정육면체 한변 사이즈(mm단위)
#define CHESS_AXISDRAW_ONOFF true //체스 패턴 위에 방향축을 그릴지 말지 여부 On/Off
#define CHESS_CUBEDRAW_ONOFF true //체스 패턴 위에 큐브를 그릴지 말지 여부 On/Off

#define TRANSLATE_X 210 //증강 이미지 위치 조정
#define TRANSLATE_Y 0 //증강 이미지 위치 조정

#define RESIZE_QUERY 210 //증강 마커 이미지 리사이징을 위한 고정 가로 사이즈
#define REALSIZE_WIDTH 210. //증강 마커 이미지 실제 가로 사이즈(mm단위)
#define REALSIZE_HEIGHT 297. //증강 마커 이미지 실제 세로 사이즈(mm단위)
#define KNN_MATCH_RATIO 0.5f //작을수록 KNN 매칭 정확도 향상(단, 매칭수가 줄어듦)
#define HOMOGRAPHY_ONOFF false //추정된 호모그래피를 원본에 그릴지 말지 여부 On/Off : 매칭 결과에는 항상 표시
#define MARKER_AXISDRAW_ONOFF true //마커 이미지 위에 방향축을 그릴지 말지 여부 On/Off
#define MARKER_CUBEDRAW_ONOFF true //마커 이미지 위에 큐브를 그릴지 말지 여부 On/Off

#define MARKER_CUBE 100.f //마커 이미지 위에 그릴 정육면체 한변 사이즈(mm단위)
#define CUBE_X REALSIZE_WIDTH/2 //마커 이미지 위에 그릴 정육면체 위치 조정
#define CUBE_Y REALSIZE_HEIGHT/2 //마커 이미지 위에 그릴 정육면체 위치 조정


/** @기본 생성자 */
VideoCalibrationInfo::VideoCalibrationInfo() {
	setBoardSize(15, 23, 1.f);
	setInputVideo("video_cali_01.mp4");
	outputFilename = "calibration_data.xml";
	cameraName = "NullCamera";
	setOutputXml("TestVideo");
	flags = 0;
}

/** @사용자에 의한 초기화 생성자 */
VideoCalibrationInfo::VideoCalibrationInfo(int w, int h, String inputFilename, String outputCameraName, float squareSize, float aspectRatio) {
	setBoardSize(w, h, squareSize, aspectRatio);
	this->inputFilename = inputFilename;
	setInputVideo(inputFilename);
	outputFilename = "calibration_data.xml";
	cameraName = outputCameraName;
	setOutputXml(outputCameraName);
	flags = 0;
}

/** @소멸자 */
VideoCalibrationInfo::~VideoCalibrationInfo() {
}

/** @체스보드 규격 입력(가로코너, 세로코너, 사각형 눈금 단위, 사각형 비율) */
void VideoCalibrationInfo::setBoardSize(int w, int h, float squareSize, float aspectRatio) {
	BoardSize.width = w;
	BoardSize.height = h;
	this->squareSize = squareSize;
	this->aspectRatio = aspectRatio;
}

/** @출력 데이터 파일명 입력 */
void VideoCalibrationInfo::setOutputXml(String cameraName) {
	outputFilename = format("%s_calibration_data.xml", cameraName);
}

/** @리프로젝션 에러율 계산 */
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

/** @체스보드 코너로부터 월드좌표(공간좌표) 배열 설정 */
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

/** @캘리브레이션 계산 및 RMS 에러 계산 */
bool VideoCalibrationInfo::runCalibration(vector<float> &reprojErrs, double &totalAvgErr) {
	CameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO) CameraMatrix.at<double>(0, 0) = aspectRatio;
	DistCoeffs = Mat::zeros(8, 1, CV_64F);
	
	//K3이상의 왜곡 지수는 0으로 고정(어안렌즈가 아니므로 없다고 가정, 보정X)
	double rms = calibrateCamera(ObjectPoints, ImagePoints, ImageSize, CameraMatrix, DistCoeffs, Rvect, Tvect, flags | CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5);
	printf("\nRMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(CameraMatrix) && checkRange(DistCoeffs);
	totalAvgErr = computeReprojectionErrors(ObjectPoints, reprojErrs);

	return ok;
}

/** @계산된 데이터 xml파일로 기록 */
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

/** @계산함수들 실행, 캘리브레이션 계산하고 저장함 */
bool VideoCalibrationInfo::runAndSave() {
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n\n", (ok ? "Calibration succeeded" : "Calibration failed"), totalAvgErr);

	if (ok) saveCameraParams(reprojErrs, totalAvgErr);

	return ok;
}

/** @입력 비디오 정보 초기화 */
void VideoCalibrationInfo::setInputVideo(String inputFilename) {
	//테스트 입력 동영상 불러오기
	InputVideo.open(inputFilename);
	if (!InputVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

	//입력 동영상 정보 
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

/** @입력영상에서 체스판 코너 검출하고 데이터 구축, 캘리브레이션 수행 및 결과 출력 */
void VideoCalibrationInfo::cameraCalibration() {
	//동영상 저장용 인코더 셋팅
	String OutputVideoName = format("%s_ChessboardDetection.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view;	//이미지 저장할 임시 데이터
	Mat viewGray;	//서브픽셀을 적용할 이미지 데이터
	int success = 0;
	
	caliFrame = nframe / FRAME_UNIT + 1; //캘리브레이션에 사용된 총 프레임 수(0-based이므로 +1)
	calcChessboardCorners(caliFrame); //체스보드의 공간좌표 계산
		
	for (int i = 0; i < caliFrame; i++) { //처리 속도상 프레임수 제한걸기
		InputVideo.set(CAP_PROP_POS_FRAMES, i*FRAME_UNIT); //전체 프레임 중 FRAME_UNIT단위로 건너뛴 i번째 프레임으로 이동
		InputVideo >> view;	//이미지 불러오기

		vector<Point2f> ImageCorners;	//영상 평면 상의 좌표
		cvtColor(view, viewGray, CV_BGR2GRAY);	//서브픽셀 함수는 그레이스케일을 인자로 받음

		//체스판 코너 검출
		bool found = findChessboardCorners(view, BoardSize, ImageCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			cornerSubPix(viewGray, ImageCorners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			drawChessboardCorners(view, BoardSize, ImageCorners, found); //코너 검출 시 원본 영상에 표시
			ImagePoints.push_back(ImageCorners);
			success++;
		}
		else {
			notFound.push_back(InputVideo.get(CAP_PROP_POS_FRAMES)); //체스보드 검출 실패 이미지 인덱스 추가(따로 제외시키기 위함)

			//이미지 실패 메시지 표기
			String msg = "Detection Failed";
			putText(view, msg, Point(view.cols / 2 - 130, view.rows / 2), 1, 5, Scalar(0, 0, 255), 4);
		}
		//이미지 프레임 번호 표기
		String msg = format("%03d/%d", i + 1, caliFrame);
		putText(view, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);

		//체스보드 검출 동영상 저장
		if (OutputVideo.isOpened()) OutputVideo << view;

		//원본 이미지가 너무 클경우 결과 출력만 작게 조정
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(view, view, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//검출 결과 재생
		imshow("ChessBoard Corners Detection", view);

		if (waitKey(100 / fps) == 27 || i == caliFrame - 1) { //ESC를 누르거나 프레임수만큼 체크하면 종료 : 1000/fps가 원본 재생 속도
			destroyWindow("ChessBoard Corners Detection");
			break;
		}
	}
	cout << endl << success << " Images Chesssboard Pattern Detection Success!!" << endl << "Computing Calibration..." << endl;

	//K3 왜곡지수를 제외시키기 위함
	DistCoeffs = Mat::zeros(5, 1, CV_64F); //K3 왜곡계수를 0으로 초기화 고정시키기 위함
	
	runAndSave();	//캘리브레이션 수행, xml 데이터 작성, 오차율 계산

	cout << endl << "Camera Matrix :" << endl << CameraMatrix << endl;
	cout << endl << "Distortion Coefficients :" << endl << DistCoeffs << endl << endl;
}

/** @캘리브레이션 후 계산된 왜곡계수로 입력영상 왜곡 보정 */
void VideoCalibrationInfo::undistort() {
	//동영상 저장용 인코더 셋팅
	String OutputVideoName = format("%s_Undistortion.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view, rview, map1, map2;
	//Mat newCameraMatrix = getOptimalNewCameraMatrix(CameraMatrix, DistCoeffs, ImageSize, 0, ImageSize, 0);
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, ImageSize, CV_16SC2, map1, map2);

	//입력 영상 다시 시작지점으로 돌리기(캘리브레이션 때 재생했으므로)
	if (InputVideo.get(CAP_PROP_POS_FRAMES) != 0) InputVideo.set(CAP_PROP_POS_FRAMES, 0);
			
	for (int i = 0; i < nframe; i++) {
		InputVideo.set(CAP_PROP_POS_FRAMES, i);
		InputVideo >> view;	//이미지 불러오기

		//undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
		remap(view, rview, map1, map2, INTER_LINEAR);

		//이미지 프레임 번호 표기
		String msg = format("%03d/%d", i + 1, nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);

		//보정 결과 동영상 저장
		if (OutputVideo.isOpened()) OutputVideo << rview;

		//보정 이미지가 너무 클경우 결과 출력만 작게 조정
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//보정 결과 재생
		imshow("Undistort Image", rview);

		if (waitKey(100 / fps) == 27 || i == nframe - 1) { //ESC를 누르거나 프레임수만큼 체크하면 종료
			destroyWindow("Undistort Image");
			break;
		}
	}
}

/** @추정된 R/T와 카메라 매트릭스로 월드좌표를 영상 평면 좌표로 변환(보정 확인용) */
void VideoCalibrationInfo::world2imageCoord(vector<Point3f> &PointWorld, vector<Point2f> &PointImage, Mat &Rmat, Mat &T) {
	CameraMatrix.convertTo(CameraMatrix, CV_64F); //곱하려면 32F->64F로 형변환
	for (int i = 0; i < PointWorld.size(); i++) {
		double worldXYZ[3] = { PointWorld[i].x, PointWorld[i].y, PointWorld[i].z };
		Mat Xobj(3, 1, CV_64FC1, worldXYZ); //월드좌표 열벡터
		Mat Xc = Rmat*Xobj + T;	//월드좌표를 카메라 좌표로 전환
		Mat_<double> xy = CameraMatrix*Xc;
		xy /= xy(2);	//호모지니어스 스케일 정규화

		PointImage.push_back(Point2f(xy(0), xy(1)));
	}
	CameraMatrix.convertTo(CameraMatrix, CV_32F); //원래 데이터 32F로 형변환
}

/** @특징점 키포인트를 평면좌표로 변환 */
void VideoCalibrationInfo::KeyPoint2Point2f(vector<KeyPoint> &InputKeypoint, vector<Point2f> &OutputKeypoint) {
	if (!OutputKeypoint.empty()) OutputKeypoint.clear();

	for (int i = 0; i < InputKeypoint.size(); i++) {
		OutputKeypoint.push_back(InputKeypoint[i].pt);
	}
}

/** @평면좌표를 특징점 키포인트로 변환 */
void VideoCalibrationInfo::Point2f2KeyPoint(vector<Point2f> &InputKeypoint, vector<KeyPoint> &OutputKeypoint) {
	if (!OutputKeypoint.empty()) OutputKeypoint.clear();

	for (int i = 0; i < InputKeypoint.size(); i++) {
		OutputKeypoint.push_back(KeyPoint(InputKeypoint[i], 1.f));
	}
}

/** @캘리브레이션 후 왜곡 보정 후 체크 패턴 위에 기하추정용 큐브 객체 그리기 */
void VideoCalibrationInfo::undistortAndDrawingChessCube() {
	//동영상 저장용 인코더 셋팅
	String OutputVideoName = format("%s_CubeDrawingChessPattern.avi", cameraName);
	VideoWriter OutputVideo(OutputVideoName, CV_FOURCC('P', 'I', 'M', '1'), fps, ImageSize);
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << endl << "Save Video File : " << OutputVideoName << endl;

	Mat view, rview, map1, map2;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, ImageSize, CV_16SC2, map1, map2);
	
	//포즈 추정에 필요한 임시변수들
	Mat Rmat, R, T;
	vector<Point2f> ImageCorners;
	vector<Point3f> ObjectCorners;
	for (int i = 0; i < BoardSize.height; i++) {
		for (int j = 0; j < BoardSize.width; j++) {
			ObjectCorners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		}
	}
	
	//체스보드 위에 그릴 3차원 큐브 꼭지점의 월드좌표 정의
	vector<Point3f> CubePoints;
	CubePoints.push_back(Point3f(0., 0., 0.));
	CubePoints.push_back(Point3f(CUBE, 0., 0.));
	CubePoints.push_back(Point3f(CUBE, CUBE, 0.));
	CubePoints.push_back(Point3f(0., CUBE, 0.));
	CubePoints.push_back(Point3f(0., 0., -CUBE));
	CubePoints.push_back(Point3f(CUBE, 0., -CUBE));
	CubePoints.push_back(Point3f(CUBE, CUBE, -CUBE));
	CubePoints.push_back(Point3f(0., CUBE, -CUBE));
	
	//입력 영상 다시 시작지점으로 돌리기(캘리브레이션 때 재생했으므로)
	if (InputVideo.get(CAP_PROP_POS_FRAMES) != 0) InputVideo.set(CAP_PROP_POS_FRAMES, 0);

	for (int i = 0; i < nframe; i++) {
		InputVideo.set(CAP_PROP_POS_FRAMES, i);
		InputVideo >> view;	//이미지 불러오기
		
		remap(view, rview, map1, map2, INTER_LINEAR);

		//체스보드 마커 인식 : 이미 왜곡 보정한 rview가 아닌 왜곡된 view에 해야함(왜곡 보정 중복 방지)
		bool found = findChessboardCorners(view, BoardSize, ImageCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {	//마커 인식 성공시 포즈(카메라의 Rvec/Tvec) 추정 실시
			solvePnP(ObjectCorners, ImageCorners, CameraMatrix, DistCoeffs, R, T);
			
			//체스보드 위에 큐브 그리기
			Rodrigues(R, Rmat); //회전행렬의 로드리게즈 변환
			vector<Point2f> DstRect; //3차원 월드좌표의 2차원 평면 투영값
			world2imageCoord(CubePoints, DstRect, Rmat, T); //3 -> 2차원 좌표 변환

			if (CHESS_AXISDRAW_ONOFF) { //체스보드 패턴 위에 방향축 그리기
				arrowedLine(rview, DstRect[0], DstRect[1], Scalar(0, 0, 255), 3); //절대좌표 X축선 표시
				arrowedLine(rview, DstRect[0], DstRect[3], Scalar(0, 255, 0), 3); //절대좌표 Y축선 표시
				arrowedLine(rview, DstRect[0], DstRect[4], Scalar(255, 0, 0), 3); //절대좌표 Z축선 표시
				circle(rview, DstRect[0], 10, Scalar(255, 255, 255), 3);	//원 그림
			}
			if (CHESS_CUBEDRAW_ONOFF) { //체스보드 패턴 위에 큐브 그리기
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

		//이미지 프레임 번호 표기
		String msg = format("%03d/%d", i + 1, nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		
		//보정 결과 동영상 저장
		if (OutputVideo.isOpened()) OutputVideo << rview;

		//보정 이미지가 너무 클경우 결과 출력만 작게 조정
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}		

		//보정 결과 재생
		imshow("Undistort & drawing Cube Image", rview);

		if (waitKey(100 / fps) == 27 || i == nframe - 1) { //ESC를 누르거나 프레임수만큼 체크하면 종료
			destroyWindow("Undistort & drawing Cube Image");
			break;
		}
	}
}

/** @평면 AR : 캘리브레이션 후 왜곡 보정 후 마커 이미지 옆에 동영상 그리기 */
void VideoCalibrationInfo::undistortAndDrawingMarkerVideo(String MarkerImage, String MarkerVideo, String ArVideo) {
	//증강 마커로 쓰일 쿼리 이미지
	Mat QueryImg = imread(MarkerImage, IMREAD_COLOR);
	if (QueryImg.empty()) { 
		cerr << "Image File Open Error" << endl; 
		exit(1); 
	}

	//쿼리 이미지가 가로 혹은 세로가 1000이상일 경우 임의의 사이즈로 축소 조정
	resize(QueryImg, QueryImg, Size(RESIZE_QUERY, (QueryImg.rows * RESIZE_QUERY) / QueryImg.cols));

	//특징 검출 기술자 선언 : 쿼리 이미지 특징 데이터 구축
	Ptr<FeatureDetector> Feature = AKAZE::create();
	vector<KeyPoint> KeyPtQuery, KeyPtTrain;
	Feature->detect(QueryImg, KeyPtQuery);
	Mat DescriptQuery, DescriptTrain;
	Feature->compute(QueryImg, KeyPtQuery, DescriptQuery);

	//증강할 마커 촬영 동영상 불러오기
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

	//동영상1 저장용 인코더 셋팅 : 큐브 그린 동영상
	String OutputVideoName01 = format("%s_VideoDrawingMarkerImageHomo.avi", cameraName);
	VideoWriter OutputVideo01(OutputVideoName01, CV_FOURCC('P', 'I', 'M', '1'), _fps, _ImageSize);
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo01.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << endl << "Save Video File : " << OutputVideoName01 << endl;

	//동영상2 저장용 인코더 셋팅 : 매칭 결과 동영상
	String OutputVideoName02 = format("%s_FeatureMatchMarkerImageHomo.avi", cameraName);
	VideoWriter OutputVideo02(OutputVideoName02, CV_FOURCC('P', 'I', 'M', '1'), _fps, Size(_cols + QueryImg.cols, (_rows > QueryImg.rows) ? _rows : QueryImg.rows));
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo02.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << "Save Video File : " << OutputVideoName02 << endl;

	Mat view, rview, map1, map2, viewAR, rviewAR;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, _ImageSize, CV_16SC2, map1, map2);


	//####### 증강할 동영상 불러오기 #######
	VideoCapture InputArVideo(ArVideo);
	if (!InputArVideo.isOpened()) {
		cerr << "Video File Open Error" << endl;
		exit(1);
	}

		
	//알고리즘 시간체크용 변수
	clock_t start, finish;

	for (int i = 0; i < _nframe; i++) {
		InputMarkerVideo.set(CAP_PROP_POS_FRAMES, i);
		InputMarkerVideo >> view;	//이미지 불러오기

		//자동 반복 재생(현재 프레임 위치가 마지막이면 처음으로 되돌림)
		if (InputArVideo.get(CAP_PROP_POS_FRAMES) == (InputArVideo.get(CAP_PROP_FRAME_COUNT) - 1)) InputArVideo.set(CAP_PROP_POS_FRAMES, 0);
		InputArVideo >> viewAR;	//증강 영상 불러오기
		resize(viewAR, viewAR, Size(RESIZE_QUERY, (viewAR.rows * RESIZE_QUERY) / viewAR.cols));

		start = clock();
		remap(view, rview, map1, map2, INTER_LINEAR);

		//###### 쿼리이미지 마커 인식 ######
		//프레임 특징 검출 및 기술 계산
		Feature->detect(rview, KeyPtTrain);
		Feature->compute(rview, KeyPtTrain, DescriptTrain);

		//기술 매칭 : BruteForce - KNN Matching(2-NN)
		Ptr<DescriptorMatcher> Matcher = DescriptorMatcher::create("BruteForce-Hamming");
		vector<vector<DMatch> > MatchListKNN;
		Matcher->knnMatch(DescriptQuery, DescriptTrain, MatchListKNN, 2);
		
		vector<KeyPoint> KeyPtQueryBest, KeyPtTrainBest;
		vector<DMatch> MatchListBest;
		for (size_t i = 0; i < MatchListKNN.size(); i++) {
			DMatch first = MatchListKNN[i][0];
			float dist1 = MatchListKNN[i][0].distance; //최근방 매칭점 거리
			float dist2 = MatchListKNN[i][1].distance; //차근방 매칭점 거리

			if (dist1 < KNN_MATCH_RATIO*dist2) { //"최근방거리/차근방거리 < 2-NN_MATCH_RATIO"을 만족하는 최근방을 정합
				KeyPtQueryBest.push_back(KeyPtQuery[first.queryIdx]);
				KeyPtTrainBest.push_back(KeyPtTrain[first.trainIdx]);
				MatchListBest.push_back(DMatch(KeyPtQueryBest.size() - 1, KeyPtTrainBest.size() - 1, 0));
			}
		}
		
		//최종 결과에 모든 특징 표시
		//for (int i = 0; i<KeyPtTrain.size(); i++) {
		//	circle(rview, KeyPtTrain[i].pt, 3, Scalar(0, 0, 255));
		//}

		Mat MatchingRview;
		drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview, Scalar(0, 0, 255), Scalar(0, 0, 255));
		//drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview);

		vector<Point2f> PointQuery, PointTrain;
		KeyPoint2Point2f(KeyPtQueryBest, PointQuery);
		KeyPoint2Point2f(KeyPtTrainBest, PointTrain);
		Mat inliersVec; //knn 매칭 중 아웃라이어 인덱스 리스트 열백터

		if (MatchListBest.size() > 4) {
			//호모그래피 추정 == 마커 인식
			Mat Homography = findHomography(PointQuery, PointTrain, RANSAC, 2.f, inliersVec);

			vector<Point2f> QueryCorners(4);
			vector<Point2f> ImageCorners(4);
			QueryCorners[0] = Point2f(0, 0);
			QueryCorners[1] = Point2f(QueryImg.cols, 0);
			QueryCorners[2] = Point2f(QueryImg.cols, QueryImg.rows);
			QueryCorners[3] = Point2f(0, QueryImg.rows);
			perspectiveTransform(QueryCorners, ImageCorners, Homography);

			//추정된 호모그래피 확인 == 마커 인식 확인
			if (HOMOGRAPHY_ONOFF) { //최종 결과 영상 위에 호모그래피 도식
				line(rview, ImageCorners[0], ImageCorners[1], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[1], ImageCorners[2], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[2], ImageCorners[3], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[3], ImageCorners[0], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[0], ImageCorners[2], Scalar(0, 255, 0), 2);
				line(rview, ImageCorners[1], ImageCorners[3], Scalar(0, 255, 0), 2);
			}
			//매칭 결과에 호모그래피 도식
			line(MatchingRview, Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[0].x + QueryImg.cols, ImageCorners[0].y), Point2f(ImageCorners[2].x + QueryImg.cols, ImageCorners[2].y), Scalar(255, 0, 255), 2);
			line(MatchingRview, Point2f(ImageCorners[1].x + QueryImg.cols, ImageCorners[1].y), Point2f(ImageCorners[3].x + QueryImg.cols, ImageCorners[3].y), Scalar(255, 0, 255), 2);
			
			Mat Mask(viewAR.rows + TRANSLATE_Y, viewAR.cols + TRANSLATE_X, viewAR.type(), Scalar::all(0));	//변형 후 영역 오려낼 마스크
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

		//수행시간 계산 및 표기
		double implementTime = (double)(finish - start) / CLOCKS_PER_SEC;
		String timeMsg = format("%0.3f sec/frame", implementTime);
		putText(rview, timeMsg, Point(60, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		
		//이미지 프레임 번호 표기
		String msg = format("%03d/%d", i + 1, _nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		putText(MatchingRview, msg, Point(_cols*0.65 + QueryImg.cols, 60), 1, 2, Scalar(0, 0, 255), 2);

		//매칭 인라이어 표시
		vector<int> inliers;
		for (int i = 0; i < inliersVec.rows; i++) {
			if (inliersVec.at<uchar>(i, 0) == 1) inliers.push_back(i);
		}
		for (int i = 0; i < inliers.size(); i++) {
			//최종 결과에 표시
			//circle(rview, Point(PointTrain[inliers[i]].x, PointTrain[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
			//매칭 결과에 표시
			circle(MatchingRview, Point(PointQuery[inliers[i]].x, PointQuery[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
			circle(MatchingRview, Point(PointTrain[inliers[i]].x + QueryImg.cols, PointTrain[inliers[i]].y), 3, Scalar(0, 255, 0), 2);
		}
		String knnMatchMsg = format("KNN Ratio %0.1f", KNN_MATCH_RATIO);
		putText(MatchingRview, knnMatchMsg, Point(_cols*0.65 + QueryImg.cols, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String matchMsg = format("Inliers %d/%d", inliers.size(), MatchListBest.size());
		putText(MatchingRview, matchMsg, Point(_cols*0.65 + QueryImg.cols, 110), 1, 1.5, Scalar(0, 0, 255), 2);
		putText(rview, matchMsg, Point(60, 110), 1, 1.5, Scalar(0, 0, 255), 2);

		//결과 동영상 저장
		//if (OutputVideo01.isOpened()) OutputVideo01 << rview;
		//if (OutputVideo02.isOpened()) OutputVideo02 << MatchingRview;

		//보정 이미지가 너무 클경우 결과 출력만 작게 조정
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}		

		//결과 출력
		imshow("Matching Test Result", MatchingRview); //매칭 결과 확인 재생
		imshow("Undistort & drawing Video Image", rview); //최종 결과 재생

		if (waitKey(100 / _fps) == 27 || i == _nframe - 1) { //ESC를 누르거나 프레임수만큼 체크하면 종료
			destroyAllWindows();
			break;
		}
	}
}

/** @캘리브레이션 후 왜곡 보정 후 마커 이미지 위에 기하추정용 큐브 객체 그리기 */
void VideoCalibrationInfo::undistortAndDrawingMarkerCube(String MarkerImage, String MarkerVideo, int solvePnpRansacFlag) {
	//증강 마커로 쓰일 쿼리 이미지
	Mat QueryImg = imread(MarkerImage, IMREAD_COLOR);
	if (QueryImg.empty()) {
		cerr << "Image File Open Error" << endl;
		exit(1);
	}

	//쿼리 이미지가 가로 혹은 세로가 1000이상일 경우 임의의 사이즈로 축소 조정
	resize(QueryImg, QueryImg, Size(RESIZE_QUERY, (QueryImg.rows * RESIZE_QUERY) / QueryImg.cols));

	//특징 검출 기술자 선언 : 쿼리 이미지 특징 데이터 구축
	Ptr<FeatureDetector> Feature = AKAZE::create();
	vector<KeyPoint> KeyPtQuery, KeyPtTrain;
	Feature->detect(QueryImg, KeyPtQuery);
	Mat DescriptQuery, DescriptTrain;
	Feature->compute(QueryImg, KeyPtQuery, DescriptQuery);

	//증강할 마커 촬영 동영상 불러오기
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

	//동영상1 저장용 인코더 셋팅 : 큐브 그린 동영상
	String OutputVideoName01 = format("%s_CubeDrawingMarkerImage_%d.avi", cameraName, solvePnpRansacFlag);
	VideoWriter OutputVideo01(OutputVideoName01, CV_FOURCC('P', 'I', 'M', '1'), _fps, _ImageSize);
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo01.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << endl << "Save Video File : " << OutputVideoName01 << endl;

	//동영상2 저장용 인코더 셋팅 : 매칭 결과 동영상
	String OutputVideoName02 = format("%s_FeatureMatchMarkerImage_KNN%0.1f.avi", cameraName, KNN_MATCH_RATIO);
	VideoWriter OutputVideo02(OutputVideoName02, CV_FOURCC('P', 'I', 'M', '1'), _fps, Size(_cols + QueryImg.cols, (_rows > QueryImg.rows) ? _rows : QueryImg.rows));
	//코덱 : CV_FOURCC('P','I','M','1'): MPEG-1, CV_FOURCC('M', 'J', 'P', 'G') : Motion-JPEG, CV_FOURCC('X', 'V', 'I', 'D') : XVID, VideoCapture::get(CV_CAP_PROP_FOURCC)
	if (!OutputVideo02.isOpened()) cerr << "Video Write Error" << endl; //저장 실패시 저장않고 그냥 실행 : exit(1) 안함
	cout << "Save Video File : " << OutputVideoName02 << endl;

	Mat view, rview, map1, map2;
	initUndistortRectifyMap(CameraMatrix, DistCoeffs, Mat(), CameraMatrix, _ImageSize, CV_16SC2, map1, map2);
	
	//마커이미지 위에 그릴 3차원 큐브 꼭지점의 월드좌표 정의 : 큐브 형태 정의
	vector<Point3f> CubePoints;
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y + MARKER_CUBE, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y + MARKER_CUBE, 0.));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2 + MARKER_CUBE, CUBE_Y + MARKER_CUBE, -MARKER_CUBE));
	CubePoints.push_back(Point3f(CUBE_X - MARKER_CUBE / 2, CUBE_Y + MARKER_CUBE, -MARKER_CUBE));

	//알고리즘 시간체크용 변수
	clock_t start, finish;

	for (int i = 0; i < _nframe; i++) {
		InputMarkerVideo.set(CAP_PROP_POS_FRAMES, i);
		InputMarkerVideo >> view;	//이미지 불러오기
		
		start = clock();
		remap(view, rview, map1, map2, INTER_LINEAR);

		//###### 쿼리이미지 마커 인식 ######
		//프레임 특징 검출 및 기술 계산
		Feature->detect(rview, KeyPtTrain);
		Feature->compute(rview, KeyPtTrain, DescriptTrain);

		//기술 매칭 : BruteForce - KNN Matching(2-NN)
		Ptr<DescriptorMatcher> Matcher = DescriptorMatcher::create("BruteForce-Hamming");
		vector<vector<DMatch> > MatchListKNN;
		Matcher->knnMatch(DescriptQuery, DescriptTrain, MatchListKNN, 2);

		vector<KeyPoint> KeyPtQueryBest, KeyPtTrainBest;
		vector<DMatch> MatchListBest;
		for (size_t i = 0; i < MatchListKNN.size(); i++) {
			DMatch first = MatchListKNN[i][0];
			float dist1 = MatchListKNN[i][0].distance; //최근방 매칭점 거리
			float dist2 = MatchListKNN[i][1].distance; //차근방 매칭점 거리

			if (dist1 < KNN_MATCH_RATIO*dist2) { //"최근방거리/차근방거리 < 2-NN_MATCH_RATIO"을 만족하는 최근방을 정합
				KeyPtQueryBest.push_back(KeyPtQuery[first.queryIdx]);
				KeyPtTrainBest.push_back(KeyPtTrain[first.trainIdx]);
				MatchListBest.push_back(DMatch(KeyPtQueryBest.size() - 1, KeyPtTrainBest.size() - 1, 0));
			}
		}

		Mat MatchingRview;
		drawMatches(QueryImg, KeyPtQueryBest, rview, KeyPtTrainBest, MatchListBest, MatchingRview, Scalar(0, 0, 255), Scalar(0, 0, 255)); //매칭 결과를 붉은 점으로 표시

		vector<Point3f> ObjectCorners;
		vector<Point2f> ImageCorners;
		for (int i = 0; i < KeyPtQueryBest.size(); i++) {
			ObjectCorners.push_back(Point3f(KeyPtQueryBest[i].pt.x, KeyPtQueryBest[i].pt.y, 0.));
		}
		KeyPoint2Point2f(KeyPtTrainBest, ImageCorners);

		//마커 인식 위치 확인 성공시 포즈(카메라의 Rvec/Tvec) 추정 실시
		Mat Rmat, R, T; //포즈 추정에 필요한 임시변수들
		vector<int> inliers; //knn 매칭 중 아웃라이어 인덱스 리스트
		cv::solvePnPRansac(ObjectCorners, ImageCorners, CameraMatrix, Mat(), R, T, false, 100, 1.3f, 0.989999999999999, inliers, solvePnpRansacFlag);
		//이미 왜곡보정 된 영상을 사용했으므로, 왜곡계수는 입력하지 않음
		/*	 enum { 
		SOLVEPNP_ITERATIVE = 0,
		SOLVEPNP_EPNP      = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
		SOLVEPNP_P3P       = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
		SOLVEPNP_DLS       = 3, // Joel A. Hesch and Stergios I. Roumeliotis. "A Direct Least-Squares (DLS) Method for PnP"
		SOLVEPNP_UPNP      = 4  // A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. "Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation" 
			}; */

		Rodrigues(R, Rmat); //회전행렬의 로드리게즈 변환
		vector<Point2f> DstRect; //3차원 월드좌표의 2차원 평면 투영값
		world2imageCoord(CubePoints, DstRect, Rmat, T); //3 -> 2차원 좌표 변환

		if (MARKER_AXISDRAW_ONOFF) { //마커이미지 위에 방향축 그리기
			arrowedLine(rview, DstRect[0], DstRect[1], Scalar(0, 0, 255), 3); //절대좌표 X축선 표시
			arrowedLine(rview, DstRect[0], DstRect[3], Scalar(0, 255, 0), 3); //절대좌표 Y축선 표시
			arrowedLine(rview, DstRect[0], DstRect[4], Scalar(255, 0, 0), 3); //절대좌표 Z축선 표시
			circle(rview, DstRect[0], 10, Scalar(255, 255, 255), 3);	//원 그림
		}
		if (MARKER_CUBEDRAW_ONOFF) { //마커이미지 위에 큐브 그리기
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

		//수행시간 계산 및 표기
		double implementTime = (double)(finish - start) / CLOCKS_PER_SEC;
		String timeMsg = format("%0.3f sec/frame", implementTime);
		putText(rview, timeMsg, Point(60, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String solvePnpMsg = format("%s", flagOption);
		putText(rview, solvePnpMsg, Point(60, 135), 1, 1.5, Scalar(0, 0, 255), 2);

		//이미지 프레임 번호 표기
		String msg = format("%03d/%d", i + 1, _nframe);
		putText(rview, msg, Point(60, 60), 1, 2, Scalar(0, 0, 255), 2);
		putText(MatchingRview, msg, Point(_cols*0.65 + QueryImg.cols, 60), 1, 2, Scalar(0, 0, 255), 2);

		//매칭 인라이어 표시
		for (int i = 0; i < inliers.size(); i++) { //매칭 화면에 표시 : 녹색점이 인라이어
			circle(MatchingRview, Point(ObjectCorners[inliers[i]].x, ObjectCorners[inliers[i]].y), 3, Scalar(30, 255, 30), 2);
			circle(MatchingRview, Point(ImageCorners[inliers[i]].x + QueryImg.cols, ImageCorners[inliers[i]].y), 3, Scalar(30, 255, 30), 2);
			//line(MatchingRview, Point(ObjectCorners[inliers[i]].x, ObjectCorners[inliers[i]].y), Point(ImageCorners[inliers[i]].x + QueryImg.cols, ImageCorners[inliers[i]].y), Scalar(255, 30, 30), 2);
		}
		for (int i = 0; i < inliers.size(); i++) { //최종 화면에 표시 : 녹색점이 인라이어
			circle(rview, ImageCorners[inliers[i]], 3, Scalar(30, 255, 30), 2);
		}
		String knnMatchMsg = format("KNN Ratio %0.1f", KNN_MATCH_RATIO);
		putText(MatchingRview, knnMatchMsg, Point(_cols*0.65 + QueryImg.cols, 85), 1, 1.5, Scalar(0, 0, 255), 2);
		String matchMsg = format("Inliers %d/%d", inliers.size(), MatchListBest.size());
		putText(MatchingRview, matchMsg, Point(_cols*0.65 + QueryImg.cols, 110), 1, 1.5, Scalar(0, 0, 255), 2);
		putText(rview, matchMsg, Point(60, 110), 1, 1.5, Scalar(0, 0, 255), 2);

		//결과 동영상 저장
		if (OutputVideo01.isOpened()) OutputVideo01 << rview;
		if (OutputVideo02.isOpened()) OutputVideo02 << MatchingRview;

		//보정 이미지가 너무 클경우 결과 출력만 작게 조정
		if (ImageSize.width > 1200 || ImageSize.height > 900) {
			resize(rview, rview, Size(800, 800 * ImageSize.height / ImageSize.width));
		}

		//결과 출력
		imshow("Matching Test Result", MatchingRview); //매칭 결과 확인 재생
		imshow("Undistort & drawing Cube Image", rview); //최종 결과 재생

		if (waitKey(100 / _fps) == 27 || i == _nframe - 1) { //ESC를 누르거나 프레임수만큼 체크하면 종료
			destroyAllWindows();
			break;
		}
	}
}