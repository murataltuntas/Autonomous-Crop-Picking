/*
*  CSE496 Autonomous Crop Picking
*
*/

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>
#include <stdio.h>
#include "Vector3.h"

using namespace cv;
using namespace std;

#define COEFFICIENT 5/1.35674

void CallBackFunc1(int event, int x, int y, int flags, void* userdata);
void CallBackFunc2(int event, int x, int y, int flags, void* userdata);
int asparagusDetect(Mat &imgOriginal, Mat &imgLines, Point &coordinate,
	vector<vector<Point>> &contours, vector<Vec4i> &, Mat &cny);

int xLeft = 0, yLeft = 0, xRight = 0, yRight = 0;
bool leftControl = false,
rightControl = false;

#if 0
// 07_L.avi ve 06_L.aviicin
int iLowH = 22;
int iHighH = 78;
int iLowS = 49;
int iHighS = 175;
int iLowV = 26;
int iHighV = 210;
#endif

#if 1
// 11_L.mp4 ve 11_L.mp4 icin
int iLowH = 23;
int iHighH = 50;
int iLowS = 30;
int iHighS = 141;
int iLowV = 103;
int iHighV = 170;
#define MINUTE 5
#endif

#if 0
// 10_R.mp4 ve 10_L.mp4 icin
int iLowH = 33;
int iHighH = 45;
int iLowS = 37;
int iHighS = 110;
int iLowV = 110;
int iHighV = 170;
#define MINUTE 3
#endif

int iLastX = -1;
int iLastY = -1;

static void print_help()
{
	printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
	printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
		"[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
		"[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}
static void saveXYZ(const char* filename, const Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

int main(int argc, char** argv)
{
	const char* algorithm_opt = "--algorithm=";
	const char* maxdisp_opt = "--max-disparity=";
	const char* blocksize_opt = "--blocksize=";
	const char* nodisplay_opt = "--no-display";
	const char* scale_opt = "--scale=";

	const char* img1_filename = 0;
	const char* img2_filename = 0;
	const char* intrinsic_filename = 0;
	const char* extrinsic_filename = 0;
	const char* disparity_filename = 0;
	const char* point_cloud_filename = "points.txt";

	char chrFileNm[200];
	int recordCount = 0;


	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3 };
	int alg = STEREO_SGBM;
	int SADWindowSize = 0, numberOfDisparities = 0;
	bool no_display = false;
	float scale = 1.f;

	Ptr<StereoBM> bm = StereoBM::create(16, 9);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
	Vector3* vector3D = new Vector3();
	Vector3* distanceVector3D = new Vector3();
	vector<vector<Point> > contours1;
	vector<vector<Point> > contours2;
	vector<Vec4i> hierarchy1;
	vector<Vec4i> hierarchy2;
	Point coordinate1;
	Point coordinate2;
	Mat img1, cny1;
	Mat img2, cny2;

	//sprintf(chrFileNm, "Frames\\Data1\\%06d_L.png", recordCount);
	//img1_filename = "Frames\\Data1\\000001_L.png";
	//img2_filename = "Frames\\Data1\\000001_R.png";
	img1_filename = "Frames\\Data1\\left2.jpg";
	img2_filename = "Frames\\Data1\\right2.jpg";
	intrinsic_filename = "intrinsics.yml";
	extrinsic_filename = "extrinsics.yml";

	VideoCapture cap1("C:/Users/Murat/Documents/Visual Studio 2013/Projects/TwoCameraExample/TwoCameraExample/Videos/11_L.mp4"); //capture the video from webcam
	VideoCapture cap2("C:/Users/Murat/Documents/Visual Studio 2013/Projects/TwoCameraExample/TwoCameraExample/Videos/11_R.mp4"); //capture the video from webcam

	cap1.set(CAP_PROP_POS_MSEC, MINUTE * 1000);
	cap2.set(CAP_PROP_POS_MSEC, MINUTE * 1000);

	if (!cap1.isOpened() || !cap2.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}


	if (!img1_filename || !img2_filename)
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		scanf("%d", &alg);
		return -1;
	}

	if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		scanf("%d", &alg);
		return -1;
	}

	if (extrinsic_filename == 0 && point_cloud_filename)
	{
		printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
		scanf("%d", &alg);
		return -1;
	}

	for (int n = 0;; n++)
	{

		int color_mode = alg == STEREO_BM ? 0 : -1;
		color_mode = 1;
		//Mat img1 = imread(img1_filename, color_mode);
		//Mat img2 = imread(img2_filename, color_mode);
		cap1 >> img1;
		cap2 >> img2;

		if (img1.empty())
		{
			printf("Command-line parameter error: could not load the first input image file\n");
			scanf("%d", &alg);
			return -1;
		}
		if (img2.empty())
		{
			printf("Command-line parameter error: could not load the second input image file\n");
			scanf("%d", &alg);
			return -1;
		}

		Mat imgLines1 = Mat::zeros(img1.size(), CV_8UC3);
		Mat imgLines2 = Mat::zeros(img2.size(), CV_8UC3);

		if (scale != 1.f)
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		}

		Size img_size = img1.size();

		Rect roi1, roi2;
		Mat Q;

		if (intrinsic_filename)
		{
			// reading intrinsic parameters
			FileStorage fs(intrinsic_filename, FileStorage::READ);
			if (!fs.isOpened())
			{
				printf("Failed to open file %s\n", intrinsic_filename);
				scanf("%d", &alg);
				return -1;
			}

			Mat M1, D1, M2, D2;
			fs["M1"] >> M1;
			fs["D1"] >> D1;
			fs["M2"] >> M2;
			fs["D2"] >> D2;

			M1 *= scale;
			M2 *= scale;

			fs.open(extrinsic_filename, FileStorage::READ);
			if (!fs.isOpened())
			{
				printf("Failed to open file %s\n", extrinsic_filename);
				scanf("%d", &alg);
				return -1;
			}

			Mat R, T, R1, P1, R2, P2;
			fs["R"] >> R;
			fs["T"] >> T;

			stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

			Mat map11, map12, map21, map22;
			initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

			Mat img1r, img2r;
			remap(img1, img1r, map11, map12, INTER_LINEAR);
			remap(img2, img2r, map21, map22, INTER_LINEAR);

			img1 = img1r;
			img2 = img2r;
		}

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterCap(31);
		bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		bm->setMinDisparity(0);
		bm->setNumDisparities(numberOfDisparities);
		bm->setTextureThreshold(10);    // texture threshold degeri
		bm->setUniquenessRatio(15);
		bm->setSpeckleWindowSize(100);  // 
		bm->setSpeckleRange(32);        // 
		bm->setDisp12MaxDiff(1);

		sgbm->setPreFilterCap(63);
		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
		sgbm->setBlockSize(sgbmWinSize);

		int cn = img1.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);
		sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

		Mat disp, disp8;
		//Mat img1p, img2p, dispp;
		//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		int64 t = getTickCount();
		if (alg == STEREO_BM)
			bm->compute(img1, img2, disp);
		else if (alg == STEREO_SGBM || alg == STEREO_HH)
			sgbm->compute(img1, img2, disp);
		t = getTickCount() - t;
		printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

		//disp = dispp.colRange(numberOfDisparities, img1p.cols);
		if (alg != STEREO_VAR){
			disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		}
		else{
			disp.convertTo(disp8, CV_8U);

		}

		if (!no_display)
		{
			int c = 10;
			int aspHeight = asparagusDetect(img1, imgLines1, coordinate1, contours1, hierarchy1,cny1);
			asparagusDetect(img2, imgLines2, coordinate2, contours2, hierarchy2,cny2);
			
			//			for (;;)
			{
				namedWindow("left", 1);
				imshow("left", img1);
				namedWindow("right", 1);
				imshow("right", img2);
				imshow("cny1", cny1);
				imshow("cny2", cny2);
				/*
				setMouseCallback("left", CallBackFunc1, NULL);
				setMouseCallback("right", CallBackFunc2, NULL);
				*/
				xLeft = coordinate1.x;
				yLeft = coordinate1.y;
				xRight = coordinate2.x;
				yRight = coordinate2.y;

				c = waitKey(20);
				/*
				if ((char)c == 27) //esc basilinca biter
				{
				break;
				}
				*/
				//				if (rightControl == true && leftControl == true) // her iki goruntuden de koordinatlar secildiyse
				if ((xLeft != 0 && yLeft != 0) && (xRight != 0 && yRight != 0))
				{
					vector<Point3f> imgPoints;
					vector<Point3f> dstPoints;

					float distX = abs(xLeft - xRight);
					cout << "Distance X : " << distX << endl;
					cout << "Distance Y : " << abs(yLeft - yRight) << endl;
					rightControl = false;
					leftControl = false;
					imgPoints.push_back(Point3f(xLeft, yLeft, distX));
					perspectiveTransform(imgPoints, dstPoints, Q);
					for (int k = 0; k < dstPoints.size(); k++)
					{
						//cout << "Z: " << dstPoints[k].z << "\tY: " << dstPoints[k].y << "\tX: " << dstPoints[k].x << endl;
						vector3D->setXYZ(0, 0, 0);
						distanceVector3D->setXYZ(dstPoints[k].z, dstPoints[k].y, dstPoints[k].x);
						// Calculate the distance between the vectors
						float distance3D = vector3D->DistanceTo(distanceVector3D);
						float realDistance = distance3D * COEFFICIENT;
						cout << "Real Distance: " << realDistance << endl;
						cout << "Real Height  : " << sqrt(aspHeight * realDistance) / COEFFICIENT << " cm" << endl;
					}

					xLeft = 0; yLeft = 0; xRight = 0; yRight = 0;
				}
			}
			/*
			for (int i = 0; i < imgPoints.size(); i++)
			{
			cout << "Z value src: " << imgPoints[i].z << endl;
			}
			*/
			//		cout << "\nApproximate Distance: " << 4*(totalDist / dstPoints.size()) << endl;


			//waitKey();
		}

		/*
		if (disparity_filename)
		imwrite(disparity_filename, disp8);

		if (point_cloud_filename)
		{
		printf("storing the point cloud...");
		fflush(stdout);
		Mat xyz;
		reprojectImageTo3D(disp, xyz, Q, true);
		saveXYZ(point_cloud_filename, xyz);
		printf("\n");
		}
		*/
	}
	return 0;
}

void CallBackFunc1(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "^LEFT IMAGE^ (" << x << ", " << y << ")" << endl;
		leftControl = true;
		xLeft = x;
		yLeft = y;
	}
}
void CallBackFunc2(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "^RIGHT IMAGE^ (" << x << ", " << y << ")" << endl;
		rightControl = true;
		xRight = x;
		yRight = y;
	}
}

int asparagusDetect(Mat &imgOriginal, Mat &imgLines, Point &coordinate,
	vector<vector<Point>> &contours, vector<Vec4i> &hierarchy, Mat &cny){

	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	GaussianBlur(imgHSV, imgHSV, Size(3, 3), 0, 0);
	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//imshow("Thresholded Image", imgThresholded); //show the thresholded image

	findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	int  max = 0, maxIndex = 0;
	for (int i = 0; i < contours.size(); i++){
		int temp = contourArea(contours[i]);

		if (max < temp){
			max = temp;
			maxIndex = i;
		}
	}



	//Calculate the moments of the thresholded image
	Moments oMoments = moments(imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00; // contour un alani
	int x = 0, y = 0;            // merkezin x ve y noktasi
	int aspHeight = 0;
	if (dArea > 30000){
		x = oMoments.m10 / dArea; // x noktasi
		y = oMoments.m01 / dArea; // y noktasi
		dArea = 0;

		cout << x << "\t" << y << endl;
		coordinate.x = x;
		coordinate.y = y;

		if (contours.size() > 0)
		{
			Rect  boundRect = boundingRect(Mat(contours[maxIndex]));
			aspHeight = boundRect.height;
			cout << "height : " << aspHeight << endl;
			rectangle(imgOriginal, boundRect, Scalar(255, 0, 0), 2, 8, 0);


			//Mat cny;
			cny = imgThresholded.clone();
			cny = cny(boundRect);
			cny = cny.rowRange(cny.rows / 3, cny.rows * 2 / 3).clone();
			//cny = cny.rowRange(cny.rows *2/ 3, cny.rows ).clone();
			Canny(cny, cny, 250, 255);
			vector<Vec4i> lines;
			HoughLinesP(cny, lines, 1, CV_PI / 180, 50, 0, 10);
			double max = 0, index = 0;
			for (int i = 0; i < lines.size(); i++)
			{
				Vec4i l = lines[i];
				double res = norm(Point(l[0], l[1]) - Point(l[2], l[3]));
				//max = max < res ? res : max;
				if (max < res)
				{
					max = res;
					index = i;
				}

			}
			if (lines.size() > 0){
				Vec4i l = lines[index];
				line(cny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 8, CV_AA);
				cout << "Begin: " << Point(l[0], l[1]) << "\tEnd: " << Point(l[2], l[3]) << endl;
//				cout << Point(l[2], l[3]).x << "\t" << Point(l[2], l[3]).y << endl;
//				coordinate.x = Point(l[2], l[3]).x;
//				coordinate.y = Point(l[2], l[3]).y;
			}

			//imshow("Canny", cny); //show the thresholded image


			//drawContours(imgOriginal, contours, maxIndex, Scalar(0, 0, 255), 2, LINE_AA, hierarchy, 0, Point());
		}
	}

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 10000)
	{
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;

		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			//Draw a red line from the previous point to the current point
			//line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
		}

		iLastX = posX;
		iLastY = posY;
	}
	imgOriginal = imgOriginal + imgLines;
	//imshow("Original", imgOriginal); //show the original image
	return aspHeight;
}