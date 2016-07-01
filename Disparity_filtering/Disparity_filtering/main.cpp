/*
*  stereo_match.cpp
*  calibration
*
*  Created by Victor  Eruhimov on 1/18/10.
*  Copyright 2010 Argus Corp. All rights reserved.
*
*/

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

void CallBackFunc1(int event, int x, int y, int flags, void* userdata);
void CallBackFunc2(int event, int x, int y, int flags, void* userdata);

int xLeft=0, yLeft=0, xRight=0, yRight=0;
bool leftControl = false, 
	 rightControl = false;

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
	vector<Point3f> imgPoints;
	vector<Point3f> dstPoints;

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3 };
	int alg = STEREO_SGBM;
	int SADWindowSize = 0, numberOfDisparities = 0;
	bool no_display = false;
	float scale = 1.f;

	Ptr<StereoBM> bm = StereoBM::create(16, 9);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

	//sprintf(chrFileNm, "Frames\\Data1\\%06d_L.png", recordCount);
	//img1_filename = "Frames\\Data1\\000001_L.png";
	//img2_filename = "Frames\\Data1\\000001_R.png";
	img1_filename = "Frames\\Data16\\left2.jpg";
	img2_filename = "Frames\\Data16\\right2.jpg";
	intrinsic_filename = "intrinsics.yml";
	extrinsic_filename = "extrinsics.yml";

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

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1 = imread(img1_filename, color_mode);
	Mat img2 = imread(img2_filename, color_mode);

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
		
	} else{
		disp.convertTo(disp8, CV_8U);
		
	}
/*
	if (!no_display)
	{
		namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);
		namedWindow("disparity", 0);
		imshow("disparity", disp8);
		printf("press any key to continue...");
		fflush(stdout);
		waitKey();
		printf("\n");
	}
*/
	if (!no_display)
	{
		int c = 10;

		for (;;)
		{
			namedWindow("left", 1);
			imshow("left", img1);
			namedWindow("right", 1);
			imshow("right", img2);
			setMouseCallback("left", CallBackFunc1, NULL);
			setMouseCallback("right", CallBackFunc2, NULL);

			c = waitKey(20);

			if ((char)c == 27) //esc basilinca biter
			{
				break;
			}
			if (rightControl == true && leftControl == true) //a basilinca gecerli frami jpg olarak kaydeder
			{
				float distX = abs(xLeft - xRight);
				cout << "Distance X : " << distX << endl;
				cout << "Distance Y : " << abs(yLeft - yRight) << endl;
				rightControl = false;
				leftControl = false;
				imgPoints.push_back(Point3f(xLeft, yLeft, distX));
			}
		}
/*
		for (int i = 0; i < imgPoints.size(); i++)
		{
			cout << "Z value src: " << imgPoints[i].z << endl;
		}
*/		

		perspectiveTransform(imgPoints,dstPoints,Q);
		double totalDistX = 0, totalDistY = 0, totalDistZ = 0;
		for (int i = 0; i < dstPoints.size(); i++)
		{
			totalDistZ += dstPoints[i].z;
			totalDistY += dstPoints[i].y;
			totalDistX += dstPoints[i].x;
			cout << "Z: " << dstPoints[i].z << "\tY: " << dstPoints[i].y << "\tX: " << dstPoints[i].x << endl;
		}
		cout << "\nAverage Distance Z: " << totalDistZ / dstPoints.size() << endl;
		cout << "Average Distance Y: " << totalDistY / dstPoints.size() << endl;
		cout << "Average Distance X: " << totalDistX / dstPoints.size() << endl;
//		cout << "\nApproximate Distance: " << 4*(totalDist / dstPoints.size()) << endl;


		waitKey();
	}

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
