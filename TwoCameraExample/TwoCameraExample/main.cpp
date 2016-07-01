#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>  // time(&timev)  icin

using namespace cv;
using namespace std;

cv::VideoWriter videoRC1, videoRC2; // kameradan vido kaydi yapmak icin.

int main(int, char)
{
	bool videoControl = false;
	bool continuousControl = false;
	time_t  timev;
	string videoFile1, videoFile2, timer;
	stringstream ss;
	char chrFileNm[200];

	VideoCapture cap1; // open the default camera
	VideoCapture cap2; // open the default camera
	cap1 = cv::VideoCapture(0);
	cap2 = cv::VideoCapture(1);
	//cap1.set(CAP_PROP_FOCUS,0);

	if (!cap1.isOpened()) // check if we succeeded
		return -1;
	if (!cap2.isOpened()) // check if we succeeded
		return -1;
	Mat edges;
	String fileName;
	//namedWindow("left", 1);
	//namedWindow("right", 1);
	//namedWindow("camera-L-R", 1);
	int count1 = 0, recordCount=0;
	int frame_width = cap1.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = cap1.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	cout << "a basilinca gecerli frami jpg olarak kaydeder\n";
	cout << "z basilinca tum framleri jpg olarak kaydeder\n";
	cout << "x basilinca framleri jpg olarak kaydetmeyi durdurur\n";
	cout << "v basilinca video kaydi baslar\n";
	cout << "n basilinca video kaydi biter\n";
	cout << "ESC basilinca programdan cikar\n";

	for (;;)
	{
		Mat frame1;
		Mat frame2;
		Mat birlesik;

		cap1 >> frame1; // get a new frame from camera
		cap2 >> frame2; // get a new frame from camera
		
		//flip(frame1, frame1,1);
		//flip(frame2, frame2,1);
		hconcat(frame2, frame1, birlesik);
		//imshow("left", frame1);
		//imshow("right", frame2);
		imshow("camera-L-R", birlesik);

		int c = waitKey(20);

		if ((char)c == 27) //esc basilinca biter
		{
			break;
		}
		
		if ((char)c == 'a') //a basilinca gecerli frami jpg olarak kaydeder
		{
			count1++;
			fileName = "left" + to_string(count1) + ".jpg";
			imwrite(fileName, frame1);
			fileName = "right" + to_string(count1) + ".jpg";
			imwrite(fileName, frame2);
		}
		
		if ((char)c == 'z') //z basilinca tüm framleri jpg olarak kaydeder
		{
			continuousControl = true;
		}
		
		if ((char)c == 'x') //x basilinca framleri jpg olarak kaydetmeyi durdurur
		{
			continuousControl = false;
		}

		if (continuousControl)
		{
			recordCount++;
			sprintf(chrFileNm, "C:\\Users\\Murat\\Documents\\Visual Studio 2013\\Projects\\TwoCameraExample\\TwoCameraExample\\Frames\\Data16\\%06d_L.png", recordCount);
			fileName = chrFileNm;
			imwrite(fileName, frame1);
			sprintf(chrFileNm, "C:\\Users\\Murat\\Documents\\Visual Studio 2013\\Projects\\TwoCameraExample\\TwoCameraExample\\Frames\\Data17\\%06d_R.png", recordCount);
			fileName = chrFileNm;
			imwrite(fileName, frame2);
		}

		if ((char)c == 'v') //video kaydi v basilinca baslar
		{
			timev = NULL;
			time(&timev);
			ss.clear();
			ss << timev;
			timer = ss.str();
			videoFile1 = "C:\\Users\\Murat\\Documents\\Visual Studio 2013\\Projects\\TwoCameraExample\\TwoCameraExample\\Videos\\" + timer + "_L.avi";
			videoFile2 = "C:\\Users\\Murat\\Documents\\Visual Studio 2013\\Projects\\TwoCameraExample\\TwoCameraExample\\Videos\\" + timer + "_R.avi";

			videoRC1 = cv::VideoWriter(videoFile1, CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height), true);
			videoRC2 = cv::VideoWriter(videoFile2, CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height), true);
			videoControl = true;
		}

		if ((char)c == 'n') //video kaydi n basilinca biter
		{
			
			timer = "";
			videoControl = false;
		}

		if (videoControl) //esc basilinca biter
		{
			videoRC1.write(frame1);  // kameradan alinan goruntuyu kaydeder.	
			videoRC2.write(frame2);  // kameradan alinan goruntuyu kaydeder.	
		}
		
		
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}