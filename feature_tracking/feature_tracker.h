/*
    Aug. 6 2018, He Zhang, hxzhang8@vcu.edu
    
    harris tracker 

*/

#pragma once

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "camodocal/camera_models/PinholeCamera.h"
// #include "pointDefinition.h"
#include <vector>

using namespace std; 

struct ImagePoint {
     float u, v;
     int ind;
};

class CTrackerParam
{
    public:
	CTrackerParam(); 
	~CTrackerParam(); 
	
	void defaultInit(); 

	// subregion 
	int mXSubregionNum;
	int mYSubregionNum; 
	int mTotalSubregionNum; // mXSubregionNum * mYSubregionNum
	double mSubregionWidth; 
	double mSubregionHeight; 

	// features in subregion
	int mMaxFeatureNumInSubregion; 
	int mMaxFeatureNum; // mMaxFeatureNumInSubregion * mTotalSubregionNum

	// boundary 
	int mXBoundary; 
	int mYBoundary; 
	
	// showing 
	int mShowSkipNum; 
	int mShowDSRate; 

	// track condition
	double mMaxTrackDis; 
	int mTrackWinSize; 

	// equalize? 
	bool mbEqualized; 
	
	// camera param 
	double mfx, mfy, mcx, mcy; 
	double mk[2];
	double mp[2]; 
	int mWidth; 
	int mHeight; 
};

class CFeatureTracker
{
    public:
	CFeatureTracker(CTrackerParam);
	virtual ~CFeatureTracker(); 
	
	void init(); 
	void handleImage(const cv::Mat &img, double img_time); 

	bool inBoard(cv::Point2f& pt); 

	// whether initialized
	bool mbInited; 
	
	// camera images 
	cv::Mat mPreImg; 
	cv::Mat mCurImg; 
	
	// show
	int mShowCnt; 
	cv::Mat mImgShow; 
	cv::Mat mHarrisPre; // extract harris for previous image  

	// features 
	vector<cv::Point2f> mvPrePts; 
	vector<cv::Point2f> mvCurPts; 

	vector<ImagePoint> mvPreImagePts; 
	vector<ImagePoint> mvCurImagePts; 

	vector<int> mvIds; 
	int mFeatureIdFromStart; // unique feature id? 

	// camera image timestamps 
	double mTimePre; 
	double mTimeCur; 
	
	// camera model 
	camodocal::CameraPtr mpCam; 
	
	// parameters
	CTrackerParam mParam; 

	int mPreTotalFeatureNum; // previous tracked feature number 
	vector<int> mvSubregionFeatureNum;  
};

