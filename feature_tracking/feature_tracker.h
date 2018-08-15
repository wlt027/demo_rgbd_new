/*
    Aug. 6 2018, He Zhang, hxzhang8@vcu.edu
    
    harris tracker 

*/

#pragma once

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include "camodocal/camera_models/CameraFactory.h"
// #include "camodocal/camera_models/PinholeCamera.h"
// #include "pointDefinition.h"
#include <vector>

using namespace std; 

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

	// harris corner threshold 
	double mHarrisThreshold;

	// dist between features 
	double mMinDist; 
};

class CFeatureTracker
{
public:
    struct ImagePoint 
    {
	float u, v;
	int ind;
    };

    public:
	CFeatureTracker(); 
	CFeatureTracker(CTrackerParam& );
	virtual ~CFeatureTracker(); 
	
	void init(); 
	bool handleImage(const cv::Mat &img, double img_time=0.); 

	bool inBoard(cv::Point2f& pt); 
	void showPreFeatImg(); 

	void setMask(); 

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

	cv::Mat mMask; // mask to avoid duplicate features

	vector<int> mvIds; 
	int mFeatureIdFromStart; // unique feature id? 

	// camera image timestamps 
	double mTimePre; 
	double mTimeCur; 
	
	// camera model 
	// camodocal::PinholeCamera* mpCam; 
	
	// parameters
	CTrackerParam mParam; 

	int mPreTotalFeatureNum; // previous tracked feature number 
	vector<int> mvSubregionFeatureNum;  
};

