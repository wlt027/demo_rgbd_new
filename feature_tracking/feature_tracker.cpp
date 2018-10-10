/*
    Aug. 6 2018, He Zhang, hxzhang8@vcu.edu
    
    harris tracker 

*/

#include "feature_tracker.h"
#include "../utility/tic_toc.h"
#include <Eigen/Core>
#include <ros/ros.h>

#define SQ(x) ((x)*(x))

CTrackerParam::CTrackerParam(){ defaultInit();}
CTrackerParam::~CTrackerParam(){}

void CTrackerParam::defaultInit()
{
    mXSubregionNum = 12; // 6; // 12; 
    mYSubregionNum = 8; // 5; // 8; 
    mTotalSubregionNum = mXSubregionNum * mYSubregionNum; 
    
    mMaxFeatureNumInSubregion = 2; // 10; // 2; 
    mMaxFeatureNum = mMaxFeatureNumInSubregion * mTotalSubregionNum; 
    
    mXBoundary = 20; 
    mYBoundary = 20;
    
    mShowSkipNum = 2; // 2; // 2; 
    mShowDSRate = 2; 
    mbShowTrackedResult = false; // true; // whether to show feature track result 

    mMaxTrackDis = 100; 
    mTrackWinSize = 15; 
    
    mbEqualized = false; // true; //false; 
    
    mfx = 617.306; // 525.; // 617.306; // 525; 
    mfy = 617.714; // 525.; // 617.714; // 525;
    mcx = 326.245; // 319.5; // 326.245; // 319.5; 
    mcy = 239.5; // 239.974; // 239.5; 
    mk[0] = 0; mk[1] = 0; 
    mp[0] = 0; mp[1] = 0; 
    mWidth = 640; 
    mHeight = 480; 

    mSubregionWidth = (double)(mWidth - 2*mXBoundary) / (double)(mXSubregionNum); 
    mSubregionHeight = (double) (mHeight - 2*mYBoundary) / (double)(mYSubregionNum); 
    
    mHarrisThreshold = 1e-8; // 1e-6; // 1e-6;    
    mMinDist = 20.0;
}

CFeatureTracker::CFeatureTracker()
{
    mParam = CTrackerParam(); 
    init(); 
}

CFeatureTracker::CFeatureTracker(CTrackerParam& p): 
mParam(p)
{
    init(); 
} 

CFeatureTracker::~CFeatureTracker(){}

void CFeatureTracker::init()
{
    mbInited = false; 

    // detect features and show img
    int downsampled_row = mParam.mHeight/mParam.mShowDSRate; 
    int downsampled_col = mParam.mWidth/mParam.mShowDSRate; 
    mHarrisPre = cv::Mat(downsampled_row, downsampled_col, CV_32FC1); 
    mImgShow = cv::Mat(downsampled_row, downsampled_col, CV_8UC1); 
    mShowCnt = 0; 

    // tracking, e.g. ids
    mPreTotalFeatureNum = 0;    
    mvSubregionFeatureNum.resize(mParam.mMaxFeatureNum, 0); 
    mFeatureIdFromStart = 0; 
    mvIds.resize(mParam.mMaxFeatureNum, 0); 

    // camera parameters 
//    camodocal::PinholeCamera::Parameters camparam;
//    camparam.fx() = mParam.mfx; camparam.fy() = mParam.mfy; 
//    camparam.cx() = mParam.mcx; camparam.cy() = mParam.mcy; 
//    camparam.k1() = mParam.mk[0]; camparam.k2() = mParam.mk[1]; 
//    camparam.p1() = mParam.mp[0]; camparam.p2() = mParam.mp[1];
//
    // string camname("rs435");
    // mpCam = new camodocal::PinholeCamera; // camodocal::CameraFactory::instance()->generateCamera(camodocal::Camera::PINHOLE, camname, cv::Size(mParam.mWidth, mParam.mHeight));
    // mpCam->setParameters(camparam); 
}

bool CFeatureTracker::handleImage(const cv::Mat& _img, double img_time)
{	
    TicToc t_ft; 

    // set time
    mTimePre = mTimeCur;
    mTimeCur = img_time; 
    
    // preprocess img 
    cv::Mat img; 
    if(mParam.mbEqualized)
    {
	static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8)); 
	clahe->apply(_img, img); 
    }else{
	img = _img.clone(); 
    }
    mPreImg = mCurImg; 
    mCurImg = img; 
    
    mvPrePts.swap(mvCurPts); 
    mvCurPts.clear();

    mvPreImagePts.swap(mvCurImagePts); 
    mvCurImagePts.clear(); 
    
    if(!mbInited) 
    {
	mbInited = true; 
	return false; 
    }

    // 
    // cv::imshow("pre img", mPreImg); 
    // cv::waitKey(0); 

    // extract features 
    cv::resize(mPreImg, mImgShow, mImgShow.size());
    // cv::imshow("show img", mImgShow); 
    // cv::waitKey(0); 

    cv::cornerHarris(mImgShow, mHarrisPre, 3, 3, 0.04); 
    // cv::imshow("harris img", mHarrisPre); 
    // cv::waitKey(0); 

    // setMask 
    setMask(); 

    int recordFeatureNum = mPreTotalFeatureNum;  
    assert(mPreTotalFeatureNum == mvPrePts.size()); 
    for(int i=0; i<mParam.mYSubregionNum; i++)
    {
	for(int j=0; j<mParam.mXSubregionNum; j++)
	{
	    int ind = i * mParam.mXSubregionNum + j; 
	    int numToFind = mParam.mMaxFeatureNumInSubregion - mvSubregionFeatureNum[ind];
	    if(numToFind > 0)
	    {
		int subregionLeft = mParam.mXBoundary + (int)(mParam.mSubregionWidth * j);
		int subregionTop = mParam.mYBoundary + (int)(mParam.mSubregionHeight * i); 
		cv::Rect subregion_rect(subregionLeft, subregionTop, (int)(mParam.mSubregionWidth+0.5), (int)(mParam.mSubregionHeight+0.5)); 
		cv::Mat subregion_img = mPreImg(subregion_rect);
		// mPreImg.copyTo(subregion_img); 
		
		// TODO: add some mask ? 
		vector<cv::Point2f> n_pts;  
		// cv::imshow("subregion_img", subregion_img); 
		// cv::waitKey(0); 
		// cv::Mat mask = cv::Mat(subregion_img.rows, subregion_img.cols, CV_8UC1, cv::Scalar(255));
		cv::Mat mask = mMask(subregion_rect); 

		cv::goodFeaturesToTrack(subregion_img, n_pts, numToFind, 0.1, mParam.mMinDist, mask, 3, 1); 
		// cout <<"feature_tracker.cpp: detect "<<n_pts.size()<<" potential features"<<endl; 
		int numFound = 0; 
		for(auto &pt : n_pts)
		{
		    pt.x += subregionLeft; 
		    pt.y += subregionTop; 
		    
		    int xInd = (pt.x + 0.5)/mParam.mShowDSRate; 
		    int yInd = (pt.y + 0.5)/mParam.mShowDSRate; 
		    
		    if(((float*)(mHarrisPre.data + mHarrisPre.step1() * yInd))[xInd] > mParam.mHarrisThreshold)
		    {
			mvPrePts.push_back(pt);  
			// mvIds.push_back(mFeatureIdFromStart);  
			mvIds[mPreTotalFeatureNum + numFound] = mFeatureIdFromStart;
			numFound++; 
			mFeatureIdFromStart++; 
		    }
		}
		mPreTotalFeatureNum += numFound; 
		mvSubregionFeatureNum[ind] += numFound;
	    }
	}
    }
  
    for(int i=0; i<mParam.mTotalSubregionNum; i++)
    {
	mvSubregionFeatureNum[i] = 0; 
    }
    int featureCount = 0; 

    // track the features 
    vector<uchar> status; 
    vector<float> err; 
    if(mvPrePts.size() > 0)
    {
	cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mvPrePts, mvCurPts, status, err, cv::Size(mParam.mTrackWinSize, mParam.mTrackWinSize), 3); 

	// check which points are valid 
	assert(mPreTotalFeatureNum == mvPrePts.size()); 
	ImagePoint point; 

	// mvPrePts only contains the points matched with mCurImg 
	// while mvPreImagePts contains the points matched with mCurImg and with previous Image 
	for(int i=0; i<mPreTotalFeatureNum; i++)
	{
	    if(!status[i] || !inBoard(mvPrePts[i]))
		continue; 
	    double trackDis = sqrt(SQ(mvCurPts[i].x - mvPrePts[i].x) + SQ(mvCurPts[i].y - mvPrePts[i].y)); 

	    if(trackDis <= mParam.mMaxTrackDis)
	    {
		int xInd = (int)((mvPrePts[i].x - mParam.mXBoundary + 0.5)/mParam.mSubregionWidth); 
		int yInd = (int)((mvPrePts[i].y - mParam.mYBoundary + 0.5)/mParam.mSubregionHeight); 

		int ind = yInd * mParam.mXSubregionNum + xInd; 

		if(mvSubregionFeatureNum[ind] < mParam.mMaxFeatureNumInSubregion)
		{
		    // this is a tracked pt in both pre and cur images
		    mvCurPts[featureCount] = mvCurPts[i];
		    mvPrePts[featureCount] = mvPrePts[i]; 
		    mvIds[featureCount] = mvIds[i]; 

		    Eigen::Vector3d tmp; 
		    // mpCam->liftProjective(Eigen::Vector2d(mvCurPts[i].x, mvCurPts[i].y), tmp); 
		    tmp(0) = (mvCurPts[i].x - mParam.mcx)/mParam.mfx; 
		    tmp(1) = (mvCurPts[i].y - mParam.mcy)/mParam.mfy; 
		    point.u = tmp(0); 
		    point.v = tmp(1); 
		    point.ind = mvIds[i]; 

		    // 
		    mvCurImagePts.push_back(point); 

		    if(i >= recordFeatureNum) // new detected feature in preImg
		    {
			// mpCam->liftProjective(Eigen::Vector2d(mvPrePts[i].x, mvPrePts[i].y), tmp);
			// cout <<"mvPrePts: "<<mvPrePts[i].x<<" "<< mvPrePts[i].y<<" output: "<<tmp(0)<<" "<<tmp(1)<<endl;
			tmp(0) = (mvPrePts[i].x - mParam.mcx)/mParam.mfx; 
			tmp(1) = (mvPrePts[i].y - mParam.mcy)/mParam.mfy; 

			point.u = tmp(0); 
			point.v = tmp(1); 
			mvPreImagePts.push_back(point); 
		    }
		    featureCount++;
		    mvSubregionFeatureNum[ind]++; 
		}
	    }
	}
    }
    ROS_DEBUG_STREAM("feature_tracker.cpp: previous Img tracked "<<featureCount<<" features!");
    ROS_DEBUG_STREAM("feature_tracker.cpp: previous Img publish "<<mvPreImagePts.size()<<" features!"); 
    // for the next loop 
    mPreTotalFeatureNum = featureCount; 
    mvCurPts.resize(featureCount); 

    static int cnt = 0; 
    ROS_DEBUG("feature_tracker num %d at time %lf cost: %f", ++cnt, mTimePre, t_ft.toc()); 

    // publish for showing 
    mShowCnt = (mShowCnt + 1) % (mParam.mShowSkipNum + 1);     
    mbSendImgForShow = (mShowCnt == mParam.mShowSkipNum);
    if(mbSendImgForShow)
    {
	// cv_bridge::CvImage bridge;
	// bridge.image = mImgShow; 
	// bridge.encoding = "mono8"; 
	// cv::imshow("show_img", mImgShow); 
	// cv::waitKey(10); 
	
	if(mParam.mbShowTrackedResult)
	    showPreFeatImg(); 
    }
    return true;
}

void CFeatureTracker::showPreFeatImg()
{
    cv::Mat show_img; 
    cv::cvtColor(mImgShow, show_img, CV_GRAY2RGB); 
    for(int i=0; i<mvPrePts.size(); i++)
    {
	cv::Point2f pt; 
	pt.x = (mvPrePts[i].x+0.5)/mParam.mShowDSRate; 
	pt.y = (mvPrePts[i].y+0.5)/mParam.mShowDSRate; 
	cv::circle(show_img, pt, 2, cv::Scalar(0, 0, 255)); 
    }
    cv::imshow("tracked_features", show_img); 
    cv::waitKey(10); 
}

void CFeatureTracker::setMask()
{
    mMask = cv::Mat(mParam.mHeight, mParam.mWidth, CV_8UC1, cv::Scalar(255)); 
    for(int i=0; i<mvPrePts.size(); i++)
    {
	cv::circle(mMask, mvPrePts[i], mParam.mMinDist, 0, -1); 
    }
}

bool CFeatureTracker::inBoard(cv::Point2f& pt)
{
    return (pt.x > mParam.mXBoundary && pt.x < mParam.mWidth - mParam.mXBoundary && 
	pt.y > mParam.mYBoundary && pt.y < mParam.mHeight - mParam.mYBoundary); 
}

