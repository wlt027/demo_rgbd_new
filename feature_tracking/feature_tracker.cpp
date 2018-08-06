/*
    Aug. 6 2018, He Zhang, hxzhang8@vcu.edu
    
    harris tracker 

*/

#include "feature_tracker.h"

#define SQ(x) ((x)*(x))

CTrackerParam::CTrackerParam(){ defaultInit();}
CTrackerParam::~CTrackerParam(){}

void CTrackerParam::defaultInit()
{
    mXSubregionNum = 12; 
    mYSubregionNum = 8; 
    mTotalSubregionNum = mXSubregionNum * mYSubregionNum; 
    
    mMaxFeatureNumInSubregion = 2; 
    mMaxFeatureNum = mMaxFeatureNumInSubregion * mTotalSubregionNum; 
    
    mXBoundary = 20; 
    mYBoundary = 20;
    
    mShowSkipNum = 2; 
    mShowDSRate = 2; 
    
    mMaxTrackDis = 100; 
    mTrackWinSize = 15; 
    
    mbEqualized = true; 
    
    mfx = 525; 
    mfy = 525;
    mcx = 319.5; 
    mcy = 239.5; 
    mk[0] = 0; mk[1] = 0; 
    mp[0] = 0; mp[1] = 0; 
    mWidth = 640; 
    mHeight = 480; 

    mSubregionWidth = (double)(mWidth - 2*mXBoundary) / (double)(mXSubregionNum); 
    mSubregionHeight = (double) (mHeight - 2*mYBoundary) / (double)(mYSubregionNum); 
    
}

CFeatureTracker::CFeatureTracker(CTrackerParam p): 
mParam(p)
{
    init(); 
} 

CFeatureTracker::~CFeatureTracker(){}

void CFeatureTracker::init()
{
    mPreTotalFeatureNum = 0;    
    
    int downsampled_row = mParam.mHeight/mParam.mShowDSRate; 
    int downsampled_col = mParam.mWidth/mParam.mShowDSRate; 
    mHarrisPre = cv::Mat(downsampled_row, downsampled_col, CV_32FC1); 
    mImgShow = cv::Mat(downsampled_row, downsampled_col, CV_8UC1); 

    mvSubregionFeatureNum.resize(mParam.mMaxFeatureNum, 0); 
    mFeatureIdFromStart = 0; 
    mvIds.resize(mParam.mMaxFeatureNum, 0); 
    
    mShowCnt = 0; 
}

void CFeatureTracker::handleImage(const cv::Mat& _img, double img_time)
{	
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
	return ; 
    }
    
    // extract features 
    cv::resize(mPreImg, mImgShow, mImgShow.size());
    cv::cornerHarris(mImgShow, mHarrisPre, 3, 3, 0.04); 

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
		mPreImg.copyTo(subregion_img); 
		
		// TODO: add some mask ? 
		vector<cv::Point2f> n_pts;  
		cv::goodFeaturesToTrack(mPreImg, n_pts, numToFind, 0.1, 5.0, NULL, 3, 1); 
		int numFound = 0; 
		for(auto &pt : n_pts)
		{
		    pt.x += subregionLeft; 
		    pt.y += subregionTop; 
		    
		    int xInd = (pt.x + 0.5)/mParam.mShowDSRate; 
		    int yInd = (pt.y + 0.5)/mParam.mShowDSRate; 
		    
		    if(((float*)(mHarrisPre.data + mHarrisPre.step1() * yInd))[xInd] > 1e-6)
		    {
			mvPrePts.push_back(pt);  
			mvIds.push_back(mFeatureIdFromStart);  
			numFound++; 
			mFeatureIdFromStart++; 
		    }

		}

		mPreTotalFeatureNum += numFound; 
		mvSubregionFeatureNum[ind] += numFound;
	    }
	}
    }
    
    // track the features 
    vector<uchar> status; 
    vector<float> err; 
    cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mvPrePts, mvCurPts, status, err, cv::Size(mParam.mTrackWinSize, mParam.mTrackWinSize), 3); 

    
    for(int i=0; i<mParam.mTotalSubregionNum; i++)
    {
	mvSubregionFeatureNum[i] = 0; 
    }
    
    // check which points are valid 
    assert(mPreTotalFeatureNum == mvPrePts.size()); 
    ImagePoint point; 
    int featureCount = 0; 
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
		mpCam->liftProjective(Eigen::Vector2d(mvCurPts[i].x, mvCurPts[i].y), tmp); 
		point.u = tmp(0); 
		point.v = tmp(1); 
		point.ind = mvIds[i]; 
		
		// 
		mvCurImagePts.push_back(point); 
		
		if(i >= recordFeatureNum)
		{
		    mpCam->liftProjective(Eigen::Vector2d(mvPrePts[i].x, mvPrePts[i].y), tmp);
		    point.u = tmp(0); 
		    point.v = tmp(1); 
		    mvPreImagePts.push_back(point); 
		}
		featureCount++;
		mvSubregionFeatureNum[ind]++; 
	    }
	}
    }
    mPreTotalFeatureNum = featureCount; 
    
    // publish for showing 
    mShowCnt = (mShowCnt + 1) % (mParam.mShowSkipNum + 1);     
    if(mShowCnt == mParam.mShowSkipNum)
    {
	// cv_bridge::CvImage bridge;
	// bridge.image = mImgShow; 
	// bridge.encoding = "mono8"; 
	cv::imshow("show_img", mImgShow); 
	cv::waitKey(10); 
    }
}

bool CFeatureTracker::inBoard(cv::Point2f& pt)
{
    return (pt.x > mParam.mXBoundary && pt.x < mParam.mWidth - mParam.mXBoundary && 
	pt.y > mParam.mYBoundary && pt.y < mParam.mHeight - mParam.mYBoundary); 
}

