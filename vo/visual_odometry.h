/*
    Aug. 9 2018, He Zhang, hzhang8@vcu.edu 
    
    Implement Visual Odometry 

*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;


class VisualOdometry 
{

public:
    VisualOdometry(); 
    ~VisualOdometry(); 
    
    void imagePointsHandler(const sensor_msgs::PointCloud2ConstPtr& imagePoints2); 
    void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2); 

    double Fy2(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 7, 1>* J);
    double Fy3(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 7, 1>* J );
    double Fy4(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 7, 1>* J );

    double mTimeLast; // current timestamp 
    double mTimeCurr; 
    
    boost::shared_ptr<pcl::PointCloud<ImagePoint> > mImgPTCurr; 
    boost::shared_ptr<pcl::PointCloud<ImagePoint> > mImgPTLast; 
    
    double mPCTime; // point cloud time 
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > mPC; 
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI> > mKDTree; 

    double mZoomDis; // must equal to that in depth_handler 
    vector<float> * mvDptCurr; 
    vector<float> * mvDptLast; 
    
    vector<tf::Transform> mFtTrans; // record the pose when the features are first observed
    boost::shared_ptr<pcl::PointCloud<ImagePoint> > mFtObs; // record the measurement 
    tf::Transform mLastPose; // pose when mImgPTLast is observed 

    double mdisThresholdForTriangulation; // 
};	


