/*
    Aug. 21 2018, He Zhang, hzhang8@vcu.edu 
    
    vio: tightly couple features [no depth, triangulated, measured] 
    with IMU integration 

*/

#include "vio.h"
#include "../vo/stereo_triangulate.h"
#include "../utility/utility.h"
#include "opencv/cv.h"
#include <iostream>
#include <string>
#include "../utility/tic_toc.h"
#include "projection_quat.h"
#include "plane.h"

using namespace QUATERNION_VIO; 

namespace{

    void printTF(tf::Transform& T, string name="")
    {
	tf::Quaternion q = T.getRotation(); 
	tf::Vector3 t = T.getOrigin(); 
	cout <<name<<" "<<t.getX()<<" "<<t.getY()<<" "<<t.getZ()<<" "<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<endl; 
    }
}

VIO::VIO():
mTimeLast(0),
mTimeCurr(0),
mImgPTLast(new pcl::PointCloud<ImagePoint>),
mImgPTCurr(new pcl::PointCloud<ImagePoint>),
mPCTime(0),
mPC(new pcl::PointCloud<pcl::PointXYZI>),
mKDTree(new pcl::KdTreeFLANN<pcl::PointXYZI>),
mZoomDis(10.),
mdisThresholdForTriangulation(1.), // 1.
mFtObsCurr(new pcl::PointCloud<ImagePoint>),
mFtObsLast(new pcl::PointCloud<ImagePoint>),
mImagePointsProj(new pcl::PointCloud<pcl::PointXYZ>),
mPCNoFloor(new pcl::PointCloud<pcl::PointXYZI>),
mPCFloor(new pcl::PointCloud<pcl::PointXYZ>),
mCurrPCFloor(new pcl::PointCloud<pcl::PointXYZ>),
mbFirstIMU(true),
mbFirstFloorObserved(false),
mbInited(false),
frame_count(0),
mFloorZ(NOT_INITIED),
mFloorRange(0.15)
{
    clearState();
}
VIO::~VIO(){}

void VIO::clearState()
{
    for(int i=0; i<WN + 1; i++)
    {
	linear_acceleration_buf[i].clear(); 
	angular_velocity_buf[i].clear(); 
	if(pre_integrations[i] != NULL)
	    delete pre_integrations[i]; 
	pre_integrations[i] = NULL; 
    }
    
    R_imu = Eigen::Matrix3d::Identity(); 

 //    if(tmp_pre_integration != NULL)
	// delete tmp_pre_integration; 
 //    tmp_pre_integration = NULL; 
}

void VIO::processIMU(double dt, Vector3d & linear_acceleration, Vector3d& angular_velocity)
{
    if(mbFirstIMU)
    {
	mbFirstIMU = false; 
	acc_0 = linear_acceleration; 
	gyr_0 = angular_velocity; 
    }

    if(!pre_integrations[frame_count])
    {
	pre_integrations[frame_count] = new IntegrationBase(acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]); 
    }
    if(frame_count != 0)
    {
    // cout<<"vio.cpp: processIMU dt = "<<dt<<" linear_acceleration: "<<linear_acceleration.transpose()<<" angular_velocity: "<<angular_velocity.transpose()<<endl;
	pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity); 

	// tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity); 
	// dt_buf[frame_count].push_back(dt); 
	// linear_acceleration_buf[frame_count].push_back(linear_acceleration);
	// angular_velocity_buf[frame_count].push_back(angular_velocity);

	int j = frame_count; 
	Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - mg; 
	Vector3d un_gyr = 0.5 *(gyr_0 + angular_velocity) - Bgs[j]; 
	Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); 
	R_imu *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); 
	Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - mg;
	Vector3d un_acc = 0.5 *(un_acc_0 + un_acc_1); 
	Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc; 
	Vs[j] += dt * un_acc; 
    }else // save for initialization 
    {
	linear_acceleration_buf[frame_count].push_back(linear_acceleration);
	angular_velocity_buf[frame_count].push_back(angular_velocity); 
    }
    acc_0 = linear_acceleration; 
    gyr_0 = angular_velocity; 
}

void VIO::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
	tic[i] = TIC[i];
	ric[i] = RIC[i];
    }
    Eigen::Quaterniond q(ric[0]); 
    mTIC = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(tic[0][0], tic[0][1], tic[0][2])); 
    printTF(mTIC, "vio.cpp: initial mTIC: ");
    // f_manager.setRic(ric);
}

void VIO::processCurrDepthCloud(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    double pctime = depthCloud2->header.stamp.toSec(); 

    // mPC->clear(); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::fromROSMsg(*depthCloud2, *tmpPC); 
    m_curr_pc_buf.lock(); 
    curr_pctime_buf.push(pctime);
    curr_pc_buf.push(tmpPC);
    m_curr_pc_buf.unlock();
}

void VIO::processDepthCloud(sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    // mPCTime =     
    double pctime = depthCloud2->header.stamp.toSec(); 

    // mPC->clear(); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZI>); 

    pcl::fromROSMsg(*depthCloud2, *tmpPC); 
    int depthCloudNum = tmpPC->points.size(); 
    // cout <<"vo receive "<<std::fixed<< mPCTime<<" dpt has: "<<depthCloudNum<<" points!"<<endl;
    if(depthCloudNum > 20)
    {
	for(int i=0; i<depthCloudNum; i++)
	{
	    tmpPC->points[i].x = tmpPC->points[i].x * mZoomDis / tmpPC->points[i].z;
	    tmpPC->points[i].y = tmpPC->points[i].y * mZoomDis / tmpPC->points[i].z;
	    tmpPC->points[i].intensity = tmpPC->points[i].z; 
	    tmpPC->points[i].z = mZoomDis;
	} 
	// mKDTree->setInputCloud(mPC); 
    }
    m_pc_buf.lock(); 
	pctime_buf.push(pctime); 
	pc_buf.push(tmpPC);
    m_pc_buf.unlock(); 
}

void VIO::processImage(sensor_msgs::PointCloud2ConstPtr& imagePoints2)
{
    // handle last imgpt and currpt
    mTimeLast = mTimeCurr; 
    mTimeCurr = imagePoints2->header.stamp.toSec(); 
    
    mImgPTLast.swap(mImgPTCurr); 
    mImgPTCurr->clear(); 
    Headers[frame_count] = imagePoints2->header;
    
    pcl::fromROSMsg(*imagePoints2, *mImgPTCurr); 
    
    vector<ip_M> vip; 
    if(!mbInited)
    {
	// initialization 
	initialize(); 
	++frame_count;
    }else{
	// associate features 
	associateFeatures(vip); 

	// reject by fundamental matrix
	rejectByF(vip); 

	// remove close triangulated point
	removeWrongTri(vip); 

	// solve odometry 
	solveOdometry(vip); 

	if(!mbStill)
	{
	    // remove outliers
	    removeOutliers(vip);
	    // solve it angin 
	    solveOdometry(vip, floor_detected()); 
	}
	// slide for next loop 
	slideWindow(); 
    }

    // for next loop
    prepareNextLoop(); 

    // for display 
    prepareForDisplay(vip);

    return ; 
}

bool VIO::floor_detected()
{
    // retrieve current point cloud 
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZI>);
    m_curr_pc_buf.lock(); 
    while(!curr_pctime_buf.empty())
    {
        double tmp = curr_pctime_buf.front(); 
        if(tmp < mTimeCurr)
        {
            curr_pctime_buf.pop(); 
            curr_pc_buf.pop(); 
        }
        else if(tmp == mTimeCurr)
        {
            ROS_DEBUG("vio.cpp: succeed set current pointcloud at t = %lf", mTimeCurr);
            tmp_pc = curr_pc_buf.front();
            curr_pctime_buf.pop(); 
            curr_pc_buf.pop(); 
            break;
        }else{ // tmp > t
            ROS_WARN("vio.cpp: no current point cloud available for t= %lf", mTimeCurr);
            break; 
        }
    }
    m_curr_pc_buf.unlock(); 

    if(tmp_pc->points.size() < 100)
        return false; 
    if(mbFirstFloorObserved == false) // first floor has not been observed
        return false; 

    // transform point cloud into world coordinate system 
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc2(new pcl::PointCloud<pcl::PointXYZ>); 
    tmp_pc2->points.reserve(tmp_pc->points.size()); 
    for(int i=0; i<tmp_pc->points.size(); i++)
    {
        pcl::PointXYZI& pt = tmp_pc->points[i]; 
        tf::Vector3 pj(pt.x, pt.y, pt.z); 
        tf::Vector3 pi = mCurrPose * pj; 
        pcl::PointXYZ ptw(pi.getX(), pi.getY(), pi.getZ());
        if(ptw.z < mFloorZ + mFloorRange && pt.z > mFloorZ - mFloorRange)
        {
            tmp_pc2->points.push_back(ptw); 
        }
    }

    if(tmp_pc2->points.size() < 200) // not a reliable observation
    {
        return false; 
    }

    // extract floor 
    Eigen::Vector3d nv; 
    double nd; 
    pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
    ((Plane*)(0))->computeByPCL<pcl::PointXYZ>(tmp_pc2, indices, nv, nd); 
    Eigen::Vector3d g(Pls[0][0], Pls[0][1], Pls[0][2]); 
    double angle = nv.dot(g); 
    const double COS30 = cos(30.*M_PI/180.);
    // cout<<"vio.cpp: Floor plane in current PC has "<<indices->indices.size()<<" points nv = "<<nv.transpose()<<endl;
    if(indices->indices.size() < 200 || angle < COS30) // NO Floor plane is detected 
    {
        return false; 
    }
    
    // save current floor points for debug 
    // mCurrPCFloor->clear();
    // mCurrPCFloor->points.reserve(indices->indices.size());
    // for(int i=0; i<indices->indices.size(); i++)
    // {
    //     mCurrPCFloor->points.push_back(tmp_pc2->points[indices->indices[i]]);
    // }
    // mCurrPCFloor->width = mCurrPCFloor->points.size(); 
    // mCurrPCFloor->height = 1;
    // mCurrPCFloor->is_dense = true;

    // reset current plane observation, within world frame,
    // now transfer it into current IMU frame 
    {
	tf::Quaternion tq = mCurrIMUPose.getRotation(); 
	tf::Vector3 tt = mCurrIMUPose.getOrigin(); 
	
	Eigen::Quaterniond q(tq.getW(), tq.getX(), tq.getY(), tq.getZ()); 
	Eigen::Vector3d t(tt.getX(), tt.getY(), tt.getZ()); 
	
	Eigen::Vector3d nl = q.inverse() * nv; 
	double dl = nv.dot(t) + nd; 
	nv = nl; 
	nd = dl; 
    }

    Pls[WN][0] = nv(0); 
    Pls[WN][1] = nv(1); 
    Pls[WN][2] = nv(2); 
    Pls[WN][3] = nd;
    return true;
}

void VIO::removeWrongTri(vector<ip_M>& ipRelations)
{
    vector<ip_M>& tmp = ipRelations; 
    for(int i=0; i<tmp.size(); i++)
    {
        ip_M& m = tmp[i];
        if(m.v == ip_M::DEPTH_TRI)
        {
            if(m.s < 3.1) // since the depth of points < 3.1 shoud be directly measured
                m.v = ip_M::INVALID;
        }
    }
}

void VIO::rejectByF(vector<ip_M>& ipRelations)
{
    TicToc t_f; 
    vector<ip_M>& tmp = ipRelations; 
    if(tmp.size() == 0) return; 
    vector<cv::Point2f> pre_pts(tmp.size()); 
    vector<cv::Point2f> cur_pts(tmp.size()); 
    for(int i=0; i<tmp.size();i++)
    {
        ip_M& m = tmp[i];
        pre_pts[i] = cv::Point2f(m.ui * FOCAL_LENGTH + CX, m.vi * FOCAL_LENGTH + CY); 
        cur_pts[i] = cv::Point2f(m.uj * FOCAL_LENGTH + CX, m.vj * FOCAL_LENGTH + CY); 
    }
    vector<uchar> status; 
    int cnt_invalid = 0;
    cv::findFundamentalMat(pre_pts, cur_pts, cv::FM_RANSAC, PIX_SIGMA, 0.99, status);
    for(int i=0; i<tmp.size(); i++)
    {
        if(!status[i])
        {
            ++cnt_invalid;
            tmp[i].v = ip_M::INVALID;
        }
    }
    ROS_DEBUG("rejectF cost %lf remove %d outlier out of %d matches ratio: %f", t_f.toc(), cnt_invalid, tmp.size(), 1.*cnt_invalid/tmp.size());
}

void VIO::removeOutliers(vector<ip_M>& ipRelations)
{
    vector<ip_M>& tmp = ipRelations; 

    double lambda = 1./PIX_SIGMA;
    Eigen::Matrix2d Info_M = lambda * lambda * Eigen::Matrix2d::Identity();
    tf::Transform Tji = mCurrPose.inverse() * mLastPose;
    tf::Transform Tij = Tji.inverse(); 
    tf::Quaternion qji = Tji.getRotation();
    tf::Vector3 tij = Tij.getOrigin(); 
    Eigen::Matrix3d R = Eigen::Quaterniond(qji.getW(), qji.getX(), qji.getY(), qji.getZ()).toRotationMatrix(); 
    Eigen::Vector3d t(tij.getX(), tij.getY(), tij.getZ()); 

    int cnt_outlier = 0;
    int cnt_depth = 0;
    int cnt_no_depth = 0; 
    int cnt_outlier_no_d = 0;
    for(int i=0; i<tmp.size(); i++)
    {
        ip_M& m = tmp[i]; 
        if(m.v == ip_M::DEPTH_MES || m.v == ip_M::DEPTH_TRI)
        {
            ++cnt_depth;

            tf::Vector3 pi(m.ui*m.s, m.vi*m.s, m.s); 
            tf::Vector3 pj = Tji*pi; 
            if(pj.getZ() <= 0.3) 
            {
                m.v = ip_M::INVALID; 
                ++cnt_outlier;
                continue;
            }

            if(m.v == ip_M::DEPTH_TRI)
            {
                if(pj.getZ() <= 3.1)
                {
                    m.v = ip_M::INVALID;
                    ++cnt_outlier;
                    continue; 
                }                
            }

            Eigen::Vector2d pix_j(m.uj * FOCAL_LENGTH + CX, m.vj * FOCAL_LENGTH + CY); 
            Eigen::Vector2d pix_j_pro(pj.getX()/pj.getZ(), pj.getY()/pj.getZ());
            pix_j_pro(0) = pix_j_pro(0) * FOCAL_LENGTH + CX; 
            pix_j_pro(1) = pix_j_pro(1) * FOCAL_LENGTH + CY; 
            Eigen::Vector2d diff_pix = pix_j - pix_j_pro;
            double chi2 = diff_pix.transpose() * Info_M * diff_pix;
            if(chi2 > 5.991)
            {
                m.v = ip_M::INVALID;
                ++cnt_outlier;
            }
        }else if(m.v == ip_M::NO_DEPTH)
        {
            ++cnt_no_depth;
            // Essential matrix test
            // Eigen::Matrix3d E = R * Utility::skewSymmetric(t); 
            // Eigen::Vector3d xj(m.uj, m.vj, 1.);
            // Eigen::Vector3d xi(m.ui, m.vi, 1.); 

            // double err_dis = xj.transpose()*E*xi; 
            // // cout <<"err_dis: "<<err_dis<<endl;
            // if(err_dis > PIX_SIGMA)
            // {
            //     m.v = ip_M::INVALID;
            //     ++cnt_outlier_no_d;
            // }

            // Fundamental matrix check 

        }

    }

    // ipRelations.swap(tmp);
    ROS_DEBUG("vio.cpp: total %d features, depth %d outlier %d outlier ratio = %f", tmp.size(), cnt_depth, cnt_outlier, cnt_outlier/(float)cnt_depth);
    ROS_DEBUG("vio.cpp: no_depth %d , outlier %d outlier ratio = %f", cnt_no_depth, cnt_outlier_no_d, cnt_outlier_no_d/(float)cnt_no_depth);
}

void VIO::prepareForDisplay(vector<ip_M>& ipRelations)
{
    // publish msg done by the node 
    mImagePointsProj->points.clear(); 
    for(int i=0; i<ipRelations.size(); i++)
    {
	ip_M ipr = ipRelations[i]; 
	if(ipr.v == ip_M::DEPTH_MES || ipr.v == ip_M::DEPTH_TRI)
	{
	    pcl::PointXYZ pt; 
	    pt.x = ipr.ui * ipr.s; 
	    pt.y = ipr.vi * ipr.s; 
	    pt.z = ipr.s; 
	    mImagePointsProj->points.push_back(pt); 
	}
    }
    {
	// transform to world coordinate 
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	tmp->points.resize(mPC->points.size()); 
	for(int i=0; i<mPC->points.size(); i++)
	{
	    pcl::PointXYZI& pt = mPC->points[i]; 
	    tf::Vector3 pj(pt.x/mZoomDis*pt.intensity, pt.y/mZoomDis*pt.intensity, pt.intensity); 
	    tf::Vector3 pi = mLastPose * pj; 
	    pcl::PointXYZ ptw(pi.getX(), pi.getY(), pi.getZ());
	    tmp->points[i] = ptw; 
	}
	// remove floor 
	removeFloorPts(tmp, mPCNoFloor); 
    }

    // save this for displaying 
    mPtRelations.swap(ipRelations); 
}

void VIO::slideWindow()
{
    Headers[0] = Headers[1]; 
    Ps[0] = Ps[1]; 
    Vs[0] = Vs[1]; 
    Rs[0] = Rs[1];
    Bas[0] = Bas[1];
    Bgs[0] = Bgs[1]; 
    // dt_buf[0].swap(dt_buf[1]); 
    // linear_acceleration_buf[0].swap(linear_acceleration_buf[1]); 
    // angular_velocity_buf[0].swap(angular_velocity_buf[1]); 
    // dt_buf[1].clear(); 
    // linear_acceleration_buf[1].clear(); 
    // angular_velocity_buf[1].clear(); 
    std::swap(pre_integrations[0], pre_integrations[1]);
    delete pre_integrations[1]; 
    pre_integrations[1] = new IntegrationBase{acc_0, gyr_0, Bas[1], Bgs[1]};
    
}

void VIO::solveOdometry(vector<ip_M>& vip, bool use_floor_plane)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);    
    
    // add pose 
    for(int i=0; i<= WN; i++)
    {
	ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
	problem.AddParameterBlock(para_Pose[i], 7, local_param); 
	problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    // fix the first pose 
    problem.SetParameterBlockConstant(para_Pose[0]);

    {
	ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
	problem.AddParameterBlock(para_Ex_Pose[0], 7, local_param); 
	// if not optimize [ric, tic]
    if(ESTIMATE_EXTRINSIC == 0)
	   problem.SetParameterBlockConstant(para_Ex_Pose[0]); 
    }
    
    if(mbStill)
    {
	Ps[WN] = Ps[WN-1]; 
	Vs[WN] = Vs[WN-1];
	Rs[WN] = Rs[WN-1]; 
    }

    priorOptimize(vip); 

    // add imu factor 
    for (int i = 0; i < WN; i++)
    {
	int j = i + 1;
	if (pre_integrations[j]->sum_dt > 10.0)
	    continue;
	IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
    // cout<<"IMU factor noise: "<<endl<<imu_factor->pre_integration->noise<<endl;
    // cout<<"IMU factor jacobian: "<<endl<<imu_factor->pre_integration->jacobian<<endl;
    // cout<<"IMU factor covariance: "<<endl<<imu_factor->pre_integration->covariance<<endl;
	problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    if(!mbStill) // motion
    {
	// add plane factor 
	if(use_floor_plane)
	{
	    // cout<<"vio.cpp: add plane_factor with plane_g: "<<Pls[0].transpose()<<" plane_l: "<<Pls[1].transpose()<<endl;
	    PlaneFactor_P1 * plane_factor = new PlaneFactor_P1(Pls[0], Pls[1]); 
	    problem.AddResidualBlock(plane_factor, NULL, para_Pose[1]);
	}

	// add feature factor 
	const float INIT_DIS = 10; 
	int N = vip.size(); 
	for(int i=0; i<N; i++)
	{
	    ip_M& pt = vip[i]; 
	    Eigen::Vector3d p1(pt.ui, pt.vi, 1.); 
	    Eigen::Vector3d p2(pt.uj, pt.vj, 1.); 
	    if(pt.v == ip_M::NO_DEPTH)
	    {
		para_Feature[i][0] = 1./INIT_DIS; 
		ProjectionFactor_Y2 * f= new ProjectionFactor_Y2(p1, p2); 
		problem.AddResidualBlock(f, loss_function, para_Pose[0], para_Pose[1], para_Ex_Pose[0], para_Feature[i]); 
	    }else if(pt.v == ip_M::DEPTH_MES || pt.v == ip_M::DEPTH_TRI)
	    {
		para_Feature[i][0] = 1./pt.s; 
		ProjectionFactor * f = new ProjectionFactor(p1, p2); 
		f->sqrt_info = 240 * Eigen::Matrix2d::Identity(); 
		problem.AddResidualBlock(f, loss_function, para_Pose[0], para_Pose[1], para_Ex_Pose[0], para_Feature[i]);
		if(pt.v == ip_M::DEPTH_MES)
		    problem.SetParameterBlockConstant(para_Feature[i]);
	    }	
	}   
    }else // camera kept still, no motion 
    {
	// fix the last pose 
	problem.SetParameterBlockConstant(para_Pose[WN]);
    }
    // optimize it 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = SOLVER_TIME; 
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());
    
    afterOptimize(vip); 

    return ; 
}

void VIO::afterOptimize(vector<ip_M>& vip)
{
     // handle pose 
    for(int i=0; i<=WN; i++)
    {
	Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
	Ps[i] = Vector3d(para_Pose[i][0],
		para_Pose[i][1] ,
		para_Pose[i][2] ) ;
	Vs[i] = Vector3d(para_SpeedBias[i][0],
		para_SpeedBias[i][1],
		para_SpeedBias[i][2]);

	Bas[i] = Vector3d(para_SpeedBias[i][3],
		para_SpeedBias[i][4],
		para_SpeedBias[i][5]);

	Bgs[i] = Vector3d(para_SpeedBias[i][6],
		para_SpeedBias[i][7],
		para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
	tic[i] = Vector3d(para_Ex_Pose[i][0],
		para_Ex_Pose[i][1],
		para_Ex_Pose[i][2]);
	ric[i] = Quaterniond(para_Ex_Pose[i][6],
		para_Ex_Pose[i][3],
		para_Ex_Pose[i][4],
		para_Ex_Pose[i][5]).toRotationMatrix();
    }
    Eigen::Quaterniond q(Rs[WN]);
    mCurrIMUPose = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(Ps[WN][0], Ps[WN][1], Ps[WN][2]));

    q = Eigen::Quaterniond(ric[0]); 
    mTIC = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(tic[0][0], tic[0][1], tic[0][2]));

    mCurrPose = mCurrIMUPose * mTIC; 
    for(int i=0; i<vip.size(); i++)
    {
	ip_M& m = vip[i];
	if(m.v == ip_M::DEPTH_TRI)
	{
	    m.s = 1./para_Feature[i][0];
	}
    }
    return ; 
}

void VIO::priorOptimize(vector<ip_M>& vip)
{
    // handle pose 
    for(int i=0; i<=WN; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
	para_Ex_Pose[i][0] = tic[i].x();
	para_Ex_Pose[i][1] = tic[i].y();
	para_Ex_Pose[i][2] = tic[i].z();
	Quaterniond q{ric[i]};
	para_Ex_Pose[i][3] = q.x();
	para_Ex_Pose[i][4] = q.y();
	para_Ex_Pose[i][5] = q.z();
	para_Ex_Pose[i][6] = q.w();
    }
    
    for(int i=0; i<vip.size() && i< NUM_OF_FEAT; i++)
    {
	ip_M& m = vip[i]; 
	para_Feature[i][0] = 1./m.s; 
    }
}

Eigen::Quaterniond VIO::rotateToG(Eigen::Vector3d& fv)
{
      // compute rotation for the first pose 
    // Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Vector3d tv(0, 0, 1);  // vn100's gz points to upwards
    Eigen::Vector3d w = fv.cross(tv).normalized(); 
    double angle = acos(fv.dot(tv)); 
    
    double half_angle = angle /2.;
    Eigen::Vector4d vq; 
    vq.head<3>() = w * sin(half_angle); 
    vq[3] = cos(half_angle); 

    // cout <<"w = "<<w.transpose()<<" angle = "<<angle<<" vq = "<<vq.transpose()<<endl; 
    Eigen::Quaterniond q(vq); 
    return q;
}

void VIO::initialize()
{
    double ax = 0; 
    double ay = 0; 
    double az = 0; 
    double gx = 0; 
    double gy = 0; 
    double gz = 0; 
    int N = linear_acceleration_buf[0].size() ; 
    if(N <= 0)
	return; 
    for(int i=0; i<N; i++)
    {
	Vector3d& acc = linear_acceleration_buf[0][i]; 
	ax += acc[0]; ay += acc[1]; az += acc[2]; 
	Vector3d& gyc = angular_velocity_buf[0][i]; 
	gx += gyc[0]; gy += gyc[1]; gz += gyc[2]; 
    }
    ax /= (float)(N);
    ay /= (float)(N); 
    az /= (float)(N); 
    gx /= (float)(N);
    gy /= (float)(N); 
    gz /= (float)(N); 
    
    double len = sqrt(ax*ax + ay*ay + az*az); 
    ax /= len; ay /= len; az /= len; 
    
    Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Quaterniond q = rotateToG(fv);
    Eigen::Matrix<double, 3, 3> m = q.toRotationMatrix(); 
    
    Eigen::Vector3d Z(0,0,0);
    Rs[1] = Rs[0] = m; 
    Ps[1] = Ps[0] = Z;  
    Vs[1] = Vs[0] = Z;
    Bas[1] = Bas[0] = Z; 
    Bgs[1] = Bgs[0] = Eigen::Vector3d(gx, gy, gz); 
    mCurrIMUPose = tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(0,0,0)); 
    
    Eigen::Vector3d tg = m * fv; 
    cout <<"q = "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
    cout <<" after transform fv = "<<tg.transpose()<<endl; 
    cout <<" initialization bg: "<<Bgs[0].transpose()<<endl; 
    mCurrPose = mCurrIMUPose * mTIC; 
    mInitCamPose = mLastPose = mCurrPose; 
    cout <<"vio.cpp: after initialization mCurrIMUPose: "<<Ps[0].transpose()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl; 
    printTF(mTIC, string("vio.cpp: after initialization mTIC: ")); 
    printTF(mCurrPose, string("vio.cpp: after initialization mCurrPose: ")); 
    
    mbInited = true; 
    return ; 
}

void VIO::prepareNextLoop()
{
    // prepare for the next loop
    // add vo to currentpose
    mCurrPose = mCurrIMUPose * mTIC; 
    tf::Transform T_ji = mCurrPose.inverse() * mLastPose;
    mLastPose = mCurrPose; 

    // set up feature points 
    int j = 0; 
    for(int i=0; i<mImgPTCurr->points.size(); i++)
    {
	bool ipFound = false;
	for(; j<mImgPTLast->points.size(); j++)
	{
	    if(mImgPTLast->points[j].ind == mImgPTCurr->points[i].ind)
	    {
		ipFound = true; 
	    }
	    if(mImgPTLast->points[j].ind >= mImgPTCurr->points[i].ind)
		break;
	}
	if(ipFound)
	{
	    mFtTransCurr.push_back(mFtTransLast[j]); 
	    mFtObsCurr->points.push_back(mFtObsLast->points[j]); 
	    if(mvDptLast[j] > 0) // transform into current pose 
	    {
		tf::Vector3 pi(mImgPTLast->points[j].u * mvDptLast[j], 
			mImgPTLast->points[j].v * mvDptLast[j], mvDptLast[j]);
		tf::Vector3 pj = T_ji * pi; 
		mvDptCurr.push_back(pj.z()); 
	    }else{
		mvDptCurr.push_back(-1);    
	    }
	}else{ // new features 
	    mFtTransCurr.push_back(mCurrPose); 
	    mFtObsCurr->points.push_back(mImgPTCurr->points[i]); 
	    mvDptCurr.push_back(-1); 
	}   
    }

    mFtTransLast.clear(); 
    mFtObsLast->clear(); 
    mvDptLast.clear();
    return ; 
}

void VIO::setPointCloudAt(double t)
{
    mPC->clear(); 
    m_pc_buf.lock(); 
	while(!pctime_buf.empty())
	{
	    double tmp = pctime_buf.front(); 
	    if(tmp < t)
	    {
		pctime_buf.pop(); 
		pc_buf.pop(); 
	    }
	    else if(tmp == t)
	    {
		ROS_DEBUG("vio.cpp: succeed set pointcloud at t = %lf", t);
		mPCTime = tmp;
		mPC = pc_buf.front(); 
		mKDTree->setInputCloud(mPC); 
		pctime_buf.pop(); 
		pc_buf.pop(); 
		break;
	    }else{ // tmp > t
		ROS_WARN("vio.cpp: no point cloud available for t= %lf", t);
		break; 
	    }
	}
    m_pc_buf.unlock(); 
}

void VIO::associateFeatures(vector<ip_M>& vip)
{
    int imgPTLastNum = mImgPTLast->points.size(); 
    int imgPTCurrNum = mImgPTCurr->points.size();

    // handle depth 
    mFtObsLast.swap(mFtObsCurr); 
    mFtTransLast.swap(mFtTransCurr); 
    mvDptLast.swap(mvDptCurr); 

    pcl::PointXYZI ips; 
    // pcl::PointXYZHSV ipr; 
    // x,y : u,v in LastImg z,h : u,v in CurrImg, 
    // s: distance along the ray, v: 1 has depth, 2 triangulate 0 no depth  
    // pcl::PointCloud<pcl::PointXYZHSV>::Ptr ipRelations(new pcl::PointCloud<pcl::PointXYZHSV>());
    vector<ip_M> ipRelations; 
    ip_M ipr; 
    // std::vector<int> ipInd;

    // find corresponding point cloud 
    setPointCloudAt(mTimeLast); 

    int j = 0;
    int cnt_matched = 0; 
    int cnt_no_depth = 0; 
    int cnt_depth_mes = 0; 
    int cnt_depth_tri = 0; 
    int cnt_not_matched = 0; 
    double disparity = 0; 
    // static ofstream fdis("disparity.log"); 
    for(int i=0; i<imgPTLastNum; i++)
    {
	bool ipFound = false; 
	for(; j<imgPTCurrNum; j++)
	{
	    if(mImgPTCurr->points[j].ind == mImgPTLast->points[i].ind) 
	    {
		ipFound = true; 
		break; 
	    }
	    if(mImgPTCurr->points[j].ind > mImgPTLast->points[i].ind)
		break; 
	}

	if(ipFound)
	{
	    ++cnt_matched; 
	    // normalized point 
	    ipr.ui = mImgPTLast->points[i].u; 
	    ipr.vi = mImgPTLast->points[i].v; 
	    ipr.uj = mImgPTCurr->points[j].u; 
	    ipr.vj = mImgPTCurr->points[j].v; 
	    
	    disparity += sqrt(SQ(ipr.ui - ipr.uj) + SQ(ipr.vi - ipr.vj)); 

	    ipr.ind = mImgPTCurr->points[j].ind;

	    ips.x = mZoomDis * ipr.ui; 
	    ips.y = mZoomDis * ipr.vi; 
	    ips.z = mZoomDis; 

	    if(mPC->points.size() > 20)
	    {
		// for kd search 
		std::vector<int> pointSearchInd;
		std::vector<float> pointSearchSqrDis;

		mKDTree->nearestKSearch(ips, 3, pointSearchInd, pointSearchSqrDis); 
		double minDepth, maxDepth; 
		if(pointSearchSqrDis[0] < 0.5 && pointSearchInd.size() == 3)
		{
		    pcl::PointXYZI depthPoint = mPC->points[pointSearchInd[0]];
		    double x1 = depthPoint.x * depthPoint.intensity / mZoomDis;
		    double y1 = depthPoint.y * depthPoint.intensity / mZoomDis;
		    double z1 = depthPoint.intensity;
		    minDepth = z1;
		    maxDepth = z1;

		    depthPoint = mPC->points[pointSearchInd[1]];
		    double x2 = depthPoint.x * depthPoint.intensity / mZoomDis;
		    double y2 = depthPoint.y * depthPoint.intensity / mZoomDis;
		    double z2 = depthPoint.intensity;
		    minDepth = (z2 < minDepth)? z2 : minDepth;
		    maxDepth = (z2 > maxDepth)? z2 : maxDepth;

		    depthPoint = mPC->points[pointSearchInd[2]];
		    double x3 = depthPoint.x * depthPoint.intensity / mZoomDis;
		    double y3 = depthPoint.y * depthPoint.intensity / mZoomDis;
		    double z3 = depthPoint.intensity;
		    minDepth = (z3 < minDepth)? z3 : minDepth;
		    maxDepth = (z3 > maxDepth)? z3 : maxDepth;

		    double u = ipr.ui;
		    double v = ipr.vi;

		    // intersection point between direction of OP and the Plane formed by [P1, P2, P3]
		    ipr.s = (x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1) 
			/ (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2 + u*y1*z2 - u*y2*z1
				- v*x1*z2 + v*x2*z1 - u*y1*z3 + u*y3*z1 + v*x1*z3 - v*x3*z1 + u*y2*z3 
				- u*y3*z2 - v*x2*z3 + v*x3*z2);
		    ipr.v = ip_M::DEPTH_MES; // meaning this point has a depth measurement 

		    // check the validity of the depth measurement 
		    if(maxDepth - minDepth > 2) // lie on an edge or noisy point? 
		    {	
			ipr.s = 0; 
			ipr.v = ip_M::NO_DEPTH; 
		    }else if(ipr.s - maxDepth > 0.2)
		    {
			ipr.s = maxDepth; 
		    }else if(ipr.s - minDepth < - 0.2)
		    {
			ipr.s = minDepth; 
		    }
		}else{
		    ipr.s = 0; 
		    ipr.v = ip_M::NO_DEPTH; 
		}
	    }else{
		ipr.s = 0; 
		ipr.v = ip_M::NO_DEPTH;
	    }

	    // if no depth 
	    if(ipr.v == ip_M::NO_DEPTH)
	    {
		// triangulation 
		// verify enough distance 
		tf::Transform first_pose = mFtTransLast[i]; // i corresponds to feature i in ImgPTLast 
		tf::Vector3 dis_ = mLastPose.getOrigin() - first_pose.getOrigin(); 
		if(dis_.length() >= mdisThresholdForTriangulation) 
		{
		    tf::Transform T12 = first_pose.inverse()*mLastPose; 
		    tf::Transform I(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)); 

		    Eigen::Vector2d p1(mFtObsLast->points[i].u, mFtObsLast->points[i].v); 
		    Eigen::Vector2d p2(ipr.ui, ipr.vi); 
		    Eigen::Vector3d pt_2 = stereo_triangulate(T12, I, p1, p2); 
		    double depth = pt_2(2); 
		    if(depth > 0.5 && depth < 50)
		    {
			ipr.s = depth; 
			ipr.v = ip_M::DEPTH_TRI; 
		    }	
		}

		// if triangulation 
		if(ipr.v == ip_M::DEPTH_TRI)
		{
		    // the feature in lastImg do not has depth but already has depth before
		    // and also being triangulated, update depth 
		    if(mvDptLast[i] > 0)
		    {
			ipr.s = 3 * ipr.s *(mvDptLast[i])/(ipr.s + 2*(mvDptLast[i])); // 
		    }
		    mvDptLast[i] = ipr.s; 
		}else if(mvDptLast[i] > 0)
		{
		    // if failed to triangulate, but already has depth 
		    ipr.s = mvDptLast[i];
		    ipr.v = ip_M::DEPTH_TRI;
		}
	    }
	    if(ipr.v == ip_M::NO_DEPTH)
	     { ++cnt_no_depth;
		// ROS_DEBUG("at match num = %d ++cnt_no_depth = %d ", cnt_matched, cnt_no_depth); 
	     }
	    else if(ipr.v == ip_M::DEPTH_MES)
	    {
		++cnt_depth_mes;
		// ROS_DEBUG("at match num = %d ++cnt_depth_mes = %d ", cnt_matched, cnt_depth_mes); 
	    }
	    else if(ipr.v == ip_M::DEPTH_TRI)
	    {
		++cnt_depth_tri; 
		// ROS_DEBUG("at match num = %d ++cnt_depth_tri = %d ", cnt_matched, cnt_depth_tri); 
	    }
	    ipr.ind = mImgPTLast->points[i].ind; 
	    ipRelations.push_back(ipr); 
	}else
	    ++cnt_not_matched; 
    }
    vip = ipRelations; 
    ROS_DEBUG("vio.cpp: total %d no matches %d matches %d, no_depth: %d depth_with_meas: %d depth_with_tri: %d",imgPTLastNum, cnt_not_matched, cnt_matched, cnt_no_depth, cnt_depth_mes, cnt_depth_tri);
    // fdis << std::fixed<<disparity<<endl; 
    
    mbStill = false; 

    if(cnt_matched > 10)
    {
	disparity /= (double)(cnt_matched); 
	if(disparity < 0.002)
	{
	    ROS_WARN("vio: still cnt_matched: %d mean disparity: %lf < 0.002", cnt_matched, disparity); 
	    mbStill = true; 
	}
    }
}


void VIO::removeFloorPts(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >& in, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >& out)
{
    TicToc t_fz; 
    if(in->points.size() < 100)
	return ; 
    out->points.clear();

    double min_z, max_z; 

    if(mFloorZ == NOT_INITIED) //     
    {	
	// double max_z = 0.1; 
	// double min_z = -1.5; 
	   min_z = 1e10; 
	   max_z = -1e10; 
        for(int i=0; i<in->points.size(); i++)
        {
            double pz = in->points[i].z; 
            if(min_z > pz ) min_z = pz; 
            if(max_z < pz ) max_z = pz; 
        }
    }
    else{
        min_z = mFloorZ - 3*mFloorRange; 
        max_z = mFloorZ + 3*mFloorRange; 
    }

	// cout <<"removeFloorPts.cpp : min_z: "<<min_z<<" max_z: "<<max_z<<endl; 

	//  histogram into different bins 
	double res = 0.1; 
	if(max_z - min_z > 2.0) max_z = min_z + 2.0; 
	int n_bins = (max_z - min_z)/res + 1; 

	map<int, vector<int> > bins; 
	for(int i=0; i<in->points.size(); i++)
	{
	    double pz = in->points[i].z; 
	    int ind = (pz - min_z + res/2.)/res;
	    bins[ind].push_back(i); 
	}

	// find the bin with most points 
	int max_n = 0; 
	int max_id = -1; 
	map<int, vector<int> >::iterator it = bins.begin();
	while(it != bins.end())
	{
	    if(it->second.size() > max_n) 
	    {
		  max_n = it->second.size(); 
		  max_id = it->first; 
	    }
	    ++it; 
	}
    if(mFloorZ == NOT_INITIED)
	   mFloorZ = min_z + max_id*res; 
    else {
        if(max_n > 100)
            mFloorZ = 0.5 * mFloorZ + 0.5 *(min_z + max_id*res);
    }

    // find floor plane 
    mPCFloor->clear(); 
    out->points.clear(); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    tmp->points.reserve(in->points.size()/2); 
    for(int i=0; i<in->points.size(); i++)
    {
        double pz = in->points[i].z; 
        if(pz < mFloorZ + mFloorRange && pz > mFloorZ - mFloorRange)
        {
          tmp->points.push_back(in->points[i]); 
        }
    }
    if(tmp->points.size() < 150)
        return ; 
    Eigen::Vector3d nv; 
    double nd; 
    pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
    ((Plane*)(0))->computeByPCL<pcl::PointXYZ>(tmp, indices, nv, nd); 
    Eigen::Vector3d g(0, 0, 1); 
    double angle = nv.dot(g); 
    const double COS30 = cos(30.*M_PI/180.);
    // cout<<"Floor plane has indices.size = "<<indices->indices.size()<<" points nv = "<<nv.transpose()<<endl;
    if(indices->indices.size() < 100 || angle < COS30) // NO Floor plane is observed 
    {
        out->points.reserve(in->points.size()); 
        for(int i=0; i<in->points.size(); i++)
        {
            double pz = in->points[i].z; 
            if(pz > mFloorZ + 3*mFloorRange)
            {
              // out->points.push_back(in->points[i]); 
		pcl::PointXYZI pt; 
		pt.x = in->points[i].x; 
		pt.y = in->points[i].y; 
		pt.z = in->points[i].z; 
		pt.intensity = pz - mFloorZ; // intensity contains distance to floor plane 
		out->points.push_back(pt); 
            }
        }
        // cout<<"failed to take it as a plane ! "<<endl;
    }else // succeed to get a floor plane 
    { 
        if(mbFirstFloorObserved == false)
        {
            mbFirstFloorObserved = true; 
            /*
            Pls[0][0] = nv(0); 
            Pls[0][1] = nv(1); 
            Pls[0][2] = nv(2); */
            Eigen::Vector3d ni(nv(0), nv(1), nv(2));
            Eigen::Quaterniond dq = rotateToG(ni); 
            Eigen::Matrix3d dR = dq.toRotationMatrix(); 
            Eigen::Vector3d new_v = dR*Vs[0];
            Eigen::Matrix3d new_R = Rs[0]*dR.transpose(); 
            Vs[0] = Vs[1] = new_v; 
            Rs[0] = Rs[1] = new_R;

            Pls[0][0] = 0; 
            Pls[0][1] = 0; 
            Pls[0][2] = 1.;
            Pls[0][3] = nd;
        }
        // save floor points for display 
        mPCFloor->points.reserve(indices->indices.size()); 
        double sum_z = 0; 
        for(int i=0; i<indices->indices.size(); i++)
        {
            mPCFloor->points.push_back(tmp->points[indices->indices[i]]); 
            sum_z += tmp->points[indices->indices[i]].z; 
        }
        mFloorZ = sum_z/(indices->indices.size()); 
        cout <<"vio: succeed to find out floor plane, reset floor_Z = "<<mFloorZ<<endl;

        mPCFloor->width = mPCFloor->points.size(); 
        mPCFloor->height = 1;
        mPCFloor->is_dense = true;
        // find the obstacle point 
        for(int i=0; i<in->points.size(); i++)
        {
            Eigen::Vector3d pt(in->points[i].x, in->points[i].y, in->points[i].z);
            double dis = nv.dot(pt) + nd; 
            if(dis > 2*mFloorRange)
            {
                // out->points.push_back(in->points[i]);
		pcl::PointXYZI pt; 
		pt.x = in->points[i].x; 
		pt.y = in->points[i].y; 
		pt.z = in->points[i].z; 
		pt.intensity = dis; // intensity contains distance to floor plane 
		out->points.push_back(pt); 
            }
        }
    }
  
    out->width = out->points.size(); 
    out->height = 1; 
    out->is_dense = true; 
    // cout <<"vio.cpp: outpc has "<<out->points.size()<<" points floor_z = "<<mFloorZ<<endl;
    ROS_DEBUG("vio: remove floor cost %f ms", t_fz.toc()); 
}
