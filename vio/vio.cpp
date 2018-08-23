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
mbFirstIMU(true),
mbInited(false),
frame_count(0)
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
	pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity); 

	// tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity); 
	// dt_buf[frame_count].push_back(dt); 
	// linear_acceleration_buf[frame_count].push_back(linear_acceleration);
	// angular_velocity_buf[frame_count].push_back(angular_velocity);

	int j = frame_count; 
	Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - mg; 
	Vector3d un_gyr = 0.5 *(gyr_0 + angular_velocity) - Bgs[j]; 
	Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); 
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
    // f_manager.setRic(ric);
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

	// solve odometry 
	solveOdometry(vip); 

	// slide for next loop 
	slideWindow(); 
    }
    
    // for next loop
    prepareNextLoop(); 

    // for display 
    prepareForDisplay(vip);

    return ; 
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

void VIO::solveOdometry(vector<ip_M>& vip)
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
	problem.SetParameterBlockConstant(para_Ex_Pose[0]); 
    }

    priorOptimize(vip); 

    // add imu factor 
    for (int i = 0; i < WN; i++)
    {
	int j = i + 1;
	if (pre_integrations[j]->sum_dt > 10.0)
	    continue;
	IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
	problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
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
    // compute rotation for the first pose 
    Eigen::Vector3d fv(ax, ay, az); 
    Eigen::Vector3d tv(0, 0, 1);  // vn100's gz points to upwards
    Eigen::Vector3d w = fv.cross(tv).normalized(); 
    double angle = acos(fv.dot(tv)); 
    
    double half_angle = angle /2.;
    Eigen::Vector4d vq; 
    vq.head<3>() = w * sin(half_angle); 
    vq[3] = cos(half_angle); 

    // cout <<"w = "<<w.transpose()<<" angle = "<<angle<<" vq = "<<vq.transpose()<<endl; 
    Eigen::Quaterniond q(vq); 
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
    mInitCamPose = mCurrPose; 
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
}


