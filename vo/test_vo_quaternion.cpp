/*
    Aug. 8 2018 He Zhang, hzhang8@vcu.edu 
    
    test visual odometry using quaternion 

*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <random>
#include <opencv/cv.h>
#include "../utility/utility.h"

#define D2R(d) (((d)*M_PI)/180.)
#define SQ(x) ((x)*(x))

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > std_3d;

using namespace std; 
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; // roll, pitch, yaw, x, y, z 
    typedef Matrix<double, 7, 1> Vector7d; // x, y, z, qx, qy, qz, qw
}

void vo_pnp(std_3d& pts1, std_3d& pts2, Eigen::Matrix<double, 7, 1>& );
Eigen::Vector7d vo_ceres(std_3d& pts1, std_3d& pts2, Eigen::Vector7d& inipose);

void test_vo();

int main(int argc, char* argv[])
{
    test_vo(); 
} 

void test_vo()
{
    int NUM = 50; 
    double err_pnp = 0; 
    double err_vo_demo = 0; 
    double err_vo_ceres = 0;
    
    // generate two point sets 
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::uniform_real_distribution<> uni_dis(-1.0, 1.0);
    std::uniform_real_distribution<> uni_z(1., 5.); 
    std::normal_distribution<> noise{0,0.02};
    std::uniform_real_distribution<> rangle(-5., 5.); 
    std::uniform_real_distribution<> rdis(-0.2, 0.2); 

     for(int k = 0; k<NUM; k++)
    {
	// generate transformation [R, t]
	Eigen::Vector7d pose; 
	double rpy[3] = {2, 3, 10}; 
	double vt[3] = {0.2, 0.1, -0.2}; 

	// double rpy[3] = {rangle(gen), rangle(gen), rangle(gen)}; 
	// double vt[3] = {rdis(gen), rdis(gen), rdis(gen)}; 
	// pose << D2R(rpy[0]), D2R(rpy[1]), D2R(rpy[2]), vt[0], vt[1], vt[2]; 
	// cout <<"gt pose: "<<endl<<pose<<endl; 
	Eigen::Vector3d yrp(rpy[2], rpy[0], rpy[1]); 
	Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
	Eigen::Vector3d t(vt[0], vt[1], vt[2]);
	Eigen::Quaterniond q(R); 
	pose << t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w() << endl; 

	int N = 40; 
	std_3d PT1(N); 
	std_3d PT2(N); 
	for(int i=0; i<N; )
	{
	    Eigen::Vector3d p1; 
	    p1 << uni_dis(gen), uni_dis(gen), 1.0; 
	    p1 = p1 * (uni_z(gen) + 1.); 
	    Eigen::Vector3d p2 = R * p1 + t; 
	    p2(0) += noise(gen); 
	    p2(1) += noise(gen); 
	    p2(2) += noise(gen); 
	    if(p1(2) <= 0.3 || p2(2) <= 0.3)
		continue; 
	    PT1[i] = p1; 
	    PT2[i] = p2; 
	    i++;
	}

	// vo_pnp 
	std_3d half_PT1(PT1.begin() + N/2, PT1.end()); 
	std_3d half_PT2(PT2.begin() + N/2, PT2.end()); 
	Eigen::Vector7d pnp_vo;
	vo_pnp(half_PT1, half_PT2, pnp_vo); 
	// cout <<"pnp vo_p: "<<endl<<pnp_vo<<endl; 
	Eigen::Vector7d delta_p = pnp_vo - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl; 
	err_pnp += delta_p.norm(); 

	// vo_demo_rgbd 
	Eigen::Vector7d inipose; 
	inipose << 0, 0, 0, 0, 0, 0, 1; 
	// Eigen::Vector6d vo_p = vo_demo_rgbd(PT1, PT2, inipose); 
	// cout <<"demo_rgbd vo_p: "<<endl<<vo_p<<endl; 
	// delta_p = vo_p - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl;
	// err_vo_demo += delta_p.norm(); 

	// vo_ceres 
	vo_p = vo_ceres(PT1, PT2, inipose); 
	// cout <<"ceres vo_p: "<<endl<<vo_p<<endl; 
	delta_p = vo_p - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl;
	err_vo_ceres += delta_p.norm(); 
    }
    cout <<"mean err_pnp: "<<err_pnp/NUM<<endl; 
   //  cout <<"mean err_demo: "<<err_vo_demo/NUM<<endl; 
    cout <<"mean err_ceres: "<<err_vo_ceres/NUM<<endl;

    return ;
}

Eigen::Vector7d vo_ceres(std_3d& pts1, std_3d& pts2, Eigen::Vector7d& inipose)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // these variables needs to be global or static 
    static double para_pose[0][7]; 

    int N = PT1.size(); 
    int M = N/2; 
    for(int i=0; i<7; i++)
	para_pose[0][i] = inipose[i]; 

    // cout <<"start to construct structure! "<<endl; 
    // ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
    // problem.AddParameterBlock(para_pose[0], 6, local_param); 
    problem.AddParameterBlock(para_pose[0], 6); 

    for(int i=0; i<N; i++)
    {
	Eigen::Vector3d p1 = PT1[i]; 
	Eigen::Vector3d p2 = PT2[i]; 
	Eigen::Vector3d np1 = p1/p1[2]; 
	Eigen::Vector3d np2 = p2/p2[2];
	Eigen::Vector6d J; 

	if(i<M)
	{
	    ProjectionFactor_Y2 * f= new ProjectionFactor_Y2(np1, np2); 
	    problem.AddResidualBlock(f, loss_function, para_pose[0]); 
	}else
	{
	    np1[2] = p1[2]; 
	    ProjectionFactor_Y3 * f3 = new ProjectionFactor_Y3(np1, np2); 
	    problem.AddResidualBlock(f3, loss_function, para_pose[0]); 

	    ProjectionFactor_Y4 * f4 = new ProjectionFactor_Y4(np1, np2); 
	    problem.AddResidualBlock(f4, loss_function, para_pose[0]); 
	}	
    }   

    ceres::Solver::Options options; 
    options.linear_solver_type = ceres::DENSE_QR; // ceres::DENSE_SCHUR; 
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // ceres::DOGLEG; 
    // options.minimizer_progress_to_stdout = true; 
    options.max_num_iterations = 150; 
    ceres::Solver::Summary summary; 
    ceres::Solve(options, &problem, &summary); 
    // std::cout<<summary.BriefReport()<<endl;

    Eigen::Vector6d vo_p; 
    for(int i=0; i<6; i++)
	vo_p[i] = para_pose[0][i]; 
    return vo_p; 


}

void vo_pnp(std_3d& pts1, std_3d& pts2, Eigen::Matrix<double, 7, 1>& pose)
{
     // compute PnP error 
    Eigen::MatrixXd SRC_PTS;
    Eigen::MatrixXd DPT_PTS;
    int N = pts1.size();
    SRC_PTS.resize(3, N); 
    DPT_PTS.resize(3, N);
    for(int i=0; i<N; i++)
    {
	SRC_PTS.col(i) = pts1[i]; 
	DPT_PTS.col(i) = pts2[i];
    } 
    
    Eigen::Matrix<double, 4, 4> T = Eigen::umeyama(SRC_PTS, DPT_PTS, false); // scaling = false 
    Eigen::Matrix<double, 3, 3> R = T.block(0, 0, 3, 3); 
    Eigen::Matrix<double, 3, 1> t = T.block(0, 3, 3, 1); 
    Eigen::Vector3d ypr = Utility::R2ypr(R);  // yrp
    // pose(0) = D2R(ypr(1)); pose(1) = D2R(ypr(2)); pose(2) = D2R(ypr(0)); 
    // pose(3) = t(0); pose(4) = t(1); pose(5) = t(2); 
    pose(0) = t(0); pose(1) = t(1); pose(2) = t(2); 
    Eigen::Quaterniond q(R); 
    pose(3) = q.x(); pose(4) = q.y(); pose(5) = q.z(); pose(6) = q.w(); 
    return ;
}


