/*
    Aug. 8 2018 He Zhang, hzhang8@vcu.edu 
    
    test visual odometry 

    
    in demo_rgbd, rotation represented by Euler angle, sequence is Z1-X2-Y3

    Z1X2Y3 = [ c1c3 - s1s2s3, -c2s1, c1s3 + c3s1s2,
	       c3s1 + c1s2s3, c1c2,  s1s3 - c1c3s2, 
	       -c2s3,          s2,     c2c3
	     ]

*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <random>
#include <opencv/cv.h>
#include "../utility/utility.h"

#include "projection_euler-ZXY.h"

#define D2R(d) (((d)*M_PI)/180.)
#define SQ(x) ((x)*(x))

using namespace EulerZXY; 

struct Jacob_E{
    union 
    {
	struct
	{
	    float de_dr; // roll 
	    float de_dp; // pitch 
	    float de_dy; // yaw
	    float de_dtx; // tx
	    float de_dty; // ty
	    float de_dtz; // tz
	};
	float de[6]; 
    };
    float err;
};

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > std_3d;

using namespace std; 
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; 
}
double Fy2(Eigen::Matrix<double, 6, 1>& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL); 
double Fy3(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J = NULL); 
double Fy4(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J = NULL ); 
void numeric_Jacobian(Eigen::Matrix<double, 6, 1>& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J); 

void pnp(std_3d& pts1, std_3d& pts2, Eigen::Matrix<double, 6, 1>& );

Eigen::Vector6d vo_demo_rgbd(std_3d& pts1, std_3d& pts2, Eigen::Vector6d& inipose);
Eigen::Vector6d vo_ceres(std_3d& pts1, std_3d& pts2, Eigen::Vector6d& inipose);

void testy2(); 
void test_vo(); 

int main(int argc, char* argv[])
{
    // testy2();
    test_vo(); 

} 

void test_vo()
{
    int NUM = 100; 
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
	Eigen::Vector6d pose; 
	double rpy[3] = {2, 3, 10}; 
	double vt[3] = {0.2, 0.1, -0.2}; 

	// double rpy[3] = {rangle(gen), rangle(gen), rangle(gen)}; 
	// double vt[3] = {rdis(gen), rdis(gen), rdis(gen)}; 

	pose << D2R(rpy[0]), D2R(rpy[1]), D2R(rpy[2]), vt[0], vt[1], vt[2]; 
	// cout <<"gt pose: "<<endl<<pose<<endl; 
	Eigen::Vector3d yrp(rpy[2], rpy[0], rpy[1]); 
	Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
	Eigen::Vector3d t(vt[0], vt[1], vt[2]);

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
	Eigen::Vector6d pnp_vo;
	pnp(half_PT1, half_PT2, pnp_vo); 
	// cout <<"pnp vo_p: "<<endl<<pnp_vo<<endl; 
	Eigen::Vector6d delta_p = pnp_vo - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl; 
	err_pnp += delta_p.norm(); 

	// vo_demo_rgbd 
	Eigen::Vector6d inipose; 
	inipose << 0, 0, 0, 0, 0, 0; 
	Eigen::Vector6d vo_p = vo_demo_rgbd(PT1, PT2, inipose); 
	// cout <<"demo_rgbd vo_p: "<<endl<<vo_p<<endl; 
	delta_p = vo_p - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl;
	err_vo_demo += delta_p.norm(); 

	// vo_ceres 
	vo_p = vo_ceres(PT1, PT2, inipose); 
	// cout <<"ceres vo_p: "<<endl<<vo_p<<endl; 
	delta_p = vo_p - pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl;
	err_vo_ceres += delta_p.norm(); 
    }
    cout <<"mean err_pnp: "<<err_pnp/NUM<<endl; 
    cout <<"mean err_demo: "<<err_vo_demo/NUM<<endl; 
    cout <<"mean err_ceres: "<<err_vo_ceres/NUM<<endl;
//
   return ;
}

Eigen::Vector6d vo_ceres(std_3d& PT1, std_3d& PT2, Eigen::Vector6d& inipose)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // these variables needs to be global or static 
    static double para_pose[0][6]; 

    int N = PT1.size(); 
    int M = N/2; 
    for(int i=0; i<6; i++)
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

Eigen::Vector6d vo_demo_rgbd(std_3d& PT1, std_3d& PT2, Eigen::Vector6d& inipose)
{
    int N = PT1.size(); 
    int M = N/2; 
    int iterNum = 150;
    double scale = 10;
    
    vector<Jacob_E> vje; 
    double sum_e = 0; 
    
    Eigen::Vector6d vo_p = inipose; 
    for(int it=0; it<iterNum; it++)
    {
	vje.clear(); 
	sum_e = 0; 
	
	// first build on 
	for(int i=0; i<N; i++)
	{
	    Eigen::Vector3d p1 = PT1[i]; 
	    Eigen::Vector3d p2 = PT2[i]; 
	    Eigen::Vector6d J; 

	    // first half for y2 
	    if(i < M)
	    {
		Eigen::Vector3d np1 = p1/p1[2]; 
		Eigen::Vector3d np2 = p2/p2[2];

		double ey2 = Fy2(vo_p, np1, np2, &J); 
		Jacob_E je; 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = scale * J(j); 
		}
		je.err = scale * ey2; 
		vje.push_back(je); 
		sum_e += SQ(je.err); 
	    }else
	    {
		Eigen::Vector3d np1 = p1/p1[2]; 
		Eigen::Vector3d np2 = p2/p2[2];
		np1(2) = p1[2]; 

		double ey3 = Fy3(vo_p, np1, np2, &J); 
		Jacob_E je; 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.err = ey3;
		vje.push_back(je);
		sum_e += SQ(je.err); 
		double ey4 = Fy4(vo_p, np1, np2, &J); 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.err = ey4;
		vje.push_back(je);
		sum_e += SQ(je.err); 
	    }
	}

	// cout <<"iterator i: "<<it<<" sum_err: "<<sum_e<<endl; 
	
	// try vo to compute the transformation 
	int N_JE = vje.size(); 
	cv::Mat matA(N_JE, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(6, N_JE, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(N_JE, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

	for(int i=0; i<N_JE; i++)
	{
	    Jacob_E& je = vje[i]; 
	    for(int j=0; j<6; j++)
		matA.at<float>(i, j) = je.de[j]; 
	    matB.at<float>(i, 0) = -0.1*je.err; 
	}
	cv::transpose(matA, matAt); 
	matAtA = matAt*matA; 
	matAtB = matAt * matB; 
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); 
	Eigen::Vector6d dmat; 

	for(int j=0; j<6; j++)
	{
	    vo_p(j) += matX.at<float>(j, 0); 
	    dmat(j) = matX.at<float>(j, 0); 
	}
	
	float delta = dmat.norm(); 
	if(delta < 0.00001)
	    break; 
    }
    return vo_p; 
}

void pnp(std_3d& pts1, std_3d& pts2, Eigen::Matrix<double, 6, 1>& pose)
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
    pose(0) = D2R(ypr(2)); pose(1) = D2R(ypr(1)); pose(2) = D2R(ypr(0)); 
    pose(3) = t(0); pose(4) = t(1); pose(5) = t(2); 
    return ;
}

void testy2()
{
    // Eigen::Vector6d pose(D2R(10), D2R(20), D2R(30), 1., 0.5, -2.); 
    Eigen::Vector6d pose; 
    pose << D2R(10), D2R(20), D2R(30), 1., 0.5, -2.; 

    Eigen::Vector3d p1(1, 1, 1);
    Eigen::Vector3d t(1., 0.5, -2);
    Eigen::Vector3d rpy(10, 20, 30);
    // Eigen::Matrix<double, 3, 3> R = Utility::rpy2R(rpy); 
    Eigen::Vector3d yrp(30, 10, 20); 
    Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
    
    // cout <<"R: "<<endl<<R<<endl;  

    Eigen::Vector3d p2 = R*p1 + t; 
    Eigen::Vector3d np1 = p1/p1[2]; 
    Eigen::Vector3d np2 = p2/p2[2];
    Eigen::Vector6d J; 
    double yy2 = Fy2(pose, np1, np2, &J); 
    // cout <<"np1: "<<np1<<endl;
    // cout <<"p2: " <<p2<<endl; 
    // cout <<"np2: "<<np2<<endl;
    // cout <<"y2 = "<<yy2<<endl; 

    Eigen::Vector6d J_numeric; 
    numeric_Jacobian(pose, np1, np2, &J_numeric); 

    cout <<"J: "<<endl<<J<<endl; 
    cout <<"J_numeric: "<<endl<<J_numeric<<endl; 
}

void numeric_Jacobian(Eigen::Matrix<double, 6, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J)
{
    double epsi = 1e-5; 
    double factor = 2.*epsi; 

    for(int i=0; i<6; i++)
    {
	Eigen::Matrix<double, 6, 1> tmp = transform; 
	tmp(i) += epsi; 
	double y2_pos = Fy2(tmp, p1, p2); 
	tmp = transform; 
	tmp(i) -= epsi; 
	double y2_neg = Fy2(tmp, p1, p2); 
	(*J)(i) = (y2_pos - y2_neg)/factor; 
    }
}


double Fy2(Eigen::Matrix<double, 6, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J)
{
    double srx = sin(transform[0]); // roll 
    double crx = cos(transform[0]);
    double sry = sin(transform[1]); // pitch
    double cry = cos(transform[1]);
    double srz = sin(transform[2]); // yaw
    double crz = cos(transform[2]);
    double tx = transform[3];  // x
    double ty = transform[4];  // y
    double tz = transform[5];  // z

    double u0 = p1(0); double v0 = p1(1); 
    double u1 = p2(0); double v1 = p2(1); 

    double yy2 = (ty - tz*v1)*(crz*sry + cry*srx*srz) - (tx - tz*u1)*(sry*srz - cry*crz*srx) 
	- v0*(srx*(ty*u1 - tx*v1) + crx*crz*(tx - tz*u1) + crx*srz*(ty - tz*v1)) 
	+ u0*((ty - tz*v1)*(cry*crz - srx*sry*srz) - (tx - tz*u1)*(cry*srz + crz*srx*sry) 
		+ crx*sry*(ty*u1 - tx*v1)) - crx*cry*(ty*u1 - tx*v1);

    /*Eigen::Matrix<double, 3, 3> R; 
    R << crz*cry - srz*srx*sry, -crx*srz, crz*sry + cry*srz*srx, 
	 cry*srz + crz*srx*sry, crz*crx,  srz*sry - crz*cry*srx,
	 -crx*sry , srx, crx*cry; 
    cout<<"R: "<<endl<<R<<endl;
    */

    if(J != NULL)
    {
	// dy2_droll 
      (*J)(0) = v0*(crz*srx*(tx - tz*u1) - crx*(ty*u1 - tx*v1) + srz*srx*(ty - tz*v1)) 
	- u0*(sry*srx*(ty*u1 - tx*v1) + crz*sry*crx*(tx - tz*u1) + sry*srz*crx*(ty - tz*v1)) 
	+ cry*srx*(ty*u1 - tx*v1) + cry*crz*crx*(tx - tz*u1) + cry*srz*crx*(ty - tz*v1);

	// dy2_dpitch
     (*J)(1) = u0*((tx - tz*u1)*(srz*sry - crz*srx*cry) - (ty - tz*v1)*(crz*sry + srx*srz*cry) 
	    + crx*cry*(ty*u1 - tx*v1)) - (tx - tz*u1)*(srz*cry + crz*srx*sry) 
	+ (ty - tz*v1)*(crz*cry - srx*srz*sry) + crx*sry*(ty*u1 - tx*v1);
	
	// dy2_dyaw
     (*J)(2) = -u0*((tx - tz*u1)*(cry*crz - srx*sry*srz) + (ty - tz*v1)*(cry*srz + srx*sry*crz)) 
	- (tx - tz*u1)*(sry*crz + cry*srx*srz) - (ty - tz*v1)*(sry*srz - cry*srx*crz) 
	- v0*(crx*crz*(ty - tz*v1) - crx*srz*(tx - tz*u1));
	
	// dy2_dx
      (*J)(3) = cry*crz*srx - v0*(crx*crz - srx*v1) - u0*(cry*srz + crz*srx*sry + crx*sry*v1) 
	- sry*srz + crx*cry*v1;

	// dy2_dy
      (*J)(4) = crz*sry - v0*(crx*srz + srx*u1) + u0*(cry*crz + crx*sry*u1 - srx*sry*srz) 
	- crx*cry*u1 + cry*srx*srz;

	// dy2_dz
	(*J)(5) = u1*(sry*srz - cry*crz*srx) - v1*(crz*sry + cry*srx*srz) + u0*(u1*(cry*srz + crz*srx*sry) 
	    - v1*(cry*crz - srx*sry*srz)) + v0*(crx*crz*u1 + crx*srz*v1);
    }
    return yy2; 
}


double Fy3(Eigen::Vector6d& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J)
{
    double srx = sin(transform[0]); // roll 
    double crx = cos(transform[0]);
    double sry = sin(transform[1]); // pitch
    double cry = cos(transform[1]);
    double srz = sin(transform[2]); // yaw
    double crz = cos(transform[2]);
    double tx = transform[3];  // x
    double ty = transform[4];  // y
    double tz = transform[5];  // z

    double u0 = p1(0); double v0 = p1(1); 
    double u1 = p2(0); double v1 = p2(1); 
    double d0 = p1(2); 
    double y3 = tx - tz*u1 + d0*(crz*sry - crx*cry*u1 + cry*srx*srz) - d0*v0*(crx*srz + srx*u1) 
	+ d0*u0*(cry*crz + crx*sry*u1 - srx*sry*srz);
    
    if(J != NULL)
    {
	(*J)(0) = d0*(cry*srz*crx + cry*u1*srx) - d0*u0*(sry*srz*crx + sry*u1*srx) - d0*v0*(u1*crx - srz*srx);
	(*J)(1) = d0*(crz*cry + crx*u1*sry - srx*srz*sry) - d0*u0*(crz*sry - crx*u1*cry + srx*srz*cry);
	(*J)(2) = -d0*(sry*srz - cry*srx*crz) - d0*u0*(cry*srz + srx*sry*crz) - crx*d0*v0*crz;
	(*J)(3) = 1; 
	(*J)(4) = 0;
	(*J)(5) = -u1; 
    }
    return y3; 
}
double Fy4(Eigen::Vector6d& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J)
{
    double srx = sin(transform[0]); // roll 
    double crx = cos(transform[0]);
    double sry = sin(transform[1]); // pitch
    double cry = cos(transform[1]);
    double srz = sin(transform[2]); // yaw
    double crz = cos(transform[2]);
    double tx = transform[3];  // x
    double ty = transform[4];  // y
    double tz = transform[5];  // z

    double u0 = p1(0); double v0 = p1(1); 
    double u1 = p2(0); double v1 = p2(1); 
    double d0 = p1(2); 
    double y4 = ty - tz*v1 - d0*(cry*crz*srx - sry*srz + crx*cry*v1) + d0*v0*(crx*crz - srx*v1) 
    + d0*u0*(cry*srz + crz*srx*sry + crx*sry*v1);
    if(J != NULL)
    {
	(*J)(0) = d0*(cry*v1*srx - cry*crz*crx) + d0*u0*(crz*sry*crx - sry*v1*srx) - d0*v0*(crz*srx + v1*crx);
	(*J)(1) = d0*(srz*cry + crz*srx*sry + crx*v1*sry) + d0*u0*(crz*srx*cry - srz*sry + crx*v1*cry);
	(*J)(2) = d0*(sry*crz + cry*srx*srz) + d0*u0*(cry*crz - srx*sry*srz) - crx*d0*v0*srz;
	(*J)(3) = 0; 
	(*J)(4) = 1; 
	(*J)(5) = -v1; 	
    }
    return y4; 
}

