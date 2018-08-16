/*
    test vo_euler using iprelation.log 

*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <random>
#include <opencv/cv.h>
#include "../utility/utility.h"

#include "projection_euler-ZXY.h"

#define D2R(d) (((d)*M_PI)/180.)
#define SQ(x) ((x)*(x))

using namespace EulerZXY; 

struct ip_M
{
    typedef enum{NO_DEPTH =0, DEPTH_MES, DEPTH_TRI, INVALID} DPT_TYPE;
    float ui, vi, uj, vj, s; // s responds to Xi = [ui,vi,1] * si
    int ind;
    DPT_TYPE v; 
};

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

using namespace std; 
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; 
}
double Fy2(Eigen::Matrix<double, 6, 1>& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL); 
double Fy3(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J = NULL); 
double Fy4(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1> * J = NULL ); 

Eigen::Vector6d vo_demo_rgbd(vector<ip_M>& vip, Eigen::Vector6d& inipose);
Eigen::Vector6d vo_ceres(vector<ip_M>& vip, Eigen::Vector6d& inipose);

string log_file("iprelations.log"); 

void read_iprelation_log(vector<ip_M>& vip, string log_file); 
void test_vo(); 

int main(int argc, char* argv[])
{
    if(argc >= 2)
	log_file = string(argv[1]); 
    test_vo(); 
} 

void test_vo()
{
    vector<ip_M> vip; 
    read_iprelation_log(vip, log_file); 
    Eigen::Vector6d inipose; inipose<< 0, 0, 0, 0, 0, 0; 
    Eigen::Vector6d inc_vo = vo_demo_rgbd(vip, inipose); 
    cout<<"demo_rgbd inc_vo: "<<endl<<inc_vo<<endl;
    
    inc_vo = vo_ceres(vip, inipose); 
    cout<<"vo_ceres: inc_vo: "<<endl<<inc_vo<<endl;

    return ;
}

void read_iprelation_log(vector<ip_M>& vip, string log_file)
{
    ifstream inf(log_file.c_str());
    while(!inf.eof())
    {
	string s;
	getline(inf, s);
	if(!s.empty())
	{
	    float v; 
	    ip_M ip; 
	    stringstream ss; 
	    ss << s; 
	    ss >> ip.ui >> ip.vi >> ip.uj >> ip.vj >> ip.s >> v; 
	    if(v == 0.){ ip.v = ip_M::NO_DEPTH; }
	    else if(v == 1.){ip.v = ip_M::DEPTH_MES; }
	    else if(v == 2.){ip.v = ip_M::DEPTH_TRI; }
	    else ip.v = ip_M::INVALID; 
	    vip.push_back(ip); 
	}
    }
} 


Eigen::Vector6d vo_ceres(vector<ip_M>& vip, Eigen::Vector6d& inipose)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // these variables needs to be global or static 
    static double para_pose[0][6]; 

    int N = vip.size(); 
    for(int i=0; i<6; i++)
	para_pose[0][i] = inipose[i]; 
    
    // cout <<"start to construct structure! "<<endl; 
    // ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
    // problem.AddParameterBlock(para_pose[0], 6, local_param); 
    problem.AddParameterBlock(para_pose[0], 6); 

    for(int i=0; i<N; i++)
    {
	ip_M & ip = vip[i]; 
	Eigen::Vector3d p1(ip.ui, ip.vi, 1.); 
	Eigen::Vector3d p2(ip.uj, ip.vj, 1.); 
	Eigen::Vector3d np1 = p1/p1[2]; 
	Eigen::Vector3d np2 = p2/p2[2];
	Eigen::Vector6d J; 

	if(ip.v == ip_M::NO_DEPTH)
	{
	    ProjectionFactor_Y2 * f= new ProjectionFactor_Y2(np1, np2); 
	    problem.AddResidualBlock(f, loss_function, para_pose[0]); 
	}else if(ip.v == ip_M::DEPTH_MES || ip.v == ip_M::DEPTH_TRI)
	{
	    np1[2] = ip.s; 
	    ProjectionFactor_Y3 * f3 = new ProjectionFactor_Y3(np1, np2); 
	    problem.AddResidualBlock(f3, loss_function, para_pose[0]); 

	    ProjectionFactor_Y4 * f4 = new ProjectionFactor_Y4(np1, np2); 
	    problem.AddResidualBlock(f4, loss_function, para_pose[0]); 
	}	
    }   

    ceres::Solver::Options options; 
    options.linear_solver_type = ceres::DENSE_QR; // ceres::DENSE_SCHUR; 
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // ceres::DOGLEG; 
    options.minimizer_progress_to_stdout = true; 
    options.max_num_iterations = 150; 
    ceres::Solver::Summary summary; 
    ceres::Solve(options, &problem, &summary); 
    std::cout<<summary.BriefReport()<<endl;
    
    Eigen::Vector6d vo_p; 
    for(int i=0; i<6; i++)
	vo_p[i] = para_pose[0][i]; 
    return vo_p; 
}

Eigen::Vector6d vo_demo_rgbd(vector<ip_M>& vip, Eigen::Vector6d& inipose)
{
    int N = vip.size(); 
    int iterNum = 150;
    double scale = 100;
    
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
	    ip_M& ip = vip[i]; 

	    Eigen::Vector3d p1(ip.ui, ip.vi, 1.);  
	    Eigen::Vector3d p2(ip.uj, ip.vj, 1.); 
	    Eigen::Vector6d J; 

	    // first half for y2 
	    if( ip.v == ip_M::NO_DEPTH)
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
	    }else if(ip.v == ip_M::DEPTH_MES || ip.v == ip_M::DEPTH_TRI)
	    {
		Eigen::Vector3d np1 = p1/p1[2]; 
		Eigen::Vector3d np2 = p2/p2[2];
		np1(2) = ip.s; //p1[2]; 

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
	if(it <= 1)
	{
	    cout<<"iter = "<<it<<endl;
	    cout<<"matAtA: "<<endl<<matAtA<<endl;
	    cout<<"matAtB: "<<endl<<matAtB<<endl;
	    cout<<"matX: "<<endl<<matX<<endl; 
	}
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

