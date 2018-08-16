/*
    Aug. 15 2018 He Zhang, hzhang8@vcu.edu 
    
    test visual odometry using quaternion 

*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <random>
#include <fstream>
#include <opencv/cv.h>
#include "../utility/utility.h"

#include "projection_quat.h"

#define D2R(d) (((d)*M_PI)/180.)
#define SQ(x) ((x)*(x))

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > std_3d;

using namespace std; 
using namespace QUATERNION; 
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; // roll, pitch, yaw, x, y, z 
    typedef Matrix<double, 7, 1> Vector7d; // x, y, z, qx, qy, qz, qw
}

struct ip_M
{
    typedef enum{NO_DEPTH =0, DEPTH_MES, DEPTH_TRI, INVALID} DPT_TYPE;
    float ui, vi, uj, vj, s; // s responds to Xi = [ui,vi,1] * si
    int ind;
    DPT_TYPE v; 
};

struct Jacob_Q{
    union 
    {
	struct
	{
	    float de_dx; // tx 
	    float de_dy; // ty 
	    float de_dz; // tz
	    float de_dq1; // theta1  
	    float de_dq2; // theta2
	    float de_dq3; // theta3
	    float de_dq4;
	};
	float de[7]; 
    };
    float err;
};

Eigen::Vector7d vo_ceres(vector<ip_M>& , Eigen::Vector7d& inipose);
Eigen::Vector7d vo_demo(vector<ip_M>&, Eigen::Vector7d& inipose);

double Fy2(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);
double Fy3(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);
double Fy4(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);

Eigen::Vector6d poseFromQuat2Euler(Eigen::Vector7d& );

string log_file("iprelations.log"); 

void read_iprelation_log(vector<ip_M>& vip, string log_file); 

void test_vo();

int main(int argc, char* argv[])
{
    if(argc >=2)
	log_file = string(argv[1]);
    test_vo();
} 

void test_vo()
{	
    vector<ip_M> vip; 
    read_iprelation_log(vip, log_file); 
    Eigen::Vector7d inipose; inipose<< 0, 0, 0, 0, 0, 0, 1; 
    Eigen::Vector7d inc_vo = vo_demo(vip, inipose); 
    Eigen::Vector6d inc_vo_euler = poseFromQuat2Euler(inc_vo); 
    cout <<"demo_rgbd inc_vo: "<<endl<<inc_vo<<endl;
    cout <<"demo_rgbd inc_vo_euler: "<<endl<<inc_vo_euler<<endl;
    inc_vo = vo_ceres(vip, inipose); 
    inc_vo_euler = poseFromQuat2Euler(inc_vo); 
    cout <<"demo_ceres inc_vo: "<<endl<<inc_vo<<endl;
    cout <<"demo_ceres inc_vo_euler: "<<endl<<inc_vo_euler<<endl;


    return ;
}

Eigen::Vector7d vo_demo(vector<ip_M>& vip, Eigen::Vector7d& inipose)
{
    int N = vip.size(); 
    int iterNum = 150;
    double scale = 100;
    
    vector<Jacob_Q> vjq; 
    double sum_e = 0; 
    Eigen::Vector7d vo_p = inipose; 
    for(int it=0; it<iterNum; it++)
    {
	vjq.clear(); 
	sum_e = 0; 
	for(int i=0; i<N; i++)
	{
	    ip_M & pt = vip[i]; 
	    Eigen::Vector3d p1(pt.ui, pt.vi, 1.); 
	    Eigen::Vector3d p2(pt.uj, pt.vj, 1.); 
	    Eigen::Vector6d J; 

	    // first half for y2 
	    if(pt.v == ip_M::NO_DEPTH)
	    {
		Eigen::Vector3d np1 = p1/p1[2]; 
		Eigen::Vector3d np2 = p2/p2[2];

		double ey2 = Fy2(vo_p, np1, np2, &J); 
		Jacob_Q je; 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = scale * J(j); 
		}
		je.de[6] = 0; 
		je.err = scale * ey2; 
		vjq.push_back(je); 
		sum_e += SQ(je.err); 
	    }else if(pt.v == ip_M::DEPTH_MES || pt.v == ip_M::DEPTH_TRI)
	    {
		Eigen::Vector3d np1 = p1/p1[2]; 
		Eigen::Vector3d np2 = p2/p2[2];
		np1(2) = pt.s; 

		double ey3 = Fy3(vo_p, np1, np2, &J); 
		Jacob_Q je; 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.de[6] = 0; 
		je.err = ey3;
		vjq.push_back(je);
		sum_e += SQ(je.err); 
		double ey4 = Fy4(vo_p, np1, np2, &J); 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.de[6] = 0;
		je.err = ey4;
		vjq.push_back(je);
		sum_e += SQ(je.err); 
	    }
	}
	// try vo to compute the transformation 
	int N_JE = vjq.size(); 
	cv::Mat matA(N_JE, 7, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(7, N_JE, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(7, 7, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(N_JE, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(7, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(7, 1, CV_32F, cv::Scalar::all(0));

	for(int i=0; i<N_JE; i++)
	{
	    Jacob_Q& jq = vjq[i]; 
	    for(int j=0; j<7; j++)
		matA.at<float>(i, j) = jq.de[j]; 
	    matB.at<float>(i, 0) = -0.1*jq.err; 
	}
	cv::transpose(matA, matAt); 
	matAtA = matAt*matA; 
	matAtB = matAt * matB; 
	cout <<"iter "<<it<<endl;
	cout <<"matAtA: "<<endl<<matAtA<<endl; 
	cout <<"matAtB: "<<endl<<matAtB<<endl;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); 
	cout <<"matX: "<<endl<<matX<<endl;
	Eigen::Vector6d dmat; 
	for(int j=0; j<3; j++)
	{
	    vo_p(j) += matX.at<float>(j, 0); 
	    dmat(j) = matX.at<float>(j, 0); 
	}
	dmat(3) = matX.at<float>(3, 0); 
	dmat(4) = matX.at<float>(4, 0); 
	dmat(5) = matX.at<float>(5, 0); 
	
	Eigen::Vector3d theta(dmat(3), dmat(4), dmat(5)); 
	Eigen::Quaterniond dq = Utility::deltaQ(theta);
	// cout <<"dq: "<<dq.x()<<" "<<dq.y()<<" "<<dq.z()<<" "<<dq.w()<<endl;
	Eigen::Quaterniond q(vo_p(6), vo_p(3), vo_p(4), vo_p(5));
	q = (q*dq).normalized(); 
	vo_p(3) = q.x(); vo_p(4) = q.y(); vo_p(5) = q.z(); vo_p(6) = q.w(); 
	// cout <<"q: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
    
	if(dmat.norm() < 0.00001)
	    break; 
    }  
    return vo_p; 
}

Eigen::Vector7d vo_ceres(vector<ip_M>& vip, Eigen::Vector7d& inipose)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // these variables needs to be global or static 
    static double para_pose[0][7]; 

    int N = vip.size(); 
    for(int i=0; i<7; i++)
	para_pose[0][i] = inipose[i]; 

    // cout <<"start to construct structure! "<<endl; 
    ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
    problem.AddParameterBlock(para_pose[0], 7, local_param); 
    // problem.AddParameterBlock(para_pose[0], 6); 

    for(int i=0; i<N; i++)
    {
	ip_M& pt = vip[i]; 
	Eigen::Vector3d p1(pt.ui, pt.vi, 1.); 
	Eigen::Vector3d p2(pt.uj, pt.vj, 1.); 
	Eigen::Vector3d np1 = p1/p1[2]; 
	Eigen::Vector3d np2 = p2/p2[2];
	Eigen::Vector6d J; 

	if(pt.v == ip_M::NO_DEPTH)
	{
	    ProjectionFactor_Y2 * f= new ProjectionFactor_Y2(np1, np2); 
	    problem.AddResidualBlock(f, loss_function, para_pose[0]); 
	}else if(pt.v == ip_M::DEPTH_MES || pt.v == ip_M::DEPTH_TRI)
	{
	    np1[2] = pt.s; 
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

    Eigen::Vector7d vo_p; 
    for(int i=0; i<7; i++)
	vo_p[i] = para_pose[0][i]; 
    
    Eigen::Quaterniond q(vo_p[6], vo_p[3], vo_p[4], vo_p[5]); 
    q.normalize(); 
    vo_p[3] = q.x(); vo_p[4] = q.y(); vo_p[5] = q.z(); vo_p[6] = q.w();
    return vo_p; 
}

double Fy4(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 6, 1>* J )
{
    double tx = pose[0];  // x
    double ty = pose[1];  // y
    double tz = pose[2];  // z
    double qx = pose[3];  // qx
    double qy = pose[4];  // qy
    double qz = pose[5];  // qz
    double qw = pose[6];  // qw
    
    // Eigen::Quaterniond seq qw, qx, qy, qz
    Eigen::Quaterniond q(qw, qx, qy, qz); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); double d0 = pts_i(2);  
    double u1 = pts_j(0); double v1 = pts_j(1); 
    double tmp1 = -tz * v1 + ty; 
    double tmp2 =  u1 * tz - tx;
    double tmp3 = -u1 * ty + v1 * tx;

    Eigen::Vector3d X0(u0*d0, v0*d0, d0); 
    Eigen::Matrix<double, 1, 3> tmp = R.row(1) - v1*R.row(2);
    double y4 = tmp * X0 + ty - v1*tz; 
    if(J)
    {
	(*J)(0) = 0; 
	(*J)(1) = 1; 
	(*J)(2) = -v1; 
	// dy4_dq
	Eigen::Matrix<double, 1, 3> dy_dq = -R.row(1)*Utility::skewSymmetric(X0) + v1 * R.row(2) * Utility::skewSymmetric(X0);
	(*J)(3) = dy_dq(0); 
	(*J)(4) = dy_dq(1); 
	(*J)(5) = dy_dq(2); 
    }
    return y4;
}


double Fy3(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 6, 1>* J )
{
    double tx = pose[0];  // x
    double ty = pose[1];  // y
    double tz = pose[2];  // z
    double qx = pose[3];  // qx
    double qy = pose[4];  // qy
    double qz = pose[5];  // qz
    double qw = pose[6];  // qw
    
    // Eigen::Quaterniond seq qw, qx, qy, qz
    Eigen::Quaterniond q(qw, qx, qy, qz); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); double d0 = pts_i(2); 
    double u1 = pts_j(0); double v1 = pts_j(1); 
    double tmp1 = -tz * v1 + ty; 
    double tmp2 =  u1 * tz - tx;
    double tmp3 = -u1 * ty + v1 * tx;

    Eigen::Vector3d X0(u0*d0, v0*d0, d0); 
    Eigen::Matrix<double, 1, 3> tmp = R.row(0) - u1*R.row(2); 
      
    double y3 = tmp * X0 + tx - u1*tz; 
    if(J)
    {
	(*J)(0) = 1; 
	(*J)(1) = 0; 
	(*J)(2) = -u1; 
	// dy3_dq
	Eigen::Matrix<double, 1, 3> dy_dq = -R.row(0)*Utility::skewSymmetric(X0) + u1*R.row(2)*Utility::skewSymmetric(X0);
	(*J)(3) = dy_dq(0); 
	(*J)(4) = dy_dq(1); 
	(*J)(5) = dy_dq(2); 
    }
    return y3;
}


double Fy2(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& pts_i, Eigen::Vector3d& pts_j, Eigen::Matrix<double, 6, 1>* J)
{
    double tx = pose[0];  // x
    double ty = pose[1];  // y
    double tz = pose[2];  // z
    double qx = pose[3];  // qx
    double qy = pose[4];  // qy
    double qz = pose[5];  // qz
    double qw = pose[6];  // qw
    
    // Eigen::Quaterniond seq qw, qx, qy, qz
    Eigen::Quaterniond q(qw, qx, qy, qz); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); 
    double u1 = pts_j(0); double v1 = pts_j(1); 
    double tmp1 = -tz * v1 + ty; 
    double tmp2 =  u1 * tz - tx;
    double tmp3 = -u1 * ty + v1 * tx;

    Eigen::Vector3d X0(u0, v0, 1.); 
    Eigen::Vector3d tmp0 = R * X0; 
    double y2 = tmp1* tmp0(0) + tmp2*tmp0(1) + tmp3*tmp0(2); 
    
//    cout <<"in Fy2"<<endl;
//    cout <<"X0: "<<endl<<X0<<endl;
//    cout <<"R: "<<endl<<R<<endl;
//    cout <<"tmp0 "<<endl<<tmp0<<endl; 
//    // cout <<"tmp1: "<<tmp1<<" tmp2: "<<tmp2<<" tmp3: "<<tmp3<<endl;

    if(J)
    {
	(*J)(0) = -tmp0(1) + v1*tmp0(2); 
	(*J)(1) = tmp0(0) - u1*tmp0(2);
	(*J)(2) = -v1*tmp0(0) + u1*tmp0(1); 
	
	// dy2_dq
	Eigen::Vector3d dy_dq; 
	Eigen::Vector3d tp1(tmp1, tmp2, tmp3); 
	Eigen::Matrix3d dtmp0_dq = R * -Utility::skewSymmetric(X0); 
	dy_dq = tp1.transpose() * dtmp0_dq;
	(*J)(3) = dy_dq(0); 
	(*J)(4) = dy_dq(1); 
	(*J)(5) = dy_dq(2);
    }
    return y2;
}

Eigen::Vector6d poseFromQuat2Euler(Eigen::Vector7d& p)
{
    Eigen::Vector6d ret; 
    Eigen::Quaterniond q(p(6), p(3), p(4), p(5)); 
    Eigen::Matrix<double , 3, 3> R = q.toRotationMatrix(); 
    Eigen::Vector3d ypr = Utility::R2ypr(R); 
    ret << D2R(ypr(2)), D2R(ypr(1)), D2R(ypr(0)), p(0), p(1), p(2); 
    return ret;
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



