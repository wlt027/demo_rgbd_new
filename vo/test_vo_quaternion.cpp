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

void vo_pnp(std_3d& pts1, std_3d& pts2, Eigen::Matrix<double, 7, 1>& );
Eigen::Vector7d vo_ceres(std_3d& pts1, std_3d& pts2, Eigen::Vector7d& inipose);

double Fy2(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);
double Fy3(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);
double Fy4(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Matrix<double, 6, 1>* J = NULL);

Eigen::Matrix<double, 6, 1> jacobian_numeric(Eigen::Matrix<double, 7, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2, 
	double (*f)(Eigen::Matrix<double, 7, 1>& , Eigen::Vector3d& , Eigen::Vector3d&, Eigen::Matrix<double, 6, 1>*)  ); 

Eigen::Vector6d poseFromQuat2Euler(Eigen::Vector7d& );

void test_vo();
void test_y2_y3_y4(); 

int main(int argc, char* argv[])
{
    test_vo();
    // test_y2_y3_y4(); 
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
	Eigen::Vector7d pose; 
	double rpy[3] = {2, 3, 10}; 
	double vt[3] = {0.2, 0.1, -0.2}; 

	// double rpy[3] = {rangle(gen), rangle(gen), rangle(gen)}; 
	// double vt[3] = {rdis(gen), rdis(gen), rdis(gen)}; 
	// euler_pose << D2R(rpy[0]), D2R(rpy[1]), D2R(rpy[2]), vt[0], vt[1], vt[2]; 
	// cout <<"gt pose: "<<endl<<pose<<endl; 
	Eigen::Vector3d yrp(rpy[2], rpy[0], rpy[1]); 
	Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
	Eigen::Vector3d t(vt[0], vt[1], vt[2]);
	Eigen::Quaterniond q(R); 
	pose << t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w();
	Eigen::Vector6d euler_pose = poseFromQuat2Euler(pose); 

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
	Eigen::Vector6d pnp_vo_e = poseFromQuat2Euler(pnp_vo); 

	// Eigen::Vector7d delta_p = pnp_vo - pose; 
	Eigen::Vector6d delta_p = pnp_vo_e - euler_pose; 

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
	Eigen::Vector7d vo_p = vo_ceres(PT1, PT2, inipose);
	Eigen::Vector6d vo_p_e = poseFromQuat2Euler(vo_p); 
	// cout <<"ceres vo_p: "<<endl<<vo_p<<endl; 
	// delta_p = vo_p - pose; 
	delta_p = vo_p_e - euler_pose; 
	// cout <<"err_norm: "<<delta_p.norm()<<endl;
	err_vo_ceres += delta_p.norm(); 
    }
    cout <<"mean err_pnp: "<<err_pnp/NUM<<endl; 
   //  cout <<"mean err_demo: "<<err_vo_demo/NUM<<endl; 
    cout <<"mean err_ceres: "<<err_vo_ceres/NUM<<endl;

    return ;
}

Eigen::Vector7d vo_ceres(std_3d& PT1, std_3d& PT2, Eigen::Vector7d& inipose)
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
    ceres::LocalParameterization *local_param = new PoseLocalPrameterization; 
    problem.AddParameterBlock(para_pose[0], 7, local_param); 
    // problem.AddParameterBlock(para_pose[0], 6); 

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

    Eigen::Vector7d vo_p; 
    for(int i=0; i<7; i++)
	vo_p[i] = para_pose[0][i]; 
    
    Eigen::Quaterniond q(vo_p[6], vo_p[3], vo_p[4], vo_p[5]); 
    q.normalize(); 
    vo_p[3] = q.x(); vo_p[4] = q.y(); vo_p[5] = q.z(); vo_p[6] = q.w();
    return vo_p; 

}




void test_y2_y3_y4()
{
    bool test_y2 = false; // true; 
    bool test_y3 = false;
    bool test_y4 = true; 
    Eigen::Vector7d pose; 
    double rpy[3] = {2, 3, 10}; 
    double vt[3] = {0.2, 0.1, -0.2}; 

    Eigen::Vector3d yrp(rpy[2], rpy[0], rpy[1]); 
    Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
    Eigen::Vector3d t(vt[0], vt[1], vt[2]);
    Eigen::Quaterniond q(R); 
    q.normalize(); 
    pose << t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w(); 
    
    double roll = D2R(rpy[0]); 
    double pitch = D2R(rpy[1]); 
    double yaw = D2R(rpy[2]); 
    double srx = sin(roll); // roll 
    double crx = cos(roll);
    double sry = sin(pitch); // pitch
    double cry = cos(pitch);
    double srz = sin(yaw); // yaw
    double crz = cos(yaw);

    double tx = t(0); double ty = t(1); double tz = t(2); 
   // generate two point sets 
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::uniform_real_distribution<> uni_dis(-1.0, 1.0);
    std::uniform_real_distribution<> uni_z(1., 5.); 
    std::normal_distribution<> noise{0,0.02};

    for(int i=0; i<3; )
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
	Eigen::Vector3d np1 = p1/p1(2); 
	Eigen::Vector3d np2 = p2/p2(2); 
	double u0 = np1(0); double v0 = np1(1); 
	double u1 = np2(0); double v1 = np2(1);
	double d0 = p1(2); 
	Eigen::Vector6d J; 

	if(test_y4)
	{
	    Eigen::Vector3d nnp1(np1(0), np1(1), d0); 
	    double y4 = Fy4(pose, nnp1, np2, &J); 

	    double yy4 = ty - tz*v1 - d0*(cry*crz*srx - sry*srz + crx*cry*v1) + d0*v0*(crx*crz - srx*v1) 
		+ d0*u0*(cry*srz + crz*srx*sry + crx*sry*v1);
	    cout <<"y4 = "<<y4<<" yy4 = "<<yy4<<endl;
	    
	    // jacobians 
	    cout <<"jacobians: "<<endl<<J<<endl; 
	    Eigen::Vector6d J_num = jacobian_numeric(pose, nnp1, np2, Fy4); 
	    cout <<"jacobians_numeric: "<<endl<<J_num<<endl;
	}

	if(test_y3)
	{
	    Eigen::Vector3d nnp1(np1(0), np1(1), d0); 
	    double y3 = Fy3(pose, nnp1, np2, &J); 
	    
	    double yy3 = tx - tz*u1 + d0*(crz*sry - crx*cry*u1 + cry*srx*srz) - d0*v0*(crx*srz + srx*u1) 
	+ d0*u0*(cry*crz + crx*sry*u1 - srx*sry*srz);

	    cout <<"y3 = "<<y3<<" yy3 = "<<yy3<<endl;
	    
	    // jacobians 
	    // cout <<"jacobians: "<<endl<<J<<endl; 
	    // Eigen::Vector6d J_num = jacobian_numeric(pose, nnp1, np2, Fy3); 
	    // cout <<"jacobians_numeric: "<<endl<<J_num<<endl;
	}

	if(test_y2)
	{ 
	    double y2 = Fy2(pose, np1, np2, &J); 
	    double yy2 = (ty - tz*v1)*(crz*sry + cry*srx*srz) - (tx - tz*u1)*(sry*srz - cry*crz*srx) 
		- v0*(srx*(ty*u1 - tx*v1) + crx*crz*(tx - tz*u1) + crx*srz*(ty - tz*v1)) 
		+ u0*((ty - tz*v1)*(cry*crz - srx*sry*srz) - (tx - tz*u1)*(cry*srz + crz*srx*sry) 
			+ crx*sry*(ty*u1 - tx*v1)) - crx*cry*(ty*u1 - tx*v1);

	    cout <<"y2 = "<<y2<<" yy2 = "<<yy2<<endl; 
	    // jacobians 
	    cout <<"jacobians: "<<endl<<J<<endl; 
	    Eigen::Vector6d J_num = jacobian_numeric(pose, np1, np2, Fy2); 
	    cout <<"jacobian_numeric: "<<endl<<J_num<<endl;
	}
	i++;
    }


}

Eigen::Vector6d jacobian_numeric(Eigen::Matrix<double, 7, 1>& pose, Eigen::Vector3d& np1, Eigen::Vector3d& np2, 
	double (*F)(Eigen::Matrix<double, 7, 1>& , Eigen::Vector3d& , Eigen::Vector3d&, Eigen::Matrix<double, 6, 1>*)  )
{
    Eigen::Vector6d J_num; 
    double eps = 1e-4; 
    double v = F(pose, np1, np2, NULL); 
    for(int i=0; i<3; i++)
    {
	Eigen::Vector7d tmp = pose;
	tmp(i) += eps; 
	double tmp_v = (*F)(tmp, np1, np2, NULL); 
	J_num(i) = (tmp_v-v)/eps; 
    }

    Eigen::Quaterniond q(pose(6), pose(3), pose(4), pose(5)); // qw qx qy qz
    for(int i=3; i<6; i++)
    {
	Eigen::Vector3d delta_q = Eigen::Vector3d(i==3, i==4, i==5) * eps; 
	Eigen::Quaterniond q2 = q*Utility::deltaQ(delta_q); 
	q2.normalize(); 
	Eigen::Vector7d tmp = pose; 
	tmp(3) = q2.x(); tmp(4) = q2.y(); tmp(5) = q2.z(); tmp(6) = q2.w(); 
	double tmp_v = F(tmp, np1, np2, NULL); 
	J_num(i) = (tmp_v- v)/eps; 
    }
    return J_num; 
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
    q.normalize(); 
    pose(3) = q.x(); pose(4) = q.y(); pose(5) = q.z(); pose(6) = q.w(); 
    return ;
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

