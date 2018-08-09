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
#include "utility.h"

#define D2R(d) (((d)*M_PI)/180.)

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

void testy2(); 
void test_vo(); 

int main(int argc, char* argv[])
{
    // testy2();
    test_vo(); 

} 

void test_vo()
{
    // generate transformation [R, t]
    Eigen::Vector6d pose; 
    pose << D2R(5), D2R(2), D2R(3), 0.1, 0.5, -0.2; 
    cout <<"gt pose: "<<endl<<pose<<endl; 
    Eigen::Vector3d yrp(3, 5, 2); 
    Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
    Eigen::Vector3d t(0.1, 0.5, -0.2);

    // generate two point sets 
    std::random_device rd{};
    std::mt19937 gen{rd()};

    int N = 40; 
    std_3d PT1(N); 
    std_3d PT2(N); 
    std::uniform_real_distribution<> uni_dis(-2.0, 2.0);
    std::uniform_real_distribution<> uni_z(1., 5.); 
    std::normal_distribution<> noise{0,0.02};
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

    int M = N/2; 
    Eigen::Vector6d inipose; 
    inipose << 0, 0, 0, 0, 0, 0; 

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
		sum_e += je.err; 
	    }else
	    {
		double ey3 = Fy3(vo_p, p1, p2, &J); 
		Jacob_E je; 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.err = ey3;
		vje.push_back(je);
		sum_e += je.err; 
		double ey4 = Fy4(vo_p, p1, p2, &J); 
		for(int j=0; j<6; j++)
		{
		    je.de[j] = J(j); 
		}
		je.err = ey4;
		vje.push_back(je);
		sum_e += je.err; 
	    }
	}

	cout <<"iterator i: "<<it<<" sum_err: "<<sum_e<<endl; 
	
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
    cout <<"estimate vo_p: "<<endl<<vo_p<<endl; 
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

