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
#include "utility.h"

#define D2R(d) (((d)*M_PI)/180.)

using namespace std; 
namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; 
}
double Fy2(Eigen::Matrix<double, 6, 1>& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2); 
// double Fy3(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2); 
// double Fy4(Eigen::Vector6d& pose, Eigen::Vector3d& p1, Eigen::Vector3d& p2); 

void testy2(); 

int main(int argc, char* argv[])
{
     
    testy2();
    

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
    
    cout <<"R: "<<endl<<R<<endl;  

    Eigen::Vector3d p2 = R*p1 + t; 
    Eigen::Vector3d np1 = p1/p1[2]; 
    Eigen::Vector3d np2 = p2/p2[2]; 
    double yy2 = Fy2(pose, np1, np2); 
    cout <<"np1: "<<np1<<endl;
    cout <<"p2: " <<p2<<endl; 
    cout <<"np2: "<<np2<<endl;
    cout <<"y2 = "<<yy2<<endl; 
}


double Fy2(Eigen::Matrix<double, 6, 1>& transform, Eigen::Vector3d& p1, Eigen::Vector3d& p2)
{
    double srx = sin(transform(0)); // roll 
    double crx = cos(transform(0));
    double sry = sin(transform[1]); // pitch
    double cry = cos(transform[1]);
    double srz = sin(transform[2]); // yaw
    double crz = cos(transform[2]);
    double tx = transform[3];  // x
    double ty = transform[4];  // y
    double tz = transform[5];  // z

    double u0 = p1(0); double v0 = p1(1); 
    double u1 = p2(0); double v1 = p2(1); 

    Eigen::Matrix<double, 3, 3> R; 
    R << crz*cry - srz*srx*sry, -crx*srz, crz*sry + cry*srz*srx, 
	 cry*srz + crz*srx*sry, crz*crx,  srz*sry - crz*cry*srx,
	 -crx*sry , srx, crx*cry; 
    cout<<"R: "<<endl<<R<<endl;

//    ipr2.x = v0*(crz*srx*(tx - tz*u1) - crx*(ty*u1 - tx*v1) + srz*srx*(ty - tz*v1)) 
//	- u0*(sry*srx*(ty*u1 - tx*v1) + crz*sry*crx*(tx - tz*u1) + sry*srz*crx*(ty - tz*v1)) 
//	+ cry*srx*(ty*u1 - tx*v1) + cry*crz*crx*(tx - tz*u1) + cry*srz*crx*(ty - tz*v1);
//
//    ipr2.y = u0*((tx - tz*u1)*(srz*sry - crz*srx*cry) - (ty - tz*v1)*(crz*sry + srx*srz*cry) 
//	    + crx*cry*(ty*u1 - tx*v1)) - (tx - tz*u1)*(srz*cry + crz*srx*sry) 
//	+ (ty - tz*v1)*(crz*cry - srx*srz*sry) + crx*sry*(ty*u1 - tx*v1);
//
//    ipr2.z = -u0*((tx - tz*u1)*(cry*crz - srx*sry*srz) + (ty - tz*v1)*(cry*srz + srx*sry*crz)) 
//	- (tx - tz*u1)*(sry*crz + cry*srx*srz) - (ty - tz*v1)*(sry*srz - cry*srx*crz) 
//	- v0*(crx*crz*(ty - tz*v1) - crx*srz*(tx - tz*u1));
//
//    ipr2.h = cry*crz*srx - v0*(crx*crz - srx*v1) - u0*(cry*srz + crz*srx*sry + crx*sry*v1) 
//	- sry*srz + crx*cry*v1;
//
//    ipr2.s = crz*sry - v0*(crx*srz + srx*u1) + u0*(cry*crz + crx*sry*u1 - srx*sry*srz) 
//	- crx*cry*u1 + cry*srx*srz;
//
//    ipr2.v = u1*(sry*srz - cry*crz*srx) - v1*(crz*sry + cry*srx*srz) + u0*(u1*(cry*srz + crz*srx*sry) 
//	    - v1*(cry*crz - srx*sry*srz)) + v0*(crx*crz*u1 + crx*srz*v1);

    double yy2 = (ty - tz*v1)*(crz*sry + cry*srx*srz) - (tx - tz*u1)*(sry*srz - cry*crz*srx) 
	- v0*(srx*(ty*u1 - tx*v1) + crx*crz*(tx - tz*u1) + crx*srz*(ty - tz*v1)) 
	+ u0*((ty - tz*v1)*(cry*crz - srx*sry*srz) - (tx - tz*u1)*(cry*srz + crz*srx*sry) 
		+ crx*sry*(ty*u1 - tx*v1)) - crx*cry*(ty*u1 - tx*v1);

    return yy2; 
}


