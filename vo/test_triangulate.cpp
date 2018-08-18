/*
    test stereo triangulation using matched points

*/

#include <iostream>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "stereo_triangulate.h"
#include "../utility/utility.h"

using namespace std; 

#define D2R(d) (((d)*M_PI)/180.)
#define R2D(r) (((r)*180.)/M_PI)

namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d; 
    typedef Matrix<double, 7, 1> Vector7d; 
}

struct pt_M
{
    double ui, vi, uj, vj; 
    double pose[7]; // x, y, z, qx, qy, qz, qw 
};

void read_triangulate_log(vector<pt_M>& vip, string log_file); 
Eigen::Vector6d poseFromQuat2Euler(Eigen::Vector7d& p); 
Eigen::Vector7d poseFromEuler2Quat(Eigen::Vector6d& p); 

void test_triangulate(); 

double demo_triangulate(Eigen::Vector6d& transformSum, Eigen::Vector2d& pi, Eigen::Vector2d& pj);
double new_triangulate(Eigen::Vector7d& transformSum);

void test_triangulate2(); 

int main(int argc, char* argv[])
{
    // test_triangulate();

     test_triangulate2(); 
    
//    Eigen::Matrix<double, 7, 1> p; 
//    p << -0.099890, -0.009580,  0.030343, -0.001491, -0.064515, -0.021949, 0.997674;
//    cout <<"p: "<<endl<<p<<endl;
//    tf::Vector3 t(p(0), p(1), p(2)); 
//    tf::Quaternion q(p(3), p(4), p(5), p(6));
//    tf::Transform Tij(q,t);
//    Eigen::Matrix<double, 3, 3> R; 
//    Eigen::Vector3d et; 
//    fromTF2Eigen(Tij, R, et); 
//    cout << "t: "<<et<<endl;
//    tf::Transform Tij2;
//    fromEigen2TF(Tij2, R, et); 
//    t = Tij2.getOrigin(); 
//    q = Tij2.getRotation(); 
//    cout<<"t: "<<t.getX()<<" "<<t.getY()<<" "<<t.getZ()<<endl; 
//    cout <<"q: "<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<endl; 
//
//    return 1; 
}

void test_triangulate2()
{
    const std::vector<Eigen::Vector3d> points3D = {
    Eigen::Vector3d(0, 0.1, 0.1),
    Eigen::Vector3d(0, 1, 3),
    Eigen::Vector3d(0, 1, 2),
    Eigen::Vector3d(0.01, 0.2, 3),
    Eigen::Vector3d(-1, 0.1, 1),
    Eigen::Vector3d(0.1, 0.1, 0.2),
    };

    Eigen::Matrix<double, 7, 1> p; 
    p << -0.099890, -0.009580,  0.030343, -0.001491, -0.064515, -0.021949, 0.997674;
    Eigen::Vector6d peu = poseFromQuat2Euler(p); 
    peu(0) = -peu(0); peu(1) = -peu(1); 
    peu(3) = -peu(3); peu(4) = -peu(4); 
    {
	cout <<"test new_depth "<<endl;
	tf::Vector3 t(p(0), p(1), p(2)); 
	tf::Quaternion q(p(3), p(4), p(5), p(6));
	tf::Transform Tij(q,t);
	tf::Transform Tji = Tij.inverse();
	tf::Transform I(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
	for(int i=0; i<points3D.size(); i++)
	{
	    Eigen::Vector3d pj = points3D[i]; 
	    tf::Vector3 tpj(pj(0), pj(1), pj(2)); 
	    tf::Vector3 tpi = Tij * tpj; 
	    Eigen::Vector2d epi(tpi.getX()/tpi.getZ(), tpi.getY()/tpi.getZ()); 
	    Eigen::Vector2d epj(pj(0)/pj(2), pj(1)/pj(2)); 
	    Eigen::Vector3d pt = stereo_triangulate(Tij, I, epi, epj); 
	    cout <<i<<" estimate depth "<<pt(2)<<" gt depth: "<<points3D[i](2)<<endl; 
	}
    }

    {
	cout <<"test demo depth "<<endl; 
	tf::Vector3 t(p(0), p(1), p(2)); 
	tf::Quaternion q(p(3), p(4), p(5), p(6));
	tf::Transform Tij(q,t);

	for(int i=0; i<points3D.size(); i++)
	{
	    Eigen::Vector3d pj = points3D[i]; 
	    tf::Vector3 tpj(pj(0), pj(1), pj(2)); 
	    tf::Vector3 tpi = Tij * tpj; 
	    Eigen::Vector2d epi(-tpi.getX()/tpi.getZ(), -tpi.getY()/tpi.getZ()); 
	    Eigen::Vector2d epj(-pj(0)/pj(2), -pj(1)/pj(2)); 
	    // Eigen::Vector3d pt = stereo_triangulate(Tij, I, epi, epj); 
	    double depth = demo_triangulate(peu, epi, epj); 
	    cout <<i<<" estimate depth "<<depth<<" gt depth: "<<points3D[i](2)<<endl; 
	}

    }
}

void test_triangulate()
{
    Eigen::Matrix<double, 7, 1> p; 
    p << -0.099890, -0.009580,  0.030343, -0.001491, -0.064515, -0.021949, 0.997674;
    Eigen::Vector6d peu = poseFromQuat2Euler(p); 
    peu(0) = -peu(0); peu(1) = -peu(1); 
    peu(3) = -peu(3); peu(4) = -peu(4); 
    cout << "peu: "<<endl<<peu<<endl; 
    // cout <<" demo_depth: "<<demo_triangulate(peu)<<endl; 
    /*
    Eigen::Vector7d pq = poseFromEuler2Quat(peu);

    peu<<0.005448, 0.129629, -0.043880, 0.100651, 0.007736, 0.030722;
    pq = poseFromEuler2Quat(peu); 
    pq(0) = -pq(0); pq(1) = -pq(1); 
    pq(3) = -pq(3); pq(4) = -pq(4);

    cout <<"pq: "<<endl<<pq<<endl;
*/
    
    cout <<"new_depth: "<<new_triangulate(p)<<endl; 

    return ; 
}

double new_triangulate(Eigen::Vector7d & p)
{
    Eigen::Vector2d pi(-0.54, -0.391429); 
    Eigen::Vector2d pj(-0.355733, -0.398209); 
    tf::Vector3 t(p(0), p(1), p(2));
    tf::Quaternion q(p(3), p(4), p(5), p(6)); 
    tf::Transform Tij(q,t); 
    tf::Transform Tji = Tij.inverse(); 
    tf::Transform I(tf::Quaternion(0 ,0 ,0, 1), tf::Vector3(0, 0, 0));
    Eigen::Vector3d pt = stereo_triangulate(Tij, I, pi, pj); 
    return pt(2); 
}

double demo_triangulate(Eigen::Vector6d& transformSum, Eigen::Vector2d& pi, Eigen::Vector2d& pj)
{
//    double u0 = 0.54; //  startPointsLast->points[i].u;
//    double v0 = 0.391429; // startPointsLast->points[i].v;
//    double u1 = 0.355733; // ipr.x;
//    double v1 = 0.398209; // ipr.y;
//    
    double u0 = pi(0); 
    double v0 = pi(1); 
    double u1 = pj(0);
    double v1 = pj(1); 

    double srx0 = sin(0);
    double crx0 = cos(0);
    double sry0 = sin(0);
    double cry0 = cos(0);
    double srz0 = sin(0);
    double crz0 = cos(0);

    double srx1 = sin(-transformSum[0]);
    double crx1 = cos(-transformSum[0]);
    double sry1 = sin(-transformSum[1]);
    double cry1 = cos(-transformSum[1]);
    double srz1 = sin(-transformSum[2]);
    double crz1 = cos(-transformSum[2]);

    double tx0 = 0; // -startTransLast->points[i].h;
    double ty0 = 0; // -startTransLast->points[i].s;
    double tz0 = 0; // -startTransLast->points[i].v;

    double tx1 = -transformSum[3];
    double ty1 = -transformSum[4];
    double tz1 = -transformSum[5];

    double x1 = crz0 * u0 + srz0 * v0;
    double y1 = -srz0 * u0 + crz0 * v0;
    double z1 = 1;

    double x2 = x1;
    double y2 = crx0 * y1 + srx0 * z1;
    double z2 = -srx0 * y1 + crx0 * z1;

    double x3 = cry0 * x2 - sry0 * z2;
    double y3 = y2;
    double z3 = sry0 * x2 + cry0 * z2;

    double x4 = cry1 * x3 + sry1 * z3;
    double y4 = y3;
    double z4 = -sry1 * x3 + cry1 * z3;

    double x5 = x4;
    double y5 = crx1 * y4 - srx1 * z4;
    double z5 = srx1 * y4 + crx1 * z4;

    double x6 = crz1 * x5 - srz1 * y5;
    double y6 = srz1 * x5 + crz1 * y5;
    double z6 = z5;

    u0 = x6 / z6;
    v0 = y6 / z6;

    x1 = cry1 * (tx1 - tx0) + sry1 * (tz1 - tz0);
    y1 = ty1 - ty0;
    z1 = -sry1 * (tx1 - tx0) + cry1 * (tz1 - tz0);

    x2 = x1;
    y2 = crx1 * y1 - srx1 * z1;
    z2 = srx1 * y1 + crx1 * z1;

    double tx = crz1 * x2 - srz1 * y2;
    double ty = srz1 * x2 + crz1 * y2;
    double tz = z2;

    double delta = sqrt((v0 - v1) * (v0 - v1) + (u0 - u1) * (u0 - u1))
	* cos(atan2(tz * v1 - ty, tz * u1 - tx) - atan2(v0 - v1, u0 - u1));
    double depth = sqrt((tz * u0 - tx) * (tz * u0 - tx) + (tz * v0 - ty) * (tz * v0 - ty)) / delta;
    return depth; 
}

Eigen::Vector6d poseFromQuat2Euler(Eigen::Vector7d& p)
{
    Eigen::Vector6d ret; 
    Eigen::Quaterniond q(p(6), p(3), p(4), p(5)); 
    Eigen::Matrix<double , 3, 3> R = q.toRotationMatrix(); 
    cout <<"quat2euler, R: "<<endl<<R<<endl;
    Eigen::Vector3d ypr = Utility::R2ypr(R); 
    ret << D2R(ypr(2)), D2R(ypr(1)), D2R(ypr(0)), p(0), p(1), p(2); 
    return ret;
}

Eigen::Vector7d poseFromEuler2Quat(Eigen::Vector6d& p)
{
    Eigen::Vector3d yrp(R2D(p(2)), R2D(p(0)), R2D(p(1)));
    Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
    cout <<"euler2quat, R: "<<endl<<R<<endl;
    Eigen::Quaterniond q(R); 
    Eigen::Vector7d ret; 
    ret<<p(3), p(4), p(5), q.x(), q.y(), q.z(), q.w();
    return ret; 
}


void read_triangulate_log(vector<pt_M>& vip, string log_file)
{
/*
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
    */
} 
