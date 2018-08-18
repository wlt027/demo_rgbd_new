/*

    Aug. 13 2018, He Zhang, hzhang8@vcu.edu
    
    stereo triangulate 

*/

#include "stereo_triangulate.h"


void fromTF2Eigen(tf::Transform & T, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t)
{
    tf::Quaternion q = T.getRotation(); 
    tf::Vector3 tt = T.getOrigin(); 
    tf::Matrix3x3 tR(q); 
    for(int i=0; i<3; i++)
	for(int j=0; j<3; j++)
	{
	    R(i,j) = tR[i][j];
	}
    t << tt.getX(), tt.getY(), tt.getZ(); 
    return ;
}

void fromEigen2TF(tf::Transform & T, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t)
{
    Eigen::Quaterniond q(R); 
    tf::Quaternion tq(q.x(), q.y(), q.z(), q.w()); 
    tf::Vector3 tt(t(0), t(1), t(2));
    T = tf::Transform(tq, tt); 
    return ; 
}

Eigen::Vector3d stereo_triangulate(tf::Transform& T1 , tf::Transform& T2, 
					    Eigen::Vector2d& pt1, Eigen::Vector2d& pt2)
{
    Eigen::Matrix<double, 3, 3> Ri, Rj; 
    Eigen::Matrix<double, 3, 1> ti, tj; 
    Eigen::Matrix<double, 3, 4> Ti = Eigen::MatrixXd::Identity(3, 4); 
    Eigen::Matrix<double, 3, 4> Tj = Eigen::MatrixXd::Identity(3, 4); 
    fromTF2Eigen(T1, Ri, ti);
    fromTF2Eigen(T2, Rj, tj);
    Ti.block<3,3>(0,0) = Ri; Ti.block<3,1>(0,3) = ti; 
    Tj.block<3,3>(0,0) = Rj; Tj.block<3,1>(0,3) = tj; 
    return stereo_triangulate(Ti, Tj, pt1, pt2); 
}
					


Eigen::Vector3d stereo_triangulate(Eigen::Matrix<double, 3, 4>& proj_m1, Eigen::Matrix<double, 3, 4>& proj_m2, 
					    Eigen::Vector2d& pt1, Eigen::Vector2d& pt2)
{
    Eigen::Matrix<double, 6, 4> A; 
    double x1 = pt1(0); 
    double y1 = pt1(1); 
    double x2 = pt2(0); 
    double y2 = pt2(1); 
    
    for(int k=0; k<4; k++)
    {
	A(0,k) = x1 * proj_m1(2, k) - proj_m1(0, k); 
	A(1,k) = y1 * proj_m1(2, k) - proj_m1(1, k);
	A(2,k) = x1 * proj_m1(1, k) - y1 * proj_m1(0, k); 
	A(3,k) = x2 * proj_m2(2, k) - proj_m2(0, k); 
	A(4,k) = y2 * proj_m2(2, k) - proj_m2(1, k); 
	A(5,k) = x2 * proj_m2(1, k) - y2 * proj_m2(0, k);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV); 
    Eigen::Matrix<double, 4, 4> V = svd.matrixV(); 
    
    for(int i=0; i<3; i++)
	V(i, 3) /= V(3, 3);
    return V.block<3,1>(0,3);
}

