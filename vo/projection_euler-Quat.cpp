/*
    Aug. 9 2018, He Zhang, hzhang8@vcu.edu 
    
    A projection factor using quaternion in ceres 
*/

#include "projection_euler-Quat.h"

namespace QUATERNION{

ProjectionFactor_Y2::ProjectionFactor_Y2(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j): 
pts_i(_pts_i), pts_j(_pts_j)
{
    sqrt_info = 1.; 
    scale = 10; 
}

bool ProjectionFactor_Y2::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const 
{
    double tx = parameters[0][0];  // x
    double ty = parameters[0][1];  // y
    double tz = parameters[0][2];  // z
    double qx = parameters[0][3];  // qx
    double qy = parameters[0][4];  // qy
    double qz = parameters[0][5];  // qz
    double qw = parameters[0][6];  // qw
    
    Eigen::Quaterniond q(qx, qy, qz, qw); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); 
    double u1 = pts_j(0); double v1 = pts_j(1); 
    
    double tmp1 = -tz * v1 + ty; 
    double tmp2 =  u1 * tz - tx;
    double tmp3 = -u1 * ty + v1 * tx;
    
    Eigen::Vector3d X0(u0, v0, 1.); 
    Eigen::Vector3d tmp0 = R * X0; 
    double y2 = tmp1* tmp0(0) + tmp2*tmp0(1) + tmp3*tmp0(2); 
  
    residuals[0] = sqrt_info * scale * y2; 
    if(jacobians && jacobians[0])
    {
	// dy2_dx
	jacobians[0][0] = -tmp0(1) + v1*tmp0(2); 
	// dy2_dy 
	jacobians[0][1] = tmp0(0) - u1*tmp0(2);
	// dy2_dz
	jacobians[0][2] = -v1*tmp0(0) + u1*tmp0(1); 

	// dy2_dq
	Eigen::Vector3d dy_dq; 
	Eigen::Vector3d tp1(tmp1, tmp2, tmp3); 
	Eigen::Matrix3d dtmp0_dq = R * -Utility::skewSymmetric(X0); 
	dy_dq = tp1.transpose() * dtmp0_dq;
	jacobians[0][3] = dy_dq(0);
	jacobians[0][4] = dy_dq(1);
	jacobians[0][5] = dy_dq(2); 
	
	for(int j=0; j<6; j++)
	    jacobians[0][j] *= scale; 
    }
    return true; 
}

ProjectionFactor_Y3::ProjectionFactor_Y3(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j):
pts_i(_pts_i), pts_j(_pts_j)
{
    sqrt_info = 1.; 
}

bool ProjectionFactor_Y3::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    double tx = parameters[0][0];  // x
    double ty = parameters[0][1];  // y
    double tz = parameters[0][2];  // z
    double qx = parameters[0][3];  // qx
    double qy = parameters[0][4];  // qy
    double qz = parameters[0][5];  // qz
    double qw = parameters[0][6];  // qw
    
    Eigen::Quaterniond q(qx, qy, qz, qw); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); double d0 = pts_i(2);
    double u1 = pts_j(0); double v1 = pts_j(1); 
 
    Eigen::Vector3d X0(u0*d0, v0*d0, d0); 
    Eigen::Matrix<double, 1, 3> tmp = R.row(0) - u1*R.row(2); 
    
    double y3 = tmp * X0 + tx - u1*tz; 
  
    *residuals = sqrt_info * y3; 
    
    if(jacobians && jacobians[0])
    {
	// dy3_dx
	jacobians[0][0] = 1; 
	// dy3_dy
	jacobians[0][1] = 0; 
	// dy3_dz
	jacobians[0][2] = -u1; 

	// dy3_dq
	Eigen::Matrix<double, 1, 3> dy_dq = -R.row(0)*Utility::skewSymmetric(X0) + u1*R.row(2)*Utility::skewSymmetric(X0);

	jacobians[0][3] = dy_dq(0); 
	jacobians[0][4] = dy_dq(1);
	jacobians[0][5] = dy_dq(2); 
    }
    return true; 
}

ProjectionFactor_Y4::ProjectionFactor_Y4(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j):
pts_i(_pts_i), pts_j(_pts_j)
{
    sqrt_info = 1.; 
}

bool ProjectionFactor_Y4::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    double tx = parameters[0][0];  // x
    double ty = parameters[0][1];  // y
    double tz = parameters[0][2];  // z
    double qx = parameters[0][3];  // qx
    double qy = parameters[0][4];  // qy
    double qz = parameters[0][5];  // qz
    double qw = parameters[0][6];  // qw
    
    Eigen::Quaterniond q(qx, qy, qz, qw); 
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix(); 

    double u0 = pts_i(0); double v0 = pts_i(1); double d0 = pts_i(2);
    double u1 = pts_j(0); double v1 = pts_j(1); 
 
    Eigen::Vector3d X0(u0*d0, v0*d0, d0); 
    Eigen::Matrix<double, 1, 3> tmp = R.row(1) - v1*R.row(2); 
    double y4 = tmp * X0 + ty - v1*tz; 

    if(jacobians && jacobians[0])
    {
	// dy4_dx/dy/dz
	jacobians[0][0] = 0;	
	jacobians[0][1] = 1;	
	jacobians[0][2] = -v1; 	
	// dy4_dq
	Eigen::Matrix<double, 1, 3> dy_dq = -R.row(1)*Utility::skewSymmetric(X0) + v1 * R.row(2) * Utility::skewSymmetric(X0); 
	
	jacobians[0][3] = dy_dq(0); 
	jacobians[0][4] = dy_dq(1);
	jacobians[0][5] = dy_dq(2); 
    }
    return true; 
}


bool PoseLocalPrameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();
    // for(int i=0; i<6; i++)
    //	*(x_plus_delta+i) = *(x+i) + *(delta+i);
    return true; 
}

bool PoseLocalPrameterization::ComputeJacobian(const double* x, double *jacobians) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > j(jacobians); 
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero(); 
    return true; 
}




}
