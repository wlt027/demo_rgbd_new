/*
    Aug. 9 2018, He Zhang, hzhang8@vcu.edu 
    
    A projection factor using euler angle Z-X-Y in ceres 
*/

#include "projection_euler-ZXY.h"

namespace EulerZXY{

ProjectionFactor_Y2::ProjectionFactor_Y2(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j): 
pts_i(_pts_i), pts_j(_pts_j)
{
    sqrt_info = 1.; 
    scale = 10; 
}

bool ProjectionFactor_Y2::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const 
{
    double srx = sin(parameters[0][0]); // roll 
    double crx = cos(parameters[0][0]);
    double sry = sin(parameters[0][1]); // pitch
    double cry = cos(parameters[0][1]);
    double srz = sin(parameters[0][2]); // yaw
    double crz = cos(parameters[0][2]);
    double tx = parameters[0][3];  // x
    double ty = parameters[0][4];  // y
    double tz = parameters[0][5];  // z

    double u0 = pts_i(0); double v0 = pts_i(1); 
    double u1 = pts_j(0); double v1 = pts_j(1); 

    double yy2 = (ty - tz*v1)*(crz*sry + cry*srx*srz) - (tx - tz*u1)*(sry*srz - cry*crz*srx) 
	- v0*(srx*(ty*u1 - tx*v1) + crx*crz*(tx - tz*u1) + crx*srz*(ty - tz*v1)) 
	+ u0*((ty - tz*v1)*(cry*crz - srx*sry*srz) - (tx - tz*u1)*(cry*srz + crz*srx*sry) 
		+ crx*sry*(ty*u1 - tx*v1)) - crx*cry*(ty*u1 - tx*v1);
    
    residuals[0] = sqrt_info * scale * yy2; 
    if(jacobians && jacobians[0])
    {
	// dy2_droll 
	jacobians[0][0] = v0*(crz*srx*(tx - tz*u1) - crx*(ty*u1 - tx*v1) + srz*srx*(ty - tz*v1)) 
	    - u0*(sry*srx*(ty*u1 - tx*v1) + crz*sry*crx*(tx - tz*u1) + sry*srz*crx*(ty - tz*v1)) 
	    + cry*srx*(ty*u1 - tx*v1) + cry*crz*crx*(tx - tz*u1) + cry*srz*crx*(ty - tz*v1);

	// dy2_dpitch
	jacobians[0][1] = u0*((tx - tz*u1)*(srz*sry - crz*srx*cry) - (ty - tz*v1)*(crz*sry + srx*srz*cry) 
		+ crx*cry*(ty*u1 - tx*v1)) - (tx - tz*u1)*(srz*cry + crz*srx*sry) 
	    + (ty - tz*v1)*(crz*cry - srx*srz*sry) + crx*sry*(ty*u1 - tx*v1);

	// dy2_dyaw
	jacobians[0][2] = -u0*((tx - tz*u1)*(cry*crz - srx*sry*srz) + (ty - tz*v1)*(cry*srz + srx*sry*crz)) 
	    - (tx - tz*u1)*(sry*crz + cry*srx*srz) - (ty - tz*v1)*(sry*srz - cry*srx*crz) 
	    - v0*(crx*crz*(ty - tz*v1) - crx*srz*(tx - tz*u1));

	// dy2_dx
	jacobians[0][3] = cry*crz*srx - v0*(crx*crz - srx*v1) - u0*(cry*srz + crz*srx*sry + crx*sry*v1) 
	    - sry*srz + crx*cry*v1;

	// dy2_dy
	jacobians[0][4] = crz*sry - v0*(crx*srz + srx*u1) + u0*(cry*crz + crx*sry*u1 - srx*sry*srz) 
	    - crx*cry*u1 + cry*srx*srz;

	// dy2_dz
	jacobians[0][5] = u1*(sry*srz - cry*crz*srx) - v1*(crz*sry + cry*srx*srz) + u0*(u1*(cry*srz + crz*srx*sry) 
		- v1*(cry*crz - srx*sry*srz)) + v0*(crx*crz*u1 + crx*srz*v1);
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
    double srx = sin(parameters[0][0]); // roll 
    double crx = cos(parameters[0][0]);
    double sry = sin(parameters[0][1]); // pitch
    double cry = cos(parameters[0][1]);
    double srz = sin(parameters[0][2]); // yaw
    double crz = cos(parameters[0][2]);
    double tx = parameters[0][3];  // x
    double ty = parameters[0][4];  // y
    double tz = parameters[0][5];  // z

    double u0 = pts_i(0); double v0 = pts_i(1);  double d0 = pts_i(2); 
    double u1 = pts_j(0); double v1 = pts_j(1); 

    double y3 = tx - tz*u1 + d0*(crz*sry - crx*cry*u1 + cry*srx*srz) - d0*v0*(crx*srz + srx*u1) 
	+ d0*u0*(cry*crz + crx*sry*u1 - srx*sry*srz);
    
    *residuals = sqrt_info * y3; 

    if(jacobians && jacobians[0])
    {
	jacobians[0][0] = d0*(cry*srz*crx + cry*u1*srx) - d0*u0*(sry*srz*crx + sry*u1*srx) - d0*v0*(u1*crx - srz*srx);
	jacobians[0][1] = d0*(crz*cry + crx*u1*sry - srx*srz*sry) - d0*u0*(crz*sry - crx*u1*cry + srx*srz*cry);
	jacobians[0][2] = -d0*(sry*srz - cry*srx*crz) - d0*u0*(cry*srz + srx*sry*crz) - crx*d0*v0*crz;
	jacobians[0][3] = 1; 
	jacobians[0][4] = 0;
	jacobians[0][5] = -u1; 
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
    double srx = sin(parameters[0][0]); // roll 
    double crx = cos(parameters[0][0]);
    double sry = sin(parameters[0][1]); // pitch
    double cry = cos(parameters[0][1]);
    double srz = sin(parameters[0][2]); // yaw
    double crz = cos(parameters[0][2]);
    double tx = parameters[0][3];  // x
    double ty = parameters[0][4];  // y
    double tz = parameters[0][5];  // z

    double u0 = pts_i(0); double v0 = pts_i(1);  double d0 = pts_i(2); 
    double u1 = pts_j(0); double v1 = pts_j(1); 
    
    double y4 = ty - tz*v1 - d0*(cry*crz*srx - sry*srz + crx*cry*v1) + d0*v0*(crx*crz - srx*v1) 
    + d0*u0*(cry*srz + crz*srx*sry + crx*sry*v1);
   
    *residuals = sqrt_info * y4; 

    if(jacobians && jacobians[0])
    {
	jacobians[0][0] = d0*(cry*v1*srx - cry*crz*crx) + d0*u0*(crz*sry*crx - sry*v1*srx) - d0*v0*(crz*srx + v1*crx);
	jacobians[0][1] = d0*(srz*cry + crz*srx*sry + crx*v1*sry) + d0*u0*(crz*srx*cry - srz*sry + crx*v1*cry);
	jacobians[0][2] = d0*(sry*crz + cry*srx*srz) + d0*u0*(cry*crz - srx*sry*srz) - crx*d0*v0*srz;
	jacobians[0][3] = 0; 
	jacobians[0][4] = 1;
	jacobians[0][5] = -v1; 
    }
    return true; 
}


bool PoseLocalPrameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    for(int i=0; i<6; i++)
	*(x_plus_delta+i) = *(x+i) + *(delta+i);
    return true; 
}

bool PoseLocalPrameterization::ComputeJacobian(const double* x, double *jacobians) const
{
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > j(jacobians); 
    j.topRows<6>().setIdentity(); 
    return true; 
}




}
