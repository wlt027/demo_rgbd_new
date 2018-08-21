/*
    Aug. 20 2018, He Zhang, hzhang8@vcu.edu 
    
    A projection factor using quaternion in ceres
*/

#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"

namespace QUATERNION_VIO 
{

class ProjectionFactor_Y2 : public ceres::SizedCostFunction<1, 7, 7, 7, 1>
{
  public:
    ProjectionFactor_Y2(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    // Eigen::Matrix<double, 2, 3> tangent_base;
    // Eigen::Matrix2d sqrt_info;
    double sqrt_info; 
    double scale; 
};

/*
class ProjectionFactor_Y3 : public ceres::SizedCostFunction<1, 7>
{
  public:
    ProjectionFactor_Y3(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    // void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    // Eigen::Matrix<double, 2, 3> tangent_base;
    // Eigen::Matrix2d sqrt_info;
    double sqrt_info; 
};

class ProjectionFactor_Y4 : public ceres::SizedCostFunction<1, 7>
{
  public:
    ProjectionFactor_Y4(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    // void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    // Eigen::Matrix<double, 2, 3> tangent_base;
    // Eigen::Matrix2d sqrt_info;
    double sqrt_info; 
};*/

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    Eigen::Matrix2d sqrt_info;
};

class PoseLocalPrameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

}
