/*

    Aug. 13 2018, He Zhang, hzhang8@vcu.edu
    
    stereo triangulate 

*/

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/tf.h>

extern void fromTF2Eigen(tf::Transform & T, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t); 
extern void fromEigen2TF(tf::Transform & T, Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 1>& t);

extern Eigen::Vector3d stereo_triangulate(Eigen::Matrix<double, 3, 4>& proj_m1, Eigen::Matrix<double, 3, 4>& proj_m2, 
					    Eigen::Vector2d& pt1, Eigen::Vector2d& pt2); 
			
extern Eigen::Vector3d stereo_triangulate(tf::Transform& T1 , tf::Transform& T2, 
					    Eigen::Vector2d& pt1, Eigen::Vector2d& pt2); 

