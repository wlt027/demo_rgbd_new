/*
    Aug. 21 2018, He Zhang, hzhang8@vcu.edu 
    
    vio: tightly couple features [no depth, triangulated, measured] 
    with IMU integration 

*/

#pragma once

#include <vector>

using namespace std; 

const static int WN = 2; 
const static int SIZE_POSE = 7;
const static int SIZE_FEATURE = 1;
const static int SIZE_SPEEDBIAS = 9; 
const static int NUM_OF_FEAT = 1000; // Maximum
const static int NUM_OF_CAM = 1; 

class VIO
{
public:
    VIO(); 
    ~VIO(); 

    void processIMU(double t, Vector3d & linear_acceleration, Vector3d& angular_velocity); 

    void processImage()

    void solveOdometry(); 
    void initialize(); 
    
    volatile bool mbInited;  

    Matrix3d ric[NUM_OF_CAM]; 
    Vector3d tic[NUM_OF_CAM]; 
    
    Vector3d Ps[WN+1]; 
    Vector3d Vs[WN+1]; 
    Matrix3d Rs[WN+1]; 
    Vector3d Bas[WN+1]; 
    Vector3d Bgs[WN+1]; 
    
    bool mbFirstIMU; 
    Vector3d acc_0; 
    Vector3d gyr_0; 
    
    IntegrationBase * pre_integrations[WN+1]; 
    vector<double> dt_buf[WN+1]; 
    vector<Vector3d> linear_acceleration_buf[WN+1]; 
    vector<Vector3d> angular_velocity_buf[WN+1]; 
    
    int frame_count; 
    FeatureManager f_manager; 
    
    double para_Pose[WN+1][SIZE_POSE];
    double para_SpeedBias[WN+1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_FEAT][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    // double para_Retrive_Pose[SIZE_POSE];
};
