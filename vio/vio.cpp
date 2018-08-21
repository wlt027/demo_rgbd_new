/*
    Aug. 21 2018, He Zhang, hzhang8@vcu.edu 
    
    vio: tightly couple features [no depth, triangulated, measured] 
    with IMU integration 

*/

#include "vio.h"

VIO::VIO():
mbFirstIMU(true),
frame_count(0)
{}
VIO::~VIO(){}

void VIO::clearState()
{
    for(int i=0; i<WN + 1; i++)
    {
	linear_acceleration_buf[i].clear(); 
	angular_velocity_buf[i].clear(); 
	if(pre_integrations[i] != NULL)
	{
	    delete pre_integrations[i]; 
	}
	pre_integrations[i] = NULL; 
    }
}

void VIO::processIMU(double t, Vector3d & linear_acceleration, Vector3d& angular_velocity)
{
    if(mbFirstIMU)
    {
	mbFirstIMU = false; 
	acc_0 = linear_acceleration; 
	gyr_0 = angular_velocity; 
    }
    
    if(!pre_integrations[frame_count])
    {
    
    }
}


