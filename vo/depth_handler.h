/*
    Aug. 10 2018, He Zhang, hzhang8@vcu.edu 

    A handler to process depth data 
*/

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/tf.h>

// template<int CLOUD_NUM>
const static int CLOUD_NUM = 5;
class DepthHandler
{
public:
    DepthHandler();
    ~DepthHandler(); 
    void cloudHandler(const sensor_msgs::Image::ConstPtr& dpt_img); 
    void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData);

    int mCloudCnt; 	// number of cloud 
    int mCloudSkip; 	// number of frames be skipped
    double mInitTime; 	// initialTime 
    int mSyncCloudId;   // synchronized pc id
    int mRegCloudId; 	// registered pc id 
    int mCloudDSRate; 	// point cloud dense rate
    double mZoomDis; 	// zoomed point cloud 
    double mCloudStamp[CLOUD_NUM];
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > mCloudArray[CLOUD_NUM];
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > mCloudRec; // recorded PointCloud 
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > mCloudPub; // published PointCloud
    double mTimeRec; 

    // depth camera param 
    double mk[4]; // fx fy cx cy
    
    // tf last transform 
    tf::Transform mLastPose; 
};