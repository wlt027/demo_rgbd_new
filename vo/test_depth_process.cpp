/*
    test process, especially point cloud downsample

*/


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <ros/ros.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <vector>

using namespace std; 

mutex img_buf_mutex; 
vector<sensor_msgs::Image::ConstPtr> img_buf; 
condition_variable con; 

void syncCloudHandler(const sensor_msgs::Image::ConstPtr& syncCloud2);

void receiver(const sensor_msgs::Image::ConstPtr& syncCloud2);

void test_pcd();

void handler(); 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_depth"); 
    ros::NodeHandle nh; 

    ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::Image>
 	("/camera/depth_registered/image", 1, syncCloudHandler);
 
    // ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::Image>
//	 ("/camera/depth_registered/image", 10, receiver);
 
    // handler(); 
    // std::thread process{handler}; 

    ros::spin(); 
       
    return 1; 
}

void handler()
{
    while(ros::ok())
    {
	vector<sensor_msgs::Image::ConstPtr> img_pc2; 
	unique_lock<std::mutex> lk(img_buf_mutex); 
	con.wait(lk, [&]{
		img_pc2.swap(img_buf); 
		return img_pc2.size() > 0; 
	    });
	lk.unlock(); 
	cout <<"handler get "<<img_pc2.size()<<" pc msg!"<<endl; 
	for(int i=0; i<img_pc2.size(); i++)
	{
	    syncCloudHandler(img_pc2[i]); 
	}
    }
}

void receiver(const sensor_msgs::Image::ConstPtr& syncCloud2)
{
    img_buf_mutex.lock(); 
    img_buf.push_back(syncCloud2); 
    img_buf_mutex.unlock();
    con.notify_one(); 
}

void syncCloudHandler(const sensor_msgs::Image::ConstPtr& syncCloud2)
{
    double time = syncCloud2->header.stamp.toSec();
    
    static pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud3(new pcl::PointCloud<pcl::PointXYZI>());

    tempCloud3->clear();
    pcl::PointXYZI point;
    double cloudDSRate = 5.; 
    double kDepth[9] = {525., 0., 319.5, 0, 525.0, 239.5, 0, 0, 1}; 
    int imageWidth = 640;
    int imageHeight = 480; 
    double halfDSRate = cloudDSRate / 2.0 - 0.5;
    const float* syncCloud2Pointer = reinterpret_cast<const float*>(&syncCloud2->data[0]);
    for (double i = halfDSRate; i < 480; i += cloudDSRate) {
	for (double j = halfDSRate; j < 640; j += cloudDSRate) {
	    int pixelCount = 0;
	    float val, valSum = 0;
	    int is = int(i - halfDSRate), ie = int(i + halfDSRate);
	    int js = int(j - halfDSRate), je = int(j + halfDSRate);
	    for (int ii = is; ii <= ie; ii++) {
		for (int jj = js; jj <= je; jj++) {
		    val = syncCloud2Pointer[ii * imageWidth + jj];
		    if (val > 0.3 && val < 7) {
			valSum += val;
			pixelCount++;
		    }
		}
	    }

	    if (pixelCount > 0) {
		double ud = (kDepth[2] - j) / kDepth[0];
		double vd = (kDepth[5] - i) / kDepth[4];

		val = valSum / pixelCount;

		point.z = val;
		point.x = ud * val;
		point.y = vd * val;
		point.intensity = 10; // timeLasted;

		tempCloud3->push_back(point);
	    }
	}
    }


    // pcl::PointCloud<pcl::PointXYZI>::Ptr syncCloudPointer = syncCloudArray[syncCloudInd];
    // syncCloudPointer->clear();

    static pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud4(new pcl::PointCloud<pcl::PointXYZI>());
    // tempCloud4.swap(tempCloud3);
    cout <<"test_depth: at timestamp: "<<std::fixed<<syncCloud2->header.stamp.toSec()<<" get "<<tempCloud3->points.size()<<" depth points! "<<endl;
    cout <<"tempCloud3 has width: "<<tempCloud3->width<<" height: "<<tempCloud3->height<<endl;
//    static int pcd_cnt = 1; 
//    stringstream ss; 
//    ss <<pcd_cnt++<<".pcd"; 
//    pcl::io::savePCDFile(ss.str().c_str(), *tempCloud4);
//
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(tempCloud3);
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    // downSizeFilter.filter(*syncCloudPointer);
    downSizeFilter.filter(*tempCloud4);
    cout <<"test_depth: syncCloudPointer "<<std::fixed<<tempCloud4->points.size()<<" depth points!"<<endl;
}



void test_pcd()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::io::loadPCDFile("1.pcd", *pc); 
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr down_pc1(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZI>); 

    {
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
	downSizeFilter.setInputCloud(pc);
	downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
	downSizeFilter.filter(*down_pc1);
	cout <<"test_depth_process: after downsample "<<down_pc1->points.size()<<" depth points!"<<endl;
	pcl::io::savePCDFile("down_pc1.pcd", *down_pc1);
    }
    {
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
	downSizeFilter.setInputCloud(down_pc1);
	downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
	downSizeFilter.filter(*depthCloud);
	int depthCloudNum = depthCloud->points.size();
	cout <<"after downsample depth cloud has "<<depthCloudNum<<endl;
	pcl::io::savePCDFile("depthCloud.pcd", *depthCloud);
    }

}
