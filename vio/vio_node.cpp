/*
    Aug. 18 2018, He Zhang, hzhang8@vcu.edu 
    
    vio node 

*/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "vio.h"
#include "parameters.h"
#include "../utility/tic_toc.h"

VIO vio; 
std::mutex m_buf;
std::mutex m_dpt_buf;

ros::Publisher *voDataPubPointer = NULL;
ros::Publisher *vioDataPubPointer = NULL;

ros::Publisher *depthPointsPubPointer = NULL;
ros::Publisher *imagePointsProjPubPointer = NULL;
ros::Publisher *obstaclePCPubPointer = NULL;
ros::Publisher *imageShowPubPointer = NULL;
ros::Publisher *floorPCPubPointer = NULL; 

tf::TransformBroadcaster * tfBroadcasterPointer = NULL; // camera_init to camera
tf::TransformBroadcaster * tfBroadcastTWI; // world to imu
tf::TransformBroadcaster * tfBroadcastTWC_init; // world to camera_init

std::condition_variable con;
std::condition_variable con_dpt; 
double current_time = -1;

double sum_vo_t = 0; 
int sum_vo_cnt = 0; 

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloud2ConstPtr> feature_buf;
queue<sensor_msgs::PointCloud2ConstPtr> dpt_buf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
    
    // cout <<"receive imu msg at "<<std::fixed<<imu_msg->header.stamp.toSec()<<endl; 

    {/* // Not need to publish state at imu's speed
	std::lock_guard<std::mutex> lg(m_state);
	predict(imu_msg);
	std_msgs::Header header = imu_msg->header;
	header.frame_id = "world";
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
	    pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    */
    }
}
void imagePointsHandler(const sensor_msgs::PointCloud2ConstPtr& imagePoints2)
{
    m_buf.lock();
    feature_buf.push(imagePoints2);
    m_buf.unlock();
    con.notify_one();
    // cout <<"receive img msg at "<<std::fixed<<imagePoints2->header.stamp.toSec()<<endl; 
}

void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    m_dpt_buf.lock(); 
    dpt_buf.push(depthCloud2); 
    m_dpt_buf.unlock(); 
    con_dpt.notify_one();
    // cout <<"receive dpt msg at "<<std::fixed<<depthCloud2->header.stamp.toSec()<<endl; 
}

void currDepthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    // cout<<"vio_node.cpp: succeed to get depthcloud at current stamp: "<<std::fixed<<depthCloud2->header.stamp.toSec()<<endl;
    vio.processCurrDepthCloud(depthCloud2);
}

void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double dx = imu_msg->linear_acceleration.x ;
    double dy = imu_msg->linear_acceleration.y ;
    double dz = imu_msg->linear_acceleration.z ;

    double rx = imu_msg->angular_velocity.x ;
    double ry = imu_msg->angular_velocity.y ;
    double rz = imu_msg->angular_velocity.z ;

    // estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
    Eigen::Vector3d acc(dx, dy, dz); 
    Eigen::Vector3d gyr(rx, ry, rz);
    vio.processIMU(dt, acc, gyr) ; 
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>>
getMeasurements();
void process(); 
void process_depthcloud();
void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData);
void depthDataHandler(const sensor_msgs::Image::ConstPtr& depthData); 
void publishMsg(sensor_msgs::PointCloud2ConstPtr& img_msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(nh);
    vio.setParameter();
    
    ROS_WARN("waiting for image and imu...");

    // registerPub(n);
    
    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;
    
    
    tf::TransformBroadcaster tfBroadcaster1;
    tfBroadcastTWI = &tfBroadcaster1;
    
    
    tf::TransformBroadcaster tfBroadcaster2;
    tfBroadcastTWC_init = &tfBroadcaster2;
    

    ros::Subscriber sub_imu = nh.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);
    
    ros::Subscriber imagePointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/image_points_last", 5, imagePointsHandler);

    ros::Subscriber depthCloudSub = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/depth_cloud", 5, depthCloudHandler); // transformed depth cloud for the last stamp 

    ros::Subscriber currDepthCloudSub = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/current_depth_cloud", 5, currDepthCloudHandler); // observed depth cloud for the curr stamp

    ros::Publisher imagePointsProjPub = nh.advertise<sensor_msgs::PointCloud2> ("/image_points_proj", 5);
    imagePointsProjPubPointer = &imagePointsProjPub;
    
    ros::Publisher pointcloudPub = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_point", 5);
    obstaclePCPubPointer = &pointcloudPub;

    ros::Publisher floorPCPub = nh.advertise<sensor_msgs::PointCloud2>("/floor_point", 5);
    floorPCPubPointer = &floorPCPub;

    ros::Publisher voDataPub = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
    voDataPubPointer = &voDataPub;
    
    ros::Publisher vioDataPub = nh.advertise<nav_msgs::Odometry>("/world_to_imu", 5); 
    vioDataPubPointer = &vioDataPub; 

    ros::Subscriber imageDataSub = nh.subscribe<sensor_msgs::Image>("/image/show", 1, imageDataHandler);

    ros::Publisher imageShowPub = nh.advertise<sensor_msgs::Image>("/image/show_2", 1);
    imageShowPubPointer = &imageShowPub;

    std::thread measurement_process{process};
    std::thread depthcloud_process{process_depthcloud}; 
    ros::spin();

    return 0;
}

void process_depthcloud()
{
    while(ros::ok())
    {
	vector<sensor_msgs::PointCloud2ConstPtr> depth_pc; 
	std::unique_lock<std::mutex> lk(m_dpt_buf); 
	con_dpt.wait(lk, [&]
	    {
		while(!dpt_buf.empty())
		{
		    depth_pc.emplace_back(dpt_buf.front());
		    dpt_buf.pop(); 
		}
		return depth_pc.size() > 0;
	    });
    lk.unlock();

	for(int i=0; i<depth_pc.size(); i++)
	    vio.processDepthCloud(depth_pc[i]);
    }
}


void process()
{
    while (ros::ok())
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

	for (auto &measurement : measurements)
	{
	    for (auto &imu_msg : measurement.first)
		send_imu(imu_msg);

	    auto img_msg = measurement.second;
	    ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

	    TicToc t_s;
	    // estimator.processImage(image, img_msg->header);
	    vio.processImage(img_msg); 
	    double whole_t = t_s.toc();
	    ROS_WARN("vio_node.cpp: vo cost %f ms", whole_t); 
	    sum_vo_t += whole_t; 
	    ROS_WARN("vio_node.cpp: average vo cost %f ms", sum_vo_t/(++sum_vo_cnt));

        publishMsg(img_msg); 
	}
    }
}


void publishMsg(sensor_msgs::PointCloud2ConstPtr& img_msg)
{
      // publish msg voData 
        nav_msgs::Odometry voData; 
        voData.header.frame_id = "/camera_init"; 
        voData.header.stamp = img_msg->header.stamp;
        voData.child_frame_id = "/camera";

        tf::Transform vo_to_init = vio.mInitCamPose.inverse() * vio.mCurrPose;  
        tf::Quaternion q = vo_to_init.getRotation(); 
        tf::Vector3 t = vo_to_init.getOrigin(); 
        voData.pose.pose.orientation.x = q.getX(); 
        voData.pose.pose.orientation.y = q.getY(); 
        voData.pose.pose.orientation.z = q.getZ(); 
        voData.pose.pose.orientation.w = q.getW(); 
        voData.pose.pose.position.x = t.getX(); 
        voData.pose.pose.position.y = t.getY();
        voData.pose.pose.position.z = t.getZ(); 
        voDataPubPointer->publish(voData);

        // cout<<"vo node at "<<std::fixed<<voData.header.stamp.toSec()<<" vo result: "<<t.getX()<<" "<<t.getY()<<" "<<t.getZ()<<endl;

        {
        // broadcast voTrans camera_init -> camera
        tf::StampedTransform voTrans;
        voTrans.frame_id_ = "/camera_init";
        voTrans.child_frame_id_ = "/camera";
        voTrans.stamp_ = img_msg->header.stamp;
        voTrans.setRotation(q); 
        voTrans.setOrigin(t); 
        tfBroadcasterPointer->sendTransform(voTrans); 
        }

        // publish vio result 
        q = vio.mCurrIMUPose.getRotation(); 
        t = vio.mCurrIMUPose.getOrigin(); 
        nav_msgs::Odometry vioData; 
        vioData.header.frame_id = "/world"; 
        vioData.header.stamp = img_msg->header.stamp;
        vioData.child_frame_id = "/imu";

        vioData.pose.pose.orientation.x = q.getX(); 
        vioData.pose.pose.orientation.y = q.getY(); 
        vioData.pose.pose.orientation.z = q.getZ(); 
        vioData.pose.pose.orientation.w = q.getW();
        vioData.pose.pose.position.x = t.getX(); 
        vioData.pose.pose.position.y = t.getY();
        vioData.pose.pose.position.z = t.getZ(); 
        vioDataPubPointer->publish(vioData);
        cout <<"vio publish: vio t "<<t.getX()<<" "<<t.getY() <<" "<<t.getZ()<<endl;
        cout <<"vio publish: vio q "<< q.getX()<<" "<< q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<endl;

        {
        // broadcast voTrans imu -> camera 
        tf::StampedTransform voTrans;
        voTrans.frame_id_ = "/world";
        voTrans.child_frame_id_ = "/camera_init";
        voTrans.stamp_ = img_msg->header.stamp;
        voTrans.setData(vio.mInitCamPose);
        tfBroadcastTWC_init->sendTransform(voTrans); 
        }


        {
        // broadcast voTrans imu -> camera 
        tf::StampedTransform voTrans;
        voTrans.frame_id_ = "/world";
        voTrans.child_frame_id_ = "/imu";
        voTrans.stamp_ = img_msg->header.stamp;
        voTrans.setData(vio.mCurrIMUPose);
        tfBroadcastTWI->sendTransform(voTrans); 
        }

        // deal with image
        {  
            // publish points with depth 
            sensor_msgs::PointCloud2 imagePointsProj2;
            pcl::toROSMsg(*(vio.mImagePointsProj), imagePointsProj2);
            imagePointsProj2.header.frame_id = "camera";
            imagePointsProj2.header.stamp = ros::Time().fromSec(vio.mTimeLast);
            imagePointsProjPubPointer->publish(imagePointsProj2);    
        }
        {
            // publish obstacle point 
            sensor_msgs::PointCloud2 obstaclePC2;
            pcl::toROSMsg(*(vio.mPCNoFloor), obstaclePC2);
            obstaclePC2.header.frame_id = "world";
            obstaclePC2.header.stamp = ros::Time().fromSec(vio.mTimeLast);
            obstaclePCPubPointer->publish(obstaclePC2);    
        }

        {
            // publish floor point 
            sensor_msgs::PointCloud2 floorPC2; 
            pcl::toROSMsg(*(vio.mPCFloor), floorPC2); 
            // pcl::toROSMsg(*(vio.mCurrPCFloor), floorPC2); 
            floorPC2.header.frame_id = "world"; 
            floorPC2.header.stamp = ros::Time().fromSec(vio.mTimeLast); 
            floorPCPubPointer->publish(floorPC2);
        }
        // cout <<"publish imagePointsProj2 with "<<imagePointsProj2.height * imagePointsProj2.width<<" points!"<<" at time "<<std::fixed<<vio.mTimeLast<<endl;

}



    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>> measurements;

    while (true)
    {
	if (imu_buf.empty() || feature_buf.empty())
	    return measurements;

	if ((imu_buf.back()->header.stamp < feature_buf.front()->header.stamp))
	{
	    ROS_WARN("wait for imu, only should happen at the beginning");
	    return measurements;
	}

	if ((imu_buf.front()->header.stamp > feature_buf.front()->header.stamp))
	{
	    ROS_WARN("throw img, only should happen at the beginning");
	    feature_buf.pop();
	    continue;
	}
	sensor_msgs::PointCloud2ConstPtr img_msg = feature_buf.front();
	feature_buf.pop();

	std::vector<sensor_msgs::ImuConstPtr> IMUs;
	while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
	{
	    IMUs.emplace_back(imu_buf.front());
	    imu_buf.pop();
	}

	measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}


void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData) 
{
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(imageData, "bgr8");

    cv::Mat show_img = ptr->image; 

    // double kImage[9] = {525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0};
    double kImage[9] = {617.306, 0.0, 326.245, 0.0, 617.714, 239.974, 0.0, 0.0, 1.0};
    double showDSRate = 2.;
    vector<ip_M> ipRelations = vio.mPtRelations; 
    int ipRelationsNum = ipRelations.size();
    // cout<<"vo_node display image at "<<std::fixed<<imageData->header.stamp.toSec()<<endl;
    for (int i = 0; i < ipRelationsNum; i++) 
    {
	ip_M pt = ipRelations[i];
	if ( pt.v == ip_M::NO_DEPTH) 
	{   
	    // cout<<"No depth: pt.uj = "<<(kImage[2] - pt.uj * kImage[0])<<" pt.vj: "<<(kImage[5] - pt.vj * kImage[4]) <<" pt.s = "<<pt.s<<endl;
	    cv::circle(show_img, cv::Point((kImage[2] + pt.uj * kImage[0]) / showDSRate, (kImage[5] + pt.vj * kImage[4]) / showDSRate), 1, CV_RGB(255, 0, 0), 2);
	} else if (pt.v == ip_M::DEPTH_MES) {
	    // cout<<"Depth MES: pt.uj = "<<(kImage[2] - pt.uj * kImage[0])<<" pt.vj: "<<(kImage[5] - pt.vj * kImage[4]) <<" pt.s = "<<pt.s<<endl;
	    cv::circle(show_img, cv::Point((kImage[2] + pt.uj * kImage[0]) / showDSRate,(kImage[5] + pt.vj * kImage[4]) / showDSRate), 1, CV_RGB(0, 255, 0), 2);
	} else if (pt.v == ip_M::DEPTH_TRI) {
	    // cout<<"Depth TRI: pt.uj = "<<(kImage[2] - pt.uj * kImage[0])<<" pt.vj: "<<(kImage[5] - pt.vj * kImage[4]) <<" pt.s = "<<pt.s<<endl;
	    cv::circle(show_img, cv::Point((kImage[2] + pt.uj * kImage[0]) / showDSRate,(kImage[5] + pt.vj * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 255), 2);
	} /*else {
	    cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
	    (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 0), 2);
	    }*/
    }
    ptr->image = show_img; 
    sensor_msgs::Image::Ptr imagePointer = ptr->toImageMsg();
    imageShowPubPointer->publish(imagePointer);
}

