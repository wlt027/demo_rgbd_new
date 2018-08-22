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
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *depthPointsPubPointer = NULL;
ros::Publisher *imagePointsProjPubPointer = NULL;
ros::Publisher *imageShowPubPointer;

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
}

void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
    m_dpt_buf.lock(); 
    dpt_buf.push(depthCloud2); 
    m_dpt_buf.unlock(); 
    con_dpt.notify_one(); 
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh("~");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(nh);
    vio.setParameter();
    
    ROS_WARN("waiting for image and imu...");

    // registerPub(n);

    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;

    ros::Subscriber sub_imu = nh.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);
    
    ros::Subscriber imagePointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/image_points_last", 5, imagePointsHandler);

    ros::Subscriber depthCloudSub = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/depth_cloud", 5, depthCloudHandler);

    ros::Publisher imagePointsProjPub = nh.advertise<sensor_msgs::PointCloud2> ("/image_points_proj", 1);
    imagePointsProjPubPointer = &imagePointsProjPub;

    // ros::Subscriber imageDataSub = nh.subscribe<sensor_msgs::Image>("/image/show", 1, imageDataHandler);

    // ros::Publisher imageShowPub = nh.advertise<sensor_msgs::Image>("/image/show_2", 1);
    // imageShowPubPointer = &imageShowPub;

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
	    ROS_DEBUG("vio_node.cpp: vo cost %f ms", whole_t); 
	    sum_vo_t += whole_t; 
	    ROS_DEBUG("vio_node.cpp: average vo cost %f ms", sum_vo_t/(++sum_vo_cnt));

	    // PUB MSG? 
	}
    }
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


