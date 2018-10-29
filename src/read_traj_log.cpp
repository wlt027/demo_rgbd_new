/*
    Oct. 27 2018, He Zhang, hzhang8@vcu.edu 

    Read trajectory file 
*/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>


#include <chrono>
#include <algorithm>
#include <sys/stat.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <vector>

using namespace std; 

struct Pose
{
	string timestamp; 
	float x, y, z; 
	float qx, qy, qz, qw; 
};

string traj_file(""); 

vector<Pose> read_traj(string filename); 

bool close(Pose& p1, Pose& p2)
{
	float norm = sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y*p2.y));
	return norm < 0.1; 
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "read_traj"); 
    ros::start(); 
    ros::NodeHandle nh; 

    ros::NodeHandle np("~"); 
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc >= 2)
	traj_file = argv[1]; 
    np.param("data_file", traj_file, traj_file); 

    ROS_DEBUG("read_traj: traj_file %s", traj_file.c_str());

    ros::Publisher traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/pub_2d_path", 1000); 
    vector<Pose> traj_path = read_traj(traj_file); 

    if(traj_path.size() > 0)
    {
	// ROS_WARN("read_traj: press any key + [ENTER] to start publish path!"); 
	// int k; 
	// cin>>k;
    }else
    {
	ROS_ERROR("read_traj_log: no path is generated!"); 
	return -1; 
    }

    // ready to publish it 
    ros::Rate r(50);
    Pose last_p; 
    vector<Pose> tmp; 
    tmp.reserve(traj_path.size()); 
    for(int i=0; i<traj_path.size() && ros::ok(); i++)
    {
	Pose& p = traj_path[i]; 
	// prepare msg 
	if(i > 0 && close(last_p, p))
	    continue; 
	tmp.push_back(p); 
	// ROS_DEBUG("read_traj: publish %i pose %f %f yaw: %f degree", i, p.x, p.y, 180.*yaw/M_PI);
	// ros::spinOnce(); 
	// r.sleep();  
	last_p = p; 
    }
    std_msgs::Float32MultiArray pt_msg; 
    pt_msg.data.resize(3*tmp.size()); 

    for(int i=0; i<tmp.size(); i++)
    {
	Pose& p = tmp[i]; 
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// float yaw = asin(2*(q.w()*q.y() - q.x()*q.z())); 
	float yaw = atan2(2*(p.qw*p.qz + p.qx*p.qy), 1-2*(p.qy*p.qy + p.qz*p.qz));  
	pt_msg.data[i*3+2] = yaw;
	pt_msg.data[i*3+0] = p.x; 
	pt_msg.data[i*3+1] = p.y; 
    }
    for(int i=0; i<5; i++)
    {
    ROS_DEBUG("read_traj: publish %i pose", tmp.size()); 
    traj_pub.publish(pt_msg); 
    ros::spinOnce(); 
    sleep(1);
    }
    ros::spin();

    return 0; 
}

vector<Pose> read_traj(string filename)
{
    vector<Pose> ret; 
    ifstream inf(filename.c_str());
    if(!inf.is_open()) 
    {
	ROS_ERROR("read_traj: failed to read trajectory file: %s", filename.c_str()); 
	return ret; 
    }


    while(!inf.eof())
    {
	string s;
	getline(inf,s);// 
	if(!s.empty())
	{
	    Pose p; 
	    replace(s.begin(), s.end(), ',', ' '); 
	    // ROS_INFO("s = %s", s.c_str());
	    stringstream ss;
	    ss << s;
	    string t; 
	    ss >> p.timestamp;
	    // mv_timestamp.push_back(t.substr(0,15));
	    ss >> p.x>>p.y>>p.z>>p.qw>>p.qx>>p.qy>>p.qz; 
	    ret.push_back(p); 
	    // ROS_DEBUG("read_traj: %s %f %f %f %f %f %f %f", p.timestamp.c_str(), p.x, p.y, p.z, p.qw, p.qx, p.qy, p.qz); 
	}
    }
    ROS_INFO("read_traj: path has %d poses", ret.size()); 
    return ret; 
}


