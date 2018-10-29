/*
    Oct. 27 2018, He Zhang, hzhang8@vcu.edu 

    Read trajectory file 
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>


#include <chrono>
#include <algorithm>
#include <sys/stat.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <ros/ros.h>
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
  	np.param("datd_file", traj_file, traj_file); 
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  	if(argc >= 2)
  		traj_file = argv[1]; 
  	ROS_DEBUG("read_traj: traj_file %s", traj_file.c_str());

    ros::Publisher traj_pub = nh.advertise<std_msgs::Float32MultiArray>("/pub_2d_path", 1000); 
    vector<Pose> traj_path = read_traj(traj_file); 

    if(traj_path.size() > 0)
    {
    	ROS_WARN("read_traj: press any key + [ENTER] to start publish path!"); 
    	int k; 
    	cin>>k;
    }

    // ready to publish it 
    ros::Rate r(50);
    Pose last_p; 
    for(int i=0; i<traj_path.size() && ros::ok(); i++)
    {
    	Pose p = traj_path[i]; 
    	last_p  = p;
 		// prepare msg 
    	std_msgs::Float32MultiArray pt_msg; 
    	pt_msg.data.resize(2); 
    	pt_msg.data[0] = p.x; 
    	pt_msg.data[1] = p.y; 
    	if(i > 0 && close(last_p, p))
    		continue; 
    	traj_pub.publish(pt_msg); 
    	ROS_DEBUG("read_traj: publish %i pose %f %f", i, p.x, p.y);
    	ros::spinOnce(); 
    	r.sleep();  
    	last_p = p; 
    }


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
      ROS_INFO("s = %s", s.c_str());
      stringstream ss;
      ss << s;
      string t; 
      ss >> p.timestamp;
      // mv_timestamp.push_back(t.substr(0,15));
      ss >> p.x>>p.y>>p.z>>p.qw>>p.qx>>p.qy>>p.qz; 
      ret.push_back(p); 
      ROS_DEBUG("read_traj: %s %f %f %f %f %f %f %f", p.timestamp, p.x, p.y, p.z, p.qw, p.qx, p.qy, p.qz); 
    }
  }
  ROS_INFO("read_traj: path has %d poses", ret.size()); 
  return ret; 
}


