#include"ros/ros.h"
#include <vn300/ins.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/String.h"
#include <sstream>

ros::Publisher pub_navsatfix;

void ins_cb(const vn300::ins& msg);

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ins2gpsfix_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_ins;
    pub_navsatfix=nh.advertise<sensor_msgs::NavSatFix>("vectornav/fix",1000);
    sub_ins = nh.subscribe("vectornav/ins",1000,ins_cb);
    ros::spin();
}
void ins_cb(const vn300::ins& msg)
{
    sensor_msgs::NavSatFix pose;
    pose.latitude=msg.LLA.x;
    pose.longitude =msg.LLA.y;
    pose.altitude = msg.LLA.z;
    /*ENU*/
    pose.position_covariance[1] =msg.PosUncertainty;
    
    pub_navsatfix.publish(pose);
}
