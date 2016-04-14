#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h" // To publish the simple occurrence of an event
#include <iostream>
#include <cerrno> // errno
#include <unistd.h> // usleep()
//#include "KeyValueMessage.h"
#include "noize_robot/Headlights.h"
#include "noize_robot/FrontContactSwitches.h"

using namespace std;

string version = "V2016-04-07";
const double ros_rate_inHz = 100.0;
ros::Publisher headlightsPub;

void OnFrontContactSwitches(const noize_robot::FrontContactSwitches::ConstPtr& msg)
{
	noize_robot::Headlights headlightsMsg;
	headlightsMsg.leftRed = 0;	headlightsMsg.leftGreen = 0;	headlightsMsg.leftBlue = 0;
	headlightsMsg.rightRed = 0;	headlightsMsg.rightGreen = 0;	headlightsMsg.rightBlue = 0;
	if (msg->leftContactSwitch)
		headlightsMsg.leftRed = 255;
	if (msg->rightContactSwitch)
		headlightsMsg.rightRed = 255;
	headlightsPub.publish(headlightsMsg);
	usleep(200000);
	headlightsMsg.leftRed = 0;	headlightsMsg.leftGreen = 0;	headlightsMsg.leftBlue = 0;
	headlightsMsg.rightRed = 0;	headlightsMsg.rightGreen = 0;	headlightsMsg.rightBlue = 0;
	headlightsPub.publish(headlightsMsg);
}

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "collision_manager");
	ros::NodeHandle nodeHdl;
	ros::Subscriber frontContactSwitchSub = nodeHdl.subscribe("FrontContactSwitches", 1000, OnFrontContactSwitches);
	headlightsPub = nodeHdl.advertise<noize_robot::Headlights>("Headlights", 1000);
	
	ROS_INFO_STREAM("collision_manager.cpp main() version = " << version);
	
	// ROS loop
	ros::Rate loop_rate(ros_rate_inHz);
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
}
