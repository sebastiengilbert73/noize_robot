#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h" // To publish the simple occurrence of an event
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cerrno> // errno
#include <unistd.h> // usleep()
#include "KeyValueMessage.h"
#include "noize_robot/Headlights.h"

using namespace std;

string version = "2016-04-07";

string serialPortName = "/dev/ttyAMA0";
int serialFileDesc = -1;

const double ros_rate_inHz = 100.0;
const char endOfMessageChar = '|';
KeyValueMessage interpret;
bool writeLock = false;


void SendHeadlightsToArduino(const noize_robot::Headlights::ConstPtr& msg)
{
	while(writeLock)
	{
		usleep(1000);
	}
	writeLock = true;
	stringstream msgStrm;	
	msgStrm << "headlights=on leftRed=" << (uint32_t) msg->leftRed << " leftGreen=" << (uint32_t) msg->leftGreen << " leftBlue=" << (uint32_t) msg->leftBlue << " rightRed=" << (uint32_t) msg->rightRed << " rightGreen=" << (uint32_t) msg->rightGreen << " rightBlue=" << (uint32_t) msg->rightBlue << " " << endOfMessageChar;
	//msgStrm << "headlights=on leftRed=255 leftGreen=0 leftBlue=0 rightRed=0 rightGreen=255 rightBlue=0 |";
	ROS_INFO_STREAM("SendHeadlightsToArduino(): msgStrm.str().c_str() = " << msgStrm.str().c_str());
	serialPuts(serialFileDesc, msgStrm.str().c_str());
	writeLock = false;
}

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "arduino_talker");
	ros::NodeHandle nodeHdl;
	ros::Subscriber headlightsSub = nodeHdl.subscribe("Headlights", 1000, SendHeadlightsToArduino);
	
	ROS_INFO_STREAM("arduino_talker.cpp main() version = " << version);
	
	// Setup wiringPi
	wiringPiSetup();
	serialFileDesc = serialOpen(serialPortName.c_str(), 115200);
	if (serialFileDesc == -1)
	{
		stringstream msg;
		msg << "arduino_listener.cpp main(): Error opening serial port " << serialPortName << ". errno = " << strerror(errno);
		ROS_ERROR_STREAM(msg.str());
	}
	
	// ROS loop
	ros::Rate loop_rate(ros_rate_inHz);
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
}