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
#include "noize_robot/TrackMotors.h"
#include "noize_robot/CameraAzimuth.h"

using namespace std;

string version = "2016-04-14";

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

void SendTrackMotorsToArduino(const noize_robot::TrackMotors::ConstPtr& msg)
{
	while (writeLock)
	{
		usleep(1000);
	}
	writeLock = true;
	stringstream msgStrm;
	msgStrm << "leftMotor=" << msg->leftMotor << " rightMotor=" << msg->rightMotor << " " << endOfMessageChar;
	ROS_INFO_STREAM("SendTrackMotorsToArduino(): msgStrm.str().c_str() = " << msgStrm.str().c_str());
	serialPuts(serialFileDesc, msgStrm.str().c_str());
	writeLock = false;
}

void SendCameraAzimuthToArduino(const noize_robot::CameraAzimuth::ConstPtr& msgPtr)
{
	while (writeLock)
	{
		usleep(1000);
	}
	writeLock = true;
	stringstream msgStrm;
	if (msgPtr->lookCenter)
	{
		msgStrm << "lookCenter=true " << endOfMessageChar;
		serialPuts(serialFileDesc, msgStrm.str().c_str());
	}
	else
	{
		if (msgPtr->deltaTheta < 0)
			msgStrm << "lookLeft=" << -(msgPtr->deltaTheta) << " " << endOfMessageChar;
		else
			msgStrm << "lookRight=" << msgPtr->deltaTheta << " " << endOfMessageChar;
		serialPuts(serialFileDesc, msgStrm.str().c_str());
	}
	ROS_INFO_STREAM("SendCameraAzimuthToArduino(): msgStrm.str().c_str() = " << msgStrm.str().c_str());
	writeLock = false;
}

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "arduino_talker");
	ros::NodeHandle nodeHdl;
	ros::Subscriber headlightsSub = nodeHdl.subscribe("Headlights", 1000, SendHeadlightsToArduino);
	ros::Subscriber trackMotorsSub = nodeHdl.subscribe("TrackMotors", 1000, SendTrackMotorsToArduino);
	ros::Subscriber cameraAzimuthSum = nodeHdl.subscribe("CameraAzimuth", 1000, SendCameraAzimuthToArduino);
	
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
