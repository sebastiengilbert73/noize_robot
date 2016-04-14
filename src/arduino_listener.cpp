#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h" // To publish the simple occurrence of an event
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cerrno> // errno
#include <unistd.h> // usleep()
#include "KeyValueMessage.h"
#include "noize_robot/FrontContactSwitches.h"

using namespace std;

string serialPortName = "/dev/ttyAMA0";
string version = "2016-03-26";
const double ros_rate_inHz = 100.0;
const char endOfMessageChar = '|';
KeyValueMessage interpret;

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "arduino_listener");
	ros::NodeHandle nodeHdl;
	// Declare publishers
	ros::Publisher frontContactSwitch_pub = nodeHdl.advertise<noize_robot::FrontContactSwitches>("FrontContactSwitches", 1000); // To do: remplace Empty by specialized message
	
	ROS_INFO_STREAM("arduino_listener.cpp main() version = " << version);
	
	// Setup wiringPi
	wiringPiSetup();
	int serialFileDesc = serialOpen(serialPortName.c_str(), 115200);
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
		// Check if characters are available at the serial port
		stringstream msg;
		bool endOfMessageCharIsFound = false;
		while (serialDataAvail(serialFileDesc) > 0 && !endOfMessageCharIsFound)
		{
			char nextChar = serialGetchar(serialFileDesc);
			if (nextChar == endOfMessageChar)
			{
				endOfMessageCharIsFound = true;				
			}
			if (nextChar != endOfMessageChar)
				msg << nextChar;
				
			usleep(50);
		}
		string msgStr = msg.str();
		if (msgStr.size() > 2) // Don't bother with carriage return
		{
			if (endOfMessageCharIsFound) // Message received
			{
				ROS_INFO_STREAM("arduino_listener.cpp main(): Received '" << msgStr << "'");
				// Determine the message topic
				if (interpret.KeyIsPresent("frontLeftContactSwitch", msgStr) || interpret.KeyIsPresent("frontRightContactSwitch", msgStr))
				{
					noize_robot::FrontContactSwitches frontContactSwitchesMsg;
					frontContactSwitchesMsg.leftContactSwitch = false;
					frontContactSwitchesMsg.rightContactSwitch = false;
					string value = interpret.ValueOf("frontLeftContactSwitch", msgStr);
					if (value == "high")
					{
						frontContactSwitchesMsg.leftContactSwitch = true;
					}
					value = interpret.ValueOf("frontRightContactSwitch", msgStr);
					if (value == "high")
					{
						frontContactSwitchesMsg.rightContactSwitch = true;
					}
					frontContactSwitch_pub.publish(frontContactSwitchesMsg);
				}
			}
			else // Error
			{
				stringstream errorMsg; errorMsg << "arduino_listener.cpp main(): End of message character was not received. errno = " << strerror(errno) << "; msgStr = " << msgStr; 
				ROS_ERROR_STREAM(errorMsg.str()); 
			}
		} // endif (msgStr.size() > 2)
		ros::spinOnce();
		loop_rate.sleep();
	} // while (ros::ok())
	
	serialClose(serialFileDesc);
	
	cout << "Done!" << endl;
	return 0;
}
