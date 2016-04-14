#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include <cerrno> // errno
#include <unistd.h> // usleep()
#include "KeyValueMessage.h"
#include "noize_robot/Headlights.h"
#include "SocketServer.h"

using namespace std;

string version = "V2016-04-11";
const double ros_rate_inHz = 100.0;
const char endOfMessageChar = '|';
KeyValueMessage interpret;
unsigned int port = 19393;

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "socket_listener");
	ros::NodeHandle nodeHdl;
	// Declare publishers
	ros::Publisher headlights_pub = nodeHdl.advertise<noize_robot::Headlights>("Headlights", 1000);
	
	ROS_INFO_STREAM("socket_listener.cpp main() version = " << version);
	
	// Create socket server
	SocketServer socketServer;
	try
	{
		socketServer.Create(port);
		socketServer.StartListening();
	}
	catch (exception e)
	{
		ROS_ERROR_STREAM("socket_listener.cpp main() : Exception caught: " << e.what());
		return -1;
	}
	
	
	// ROS loop
	ros::Rate loop_rate(ros_rate_inHz);
	while (ros::ok())
    {
		if (socketServer.ThereIsAReceivedMessage())
		{
			string msg = socketServer.NextReceivedMessage();
			ROS_INFO_STREAM("socket_listener.cpp main(): received message = " << msg);
			if (interpret.KeyIsPresent("headlights", msg))
			{
				noize_robot::Headlights headlightsMsg;
				headlightsMsg.leftRed = 0;	headlightsMsg.leftGreen = 0;	headlightsMsg.leftBlue = 0;
				headlightsMsg.rightRed = 0;	headlightsMsg.rightGreen = 0;	headlightsMsg.rightBlue = 0;
				string color = interpret.ValueOf("headlights", msg);
				if (color == "off")
					headlights_pub.publish(headlightsMsg);
				else if (color == "red")
				{
					headlightsMsg.leftRed = 255;	headlightsMsg.rightRed = 255;
					headlights_pub.publish(headlightsMsg);
				}
				else if (color == "green")
				{
					headlightsMsg.leftGreen = 255;	headlightsMsg.rightGreen = 255;
					headlights_pub.publish(headlightsMsg);
				}
				else if (color == "blue")
				{
					headlightsMsg.leftBlue = 255;	headlightsMsg.rightBlue = 255;
					headlights_pub.publish(headlightsMsg);
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	} // end of while (ros::ok())
}
