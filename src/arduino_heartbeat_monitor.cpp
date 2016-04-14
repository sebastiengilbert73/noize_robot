#include "ros/ros.h"
#include "std_msgs/String.h" // To publish the simple occurrence of an event
#include <iostream>
#include <wiringPi.h>
//#include <unistd.h> // usleep()
#include "ArduinoHeartbeat.h"

using namespace std;

const int ARDUINO_HEARTBEAT_PIN = 0; // <- wiringPi pin 0; Corresponds to physical pin 11;
const int ARDUINO_RESET_PIN = 7; // <- wiringPi pin 7; Corresponds to physical pin 7;
const double arduinoHeartbeatEmergencyDelay = 2.0;
const double arduinoHeartbeatResetDelay = 4.0;
const double arduinoHeartbeatNormalPeriod = 1.0;
const double arduinoHeartbeatMessagePeriod = 1.0;

int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "arduino_heartbeat_monitor");
	ros::NodeHandle nodeHdl;
	ros::Publisher heartStroke_pub = nodeHdl.advertise<std_msgs::String>("ArduinoHeartStroke", 1000);

	// Setup wiringPi
	wiringPiSetup();
	pinMode(ARDUINO_HEARTBEAT_PIN, INPUT);
	pullUpDnControl(ARDUINO_HEARTBEAT_PIN, PUD_DOWN);
	
	// Setup ArduinoHeartbeat
	ArduinoHeartbeat arduinoHeartbeat(
		ARDUINO_RESET_PIN,
		ARDUINO_HEARTBEAT_PIN,
		arduinoHeartbeatEmergencyDelay,
		arduinoHeartbeatResetDelay,
		arduinoHeartbeatNormalPeriod,
		arduinoHeartbeatMessagePeriod,
		heartStroke_pub);
	arduinoHeartbeat.WatchArduinoHeartbeat();
	
	// ROS loop
	ros::Rate loop_rate(2);
	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
	}
	cout << "Done!" << endl;
	return 0;
}
