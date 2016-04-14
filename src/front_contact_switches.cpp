#include "ros/ros.h"
#include "std_msgs/Empty.h" // To publish the simple occurrence of an event
#include <iostream>
#include <wiringPi.h>
#include <unistd.h> // usleep()

using namespace std;

const int LEFT_SWITCH_PIN = 1; // <- wiringPi pin 1; Corresponds to physical pin 12;
int main(int argc, char **argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "front_contact_switches");
	ros::NodeHandle nodeHdl;
	ros::Publisher switchHigh_pub = nodeHdl.advertise<std_msgs::Empty>("LeftContactSwitch", 1000);

	// Setup wiringPi
	wiringPiSetup();
	pinMode(LEFT_SWITCH_PIN, INPUT);
	pullUpDnControl(LEFT_SWITCH_PIN, PUD_DOWN);
	
	// ROS loop
	ros::Rate loop_rate(10);
	while (ros::ok())
    {
		if (digitalRead(LEFT_SWITCH_PIN) == HIGH)
		{
			std_msgs::Empty switchHighEvent;
			ROS_INFO("%s", "Contact detected");
			switchHigh_pub.publish(switchHighEvent);		
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	
	
	
/*	for(;;)
	{
		if (digitalRead(LEFT_SWITCH_PIN) == HIGH)
		{
			cout << "Contact!" << endl;
		}
		if (usleep(1000) == -1)
		{
			cout << "usleep() interrupted." << endl;
			return -1;
		}
	}*/
	cout << "Done!" << endl;
	return 0;
}
