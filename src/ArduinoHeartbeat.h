#ifndef ArduinoHeartbeat_h
#define ArduinoHeartbeat_h

#include <iostream>
#include <string>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include "wiringPi.h"
#include <unistd.h> // usleep()
#include "ros/ros.h"
#include "std_msgs/String.h"

enum ArduinoHeartbeatPeriodMeaning
{
	NORMAL, MESSAGE_TO_SEND
};

struct WatchArduinoHeartbeatData
{
	int arduinoResetPin;
	int arduinoHeartbeatPin;
	double arduinoHeartbeatEmergencyDelay;
	double arduinoHeartbeatResetDelay;
	double arduinoHeartbeatNormalPeriod;
	double arduinoHeartbeatMessagePeriod;
	struct timeval lastArduinoResetTime;
	ros::Publisher rosPublisher;
};

class ArduinoHeartbeat
{
public:
	ArduinoHeartbeat(
		int arduinoResetPin,
		int arduinoHeartbeatPin,
		double arduinoHeartbeatEmergencyDelay,
		double arduinoHeartbeatResetDelay,
		double arduinoHeartbeatNormalPeriod,
		double arduinoHeartbeatMessagePeriod,
		ros::Publisher rosPublisher);
	~ArduinoHeartbeat();
	void ResetArduino();
	void WatchArduinoHeartbeat();
	
private:

	WatchArduinoHeartbeatData* heartbeatDataPtr;
};



void* WatchArduinoHeartbeatThread(void* castWatchArduinoHeartbeatDataPtr);
void ResetArduino(int arduinoResetPin, struct timeval* lastArduinoResetTimePtr);
ArduinoHeartbeatPeriodMeaning PeriodMeaning(double period);

#endif
