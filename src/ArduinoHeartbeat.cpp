#include "ArduinoHeartbeat.h"

using namespace std;

ArduinoHeartbeat::ArduinoHeartbeat(
		int arduinoResetPin,
		int arduinoHeartbeatPin,
		double arduinoHeartbeatEmergencyDelay,
		double arduinoHeartbeatResetDelay,
		double arduinoHeartbeatNormalPeriod,
		double arduinoHeartbeatMessagePeriod,
		ros::Publisher rosPublisher)
{
	heartbeatDataPtr = new WatchArduinoHeartbeatData;
	heartbeatDataPtr->arduinoResetPin = arduinoResetPin;
	heartbeatDataPtr->arduinoHeartbeatPin = arduinoHeartbeatPin;
	heartbeatDataPtr->arduinoHeartbeatEmergencyDelay = arduinoHeartbeatEmergencyDelay;
	heartbeatDataPtr->arduinoHeartbeatResetDelay = arduinoHeartbeatResetDelay;
	heartbeatDataPtr->arduinoHeartbeatNormalPeriod = arduinoHeartbeatNormalPeriod;
	heartbeatDataPtr->arduinoHeartbeatMessagePeriod = arduinoHeartbeatMessagePeriod;
	heartbeatDataPtr->rosPublisher = rosPublisher;
	gettimeofday(&heartbeatDataPtr->lastArduinoResetTime, NULL);
}

ArduinoHeartbeat::~ArduinoHeartbeat()
{
	delete heartbeatDataPtr;
}
		
void ResetArduino(int arduinoResetPin, struct timeval* lastArduinoResetTimePtr)
{
	digitalWrite(arduinoResetPin, LOW);
	usleep(1000);
	digitalWrite(arduinoResetPin, HIGH);
	gettimeofday(lastArduinoResetTimePtr, NULL);
}

void ArduinoHeartbeat::ResetArduino()
{
	::ResetArduino(heartbeatDataPtr->arduinoResetPin, &heartbeatDataPtr->lastArduinoResetTime);
}

ArduinoHeartbeatPeriodMeaning PeriodMeaning(double period, double arduinoHeartbeatNormalPeriod, double arduinoHeartbeatMessagePeriod)
{
	double normalDelta = fabs(period - arduinoHeartbeatNormalPeriod);
	double messageToSendDelta = fabs(period - arduinoHeartbeatMessagePeriod);
	if (normalDelta < messageToSendDelta)
		return NORMAL;
	else
		return MESSAGE_TO_SEND;
}

void* WatchArduinoHeartbeatThread(void* castWatchArduinoHeartbeatDataPtr)
{
	cout << "WatchArduinoHeartbeatThread()" << endl;
	WatchArduinoHeartbeatData* dataPtr = (WatchArduinoHeartbeatData*)  castWatchArduinoHeartbeatDataPtr;
	struct timeval lastUpFront;
	struct timeval previousUpFront;
	struct timeval currentTime;
	ResetArduino(dataPtr->arduinoResetPin, &dataPtr->lastArduinoResetTime);

	double heartbeatPeriod = 0;
	int previousHeartbeatValue = digitalRead(dataPtr->arduinoHeartbeatPin);
	do
	{
		gettimeofday(&currentTime, NULL);
		int heartbeatValue = digitalRead(dataPtr->arduinoHeartbeatPin);
		if (heartbeatValue != previousHeartbeatValue)
		{
			previousHeartbeatValue = heartbeatValue;
			if (heartbeatValue == 1)
			{
				previousUpFront = lastUpFront;
				//lastUpFront = clock();
				gettimeofday(&lastUpFront, NULL);
				//double heartbeatPeriod = (double) (lastUpFront - previousUpFront)/CLOCKS_PER_SEC;
				heartbeatPeriod = (lastUpFront.tv_sec - previousUpFront.tv_sec) + 1E-6 *(lastUpFront.tv_usec - previousUpFront.tv_usec);
				//cout << "heartbeatPeriod = " << heartbeatPeriod << endl;
				ArduinoHeartbeatPeriodMeaning periodMeaning = PeriodMeaning(heartbeatPeriod, dataPtr->arduinoHeartbeatNormalPeriod,
					dataPtr->arduinoHeartbeatMessagePeriod);
				if (periodMeaning == MESSAGE_TO_SEND)
				{
					cout << "Arduino has a message to send." << endl;
				}
			}
		}
		double timeSinceLastUpFront = (currentTime.tv_sec - lastUpFront.tv_sec) + 1E-6 * (currentTime.tv_usec - lastUpFront.tv_usec);
		double timeSinceLastArduinoReset = (currentTime.tv_sec - dataPtr->lastArduinoResetTime.tv_sec) + 1E-6 * (currentTime.tv_usec - dataPtr->lastArduinoResetTime.tv_usec);
		//cout << "timeSinceLastUpFront = " << timeSinceLastUpFront << ", timeSinceLastArduinoReset = " << timeSinceLastArduinoReset << endl;
		if (timeSinceLastUpFront > dataPtr->arduinoHeartbeatEmergencyDelay && timeSinceLastArduinoReset > dataPtr->arduinoHeartbeatResetDelay)
		{
			cout << "Resetting Arduino..." << endl;
			std::string msgStr = "Resetting Arduino...";
			std_msgs::String msg;
			msg.data = msgStr;
			dataPtr->rosPublisher.publish(msg);
			ResetArduino(dataPtr->arduinoResetPin, &dataPtr->lastArduinoResetTime);
		}
		
	} while(true);
	delete dataPtr;
	return NULL;
}

void ArduinoHeartbeat::WatchArduinoHeartbeat()
{
	pthread_t thread1;
	pthread_create(&thread1, NULL, WatchArduinoHeartbeatThread, 
		(void*)(heartbeatDataPtr));
}
