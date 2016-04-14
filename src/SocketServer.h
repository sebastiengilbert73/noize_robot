#ifndef SocketServer_h
#define SocketServer_h

#include <iostream>
#include <string.h>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h> // usleep(), read(), write()
#include "SocketCommunicationData.h"


void* WaitForConnectionThread(void* castSocketCommunicationDataPtr);
void* ListeningThread(void* castSocketCommunicationDataPtr);
void* TalkingThread(void* castSocketCommunicationDataPtr);

class SocketServer
{

public:
	SocketServer();
	void Create();
	SocketServer(unsigned int port);
	void Create(unsigned int port);
	void StartListening();
	bool ThereIsAReceivedMessage() { return _socketCommunicationData.ThereIsAReceivedMessage(); }
	std::string NextReceivedMessage();
	void SendMessage(std::string message);
	
protected:
	
	
	SocketCommunicationData _socketCommunicationData;
	int _port;
	
	pthread_t _waitForConnectionThread;
};

#endif
