#include "SocketServer.h"

using namespace std;



SocketServer::SocketServer()
{
	Create();
}

void SocketServer::Create()
{
	_socketCommunicationData.Create();
}

SocketServer::SocketServer(unsigned int port)
{
	Create(port);
}

void SocketServer::Create(unsigned int port)
{
	Create();
	_socketCommunicationData.Create();
	_socketCommunicationData.SetServerPort(port);
}

void SocketServer::StartListening()
{
	pthread_create(&_waitForConnectionThread, NULL, WaitForConnectionThread, 
		(void*)(&_socketCommunicationData));
}

std::string SocketServer::NextReceivedMessage()
{
	return _socketCommunicationData.PopNextReceivedMessage();
}

void SocketServer::SendMessage(std::string message)
{
	_socketCommunicationData.AddMessageToSend(message);
}



void* ListeningThread(void* castSocketCommunicationDataPtr)
{
	SocketCommunicationData* socketCommunicationDataPtr = (SocketCommunicationData*) castSocketCommunicationDataPtr;
	while (socketCommunicationDataPtr->MustRun() && socketCommunicationDataPtr->ClientIsConnected())
	{
		char buffer[256];
		bzero(buffer, 256);
		int n = read(socketCommunicationDataPtr->SocketFileDescriptor(), buffer, 255); // Blocking
		if (n < 0)
		{
			/*stringstream msg;
			msg << "SocketServer.cpp > ListeningThread(): Error reading from socket";
			throw runtime_error(msg.str());*/
			socketCommunicationDataPtr->SetClientIsConnected(false);
		}
		else if (n == 0) // Client is disconnected
		{
			socketCommunicationDataPtr->SetClientIsConnected(false);
		}
		else
			socketCommunicationDataPtr->AddReceivedMessage(string(buffer));
	}
	/*	//socketServerDataPtr->AddReceivedMessage("Received inside ListeningThread()");
	int sockfd,newsockfd;
	socklen_t clilen;
	char buffer[256];
	struct sockaddr_in serverAddr, clientAddr;
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
	{
		stringstream msg;
		msg << "SocketServer.cpp > ListeningThread(): Error opening socket";
		throw runtime_error(msg.str());
	}
	memset((void*) (&serverAddr), 0, sizeof(sockaddr_in));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = htons(_port);
	
	// Binding
	if (bind(sockfd, (struct sockaddr*) &serverAddr,
		sizeof(serverAddr)) < 0)
	{
		stringstream msg;
		msg << "SocketServer.cpp > ListeningThread(): Error on binding";
		throw runtime_error(msg.str());
	}
	listen(sockfd, 5);
	clilen = sizeof(clientAddr);
	while(socketCommunicationDataPtr->MustRun())
	{
		//cout << "SocketServer.cpp > ListeningThread(): Before newsockfd = accept(...)" << endl;
		newsockfd = accept(sockfd, (struct sockaddr*) &clientAddr, &clilen); // Blocking
		//cout << "SocketServer.cpp > ListeningThread(): After newsockfd = accept(...)" << endl;
		if (sockfd < 0)
		{
			stringstream msg;
			msg << "Error on accept";
			throw runtime_error(msg.str());
		}
		bool clientIsConnected = true;
		int loopNbr = 1;
		while (clientIsConnected)
		{
			cout << "SocketServer.cpp > ListeningThread(): loopNbr = " << loopNbr << endl;
			loopNbr++;
			// Send messages
			while(socketCommunicationDataPtr->ThereIsAMessageToSend())
			{
				string message = socketCommunicationDataPtr->PopNextMessageToSend();
				cout << "SocketServer.cpp > ListeningThread(): message = " << message << endl;
				int n = write(newsockfd, message.c_str(), message.length());
				if (n < 0)
				{
					stringstream msg;
					msg << "SocketServer.cpp > ListeningThread(): Error writing to socket";
					throw runtime_error(msg.str());
				}
			}
			memset((void*) buffer, 0, 256);
			int n = read(newsockfd, buffer, 255); // Blocking
			if (n < 0)
			{
				stringstream msg;
				msg << "SocketServer.cpp > ListeningThread(): Error reading from socket";
				throw runtime_error(msg.str());
			}
			else if (n == 0)
			{
				clientIsConnected = false;
				cout << "SocketServer.cpp > ListeningThread(): Client is disconnected" << endl;
			}
			else
				socketCommunicationDataPtr->AddReceivedMessage(string(buffer));
		}
	}
*/	
	return NULL;
}

void* TalkingThread(void* castSocketCommunicationDataPtr)
{
	SocketCommunicationData* socketCommunicationDataPtr = (SocketCommunicationData*) castSocketCommunicationDataPtr;
	while (socketCommunicationDataPtr->MustRun() && socketCommunicationDataPtr->ClientIsConnected())
	{
		// Send messages
		while(socketCommunicationDataPtr->ThereIsAMessageToSend())
		{
			string message = socketCommunicationDataPtr->PopNextMessageToSend();
			int n = write(socketCommunicationDataPtr->SocketFileDescriptor(), message.c_str(), message.length());
			if (n < 0)
			{
				stringstream msg;
				msg << "SocketServer.cpp > TalkingThread(): Error writing to socket";
				throw runtime_error(msg.str());
			}
		}
	}
	return NULL;
}

void* WaitForConnectionThread(void* castSocketCommunicationDataPtr)
{
	//cout << "WaitForConnectionThread()" << endl;
	SocketCommunicationData* socketCommunicationDataPtr = (SocketCommunicationData*) castSocketCommunicationDataPtr;
	
	int sockfd,newsockfd;
	socklen_t clilen;
	//char buffer[256];
	struct sockaddr_in serverAddr, clientAddr;
	//cout << "WaitForConnectionThread(): Opening socket" << endl;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
	{
		stringstream msg;
		msg << "SocketServer.cpp > WaitForConnectionThread(): Error opening socket";
		throw runtime_error(msg.str());
	}
	memset((void*) (&serverAddr), 0, sizeof(sockaddr_in));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = htons(socketCommunicationDataPtr->ServerPort());
	
	// Binding
	//cout << "WaitForConnectionThread(): Binding" << endl;
	if (bind(sockfd, (struct sockaddr*) &serverAddr,
		sizeof(serverAddr)) < 0)
	{
		stringstream msg;
		msg << "SocketServer.cpp > WaitForConnectionThread(): Error on binding";
		throw runtime_error(msg.str());
	}
	listen(sockfd, 5);
	clilen = sizeof(clientAddr);
	while(socketCommunicationDataPtr->MustRun())
	{
		//cout << "WaitForConnectionThread(): Before newsockfd = accept(sockfd, (struct sockaddr*) &clientAddr, &clilen); // Blocking" << endl;
		newsockfd = accept(sockfd, (struct sockaddr*) &clientAddr, &clilen); // Blocking
		//cout << "WaitForConnectionThread(): After newsockfd = accept(sockfd, (struct sockaddr*) &clientAddr, &clilen); // Blocking" << endl;
		if (sockfd < 0)
		{
			stringstream msg;
			msg << "SocketServer.cpp > WaitForConnectionThread(): Error on accept";
			throw runtime_error(msg.str());
		}
		socketCommunicationDataPtr->SetClientIsConnected(true);
		socketCommunicationDataPtr->SetSocketFileDescriptor(newsockfd);
		// Start listening and talking threads
		pthread_t listeningThread, talkingThread;
		pthread_create(&listeningThread, NULL, ListeningThread, 
			(void*)(socketCommunicationDataPtr));
		pthread_create(&talkingThread, NULL, TalkingThread, 
			(void*)(socketCommunicationDataPtr));
		
		// Wait for connection to be closed
		while (socketCommunicationDataPtr->MustRun() && socketCommunicationDataPtr->ClientIsConnected())
		{
			usleep(1000);
		}
	}
	return NULL;
}
