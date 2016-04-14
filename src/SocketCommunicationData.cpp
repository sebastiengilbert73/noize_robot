#include "SocketCommunicationData.h"

using namespace std;

SocketCommunicationData::SocketCommunicationData()
{
	Create();
}

void SocketCommunicationData::Create()
{
	_receivedMessagesVct.clear();
	_messagesToSendVct.clear();
	_mustRun = true;
	_socketFileDescriptor = 0;
	_serverIPAddress = "NotSet";
	_serverPort = 0;
	_clientIsConnected = false;
}

void SocketCommunicationData::AddReceivedMessage(std::string message)
{
	_receivedMessagesVct.push_back(message);
}

std::string SocketCommunicationData::PopNextReceivedMessage()
{
	if (_receivedMessagesVct.size() < 1)
		return "";
	string nextMsg = _receivedMessagesVct[0];
	_receivedMessagesVct.erase(_receivedMessagesVct.begin());
	return nextMsg;
}

void SocketCommunicationData::AddMessageToSend(std::string message)
{
	_messagesToSendVct.push_back(message);
}

bool SocketCommunicationData::MustRun()
{
	return _mustRun;
}

void SocketCommunicationData::Stop()
{
	_mustRun = false;
}

std::string SocketCommunicationData::PopNextMessageToSend()
{
	if (ThereIsAMessageToSend())
	{
		string nextMessage = _messagesToSendVct[0];
		_messagesToSendVct.erase(_messagesToSendVct.begin());
		return nextMessage;
	}
	else
		return "";
}
