#include <sstream>
#include <vector>

class SocketCommunicationData
{
public:
	 SocketCommunicationData();
	 void Create();
	 void AddReceivedMessage(std::string message);
	 bool ThereIsAReceivedMessage() { return _receivedMessagesVct.size() > 0; }
	 std::string PopNextReceivedMessage();
	 void AddMessageToSend(std::string message);
	 bool ThereIsAMessageToSend() { return _messagesToSendVct.size() > 0; }
	 std::string PopNextMessageToSend();
	 bool MustRun();
	 void Stop();
	 int SocketFileDescriptor() { return _socketFileDescriptor; }
	 void SetSocketFileDescriptor(int fileDescriptor) { _socketFileDescriptor = fileDescriptor; }
	 std::string ServerIPAddress() { return _serverIPAddress; }
	 void SetServerIPAddress(std::string serverIPAddress) { _serverIPAddress = serverIPAddress; }
	 unsigned int ServerPort() { return _serverPort; }
	 void SetServerPort(unsigned int serverPort ) { _serverPort = serverPort; }
	 bool ClientIsConnected() { return _clientIsConnected; }
	 void SetClientIsConnected(bool setValue) { _clientIsConnected = setValue; }
	 pthread_t ListeningThread();
	 pthread_t TalkingThread();
	
protected:
	std::vector<std::string> _receivedMessagesVct;
	std::vector<std::string> _messagesToSendVct;
	bool _mustRun;
	int _socketFileDescriptor;
	std::string _serverIPAddress;
	unsigned int _serverPort;
	bool _clientIsConnected;
	pthread_t _listeningThread;
	pthread_t _talkingThread;
};

