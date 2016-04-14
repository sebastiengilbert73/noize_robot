#include "KeyValueMessage.h"

using namespace std;

string KeyValueMessage::KeyValueString(string key, string value)
{
  string keyValueStr = key;
  keyValueStr += "=";
  keyValueStr += value;
  return keyValueStr;
}

string KeyValueMessage::KeyValueAsIntString(string key, int value)
{
  stringstream keyValueStr;
  keyValueStr << key;
  keyValueStr << "=";
  keyValueStr << value;
  return keyValueStr.str();
}

string KeyValueMessage::KeyValueAsDoubleString(string key, double value)
{
/*  string keyValueStr = key;
  keyValueStr += "=";
  char valueArr[15];
  dtostrf(value, 15, 15, valueArr);
  string valueStr = valueArr;
  keyValueStr += valueStr;
  return keyValueStr;
*/  
  stringstream keyValueStr;
  keyValueStr << key << "=" << value;
  return keyValueStr.str();
}

string KeyValueMessage::KeyValueAsBoolString(string key, bool value)
{
  string keyValueStr = key;
  keyValueStr += "=";
  keyValueStr += (value == true ? "true" : "false");
  return keyValueStr;
}

string KeyValueMessage::ValueOf(string key, string message)
{
/*  int keyStart = message.find(key);//StartPositionOfKey(message, key);
  if (keyStart != (int) string::npos && keyStart + key.length() < message.length() && message.at(keyStart + key.length()) == '=')
  {
	  if (keyStart == 0 || message.at(keyStart - 1) == _separator)
	  {
		int separatorPosition = message.find(_separator, keyStart + key.length() + 1);
		if (separatorPosition != (int) string::npos)
		{
			return message.substr(keyStart + key.length() + 1, separatorPosition - (keyStart + key.length() + 1));
		}
		else // To the end of the string
			return message.substr(keyStart + key.length() + 1);
	  }
      else // "coronation=1 nation=2" -> we found the first "nation", but it's not the one we are searching for
      {
		  string truncatedMsg = message.substr(keyStart + 1);
		  return ValueOf(key, truncatedMsg);
	  }
  }
  else return "";*/
  std::vector<KeyValuePair> allKeyValuesPairsVct = AllKeyValues(message);
  for (unsigned int pairNdx = 0; pairNdx < allKeyValuesPairsVct.size(); pairNdx++)
  {
	  if (allKeyValuesPairsVct[pairNdx].key.compare(key) == 0)
		return allKeyValuesPairsVct[pairNdx].value;
  }
  return "";
}

/*string KeyValueMessage::AvailableMessage(int portID, char endOfMessageChar,
	double maximumDelayInMilliseconds)
{

	double initialTime = (double) clock()/CLOCKS_PER_SEC;
	string inputMsg;
	if (serialDataAvail(portID))
	{
		bool endOfMessageCharReceived = false;
		double elapsedTimeInMilliseconds = 1000 * ((double) clock()/CLOCKS_PER_SEC - initialTime);
		while (!endOfMessageCharReceived && elapsedTimeInMilliseconds <= maximumDelayInMilliseconds)
		{
			char receivedChar = (char) serialGetchar(portID);
			if ((int) receivedChar > 0) // Ignore the null character received (there is typically one at the beginning)
				inputMsg += receivedChar;
			endOfMessageCharReceived = (receivedChar == endOfMessageChar);
			elapsedTimeInMilliseconds = 1000 * ((double) clock()/CLOCKS_PER_SEC - initialTime);
		}
		// Remove the end of message char at the end
		inputMsg = inputMsg.substr(0, inputMsg.length() - 1);
	}
	return inputMsg;
}*/

std::vector<KeyValuePair> KeyValueMessage::AllKeyValues(string message)
{
   vector<KeyValuePair> keyValuePairsVct;
   // Find the separators: key start are at space (i.e. separator) index + 1
   // "key1=value1 key2=value2 key3=value3 ... "
   //  ^           ^           ^
   vector<int> keyStartPositionsVct;
   keyStartPositionsVct.push_back(0);
   int lastFoundNdx = 0;
   while (lastFoundNdx != (int) string::npos)
   {
	lastFoundNdx = message.find_first_of(_separator, lastFoundNdx + 1);
	if (lastFoundNdx != (int) string::npos)
	   keyStartPositionsVct.push_back(lastFoundNdx + 1);
   }
   // Find the equal signs and build the key-value pairs
   // "key1=value1 key2=value2 key3=value3 ... "
   //      ^           ^           ^
   
   for (unsigned int keyNdx = 0; keyNdx < keyStartPositionsVct.size(); keyNdx++)
   {
	int equalSignPosition = message.find_first_of('=', keyStartPositionsVct[keyNdx]);
	if (equalSignPosition != (int) string::npos);
	{
		KeyValuePair pair;
		pair.key = message.substr(keyStartPositionsVct[keyNdx], equalSignPosition - keyStartPositionsVct[keyNdx]);
		if (keyNdx < keyStartPositionsVct.size() - 1) // Not the last value
			pair.value = message.substr(equalSignPosition + 1, keyStartPositionsVct[keyNdx + 1] - 1 - (equalSignPosition + 1));
		else // Last value
			pair.value = message.substr(equalSignPosition + 1);
		keyValuePairsVct.push_back(pair);
	}
   }
   return keyValuePairsVct;
}

bool KeyValueMessage::ConvertToBool(string value)
{
	// Convert to uppercase
	std::transform(value.begin(), value.end(), value.begin(), ::toupper);
	if (value.compare("TRUE") == 0)
		return true;
	else
		return false;
}

bool KeyValueMessage::KeyIsPresent(string key, string& message)
{
	string value = ValueOf(key, message);
	if (value.length() == 0)
		return false;
	else
		return true;
}

bool KeyValueMessage::ValueAsInt(string key, string& message, int* dstIntValuePtr)
{
	if (KeyIsPresent(key, message))
	{
		string value = ValueOf(key, message);
		*dstIntValuePtr = atoi(value.c_str());
		return true;
	}

	else
	{
		*dstIntValuePtr = 0;
		return false;
	}
}

bool KeyValueMessage::ValueAsDouble(string key, string& message, double* dstDoubleValuePtr)
{
	if (KeyIsPresent(key, message))
	{
		string value = ValueOf(key, message);
		double valueDbl = atof(value.c_str());
		*dstDoubleValuePtr = valueDbl;
		return true;
	}
	else
	{
		*dstDoubleValuePtr = 0;
		return false;
	}
}

