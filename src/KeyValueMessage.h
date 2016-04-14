#ifndef KeyValueMessage_h
#define KeyValueMessage_h

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
//#include <wiringSerial.h>


struct KeyValuePair
{
   std::string key;
   std::string value;
};

class KeyValueMessage
{
  public:
    KeyValueMessage(char separator = ' ') { _separator = separator; }
    std::string KeyValueString(std::string key, std::string value);
    std::string KeyValueAsIntString(std::string key, int value);
    std::string KeyValueAsDoubleString(std::string key, double value);
    std::string KeyValueAsBoolString(std::string key, bool value);
    std::string ValueOf(std::string key, std::string message);
    //static std::string AvailableMessage(int portID, char endOfMessageChar,
	//	double maximumDelayInMilliseconds = 500);
    std::vector<KeyValuePair> AllKeyValues(std::string message);
    static bool ConvertToBool(std::string value);
    bool KeyIsPresent(std::string key, std::string& message);
    bool ValueAsInt(std::string key, std::string& message, int* dstIntValuePtr);
    bool ValueAsDouble(std::string key, std::string& message, double* dstDoubleValuePtr);

  private:
    char _separator;
};

#endif
