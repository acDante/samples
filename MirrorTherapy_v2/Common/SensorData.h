#pragma once
#include <boost\algorithm\string\split.hpp>
#include <boost\algorithm\string.hpp>
#include <vector>
#include <map>
#include <iostream>

// Define delimiters to split string.
#define PAIRS_DELIMITER			";" // Delimiter to make pair(s) from whole message.
#define KEYandVALUE_DELIMITER	":" // Delimter to split to KEY and VALUE from one pair.
#define VECTOR_DELIMITER		"," // Delimiter to vector(multi dimentional values) from one VALUE.

static const std::string pairsDelimDefault		= PAIRS_DELIMITER;
static const std::string keyValueDelimDefault	= KEYandVALUE_DELIMITER;
static const std::string vectorDelimDefault		= VECTOR_DELIMITER;

class SensorData
{
public:
	virtual ~SensorData() {};

	///@brief Convert sensor information to string.
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split records. Record is pair of KEY and VALUE.
	///@param valueDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual std::string encodeSensorData2Message(const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim) = 0;
	
	///@brief Convert message(string) to sensor data.
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split records. Record is pair of KEY and VALUE.
	///@param valueDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual void decodeMessage2SensorData(const std::string &message, const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim) = 0;

	///@brief Convert message to std::map. 
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split whole message.
	static std::map<std::string, std::string> convertMessage2Map(const std::string &message, const std::string &keyDelim, const std::string &recordDelim);

	///@brief Delete parentheses from string.
	static std::string deleteParenthesesFromString(const std::string &input, const std::string &pBegin = "(", const std::string &pEnd = ")");
};