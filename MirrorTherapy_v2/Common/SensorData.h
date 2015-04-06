#pragma once
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <map>
#include <iostream>

// Define delimiters to split string.
#define PAIRS_DELIMITER       ";" // Delimiter to make pair(s) from whole message.
#define KEYandVALUE_DELIMITER ":" // Delimter to split to KEY and VALUE from one pair.
#define VECTOR_DELIMITER      "," // Delimiter to vector(multi dimentional values) from one VALUE.

static const std::string pairsDelimDefault    = PAIRS_DELIMITER;
static const std::string keyValueDelimDefault = KEYandVALUE_DELIMITER;
static const std::string vectorDelimDefault   = VECTOR_DELIMITER;

class SensorData
{
public:
	virtual ~SensorData() {};

	///@brief Convert sensor information to string.
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split records. Record is pair of KEY and VALUE.
	///@param valueDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual std::string encodeSensorData(const std::string &pairsDelim = pairsDelimDefault, const std::string &keyValueDelim = keyValueDelimDefault, const std::string &vectorDelim = vectorDelimDefault) = 0;
	
	///@brief Convert message(string) to sensor data.
	///@param pairsDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split records. Record is pair of KEY and VALUE.
	///@param valueDelim Symbol to split each componet of VALUE. When VALUE is multidimensional data, you use this symbol.
	virtual void decodeSensorData(const std::string &message, const std::string &pairsDelim = pairsDelimDefault, const std::string &keyValueDelim = keyValueDelimDefault, const std::string &vectorDelim = vectorDelimDefault) = 0;

	///@brief Convert message to std::map. 
	///@param keyDelim Symbol to split KEY and VALUE.
	///@param recordDelim Symbol to split whole message.
	//static std::map<std::string, std::string> convertMessage2Map(const std::string &message, const std::string &keyDelim, const std::string &recordDelim);
	static std::map<std::string, std::vector<std::string> > convertMessage2Map(const std::string &messages, const std::string &pairsDelim = pairsDelimDefault, const std::string &keyValueDelim = keyValueDelimDefault, const std::string &vectorDelim = vectorDelimDefault);

	// TODO です．
	// センサーデータを std::map<std::string, std::vector<std::string> > に変換したものを渡せば，それに基づくメッセージを作ってくれる．というもの．
	// なぜマップにする必要があるのか？という説明は以下．(野崎)
	// 現状では encodeSensorData() でメッセージ作成のほぼすべてを賄っているが，メッセージ作成の規則（:や,の置き場所など）を実装する人が理解しておく必要がある．
	// SensorDataの実体をマップに変換する実装さえあれば，メッセージ作成の規則に従ってメッセージを作成するという機能を持たせたい．
	// つまり，SensorDataの実体を「[KEYの文字列]と[VALUEの文字列のベクタ]からなるマップ」に変換する実装をencodeSensorData()内に書き，
	// その直後（encodeSensorData()内）にこのconvertMap2Message()を呼んで第一引数に与えれば，送るべきメッセージが生成される．
	// という機能を実装したい．ということです．
	// 推測されるインタフェースだけ書いてみました．
	// static std::string convertMap2Message(std::map<std::string, std::vector<std::string> > &map, const std::string &keyDelim, const std::string &recordDelim, const std::string &vectorDelim);

	///@brief Delete parentheses '(' and ')' from string.
	static std::string deleteParenthesesFromString(const std::string &input, const std::string &pBegin = "(", const std::string &pEnd = ")");
};
