#include "SensorData.h"

std::map<std::string, std::vector<std::string> > SensorData::convertMessage2Map(const std::string &message, const std::string &pairsDelim, const std::string &keyValueDelim, const std::string &vectorDelim)
{
	std::map<std::string, std::vector<std::string> > map;

	try {
		// まずは「;」でKEY:VALUEごとに文字列を取得する
		std::vector<std::string> pairs;
		boost::split(pairs, message, boost::is_any_of(pairsDelim));

		for(int i = 0; i < (int)pairs.size(); i++) 
		{
			if (pairs[i] == "") continue;

			//std::cout << pairs[i] << std::endl;
			
			// KEY:VALUE となっている文字列から KEY と VALUE を抽出してマップにする
			std::vector<std::string> keyAndValues;
			boost::split(keyAndValues, pairs[i], boost::is_any_of(keyValueDelim));

			// 「(」と「)」を削除する（無い場合は何もしない）．
			// Delete "(" and ")" from message.
			const std::string valuesStringDeletedParentheses = SensorData::deleteParenthesesFromString(keyAndValues[1]);

			// VALUE部分を「,」で分割する
			std::vector<std::string> valuesVector;
			boost::split(valuesVector, valuesStringDeletedParentheses, boost::is_any_of(vectorDelim));

			map.insert( std::map<std::string, std::vector<std::string> >::value_type(keyAndValues[0], valuesVector) );
		}
	}
	catch (std::exception &ex) 
	{
		std::cout << ex.what() << std::endl;
	}
	return map;
}

std::string SensorData::deleteParenthesesFromString(const std::string &input, const std::string &pBegin, const std::string &pEnd)
{
	// 「(」と「)」を削除する．
	// Delete "(" and ")" from message.
	
	// ( を探す
	std::string tmpValuesString = input;
	const std::string::size_type pos1(tmpValuesString.find("("));
	
	// ( が無い場合は入力の文字列を返して終了する．
	if (pos1 == std::string::npos) {
		return input;
	}

	tmpValuesString.replace(pos1, 1, "");
	const std::string::size_type pos2(tmpValuesString.find(")"));
	tmpValuesString.replace(pos2, 1, "");
	const std::string valuesStringDeletedParenthesis = tmpValuesString;
	return tmpValuesString;
}
