#include <Common\SensorData.h>

std::map<std::string, std::string> SensorData::convertMessage2Map(const std::string &message, const std::string &keyDelim, const std::string &recordDelim)
{
	std::map<std::string, std::string> map;
	try {
		// まずは「;」でKEY:VALUEごとに文字列を取得する
		std::vector<std::string> pairs;
		boost::split(pairs, message, boost::is_any_of(pairsDelimDefault));

		for(int i = 0; i < (int)pairs.size() - 1; i++) {
			
			// KEY:VALUE となっている文字列から KEY と VALUE を抽出してマップにする
			std::vector<std::string> keyAndValues;
			boost::split(keyAndValues, pairs[i], boost::is_any_of(keyValueDelimDefault));

			map.insert( std::map<std::string, std::string>::value_type(keyAndValues[0], keyAndValues[1]) );
		}

	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
	}
	return map;
}

std::string SensorData::deleteParenthesesFromString(const std::string &input, const std::string &pBegin, const std::string &pEnd)
{
	// 「(」と「)」を削除する．
	// Delete "(" and ")" from message.
	std::string tmpValuesString = input;
	const std::string::size_type pos1(tmpValuesString.find("("));
	tmpValuesString.replace(pos1, 1, "");
	const std::string::size_type pos2(tmpValuesString.find(")"));
	tmpValuesString.replace(pos2, 1, "");
	const std::string valuesStringDeletedParenthesis = tmpValuesString;
	return tmpValuesString;
}