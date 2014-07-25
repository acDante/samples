/* $Id: Encode.h,v 1.2 2011-03-31 08:15:56 okamoto Exp $ */ 
#ifndef Encode_h
#define Encode_h

enum {
	ENCODE_NOT_SET = -1,
	ENCODE_ASCII,
	ENCODE_EUCJP,
	ENCODE_JIS,
	ENCODE_SHIFT_JIS,
	ENCODE_UTF8,
	ENCODE_NUM,
};

typedef short Encode;


#endif // Encode_h
 
