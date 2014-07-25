/* $Id: Size.h,v 1.2 2011-03-31 08:15:57 okamoto Exp $ */ 
#ifndef Size_h
#define Size_h

#include "Position.h"

class Size : public Position
{
	typedef Position Super;
public:
	Size() : Super() {;}
	Size(dReal x_, dReal y_, dReal z_) : Super(x_, y_, z_) {;}
	Size(const Size &s) : Super(s) {;}
};


#endif // Size_h
 

