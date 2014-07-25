/* $Id: ViewImageInfo.h,v 1.3 2011-09-16 03:54:26 okamoto Exp $ */ 
#ifndef ViewImageInfo_h
#define ViewImageInfo_h

#include <assert.h>

typedef unsigned short ImageDataType;

enum {
	IMAGE_DATA_TYPE_ANY = 0,
	IMAGE_DATA_WINDOWS_BMP,
};


typedef unsigned short ColorBitType;

enum {
	COLORBIT_ANY = 0,
	COLORBIT_24,
	DEPTHBIT_8,
};

enum ImageDataSize {
	IMAGE_320X240 = 0,
	IMAGE_320X1,
};

class ViewImageInfo
{
private:
	ImageDataType	m_dataType;
	ColorBitType	m_cbType;
	int		m_width;
	int		m_height;

public:
	ViewImageInfo(ImageDataType dataType, ColorBitType cbType, ImageDataSize sz) : m_dataType(dataType), m_cbType(cbType)
	{
		
		switch(sz) {
		case IMAGE_320X240:
			m_width = 320; m_height = 240;
			break;
		case IMAGE_320X1:
			m_width = 320; m_height = 1;
			break;
		default:
			assert(0);
			break;
		}
	}
	ViewImageInfo(ImageDataType dataType, ColorBitType cbType, int w, int h)
		: m_dataType(dataType), m_cbType(cbType), m_width(w), m_height(h)
	{
	}
	ViewImageInfo(const ViewImageInfo &o)
		: m_dataType(o.m_dataType),
		  m_cbType(o.m_cbType),
		  m_width(o.m_width), m_height(o.m_height)
	{
	}
	ImageDataType getDataType() const { return m_dataType; }
	ColorBitType  getColorBitType() const { return m_cbType; }
	int	      getWidth() const { return m_width; }
	int	      getHeight() const { return m_height; }

	int	getBytesPerOnePixel() const {
		int b;
		switch(m_cbType) {
			case COLORBIT_24:
				b = 3; break;
			case DEPTHBIT_8:
				b = 1; break;
			default:
				assert(0);	// error
				b = 0; break;
		}

		return b;
	}
};


#endif // ViewImageInfo_h
 

