/* $Id: ViewImage.h,v 1.5 2012-02-07 05:41:34 yahara Exp $ */ 
#ifndef ViewImage_h
#define ViewImage_h

#include <stdlib.h>

#include "ViewImageInfo.h"

class ViewImage
{
 private:
        ViewImageInfo	m_info;
	char *		m_buf;
	int		m_buflen;
	
	double m_fov;
	double m_ar;   //aspect ratio
public:
	ViewImage(const ViewImageInfo &info)
		: m_info(info), m_buf(0), m_buflen(0)
	{
		m_buflen = calcBufferSize(info);
		m_buf = new char [m_buflen];
	}

	~ViewImage() {
		if (m_buf) {
			delete [] m_buf; m_buf = 0;
		}
	}

private:
	// sekikawa(2007/10/12)
	int getWidthBytes(int width, int bytesPerOnePixel);

	int	calcBufferSize(const ViewImageInfo &info);

public:
	const ViewImageInfo & getInfo() { return m_info; }

	int  getWidth() const { return m_info.getWidth(); }
	int  getHeight() const { return m_info.getHeight(); }
	char * getBuffer() const { return m_buf; }
	void setBuffer(char *buf) { m_buf = buf; }
	int   getBufferLength() const { return m_buflen; }
	void setFOVy(double fov){ m_fov = fov;} 
	void setAspectRatio(double ar){ m_ar = ar;}
	double getFOVy(){ return m_fov;} 
	double getAspectRatio(){ return m_ar;}

	//virtual void setDimension(int n){} 

	
	bool	saveAsWindowsBMP(const char *fname);

#ifdef WIN32
	// sekikawa(2007/10/12)
	// convert RGBA format to BGR and turn y-axis upside down
	void setBitImageAsWindowsBMP(unsigned char *bitImage);
#endif

#if (defined _DEBUG || defined UNIT_TEST || defined IRWAS_TEST_CLIENT)
	static ViewImage *createSample();
#endif
};

#endif // ViewImage_h
 

