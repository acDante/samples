/* $Id: ViewImage.h,v 1.5 2012-02-07 05:41:34 yahara Exp $ */ 
#ifndef ViewImage_h
#define ViewImage_h

#include <stdlib.h>

#include "ViewImageInfo.h"

/**
 * @brief ���o�f�[�^�N���X
 */
class ViewImage
{
 private:
        ViewImageInfo	m_info;
	char *		m_buf;
	int		m_buflen;
	
	double m_fov;
	double m_ar;   //aspect ratio
public:
	/**
	 *  @brief �R���X�g���N�^
	 *
	 *  @param info ���o�f�[�^���
	 */
	ViewImage(const ViewImageInfo &info)
		: m_info(info), m_buf(0), m_buflen(0)
	{
		m_buflen = calcBufferSize(info);
		m_buf = new char [m_buflen];
	}

	//! �f�X�g���N�^
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
	//! ���o�f�[�^���̎擾
	const ViewImageInfo & getInfo() { return m_info; }

	//! �摜�̕�(pixel)�̎擾
	int  getWidth() const { return m_info.getWidth(); }
	//! �摜�̍���(pixel)�̎擾
	int  getHeight() const { return m_info.getHeight(); }
	//! �摜�f�[�^�̎擾
	char * getBuffer() const { return m_buf; }
	//! �摜�f�[�^�ݒ�
	void setBuffer(char *buf) { m_buf = buf; }
	//! �摜�f�[�^���̎擾
	int   getBufferLength() const { return m_buflen; }
	//! ����p�ݒ�
	void setFOVy(double fov){ m_fov = fov;} 
	//! �A�X�y�N�g��ݒ�
	void setAspectRatio(double ar){ m_ar = ar;}
	//! ����p�擾
	double getFOVy(){ return m_fov;} 
	//! �A�X�y�N�g��擾
	double getAspectRatio(){ return m_ar;}

	//virtual void setDimension(int n){} 

	/**
	 * @brief �摜�f�[�^�� Windows �r�b�g�}�b�v�`���ŕۑ�����
	 *
	 * @param fname �ۑ���t�@�C����
	 */
	
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

/*
class DepthImage : public ViewImage
{
 public:
 DepthImage(const ViewImageInfo &info)
   : ViewImage(info)
  {
  }
  //! �s�N�Z���̈ʒu����depth�f�[�^�擾
  unsigned char getDepthFromPixel(int w, int h=0);

  //! �s�N�Z���̈ʒu���狗���f�[�^�擾
  double getDistanceFromPixel(int w, int h=0);

  //! �����i�p�x�j���狗���f�[�^�擾
  unsigned char getDepthFromAngle(double theta, double phi=0);

  void setDimension(int dim){m_dim = dim;}
 private:

  //dimension
  int m_dim;

};
*/
#endif // ViewImage_h
 
