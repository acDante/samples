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

/**
 * @brief �摜�f�[�^���N���X
 */
class ViewImageInfo
{
private:
	ImageDataType	m_dataType;
	ColorBitType	m_cbType;
	int		m_width;
	int		m_height;

public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param dataType �摜�f�[�^�^�C�v
	 * @param cbType   �J���[�r�b�g�^�C�v
	 * @param sz       �摜�T�C�Y
	 */
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
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param dataType �摜�f�[�^�^�C�v
	 * @param cbType   �J���[�r�b�g�^�C�v
	 * @param w        ��(pixel)
	 * @param h        ����(pixel)
	 */
	ViewImageInfo(ImageDataType dataType, ColorBitType cbType, int w, int h)
		: m_dataType(dataType), m_cbType(cbType), m_width(w), m_height(h)
	{
	}
	//! �R�s�[�R���X�g���N�^
	ViewImageInfo(const ViewImageInfo &o)
		: m_dataType(o.m_dataType),
		  m_cbType(o.m_cbType),
		  m_width(o.m_width), m_height(o.m_height)
	{
	}
	//! �摜�f�[�^�`��(BMP etc...)�𓾂�
	ImageDataType getDataType() const { return m_dataType; }
	//! �J���[�^�C�v�𓾂�
	ColorBitType  getColorBitType() const { return m_cbType; }
	//! �摜�̕�(pixel)�𓾂�
	int	      getWidth() const { return m_width; }
	//! �摜�̍���(pixel)�𓾂�
	int	      getHeight() const { return m_height; }

	//! 1�s�N�Z��������̃f�[�^�T�C�Y�𓾂�
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
 
