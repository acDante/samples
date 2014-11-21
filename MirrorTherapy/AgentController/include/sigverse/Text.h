/* $Id: Text.h,v 1.2 2011-03-31 08:15:57 okamoto Exp $ */ 
#ifndef Text_h
#define Text_h

#include "Encode.h"

#include <string>

/**
 * @brief �e�L�X�g�f�[�^�N���X
 *
 * �V�~�����[�V�������E���ł��Ƃ肳���e�L�X�g�f�[�^�N���X
 *
 */
    
class Text
{
private:
	Encode m_encode;
	std::string m_text;
public:
	/**
	 * @brief �R���X�g���N�^
	 * @param enc �G���R�[�h
	 * @param text �e�L�X�g������
	 */
	Text(Encode enc, const char *text) : m_encode(enc){
		if (text) {
			m_text = text;
		}
	}

	//! �G���R�[�h���擾����
	Encode getEncode() { return m_encode; }
	//! �e�L�X�g��������擾����
	const char *getString() { return m_text.length() > 0? m_text.c_str(): NULL; }
};


#endif // Text_h
 
