/* $Id: Attribute.h,v 1.2 2011-03-31 08:15:56 okamoto Exp $ */ 
#ifndef Attribute_h
#define Attribute_h

#include <string>
#include "Value.h"

/**
 * @brief �����N���X
 *
 * �G�[�W�F���g�̑�����\��
 */
   
class Attribute
{
	typedef std::string S;
private:
	S	m_name;
	Value * m_value;
	S	m_group;
	char  * m_buf;
	int	m_bufsize;
	int	m_voffset;
private:
	int	binarySize() const;
	void	free_();
public:
	/**
	 * @brief �R���X�g���N�^
	 * @param n ������
	 * @param v �l
	 */
	Attribute(S n, Value *v) : m_name(n), m_value(v), m_buf(0), m_bufsize(0) {;}
	/**
	 * @brief �R���X�g���N�^
	 * @param n ������
	 * @param v �����l
	 * @param g �����O���[�v��
	 */
	Attribute(S n, Value *v, S g) : m_name(n), m_value(v), m_group(g), m_buf(0), m_bufsize(0)  {;}
	/**
	 * @brief �f�t�H���g�R���X�g���N�^
	 */
	Attribute() : m_buf(0), m_bufsize(0), m_voffset(0) {;}
	/**
	 * @brief �R�s�[�R���X�g���N�^
	 */
	Attribute(const Attribute &o);
	
	/**
	 * @brief �f�X�g���N�^
	 */
	~Attribute() { free_(); }

	/**
	 * @brief �������𓾂�
	 */
	const char *name() const { return m_name.c_str(); }
	/**
	 * @brief �����O���[�v���𓾂�
	 */
	const char *group() const { return m_group.c_str(); }
	/**
	 * @brief �����l�𓾂�
	 */
	Value & value() const { return *m_value; }

	/**
	 * @brief double�^�̑����l�𓾂�
	 */
	double	getDouble() const { return value().getDouble(); }

	/**
	 * @brief �I�u�W�F�N�g�̓��e�𕶎��񉻂���
	 */ 
	const char *toString();

	/**
	 * @brief �I�u�W�F�N�g�̓��e���o�C�i��������
	 * @param n �o�C�i���f�[�^�T�C�Y
	 * @return �o�C�i���f�[�^
	 */ 
	char *	toBinary(int &n);
	/**
	 * @brief �o�C�i���f�[�^���畜������
	 * @param data �o�C�i�������ꂽ�����f�[�^
	 * @param n    �o�C�i���f�[�^�T�C�Y
	 */
	int	setBinary(char *data, int n);
	/**
	 * @brief �_���v���\�b�h(�f�o�b�O�p)
	 */
	void	dump();
};

#endif // Attribute_h
 
