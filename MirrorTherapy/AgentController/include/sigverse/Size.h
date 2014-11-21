/* $Id: Size.h,v 1.2 2011-03-31 08:15:57 okamoto Exp $ */ 
#ifndef Size_h
#define Size_h

#include "Position.h"

//! �V�~�����[�V�������E���ɂ�����3���������̂̑傫����\���N���X
class Size : public Position
{
	typedef Position Super;
public:
	//! �R���X�g���N�^
	Size() : Super() {;}
	/**
	 * @brief �R���X�g���N�^
	 * @param x X�����̕ӂ̒���
	 * @param y Y�����̕ӂ̒���
	 * @param z Z�����̕ӂ̒���
	 */
	Size(dReal x_, dReal y_, dReal z_) : Super(x_, y_, z_) {;}
	//! �R�s�[�R���X�g���N�^
	Size(const Size &s) : Super(s) {;}
};


#endif // Size_h
 
