/* $Id: NoData.h,v 1.1.1.1 2011-03-25 02:18:50 okamoto Exp $ */ 
#ifndef CommController_NoData_h
#define CommController_NoData_h

/**
 * @brief �f�[�^�̂Ȃ��C�x���g�N���X
 *
 * �C�x���g�̒ʒm�݂̂��s���A�f�[�^�������Ȃ��R���g���[���C�x���g�B
 */
class NoDataEvent
{
public:
	bool	set(int packetNum, int seq, char *data, int n) { return true;} 
};

/**
 * @brief �R���g���[��onInit �C�x���g�N���X
 *
 * @see CommData::InvokeOnInit
 */
class InitEvent : public NoDataEvent {};

#endif // CommController_NoData_h
 
