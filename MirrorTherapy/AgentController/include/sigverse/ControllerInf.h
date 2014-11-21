/* $Id: ControllerInf.h,v 1.2 2011-03-31 08:15:56 okamoto Exp $ */ 
#ifndef ControllerInf_h
#define ControllerInf_h

class InitEvent;
class ActionEvent;
class RecvTextEvent;
class RecvSoundEvent;
class RecvMessageEvent;
class RecvMsgEvent;
class CollisionEvent;

/**
 * @brief �R���g���[���C���^�[�t�F�[�X
 */
class ControllerInf
{
public:
 
 ControllerInf(): m_startSim(false), m_excon(false) {;}

	//! �f�X�g���N�^
	virtual ~ControllerInf() {;}

	/**
	 * @brief �R���g���[���̏������R�[���o�b�N�֐�
	 *
	 * �V�~�����[�V�����J�n���ɌĂяo�����
	 */
	virtual void 	onInit(InitEvent &evt) {}
	
	/**
	 * @brief �s���R�[���o�b�N�֐�
	 *
	 * �G�[�W�F���g�̎�ȓ�������������B�ʏ�A�V�~�����[�V�������Ԃ��i�ޖ��ɌĂяo�����B
	 *
	 * @return ���ɌĂяo�����܂ł̎��ԁB
	 */
	virtual double 	onAction(ActionEvent &evt) { return 0.0; }

	//! �e�L�X�g��M�R�[���o�b�N�֐�
	virtual void 	onRecvText(RecvTextEvent &evt) {}
	//! ������M�R�[���o�b�N�֐�
	virtual void	onRecvSound(RecvSoundEvent &evt) {}
	//old
	virtual void	onRecvMessage(RecvMessageEvent &evt) {}
	//! ���b�Z�[�W��M�R�[���o�b�N�֐�(2012/9/10�ǉ�)
	virtual void	onRecvMsg(RecvMsgEvent &evt) {}
	//! �Փˌ��o�R�[���o�b�N�֐� 
	virtual void 	onCollision(CollisionEvent &evt) {}

#ifndef UNIT_TEST
	//! �R���g���[�����������ł��邩�H
	virtual	bool	isProcessing() = 0;
	//! �R�[���o�b�N�֐����Ăяo���O�̏�������������
	virtual void	onPreEvent() = 0;
	//! �R�[���o�b�N�֐����Ăяo������̏�������������
	virtual void	onPostEvent() = 0;
#else
	virtual	bool	isProcessing() { return false; };
	virtual void	onPreEvent() {};
	virtual void	onPostEvent() {};
#endif


	//! �V�~�����[�V�������N�������ǂ����擾����
	bool getSimState(){ return m_startSim; }

	//! �V�~�����[�V�������N�������ǂ����ݒ肷��
	void setSimState(bool sim){ m_startSim = sim; }

	// ���̃X���b�h���֐����g�p�����ǂ������擾����
	bool getExCon(){ return m_excon; }

	// �֐����g�p���ł��邱�Ƃ�o�^����
	void setExCon(bool excon){ m_excon = excon; }

 protected:
	//! �V�~�����[�V�����N�������ǂ���
	bool m_startSim;

	//! �r������p�ϐ�
	bool m_excon;
};


#endif // ControllerInf_h
 
 
