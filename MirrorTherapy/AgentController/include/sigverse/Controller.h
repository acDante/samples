/* $Id: Controller.h,v 1.19 2011-12-14 08:13:44 okamoto Exp $ */ 
#ifndef Controller_h
#define Controller_h

#include <string>
#include <vector>
#include <map>

#include "ControllerImpl.h"
#include "ViewImageInfo.h"
#include "SimObj.h"

//#include "DynamicsController.h"


class ViewImage;
class DepthImage;
class Text;
class RawSound;





class DynamicsController {
public:
	DynamicsController();
	/**
	 * @brief Webots�݊��̃p�����[�^��ݒ肵�܂��B�ϐ��̐擪��left�������͍̂����ԗւ̐ݒ�Aright�̂����͉̂E���ԗւ̐ݒ�
	 *
	 * @param leftWheelName          �ԗւ̃W���C���g��
	 * @param leftMotorConsumption   �ԗւ̏���d��(W)
	 * @param leftWheelMaxSpeed      �ԗւ̍ő�p���x(rad/s)
	 * @param leftWheelSpeedUnit     wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param leftSlipNoise          ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param leftEncoderResolution  1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param leftMaxForce           �ő�g���N
	 * @param rightWheelName         �ԗւ̃W���C���g��
	 * @param rightMotorConsumption  �ԗւ̏���d��(W)
	 * @param rightWheelMaxSpeed     �ԗւ̍ő�p���x(rad/s)
	 * @param rightWheelSpeedUnit    wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param rightSlipNoise         ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param rightEncoderResolution 1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param rightMaxForce          �ő�g���N
	 * @param axleLength             �ԗ֊Ԃ̋���
	 */
	void setWheelProperty(
		SimObj *my,
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	/**
	 * ���E�̎ԗւɂ��ꂼ�ꂱ�ƂȂ�X�s�[�h��^����
	 * @param left ���ԗւ̊p���x
	 * @param right �E�ԗւ̊p���x
	 */
	void differentialWheelsSetSpeed(SimObj *my,double left,double right);

	/**
	 * �Ԏ��̒������擾���܂�
	 */
	double getAxleLength();

	/**
	 * ���ԗւ̔��a���擾���܂�
	 */
	double getLeftWheelRadius(SimObj *my);

	/**
	 * �E�ԗւ̔��a���擾���܂�
	 */
	double getRightWheelRadius(SimObj *my);

	/**
	 * ���ԗւ̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getLeftEncoderNoise();

	/**
	 * �E�ԗւ̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getRightEncoderNoise();

	/**
	 * ���ԗւ̃X���b�v���擾���܂�
	 */
	double getLeftSlipNoise();

	/**
	 * �E�ԗւ̃X���b�v���擾���܂�
	 */
	double getRightSlipNoise();

	/**
	 * ���݂̍��ԗւ̃X�s�[�h���擾���܂�
	 */
	double getCurrentLeftWheelSpeed();

	/**
	 * ���݂̉E�ԗւ̃X�s�[�h���擾���܂�
	 */
	double getCurrentRightWheelSpeed();

private:

	#define ACCURACY (0.00000001)

	/** �����̎ԗւ̖��O */
	char   *leftWheelName;
	/** ���ԗւ̓d��[W] */
	double leftMotorConsumption;
	/** ���ԗւ̍ő�p���x[rad/s] */
	double leftWheelMaxSpeed;
	/** ���ԗւ̑��x�P�ʐݒ� */
	double leftWheelSpeedUnit;
	/** ���ԗւ̂��ׂ� */
	double leftSlipNoise;
	/** ���ԗւ̃X���b�v�m�C�Y�̒~�ϗ� */
	//double leftCumulativeNoise;
	/** 1[rad]������ǂ̒��x�̃X���b�v�������邩 */
	double leftEncoderResolution;
	/** �ő�g���N */
	double leftMaxForce;
	/** �E���̎ԗւ̖��O */
	char   *rightWheelName;
	/** �E���̃��[�^�̏���d�� */
	double rightMotorConsumption;
	/** �E���̃z�C�[���̍ő�p���x */
	double rightWheelMaxSpeed;
	/** �E�ԗւ̑��x�P�ʐݒ� */
	double rightWheelSpeedUnit;
	double rightSlipNoise;
	//double rightCumulativeNoise;
	double rightEncoderResolution;
	double rightMaxForce;


	/** �ԗ֊Ԃ̋��� */
	double axleLength;
	/** ���ԗւ̔��a[m] */
	double leftWheelRadius;
	/** �E���̃z�C�[���̔��a */
	double rightWheelRadius;
	/** ���ԗւ̃m�C�Y�̒~�� */
	double leftEncoderNoise;
	/** �E�ԗւ̃m�C�Y�̒~�� */
	double rightEncoderNoise;

	/** ���݂̍��ԗւ̑��x */
	double currentLeftWheelSpeed;
	/** ���݂̉E�ԗւ̑��x */
	double currentRightWheelSpeed;

};





/**
 * @brief ���[�U��`�R���g���[����{�N���X
 *
 * �G�[�W�F���g�R���g���[���͂��̃N���X���p�������N���X����
 * �����A�����ɃG�[�W�F���g�̐U�������L�q����B
 */
class Controller : public ControllerImpl, public SimObj::RequestSender
{
private:
	typedef ControllerImpl Super;
	typedef std::string S;
	typedef std::map<S, SimObj*> M;
	
private:
	M		m_objs;
	bool		m_in;
private:
	SimObj *	find(const char *name)
	{
	  return m_objs[name];
	}
 public:
	void	updateObjs();
	void	clearObjs();
private:
	bool			send(CommDataEncoder &);
	CommDataResult *	recv(int bufsize);
	SOCKET getControllerSocket() { SOCKET sock; return sock; }

protected:
	//! �R���X�g���N�^
	Controller();

	/**
	 * 0 - 1�܂ł̈�l���z�̗����𐶐����܂�
	 */
	double getRand();

	/**
	 * @brief Webots�݊��̃p�����[�^��ݒ肵�܂��B�ϐ��̐擪��left�������͍̂����ԗւ̐ݒ�Aright�̂����͉̂E���ԗւ̐ݒ�
	 *
	 * @param leftWheelName          �ԗւ̃W���C���g��
	 * @param leftMotorConsumption   �ԗւ̏���d��(W)
	 * @param leftWheelRadius        �ԗւ̔��a(m)
	 * @param leftWheelMaxSpeed      �ԗւ̍ő�p���x(rad/s)
	 * @param leftWheelSpeedUnit     wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param leftSlipNoise          ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param leftEncoderNoise       �X���b�v�m�C�Y�̒~��
	 * @param leftEncoderResolution  1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param leftMaxForce           �ő�g���N
	 * @param rightWheelName         �ԗւ̃W���C���g��
	 * @param rightMotorConsumption  �ԗւ̏���d��(W)
	 * @param rightWheelRadius       �ԗւ̔��a(m)
	 * @param rightWheelMaxSpeed     �ԗւ̍ő�p���x(rad/s)
	 * @param rightWheelSpeedUnit    wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param rightSlipNoise         ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param rightEncoderNoise      �X���b�v�m�C�Y�̒~��
	 * @param rightEncoderResolution 1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param rightMaxForce          �ő�g���N
	 */
	void setWheelProperty(
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	/**
	 * @brief Webots�݊��̃p�����[�^��ݒ肵�܂��B�ϐ��̐擪��left�������͍̂����ԗւ̐ݒ�Aright�̂����͉̂E���ԗւ̐ݒ�
	 *
	 * @param objectName             �G�[�W�F���g�̖��O
	 * @param leftWheelName          �ԗւ̃W���C���g��
	 * @param leftMotorConsumption   �ԗւ̏���d��(W)
	 * @param leftWheelRadius        �ԗւ̔��a(m)
	 * @param leftWheelMaxSpeed      �ԗւ̍ő�p���x(rad/s)
	 * @param leftWheelSpeedUnit     wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param leftSlipNoise          ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param leftEncoderNoise       �X���b�v�m�C�Y�̒~��
	 * @param leftEncoderResolution  1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param leftMaxForce           �ő�g���N
	 * @param rightWheelName         �ԗւ̃W���C���g��
	 * @param rightMotorConsumption  �ԗւ̏���d��(W)
	 * @param rightWheelRadius       �ԗւ̔��a(m)
	 * @param rightWheelMaxSpeed     �ԗւ̍ő�p���x(rad/s)
	 * @param rightWheelSpeedUnit    wb_differential_wheels_set_speed �Őݒ肳��鑬�x�̒P�ʂ��Z�b�g(rad/s)
	 * @param rightSlipNoise         ��l���z�ɂ��������ԗւ̃X���b�v(0.0 - 1.0)
	 * @param rightEncoderNoise      �X���b�v�m�C�Y�̒~��
	 * @param rightEncoderResolution 1[rad]������ǂ̒��x�̃X���b�v�������邩
	 * @param rightMaxForce          �ő�g���N
	 */
	void setWheelProperty(
		const char   *objectName,
		char   *leftWheelName,
		double leftMotorConsumption,
		double leftWheelMaxSpeed,
		double leftWheelSpeedUnit,
		double leftSlipNoise,
		double leftEncoderResolution,
		double leftMaxForce,
		char   *rightWheelName,
		double rightMotorConsumption,
		double rightWheelMaxSpeed,
		double rightWheelSpeedUnit,
		double rightSlipNoise,
		double rightEncoderResolution,
		double rightMaxForce
	);

	/**
	 * �������g�̎Ԏ��̒������擾���܂�
	 */
	double getAxleLength();

	/**
	 * �Ԏ��̒������擾���܂�
	 */
	double getAxleLength(const char *simObjName);

	/**
	 * �������g�̍��ԗւ̔��a���擾���܂�
	 */
	double getLeftWheelRadius();

	/**
	 * ���ԗւ̔��a���擾���܂�
	 */
	double getLeftWheelRadius(const char *simObjName);

	/**
	 * �������g�̉E�ԗւ̔��a���擾���܂�
	 */
	double getRightWheelRadius();

	/**
	 * �E�ԗւ̔��a���擾���܂�
	 */
	double getRightWheelRadius(const char *simObjName);

	/**
	 * �������g�̍��ԗւ̃G���R�[�_�̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getLeftEncoderNoise();

	/**
	 * ���ԗւ̃G���R�[�_�̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getLeftEncoderNoise(const char *simObjName);

	/**
	 * �������g�̉E�ԗւ̃G���R�[�_�̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getRightEncoderNoise();

	/**
	 * �E�ԗւ̃G���R�[�_�̃m�C�Y�~�ϗʂ��擾���܂�
	 */
	double getRightEncoderNoise(const char *simObjName);

	/**
	 * �������g�̍��E�̎ԗւɂ��ꂼ�ꂱ�ƂȂ�X�s�[�h��^����
	 * @param left ���ԗւ̊p���x
	 * @param right �E�ԗւ̊p���x
	 */
	void differentialWheelsSetSpeed(double left,double right);

	/**
	 * ���E�̎ԗւɂ��ꂼ�ꂱ�ƂȂ�X�s�[�h��^����
	 * @param simObjName SimObj�̖��O
	 * @param left ���ԗւ̊p���x
	 * @param right �E�ԗւ̊p���x
	 */
	void differentialWheelsSetSpeed(const char *simObjName,double left,double right);

	/**
	 * @brief �w��̃G���e�B�e�B���擾����
	 * @param name �G���e�B�e�B��
	 */
	SimObj *	getObj(const char *name);

	/**
	 * @brief �V�~�����[�V�������Ԃ��擾����
	 */
	double          getSimulationTime();

	/**
	 * @brief robot�G���e�B�e�B���擾����
	 * @param name �G���e�B�e�B��
	 */
	RobotObj *	getRobotObj(const char *name);

	// old
	ViewImage * 	captureView(ColorBitType cbtype, ImageDataSize size);

	// old
	ViewImage * 	captureView(ColorBitType cbtype, ImageDataSize size, int id);

	// old
	bool	       	detectEntities(std::vector<std::string> &v);

	// old
	bool	       	detectEntities(std::vector<std::string> &v, int id);

	/**
	 * @brief SIGVerse���E��1�X�e�b�v�X�V����
	 * @param stepsize �X�e�b�v��(sec)
	 */
	void            worldStep(double stepsize);

	/**
	 * @brief SIGVerse���E��1�X�e�b�v�X�V����(worldStep�ɔ�ׂđ��x�����������x�͒Ⴂ)
	 * @param stepsize �X�e�b�v��(sec)
	 */
	void            worldQuickStep(double stepsize);

	/**

	 * @brief �����f�[�^�������F�������Ɋ|����
	 *
	 * �����F���T�[�r�X�v���o�C�_�ɉ����f�[�^��]�����A�����F������(�e�L�X�g)�𓾂�B
	 *
	 * @param sound  recvSound���\�b�h�ɂ��擾���������f�[�^
	 *
	 * @retval !=NULL ����
	 * @retval NULL ���s
	 */
	Text *		getText(RawSound &sound);

	/**
	 * @brief �V�~�����[�V�������E���ɑ��݂��邷�ׂẴG���e�B�e�B�̖��O���擾����
	 * 
	 * @param v �G���e�B�e�B�����i�[����R���e�i
	 * @retval true ����
	 * @retval false ���s
	 */
	bool		getAllEntities(std::vector<std::string> &v);

	/**
	 * @brief  �ڑ�����T�[�r�X��
	 * 
	 * @param  name �ڑ�����T�[�r�X��
	 * @retval true ����
	 * @retval false ���s
	 */
	//bool		connectToService(std::string name);


	bool	isProcessing()
	{
		return m_in;
	}
	void	onPreEvent()
	{
		m_in = true;
	}
	void	onPostEvent()
	{
	  //updateObjs();
	  //clearObjs();
	  m_in = false;
	}

private:
	typedef std::map<std::string, DynamicsController*> DYNAMICS_CONTROLLER_LIST;
	DYNAMICS_CONTROLLER_LIST dynamicsDataList;


	void	add(SimObj *obj);

	/**
	 * �ԗ��̃_�C�i�~�N�X�𗘗p���邩�ǂ���
	 */
	enum{
		MODE_NOT_USE_WHEEL = 0,	//	�ԗ��_�C�i�~�N�X���g��Ȃ�
		MODE_USE_WHEEL		//	�ԗ��_�C�i�~�N�X���g�p
	};
	/**
	 * �ԗ��͊w���[�h�̃t���O
	 */
	int dynamicsMode;

	void slipWheel();

	//
	// ���ԗւ��X���b�v�����܂��B
	// �X���b�v�ʂ�leftSlipNoise����Z�o�����B
	//
	//void slipLeftWheel();

	//
	// �E�ԗւ��X���b�v�����܂��B
	// �X���b�v�ʂ�leftSlipNoise����Z�o�����B
	//
	//void slipRightWheel();

	//
	// �p���x�Ɠd�͂���A�g���N���Z�o����B
	// ���[�^�̌v�Z���͉��L�̎��ɏ]���Ƃ���B
	//             T
	// P = F�Er�� = �[�[ �E r�� = 2��Tf
	//             r
	// P : ����d��[W]
	// F : �^�C���ɂ������[N]
	// r : �^�C���̔��a[m]
	//�� : �p���g��[rad/sec]
	// T : �g���N[N�Em]
	// f : ���g��(��]��)[Hz]
	//
	// @param motorConsumption ���[�^�̓d��[W]
	// @param radius �ԗւ̔��a[m]
	// @param wheelSpeed �ԗւ̊p���x[rad/sec]
	// @return �ԗւɂ�����g���N[N�Em]
	//
	//double getTorqueFromMotor(double motorConsumption,double radius,double wheelSpeed);

	//
	// @brief ���[�^�𓮂����܂�
	// @param jointName        �W���C���g��
	// @param motorConsumption �d��[W]
	///
	//void moveMotor(char *jointName,double motorConsumption);

public:
	/**
	 * @brief �G�[�W�F���g�ɃR���g���[�������蓖�Ă�
	 *
	 * �V�~�����[�V�����T�[�o�ɐڑ����A�w�肵���G�[�W�F���g�ɓ��R���g���[�������蓖�Ă�
	 *
	 * @param server  �V�~�����[�V�����T�[�o(IP�A�h���X�܂��̓z�X�g��)
	 * @param port    �V�~�����[�V�����T�[�o�|�[�g�ԍ�
	 * @param myname  �G�[�W�F���g��
	 * 
	 */
	bool	attach(const char *server, int port, const char *myname);
	//! �R���g���[�����[�v�֐�
	void	loopMain();

	//!! �R���g���[���������֐�
	static void init();
};


#endif // Controller_h
 
