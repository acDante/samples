/* $Id: ControllerImpl.h,v 1.6 2012-03-27 04:07:53 noma Exp $ */
#ifndef ControllerImpl_h
#define ControllerImpl_h

#include "ControllerInf.h"
#include "RawSound.h"
#include "systemdef.h"
#include "Logger.h"
#include "ViewImage.h"
#include <vector>
#include <map>
#include <string>


class Command;
class CTSimObj;
class ControllerImpl;

namespace SocketUtil {
  bool sendData(SOCKET sock, const char* msg, int size);
  bool recvData(SOCKET sock, char* msg, int size);
}
//! �T�[�r�X�v���o�C�_�ƃf�[�^�̑���M���s���N���X
class BaseService 
{
 public:
  BaseService(){;} 
  
 BaseService(std::string name, unsigned short port, SOCKET sock) 
   : m_name(name), m_port(port), m_sock(sock), m_getData(false) {;}
  ~BaseService(){
    /*
    if(m_image != NULL) {
      delete m_image;
      m_image = NULL;
    }
    */
  }
  
  //! �f�[�^��M���[�v���J�n����
  bool startServiceLoop(ControllerImpl *con);

  //! �f�[�^��M���[�v���I������
  void endServiceLoop();
  
  //! �T�[�r�X�v���o�C�_�̖��O�擾   
  std::string getName() { return m_name; }

  //! �T�[�r�X�v���o�C�_��t�\�P�b�g���擾����
  SOCKET getSocket() { return m_sock; }

  //! �T�[�r�X�v���o�C�_�ڑ��\�P�b�g���擾����
  SOCKET getClientSocket() { return m_clientSock; }

  //! �T�[�r�X�v���o�C�_�ڑ��\�P�b�g�̐ݒ�
  void setClientSocket(SOCKET sock) { m_clientSock = sock; }

  //! ���b�Z�[�W���M
  bool sendMsgToSrv(std::string msg);

  //! �f�[�^��M�������Ƃ�m�点��
  void setGetData(bool get){ m_getData = get; }

  //! ���o�����I�u�W�F�N�g����ݒ肷��
  void setDetectedNames(std::vector<std::string> names) { m_detectedNames = names; }
  /*
  ViewImage* getImage() {
    return m_image;
  }
  */
  //! �T�[�r�X�𗘗p����G���e�B�e�B�̖��O�擾
  std::string getEntityName() { 
    return m_entname; 
  }

  //! �T�[�r�X�𗘗p����G���e�B�e�B�̖��O�擾
  void setEntityName(std::string name) {
    m_entname = name;
  }

 protected:
  // �ڑ���̃T�[�r�X��
  std::string m_name;

  // port number(�R���g���[��)
  unsigned short m_port;

  // socket
  SOCKET m_sock;

  // �N���C�A���g���\�P�b�g
  SOCKET m_clientSock;

  //! �T�[�r�X���f�[�^����M���Ă��邩
  bool m_getData;

  // �T�[�r�X�𗘗p����G���e�B�e�B��
  std::string m_entname;

  // �摜�f�[�^
  std::vector<ViewImage*> m_images;

  // �����f�[�^
  unsigned char         m_distance;

  // detectEntities�̌���
  std::vector<std::string> m_detectedNames;
};

class ViewService : public BaseService
{


 public:

  ViewService(){;} 
  
 ViewService(std::string name, unsigned short port, SOCKET sock) 
   : BaseService(name, port, sock) {;}
  
  /**
   * @brief ������ɑ��݂���G���e�B�e�B�̌��o(�T�[�r�X�v���o�C�_�@�\�g�p)
   *
   * @param v �G���e�B�e�B�����i�[����R���e�i
   * @param id �J����ID
   * @retval true ����
   * @retval false ���s
   */
  bool  detectEntities(std::vector<std::string> &v, int id = 1);

  /**
   * @brief �G�[�W�F���g���_�摜���擾����
   * @param camID �J����ID
   * @param ctype �摜�^�C�v
   * @param size  �摜�T�C�Y
   * @retval != NULL �摜�f�[�^
   * @retval NULL    ���s
   */
  ViewImage *captureView(int camID = 1, ColorBitType ctype = COLORBIT_24, ImageDataSize size = IMAGE_320X240);
  
  /**
   * @brief �J�������������ɂ���I�u�W�F�N�g�܂ł̋����𓾂�
   * @param start �ŏ�����(�߂�l��0�ƂȂ鋗��)
   * @param end   �ő勗��(�߂�l��255�ƂȂ鋗��)
   * @param ctype �r�b�g�[�x
   * @param camID �J����ID
   * @retval 0~255 �����f�[�^
   */
  unsigned char distanceSensor(double start = 0.0, double end = 255.0, int camID =1, ColorBitType ctype = DEPTHBIT_8);

  /**
   * @brief �J����������ɂ���I�u�W�F�N�g�܂ł�1���������f�[�^�𓾂�
   * @param start �ŏ�����(�߂�l��0�ƂȂ鋗��)
   * @param end   �ő勗��(�߂�l��255�ƂȂ鋗��)
   * @param camID �J����ID
   * @param ctype �摜�^�C�v
   * @param size  �����f�[�^�T�C�Y
   */
  ViewImage *distanceSensor1D(double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X1);

  /**
   * @brief �J����������ɂ���I�u�W�F�N�g�܂ł̋����f�[�^�̂Q���f�[�^�𓾂�
   * @param start �ŏ�����(�߂�l��0�ƂȂ鋗��)
   * @param end   �ő勗��(�߂�l��255�ƂȂ鋗��)
   * @param camID �J����ID
   * @param ctype �摜�^�C�v
   * @param size  �摜�T�C�Y
   */
  ViewImage *distanceSensor2D(double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X240);

 private:
  /**
  * @brief distanceSensor���N�G�X�g���T�[�r�X�v���o�C�_�ɑ��M����
  * @param type ���N�G�X�g�^�C�v�i�擾����f�[�^�̎����j
  * @param start �ŏ�����(�߂�l��0�ƂȂ鋗��)
  * @param end   �ő勗��(�߂�l��255�ƂȂ鋗��)
  * @param camID �J����ID
  * @param ctype �摜�^�C�v
  * @param size  �摜�T�C�Y
  */
  bool sendDSRequest(int type, double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X240);

};

/**
 * @brief �l�b�g���[�N�ʐM�R���g���[���N���X
 *
 * �V�~�����[�V�����T�[�o�Ƃ̒ʐM���s�Ȃ��R���g���[���N���X
 */
class ControllerImpl : public ControllerInf
{
 private:
  typedef ControllerInf Super;

 private:
  std::string 	m_server;
  int		m_port;
  std::string 	m_myname;
  
  bool m_isAttached;	// added by sekikawa(2007/10/15)


 protected:
  SOCKET		m_cmdSock;
  SOCKET	       m_dataSock;
  SOCKET                m_srvSock; //�T�[�r�X�v���o�C�_�p
  unsigned short        m_srvPort; //�T�[�r�X�v���o�C�_�p�|�[�g�ԍ�
  SOCKET                m_tmpSock; // tmpSock

  // �T�[�r�X�v���o�C�_���Ƃ̃\�P�b�g�ۑ�
  std::map<std::string, SOCKET> m_srvSocks; 

  // �T�[�r�X�v���o�C�_�ڑ�������
  bool m_connected;



  // �T�[�r�X�v���o�C�_�ɐڑ������������ǂ���
  bool           m_connectService;

  // �T�[�r�X�v���o�C�_�p�\�P�b�g<�T�[�r�X���A�\�P�b�g>
  std::vector<BaseService*> m_services;

  // changed by sekikawa (2007/10/16)
  // (********* this is only experiment. need to recover later *********)
  //private:
 private:
  CTSimObj * m_ctSimObj;
 protected:
  CTSimObj & getCTSimObj();

  /**
   * @brief �T�[�r�X�v���o�C�_�ɐڑ�����(�|�[�g�ԍ��̓��C���|�[�g+1)
   *        
   * @param name	  �T�[�r�X�v���o�C�_��
   * @retval BaseService  �T�[�r�X
   * @retval NULL �@�@�@�@���s
   */
  BaseService* connectToService(std::string name);

  /**
   * @brief �T�[�r�X�v���o�C�_�ɐڑ�����(�|�[�g�ԍ��w��)
   *        
   * @param name	  �T�[�r�X�v���o�C�_��
   * @param port          �T�[�o���Ŏg�p����|�[�g�ԍ�
   * @retval BaseService  �T�[�r�X
   * @retval NULL �@�@�@�@���s
   */
  BaseService* connectToService(std::string name, unsigned short port);

  /**
   * @brief �T�[�r�X�v���o�C�_�Ɛؒf����
   *        
   * @param name	  �T�[�r�X�v���o�C�_��
   */
  void disconnectToService(std::string name);

  /**
   * @brief  �T�[�r�X�v���o�C�_���g�p�\���T�[�o�ɖ₢���킹��
   * @param  name  ���ׂ����T�[�r�X��  
   * @retval true  �T�[�r�X�g�p�\
   * @retval false �T�[�r�X�g�p�s��
   */
  bool checkService(std::string name);
  
 private:
  // bool recvData(SOCKET sock, char *msg, int size);


 public:
  static void *serviceThread(void *pParam);


 protected:
  void	close_();
	
 protected:
  //! �R���X�g���N�^
 ControllerImpl()
   : Super(),
    m_port(-1), 
    m_isAttached(false), 
    m_cmdSock(-1), 
    m_dataSock(-1),
    m_srvSock(-1), 
    m_tmpSock(-1), 
    m_ctSimObj(NULL), 
    m_connectService(false),
    m_connected(false){;}

  //! �f�X�g���N�^
  ~ControllerImpl() {
    close_();
  }

 public:
  // �T�[�r�X�v���o�C�_�ɐڑ������ǂ���
  bool connected() {return m_connected;}

  // tmp�\�P�b�g�ݒ�
  void setTmpSock(SOCKET sock) {m_tmpSock = sock;}

  //! �ڑ����̃T�[�r�X�ƃ\�P�b�g�̃}�b�v�擾
  std::map<std::string, SOCKET> getSrvSocks() { return m_srvSocks; }

  // �T�[�r�X�v���o�C�_�ڑ���Ԑݒ�
  void setConnected(bool connected) {m_connected = connected;}

  //! �T�[�o�����擾����
  const char * 	server() { return m_server.c_str(); }

  //! �T�[�o�|�[�g�ԍ����擾����
  int		port() { return m_port; }

  //! �A�^�b�`����G�[�W�F���g�����擾����
  const char *	myname() { return m_myname.c_str(); }

  //! �T�[�o���̃G�[�W�F���g�ɃA�^�b�`���Ă��邩�H
  bool	isAttached() { return m_isAttached; }	// added by sekikawa(2007/10/15)

  /** �T�[�o���̎w��̃G�[�W�F���g�ɃR���g���[��������
   * ���Ă�
   *
   * @param server �V�~�����[�V�����T�[�o��
   * @param port	 �V�~�����[�V�����T�[�o�|�[�g�ԍ�
   * @param myname �A�^�b�`����G�[�W�F���g��
   */
  bool	attach(const char *server, int port, const char *myname);

  /**
   * @brief �G�[�W�F���g����R���g���[����ؗ���
   *
   * �ؗ�����A�T�[�o�Ƃ̒ʐM��ؒf����
   */
  void	detach() { close_(); }

  /**
   * @brief �G���e�B�e�B�f�[�^����M�p�\�P�b�g�𓾂�
   *
   */
  SOCKET  getDataSock() { return m_dataSock; }

  /**
   * @brief �T�[�r�X�v���o�C�_�󂯕t���p�\�P�b�g�𓾂�
   *
   */
  SOCKET  getSrvSock() { return m_srvSock; }

  // old
  void	sendText(double t, const char *to, const char *text, double reachRadius);

  // old
  void	sendText(double t, const char *to, const char *text);

  void	sendDisplayText(double t, const char *to, const char *text, int fs, const char *color, double reachRadius = -1.0);

  void	displayText(const char *text, int fs, const char *color, double dummy = -1.0);

  void	sendMessage(const char *to, int argc, char **argv);

  /**
   * @brief ���b�Z�[�W�𑗐M����(���b�Z�[�W���󂯎�����G���e�B�e�B��onRecvMsg���Ăяo�����)
   *
   * @param to		  ���M��G�[�W�F���g��
   * @param msg		  ���b�Z�[�W
   * @param distance	  ���b�Z�[�W���͂�����(�w�肵�Ȃ���ΕK���͂�)
   */
  bool	sendMsg(std::string to, std::string msg, double distance = -1.0);

  /**
   * @brief �����̃G���e�B�e�B�Ƀ��b�Z�[�W�𑗐M����
   *        (���b�Z�[�W���󂯎�����G���e�B�e�B��onRecvMsg���Ăяo�����)
   *
   * @param to		  ���M��G�[�W�F���g��(Vecotr)
   * @param msg		  ���b�Z�[�W
   * @param distance	  ���b�Z�[�W���͂�����(�w�肵�Ȃ���ΕK���͂�)
   */
  bool	sendMsg(std::vector<std::string> to, std::string msg, double distance = -1.0);

  void	sendDisplayMessage(const char *to, int argc, char **argv, int fs, const char *color);

  /**
   * @brief �ڑ����̑S�T�[�r�X�v���o�C�_�Ƀ��b�Z�[�W�𑗐M����
   * 
   * @param msg		  ���b�Z�[�W
   */
  bool	broadcastMsgToSrv(std::string msg);

  /**
   * @brief �ڑ����̃R���g���[�������S�G���e�B�e�B�Ƀ��b�Z�[�W�𑗐M����
   * 
   * @param msg		  ���b�Z�[�W
   * @param distance	  ���b�Z�[�W���͂�����(�w�肵�Ȃ���ΕK���͂�)
   */
  bool	broadcastMsgToCtl(std::string msg, double distance = -1.0);

  // old
  void	broadcastMessage(int argc, char **argv);

  /**
   * @brief ���b�Z�[�W���u���[�h�L���X�g����
   *
   * @param msg		  ���b�Z�[�W
   * @param distance	  ���b�Z�[�W���͂������i�w�肵�Ȃ���ΑS���[�U�[�ɓ͂�)	  
   */
  bool	broadcastMsg(std::string msg, double distance = -1.0);

  // old
  void	broadcastDisplayMessage(int argc, char **argv, int fs, const char *color);

  // begin(FIX20110401)
  /**
   * @brief ���̃G�[�W�F���g�ɉ����f�[�^�𑗐M����
   *
   * @param t			  ���݂̎���
   * @param to		  ���M��(NULL�̏ꍇ���ׂẴG�[�W�F���g�ɑ��M����)
   * @param text		  �����f�[�^
   */
  void sendSound(double t, const char *to, RawSound &sound);
  // end(FIX20110401)

  //! �f�[�^�𑗐M����
  bool sendData(SOCKET sock, const char *msg, int size);

#ifdef DEPRECATED
  void	send(Command &cmd);
  void	moveTo(double x, double z, double velocity);
#endif


  /**
   * @brief ���b�Z�[�W���u���[�h�L���X�g����
   *
   * @param msg		  ���b�Z�[�W
   * @param distance	  ���b�Z�[�W���͂�����(-1.0�̏ꍇ�͕K���͂�)	  
   * @param to              -1�̏ꍇ�͂��ׂāA-2�̏ꍇ�̓T�[�r�X�v���o�C�_�A-3�̏ꍇ�̓R���g���[��
   */
  bool broadcast(std::string msg, double distance, int to);

  // �Ō�ɐڑ������T�[�r�X���폜
  void deleteLastService() {
    BaseService* srv = m_services.back();
    delete srv;
    m_services.pop_back();
  }

  // �ڑ����̎w�肵���T�[�r�X���폜
  void deleteService(std::string sname) {
    std::vector<BaseService*>::iterator it;
    it = m_services.begin();
    while(it != m_services.end()) {
      std::string name = (*it)->getName();
      if(name == sname) {
	delete *it;
	m_services.erase(it);
	break;
      }
      it++;
    }

    // �T�[�r�X���ƃ\�P�b�g�̃}�b�v����폜
    std::map<std::string, SOCKET>::iterator mit;
    mit = m_srvSocks.begin();
    while(mit != m_srvSocks.end()) {
      if((*mit).first == sname) {
	m_srvSocks.erase(mit);
	return;
      }
      mit++;
    }
    LOG_ERR(("deleteService: cannot find %s", sname.c_str()));
  }

  //! �Ō�ɐڑ������T�[�r�X���擾����
  BaseService* getLastService() {
    return m_services.back();
  }

  //! �ڑ����̎w�肵���T�[�r�X���擾����
  BaseService* getService(std::string sname) {
    std::vector<BaseService*>::iterator it;
    it = m_services.begin();
   
    while(it != m_services.end()) {
      std::string name = (*it)->getName();
      if(name == sname) {
	return (*it);
      }
      it++;
    }
    LOG_ERR(("getService: cannot find %s", sname.c_str()));
    return NULL;
  }


};



#endif // ControllerImpl_h
