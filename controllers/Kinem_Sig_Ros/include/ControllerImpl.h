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
  
  bool startServiceLoop(ControllerImpl *con);

  void endServiceLoop();
  
  std::string getName() { return m_name; }

  SOCKET getSocket() { return m_sock; }

  SOCKET getClientSocket() { return m_clientSock; }

  void setClientSocket(SOCKET sock) { m_clientSock = sock; }

  bool sendMsgToSrv(std::string msg);

  void setGetData(bool get){ m_getData = get; }

  void setDetectedNames(std::vector<std::string> names) { m_detectedNames = names; }
  /*
  ViewImage* getImage() {
    return m_image;
  }
  */
  std::string getEntityName() { 
    return m_entname; 
  }

  void setEntityName(std::string name) {
    m_entname = name;
  }

 protected:
  std::string m_name;

  unsigned short m_port;

  // socket
  SOCKET m_sock;

  SOCKET m_clientSock;

  bool m_getData;

  std::string m_entname;

  std::vector<ViewImage*> m_images;

  unsigned char         m_distance;

  std::vector<std::string> m_detectedNames;
};

class ViewService : public BaseService
{


 public:

  ViewService(){;} 
  
 ViewService(std::string name, unsigned short port, SOCKET sock) 
   : BaseService(name, port, sock) {;}
  
  bool  detectEntities(std::vector<std::string> &v, int id = 1);

  ViewImage *captureView(int camID = 1, ColorBitType ctype = COLORBIT_24, ImageDataSize size = IMAGE_320X240);
  
  unsigned char distanceSensor(double start = 0.0, double end = 255.0, int camID =1, ColorBitType ctype = DEPTHBIT_8);

  ViewImage *distanceSensor1D(double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X1);

  ViewImage *distanceSensor2D(double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X240);

 private:
  bool sendDSRequest(int type, double start = 0.0, double end = 255.0, int camID = 1, ColorBitType ctype = DEPTHBIT_8, ImageDataSize size = IMAGE_320X240);

};

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
  SOCKET                m_srvSock; 
  unsigned short        m_srvPort; 
  SOCKET                m_tmpSock; // tmpSock

  std::map<std::string, SOCKET> m_srvSocks; 

  bool m_connected;



  bool           m_connectService;

  std::vector<BaseService*> m_services;

  // changed by sekikawa (2007/10/16)
  // (********* this is only experiment. need to recover later *********)
  //private:
 private:
  CTSimObj * m_ctSimObj;
 protected:
  CTSimObj & getCTSimObj();

  BaseService* connectToService(std::string name);

  BaseService* connectToService(std::string name, unsigned short port);

  void disconnectToService(std::string name);

  bool checkService(std::string name);
  
 private:
  // bool recvData(SOCKET sock, char *msg, int size);


 public:
  static void *serviceThread(void *pParam);


 protected:
  void	close_();
	
 protected:
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

  ~ControllerImpl() {
    close_();
  }

 public:
  bool connected() {return m_connected;}

  void setTmpSock(SOCKET sock) {m_tmpSock = sock;}

  std::map<std::string, SOCKET> getSrvSocks() { return m_srvSocks; }

  void setConnected(bool connected) {m_connected = connected;}

  const char * 	server() { return m_server.c_str(); }

  int		port() { return m_port; }

  const char *	myname() { return m_myname.c_str(); }

  bool	isAttached() { return m_isAttached; }	// added by sekikawa(2007/10/15)

  bool	attach(const char *server, int port, const char *myname);

  void	detach() { close_(); }

  SOCKET  getDataSock() { return m_dataSock; }

  SOCKET  getSrvSock() { return m_srvSock; }

  // old
  void	sendText(double t, const char *to, const char *text, double reachRadius);

  // old
  void	sendText(double t, const char *to, const char *text);

  void	sendDisplayText(double t, const char *to, const char *text, int fs, const char *color, double reachRadius = -1.0);

  void	displayText(const char *text, int fs, const char *color, double dummy = -1.0);

  void	sendMessage(const char *to, int argc, char **argv);

  bool	sendMsg(std::string to, std::string msg, double distance = -1.0);

  bool	sendMsg(std::vector<std::string> to, std::string msg, double distance = -1.0);

  void	sendDisplayMessage(const char *to, int argc, char **argv, int fs, const char *color);

  bool	broadcastMsgToSrv(std::string msg);

  bool	broadcastMsgToCtl(std::string msg, double distance = -1.0);

  // old
  void	broadcastMessage(int argc, char **argv);

  bool	broadcastMsg(std::string msg, double distance = -1.0);

  // old
  void	broadcastDisplayMessage(int argc, char **argv, int fs, const char *color);

  // begin(FIX20110401)
  void sendSound(double t, const char *to, RawSound &sound);
  // end(FIX20110401)

  bool sendData(SOCKET sock, const char *msg, int size);

#ifdef DEPRECATED
  void	send(Command &cmd);
  void	moveTo(double x, double z, double velocity);
#endif


  bool broadcast(std::string msg, double distance, int to);

  void deleteLastService() {
    BaseService* srv = m_services.back();
    delete srv;
    m_services.pop_back();
  }

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

  BaseService* getLastService() {
    return m_services.back();
  }

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

