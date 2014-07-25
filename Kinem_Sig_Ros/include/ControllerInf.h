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

class ControllerInf
{
public:
 
 ControllerInf(): m_startSim(false), m_excon(false) {;}

	virtual ~ControllerInf() {;}

	virtual void 	onInit(InitEvent &evt) {}
	
	virtual double 	onAction(ActionEvent &evt) { return 0.0; }

	virtual void 	onRecvText(RecvTextEvent &evt) {}
	virtual void	onRecvSound(RecvSoundEvent &evt) {}
	//old
	virtual void	onRecvMessage(RecvMessageEvent &evt) {}
	virtual void	onRecvMsg(RecvMsgEvent &evt) {}
	virtual void 	onCollision(CollisionEvent &evt) {}

#ifndef UNIT_TEST
	virtual	bool	isProcessing() = 0;
	virtual void	onPreEvent() = 0;
	virtual void	onPostEvent() = 0;
#else
	virtual	bool	isProcessing() { return false; };
	virtual void	onPreEvent() {};
	virtual void	onPostEvent() {};
#endif


	bool getSimState(){ return m_startSim; }

	void setSimState(bool sim){ m_startSim = sim; }

	bool getExCon(){ return m_excon; }

	void setExCon(bool excon){ m_excon = excon; }

 protected:
	bool m_startSim;

	bool m_excon;
};


#endif // ControllerInf_h
 
 

