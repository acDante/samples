/*
 * AvatarController.h
 *
 * This is basic(abstract) class.
 * Any avatar class should inherit this class.
 *
 * 12nd, Feb., 2015.
 * wrote by Nozaki.
 */

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/SimObj.h>

//#include "Common.h"

///@brief General avatar controller class (abstract class).
class AvatarController: public Controller
{
private:

public:
	///@brief Constructor.
	AvatarController(){};
	
	///@brief Virtual destructor.
	virtual ~AvatarController(){};
	
	///@brief 受け取ったメッセージを姿勢情報に反映する
	///@brief Convert the received message to avatar's posture information.
	//virtual void convertMessage2Posture(const RecvMsgEvent &evt) = 0;

	///@brief 姿勢情報をSIGViewer上のアバターに適用する
	///@brief Set the joint quaternions of sim-object from avatar's posture.
	//virtual void setJointQuaternionsFromPosture(SimObj *simObj) = 0;
};
