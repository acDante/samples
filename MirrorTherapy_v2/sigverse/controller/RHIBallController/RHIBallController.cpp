/*
 * RHIBallController.cpp
 *
 *  Created on: 2015/08/06
 *      Author: Yamada@tome
 */

#include <sigverse/controller/RHIBallController/RHIBallController.h>

#include <sys/time.h>
#include <boost/exception/diagnostic_information.hpp>
#include <math.h>

const std::string RHIBallController::mirrorTherapyManName = "mirror_therapy_man";


///@brief Initialize this controller.
void RHIBallController::onInit(InitEvent &evt)
{
	//Connect Service.
	this->m_view = (ViewService*)connectToService("SIGViewer");

	this->ballObj             = getObj(myname());
	this->mirrorTherapyManObj = getObj(RHIBallController::mirrorTherapyManName.c_str());

	this->zOffset = 1000.0;
	this->totalTime = 0.0;
}

///@brief Movement of the robot.
double RHIBallController::onAction(ActionEvent &evt)
{
	this->yOffset = 9.0 + 10.0 * (std::sin(this->totalTime * M_PI)+1.0);

	Vector3d mirrorVec, ballVec;
	this->mirrorTherapyManObj->getJointPosition(mirrorVec, "RARM_JOINT5");

	ballVec.set(mirrorVec.x(), mirrorVec.y()+this->yOffset, mirrorVec.z()+this->zOffset);

	this->ballObj->setPosition(ballVec);

	this->totalTime+=0.1;

	return 0.1;
}


///@brief Message from device.
void RHIBallController::onRecvMsg(RecvMsgEvent &evt)
{
	try
	{
		// Receive message.
		const std::string allMsg = evt.getMsg();

		std::cout << "RHIBallController::onRecvMsg " << allMsg << std::endl;

		if (this->m_view != NULL && allMsg=="ball")
		{
			std::cout << "RHIBallController::zOffset= " << this->zOffset << std::endl;

			if(this->zOffset<500.0){ this->zOffset = 1000.0; }
			else if(this->zOffset>500.0){ this->zOffset =    0.0; }
		}
	}
	catch(...)
	{
		LOG_MSG(("SIGViewer message is invalid."));
	}
}


extern "C" Controller * createController()
{
	return new RHIBallController;
}

