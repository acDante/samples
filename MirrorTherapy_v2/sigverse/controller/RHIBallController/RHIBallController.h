/*
 * RHIBallController.h
 *
 *  Created on: 2015/08/06
 *      Author: Yamada@tome
 */
#ifndef SIGVERSE_RHI_BALL_CONTROLLER_H
#define SIGVERSE_RHI_BALL_CONTROLLER_H

#include <sigverse/Controller.h>
#include <sigverse/ControllerEvent.h>
#include <sigverse/comm/controller/Controller.h>


class RHIBallController : public Controller
{
public:
	///@brief Initialize this controller.
	void onInit(InitEvent &evt);

	///@brief Movement of the robot.
	double onAction(ActionEvent &evt);

	///@brief Message heard by the robot.
	void onRecvMsg(RecvMsgEvent &evt);

	static const std::string mirrorTherapyManName;

private :
	ViewService* m_view;

	SimObj *ballObj;
	SimObj *mirrorTherapyManObj;

	double yOffset;
	double zOffset;
	double totalTime;
};

#endif // SIGVERSE_RHI_BALL_CONTROLLER_H
