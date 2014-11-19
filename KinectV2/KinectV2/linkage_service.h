#ifndef _LINKAGE_SERVICE_
#define _LINKAGE_SERVICE_
/*!
* ==========================================================================================
*  @brief  service of rehabilitation to grasp a obon with left hand and Kinect
*  @file   linkage_service.h
*  @date   2013/5/8
*  @author National Institute of Informatics
*  @par    1.0.0
* ==========================================================================================
*/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "linkage.h"

class SIGKINECT_Linkage : public sigverse::SIGService {

public:
	/*!
	* ------------------------------------------------------------------------------------------
	* @brief execute service initializeing
	* @param[in]  XnSkeltonJointPositionstd::string  transmit service name
	* ------------------------------------------------------------------------------------------
	*/
	SIGKINECT_Linkage(std::string name) : SIGService(name) {
		/* input default values*/
		this->bgrasp = false;
		this->bfirst = false;

		printf("[%s][%d]SIGKINECT_Service invoked\n", __FUNCTION__, __LINE__);
	};

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief execute close process of service
	* ------------------------------------------------------------------------------------------
	*/
	~SIGKINECT_Linkage() {
		printf("[%s][%d]SIGKINECT_Service deleted\n", __FUNCTION__, __LINE__);
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief get messages from other services
	* @param[in]  sigverse::RecvMsgEvent input messages
	* ------------------------------------------------------------------------------------------
	*/
	void onRecvMsg(sigverse::RecvMsgEvent &evt) {
		printf("[%s][%d]SIGKINECT_Service received message of %s\n", __FUNCTION__, __LINE__, evt.getMsg());

		if (strstr(evt.getMsg(), "RELEASE") != NULL) {
			/* transit to release status when release something */
			this->bgrasp = false;

			printf("[%s][%d]SIGKINECT_Service knows that agent release object\n", __FUNCTION__, __LINE__);

		}
		else if (strstr(evt.getMsg(), "GRASP") != NULL) {
			/* transit to grasp status*/
			this->bgrasp = true;
			this->bfirst = true;

			printf("[%s][%d]SIGKINECT_Service knows that agent grasp object\n", __FUNCTION__, __LINE__);

		}
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief regularly called
	* ------------------------------------------------------------------------------------------
	*/
	double onAction() {
		printf("SIGKINECT_Linkage service processing\n");
		return 5.0;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief wheter grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	bool grasp() {
		return this->bgrasp;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief wheter first time of movement after grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	bool first() {
		return this->bfirst;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief wheter first time of movement after grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	void next() {
		this->bfirst = false;
	}

private:
	/*!
	* @brief whether grasp or not
	*/
	bool bgrasp;
	/*
	* @brief whether first time of get position after state transition or not 
	*/
	bool bfirst;

};
#endif