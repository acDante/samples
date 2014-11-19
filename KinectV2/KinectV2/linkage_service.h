#ifndef _LINKAGE_SERVICE_
#define _LINKAGE_SERVICE_
/*!
* ==========================================================================================
*  @brief  Kinectを用いた左手のみでお盆を掴むリハビリテーション用のサービスを提供します。 service of rehabilitation to grasp a obon by left hand with Kinect
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
	* @brief サービスの初期処理を実施します。 execute service initializeing
	* @param[in]  XnSkeltonJointPositionstd::string サービス名を渡します。 transmit service name
	* ------------------------------------------------------------------------------------------
	*/
	SIGKINECT_Linkage(std::string name) : SIGService(name) {
		/* 変数の初期値を代入します  input default values*/
		this->bgrasp = false;
		this->bfirst = false;

		/* セントラルサーバへの接続は外部関数が（親クラスの機能を用いて）行います */
		printf("[%s][%d]SIGKINECT_Service invoked\n", __FUNCTION__, __LINE__);
	};

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief サービスの終了処理を実施します。
	* ------------------------------------------------------------------------------------------
	*/
	~SIGKINECT_Linkage() {
		/* セントラルサーバとの切断は外部関数が（親クラスの機能を用いて）行います */
		printf("[%s][%d]SIGKINECT_Service deleted\n", __FUNCTION__, __LINE__);
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief 他サービスからのメッセージを受け取ります。 get messages from other services
	* @param[in]  sigverse::RecvMsgEvent メッセージを渡します。 input messages
	* ------------------------------------------------------------------------------------------
	*/
	void onRecvMsg(sigverse::RecvMsgEvent &evt) {
		printf("[%s][%d]SIGKINECT_Service received message of %s\n", __FUNCTION__, __LINE__, evt.getMsg());

		if (strstr(evt.getMsg(), "RELEASE") != NULL) {
			/* 物を離したために状態を離した状態に遷移します transit to release status */
			this->bgrasp = false;

			printf("[%s][%d]SIGKINECT_Service knows that agent release object\n", __FUNCTION__, __LINE__);

		}
		else if (strstr(evt.getMsg(), "GRASP") != NULL) {
			/* 物を掴んだために状態を掴んだ状態に遷移します  transit to grasp status*/
			this->bgrasp = true;
			this->bfirst = true;

			printf("[%s][%d]SIGKINECT_Service knows that agent grasp object\n", __FUNCTION__, __LINE__);

		}
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief 定期的に呼ばれます。
	* @brief regularly called
	* ------------------------------------------------------------------------------------------
	*/
	double onAction() {
		printf("SIGKINECT_Linkage service processing\n");
		return 5.0;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief 掴んでいるか離しているかの状態を返します。
	* @brief wheter grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	bool grasp() {
		return this->bgrasp;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief 物を掴んで最初の動作か否か返します。
	* @brief wheter first time of movement after grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	bool first() {
		return this->bfirst;
	}

	/*!
	* ------------------------------------------------------------------------------------------
	* @brief 物を掴んで最初の動作か否か返します。
	* @brief wheter first time of movement after grasp or not
	* ------------------------------------------------------------------------------------------
	*/
	void next() {
		this->bfirst = false;
	}

private:
	/*!
	* @brief 物を掴んでいる状態か否か保持します。
	* @brief whether grasp or not
	*/
	bool bgrasp;
	/*
	* @brief 状態遷移後の初めての位置の取得か否か保持します。
	* @brief whether first time of get position after state transition or not 
	*/
	bool bfirst;

};
#endif