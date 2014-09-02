#include "Controller.h"
#include <ControllerEvent.h>
#include "Logger.h"
#include <unistd.h>
#include <fstream>

// PSMoveAPI
#include "PSMoveData.h"
#include <psmoveapi/psmove.h> // only for buttons numerical code

#define PI 3.141592

//角度からラジアンに変換します。
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )

class CameraController : public Controller
{
public:
  void onInit(InitEvent &evt);
  double onAction(ActionEvent &evt);
  void rotateTowardRobot(Vector3d pos);
  void onRecvMsg(RecvMsgEvent &evt);
  void rotateTowardGround();

private:
  // カメラ関係
  SimObj *m_my;
  Vector3d m_pos;
  Vector3d m_rotatePos;
  Vector3d m_pathPos;
  std::string m_name;

  // ロボット関係
  RobotObj *r_my;
  Vector3d r_pos;

  bool configuringPath;
  std::vector<Vector3d> path;

};

void CameraController::onInit(InitEvent &evt)
{
  // カメラ初期方向　1:45 2:135 3:315 4:225
  // ロボット初期位置取得
  r_my = getRobotObj("robot_004");
  r_my->getPosition(r_pos);

  // カメラ番号取得
  m_my = getObj(myname());
  m_my->getPosition(m_rotatePos);
  m_pathPos = Vector3d(m_rotatePos.x() -500, m_rotatePos.y() + 1700, m_rotatePos.z());
  m_name = m_my->name();

  configuringPath = false;

  // 定点カメラをロボットの方向に回転
  // rotateTowardRobot(r_pos);
  rotateTowardGround();

}

//定期的に呼び出される関数です。
double CameraController::onAction(ActionEvent &evt)
{
  // ロボット位置取得
  r_my->getPosition(r_pos);

  // カメラ番号取得
  m_my = getObj(myname());
  m_name = m_my->name();

  // std::cout << "configuringPath : " << configuringPath << std::endl;

  if (!configuringPath) {
    // 定点カメラをロボットの方向に回転
    // rotateTowardRobot(r_pos);
  }

  return 0.05;
}

void CameraController::rotateTowardRobot(Vector3d r_pos)
{
  m_my->setPosition(m_rotatePos);
  // カメラの位置を得る
  m_my->getPosition(m_pos);

  // カメラの位置からロボットを結ぶベクトル
  Vector3d tmpp = r_pos;
  tmpp -= m_pos;

  // y方向は考えない
  tmpp.y(0);

  // カメラの回転角度を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // カメラのy軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();
  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // ロボットまでの回転角度を得る
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;

  // 角度差から回転量を得る
  targetAngle += theta;
  m_my->setAxisAndAngle(0, 1, 0, -targetAngle, 0);
}


void CameraController::onRecvMsg(RecvMsgEvent &evt) {
  std::string msg = evt.getMsg();
  std::vector<MoveData> moves = buildMoveData(msg);
  if (!moves.empty()) { // seg fault if task is not started
    MoveData move = moves[0]; // only one move needed for this operation
    if (move.releasedButtons & Btn_MOVE) {
      if (! configuringPath) {
	rotateTowardGround();
	std::cout << "starting path definition" << std::endl;
	configuringPath = true;
      }
      else {
	// save path --> write path to file
	if (!path.empty()) {
	  std::ofstream output;
	  std::stringstream filename;

	  time_t now = time(0);
	  struct tm  tstruct;
	  char buf[80];
	  tstruct = *localtime(&now);

	  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	  filename << "nodes/"
		   << buf
		   << ".txt";

	  output.open(filename.str().c_str());
	  std::vector<Vector3d>::iterator node;
	  for (node = path.begin(); node != path.end(); ++node) {
	    std::cout << "Node : " << node->x() << ", " << node->y() << ", " << node->z() << std::endl;
	    output << node->x() << "," << node->y() << "," << node->z() << "," << "0" << std::endl;
	  }
	  output.close();
	  std::stringstream msg;
	  msg << "new path wrote to " << filename.str();
	  std::cout << msg << std::endl;
	  broadcastMsg(msg.str());
	}
	path.clear();
	configuringPath = false;
      }
    }
    else if (configuringPath) {
      // update the position of the cursor.
      SimObj* cursor = getObj("pointer1");

      double xPosition = -300*move.tracker.y;
      double yPosition = 100;
      double zPosition = -300*move.tracker.x;

      cursor->setPosition(xPosition, yPosition, zPosition);


      if (move.releasedButtons & Btn_CROSS) {
	// add a step to the current path
	path.push_back(Vector3d(xPosition, yPosition, zPosition));
      }
      else if (move.releasedButtons & Btn_CIRCLE) {
	// remove the last step of the path
	if (! path.empty()) {
	  path.pop_back(); 
	}
      }

    }
  }

}

void CameraController::rotateTowardGround() {
  m_my->setPosition(m_pathPos);
  m_my->setRotation(Rotation(0.6123724356957946, 0.35355339059327373, -0.6123724356957945, 0.35355339059327373));
  // m_my->setRotation(Rotation(60, 0, 1, 0));
  // m_my->setAxisAndAngle(1, 0, 0, 90);
}



extern "C"  Controller * createController ()
{
  return new CameraController;
}

