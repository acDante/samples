#include "sigverse/commonlib/ControllerEvent.h"
#include "sigverse/commonlib/Controller.h"
#include "sigverse/commonlib/Logger.h"
#include "sigverse/commonlib/Size.h"
#include <sstream>

using namespace std;

typedef map<string, CParts *>::iterator partCollectionIterator;
typedef map<string, CParts *> partCollection;

class MyController : public Controller
{
	public:
		void onInit(InitEvent &evt);
		double onAction(ActionEvent&);
		void onRecvMsg(RecvMsgEvent &evt);
		void onCollision(CollisionEvent &evt);
		template<typename T>
		string to_string(T value);
	private:
		Vector3d initialPos;

};

void MyController::onInit(InitEvent &evt) {
	SimObj* hand = getObj(myname());
	hand->setRotation(Rotation(0.707, -0.707, 0, 0));
	hand->getPosition(initialPos);
}

double MyController::onAction(ActionEvent &evt)
{
	return 0.01;
}

/*
	different tokens:
		- "/" : separate different objects in one line of data (rotation data,
		        position data, different bones rotation data)
		- ":" : separate finger identifier in finger line of data
		- ";" : separate two lines of data
		- "," : separate each data
*/
void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	string msg = evt.getMsg();
	size_t pos = msg.find(";", 0);
	string line = msg.substr(0, pos);
	SimObj* hand = getObj(myname());
	double qw,  qx,  qy,  qz;
	double qwF, qxF, qyF, qzF;
	double x, y, z;

	if (!line.empty()) {
		string part1;

		// reducing the length of the msg to be able to search from the begining of the msg at each phase
		msg.erase(0, pos + 1);

		pos = line.find("/", 0);
		part1 = line.substr(0, pos);

		line.erase(0, pos + 1);

		// hand rotation data
		pos = part1.find(",", 0);
		qw = atof(part1.substr(0, pos).c_str());
		part1.erase(0, pos + 1);

		pos = part1.find(",", 0);
		qx = atof(part1.substr(0, pos).c_str());
		part1.erase(0, pos + 1);

		pos = part1.find(",", 0);
		qy = atof(part1.substr(0, pos).c_str());
		part1.erase(0, pos + 1);

		pos = part1.find(",", 0);
		qz = atof(part1.substr(0, pos).c_str());

		// hand position data
		pos = line.find(",", 0);
		x = atof(line.substr(0, pos).c_str()) / 10;
		line.erase(0, pos + 1);

		pos = line.find(",", 0);
		y = atof(line.substr(0, pos).c_str()) / 10;
		line.erase(0, pos + 1);

		pos = line.find(",", 0);
		z = atof(line.substr(0, pos).c_str()) / 10;

		while (!msg.empty()) {
			int i = 0;

			pos = msg.find(";");

			if (pos != string::npos) {
				line = msg.substr(0, pos);
				msg.erase(0, pos + 1);
			}

			else {
				line = msg;
				msg = "";
			}

			pos = line.find(":");
			string fingerName = line.substr(0, pos);
			line.erase(0, pos + 1);

			while (!line.empty()) {
				string data;
				string fingerJoint = fingerName + "_JOINT" + to_string(i);
				pos = line.find("/");

				if (pos != string::npos) {
					data = line.substr(0, pos);
					line.erase(0, pos + 1);
				}
				else {
					data = line;
					line = "";
				}

				// finger joint rotation data
				pos = data.find(",", 0);
				qwF = atof(data.substr(0, pos).c_str());
				data.erase(0, pos + 1);

				pos = data.find(",", 0);
				qxF = atof(data.substr(0, pos).c_str());
				data.erase(0, pos + 1);

				pos = data.find(",", 0);
				qyF = atof(data.substr(0, pos).c_str());
				data.erase(0, pos + 1);

				pos = data.find(",", 0);
				qzF = atof(data.substr(0, pos).c_str());

				hand->setJointQuaternion(fingerJoint.c_str(), qwF, qxF, qyF, qzF);

				i++;
			}

			hand->setRotation(Rotation(qw, qx, qy, qz));
			hand->setPosition(initialPos.x() + x, initialPos.y() + y, initialPos.z() + z);
		}
	}
}

void MyController::onCollision(CollisionEvent &evt)
{
}


template<typename T>
string MyController::to_string(T value)
{
	stringstream ss(stringstream::in | stringstream::out);
	ss << value;
	return ss.str();
}


extern "C" Controller * createController(){
	return new MyController;
}

