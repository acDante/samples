#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include "Size.h"
#include <sstream>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

using namespace std;

typedef map<string, CParts *>::iterator partCollectionIterator;
typedef map<string, CParts *> partCollection;

class MyController : public Controller {
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

double MyController::onAction(ActionEvent &evt) {
    // if(strcmp(myname(), "rightHand") == 0){
    //     SimObj* hand = getObj("rightHand");
    //     partCollection parts = hand->getPartsCollection();
    //     std::vector<std::string> s;

    //     for(partCollectionIterator it = parts.begin(); it != parts.end(); ++it){
    //         if(it->first != "body")
    //             s.push_back(it->first);
    //     }

    //     cout << "The total links are " << s.size() << endl;

    //     Vector3d pos;
    //     Size si;
    //     Rotation r;

    //     for (int i = 0; i < s.size(); i++){
    //         CParts *link = hand->getParts(s[i].c_str());


    //         // if (link->getType() == 0){
    //         //     BoxParts* box = (BoxParts*) link;
    //         //     si = box->getSize();
    //         //     cout << "Box Position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
    //         //     cout << "Box Size : x = " << si.x() << "  y = " << si.y() << "  z = " << si.z() << endl;
    //         //     box->dump();
    //         // }

    //         // else if(link->getType() == 1){
    //         //     CylinderParts* cyl = (CylinderParts*) link;
    //         //     cout << "Cylinder Position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
    //         //     cout << "Cylinder Length : length = " << cyl->getLength() << endl;
    //         //     cout << "Cylinder Radius : rad = " << cyl->getRad() << endl;
    //         //     cyl->dump();
    //         // }

    //         if(link->getType() == 2){
    //             link->getRotation(r);
    //             link->getPosition(pos);
    //             SphereParts* sph = (SphereParts*) link;
    //             cout << "Sphere Position : x = " << pos.x() << "  y = " << pos.y() << "  z = " << pos.z() << endl;
    //             cout << "Sphere Radius : rad = " << sph->getRad() << endl;
    //             cout << "Part Rotation : qx = " << r.qx() << "  qy = " << r.qy() << "  qz = " << r.qz() << "  qw = " << r.qw() << endl;
    //             //sph->dump();
    //         }

    //     }
    // }

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
void MyController::onRecvMsg(RecvMsgEvent &evt) {
    string msg = evt.getMsg();
    size_t pos = msg.find(";", 0);
    string line = msg.substr(0, pos);
    SimObj* hand = getObj(myname());
    float qw, qx, qy, qz;
    float qwF, qxF, qyF, qzF;
    float x, y, z;

    if(!line.empty()){
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

        while(!msg.empty()){
            int i = 0;

            pos = msg.find(";");

            if(pos != string::npos){
                line = msg.substr(0, pos);
                msg.erase(0, pos + 1);
            }

            else{
                line = msg;
                msg = "";
            }

            pos = line.find(":");
            string fingerName = line.substr(0, pos);
            line.erase(0, pos + 1);

            while(!line.empty()){
                string data;
                string fingerJoint = fingerName + "_JOINT" + to_string(i);
                pos = line.find("/");

                if(pos != string::npos){
                    data = line.substr(0, pos);
                    line.erase(0, pos + 1);
                }

                else{
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

void MyController::onCollision(CollisionEvent &evt) {

}

template<typename T>
string MyController::to_string(T value){
    stringstream ss(stringstream::in | stringstream::out);
    ss << value;
    return ss.str();
}

extern "C" Controller * createController(){
    return new MyController;
}

