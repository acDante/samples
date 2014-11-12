#include "KinectApp.h"
#include "Logger.h"
#include <time.h>
#include <tchar.h> 
// Visual Studio Professional以上を使う場合はCComPtrの利用を検討してください。
//#include <atlbase.h>




// クラスMyServiceのインスタンスを作成します  
MyService* srv;

/**main loop
*/
void mainLoop(std::string saddr, KinectApp kinectapp,MyService* srv)
{
	printf("start loop\n");

	printf("20141027\n");


	bool m_start;
	m_start = true;
	static clock_t start;
	clock_t now;
	start = clock();

	bool m_onLoop;
	m_onLoop = true;
	double ltime;
	ltime = -1.0;


	while (1) {
		kinectapp.update();
		kinectapp.draw();
		auto key = cv::waitKey(10);
		if (key == 'q'){
			break;
		}

		//srv.sendMsgToCtr(saddr, kinectapp.strMsg);
		//srv.setMessage(kinectapp.strMsg);
		kinectapp.sendMessage();
		Sleep(100); /* wait 0.1 seconds */
		if (m_onLoop){
			if (ltime > 0){
				now = clock();
				double tmp = (double)(now - start) / CLOCKS_PER_SEC;
				if (tmp > ltime) {
					m_onLoop = false;
					break;
				}
			}
			if (!srv->checkRecvData(100)) {
				m_onLoop = false;
				break;
			}
		}
		else break;
	}

	std::printf("end  loop\n");
}
//void main()
int main(int argc, char **argv)
{

	std::string sname = "SIGKINECT";
	TCHAR spd[64] = "";
	GetPrivateProfileString(_T("SETTING"), _T("SERVICE_NAME"), '\0', spd, 256, _T("./SIGNiUserTracker.ini"));
	printf("spd=%s\n",spd);
	if (spd[0] != '\0') {
		sname = spd;
	}
	std::printf("sname=%s\n",sname.c_str());
	//MyService srv("SIGKINECT");
	printf("SERVICENAME=%s\n",sname.c_str());
	MyService srv(sname);

	char saddr[128];
	int portnumber;

	//1.connect to SIGVerse server.

//#define TEST

#ifndef TEST
	if (argc == 1) {
		std::cout << "Cannot find Server host name and port number.\n";
		std::cout << "Enter SIGVerse server host name (or IP address)\n>>";
		std::cin >> saddr;
		std::cout << "Enter port number\n>>";
		std::cin >> portnumber;
	}

	if (argc > 1) {
		sprintf_s(saddr, 128, "%s", argv[1]);

		portnumber = atoi(argv[2]);
		std::cout << "Connect to server [" << saddr << "]\n";
		std::cout << "Portnumber [" << portnumber << "]\n\n";
		std::cout << "[ESC]: Quit service.\n";
	}
#else
	strcpy_s(saddr, "192.168.244.129");
	portnumber = 9000;
#endif


	//  unsigned short port = (unsigned short)(atoi(argv[2]));  
	//  srv.connect(argv[1], port);  

	// サーバに接続します  
	if (srv.connect(saddr, portnumber)){
		printf("success to connect to SIGServer.\n");
	}else
	{
		printf("fail to connect to SIGServer.\n");
	}

	// ビューワーに接続します  
	if (srv.connectToViewer()){
		printf("success to connect to SIGViewer.\n");
		// ビューワーがサーバからdisconnectしたらloopを自動で抜けます  
		srv.setAutoExitLoop(true);
	}
	else{
		printf("fail to connect to SIGViewer.\n");
	}

    try {
		KinectApp kinectapp;
		kinectapp.initialize(&srv);
        //app.run();
		mainLoop(saddr, kinectapp, &srv);

    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }


}
