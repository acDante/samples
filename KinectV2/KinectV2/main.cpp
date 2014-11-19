//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "KinectApp.h"
// Visual Studio Professional以上を使う場合はCComPtrの利用を検討してください。 consider to use  "CComPtr" if you use Visual Studio Professional.




// クラスSIGKINECT_Linkageのインスタンスを作成します   create instance of "SIGKINECT_Linkage"
SIGKINECT_Linkage* srv;


std::string agent_name;

/*
 *main loop
 */
void mainLoop(std::string saddr, KinectApp kinectapp, SIGKINECT_Linkage* srv)
{
	//printf("mainloop\n");
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
		kinectapp.sendMessage(srv,agent_name);
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

int main(int argc, char **argv)
{
	agent_name = "Linkage-Robot";
	std::string sname = "SIGKINECT_Linkage";
	TCHAR spd[64] = "";
	GetPrivateProfileString(_T("SETTING"), _T("SERVICE_NAME"), '\0', spd, 256, _T("./SIGNiUserTracker.ini"));
	std::printf("sname=%s\n",sname.c_str());
	printf("SERVICENAME=%s\n",sname.c_str());
	SIGKINECT_Linkage srv(sname);
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

	// サーバに接続します   connect to server
	if (srv.connect(saddr, portnumber)){
		printf("success to connect to SIGServer.\n");
	}else
	{
		printf("fail to connect to SIGServer.\n");
	}

	// ビューワーに接続します   connect to viewer
	if (srv.connectToViewer()){
		printf("success to connect to SIGViewer.\n");
		// ビューワーがサーバからdisconnectしたらloopを自動で抜けます   automatically exit loop when the viewer disconnect
		srv.setAutoExitLoop(true);
	}
	else{
		printf("fail to connect to SIGViewer.\n");
	}

    try {
		KinectApp kinectapp;
		kinectapp.initialize();
        //app.run();
		mainLoop(saddr, kinectapp, &srv);

    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }


}
