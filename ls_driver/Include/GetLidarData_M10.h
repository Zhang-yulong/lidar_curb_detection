#define LINUX 1 
#pragma once
#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <cmath>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#ifdef LINUX
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include "GetLidarData.h"
typedef uint64_t u_int64;
#else
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

typedef unsigned __int64 u_int64;

#define timegm _mkgmtime
#endif

#if 0
typedef struct _Point_XYZ
{
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
}m_PointXYZ;

typedef struct _MuchLidarData
{
	float X = 0.0;
	float Y = 0.0;
	float Z = 0.0;
	int ID = 0;
	float H_angle = 0.0;
	float Distance = 0.0;
	int Intensity = 0.0;
	u_int64 Mtimestamp_nsce = 0.0;
}MuchLidarData;
#endif

typedef std::function<void(std::vector<MuchLidarData>, int)> Fun;

class GetLidarData_M10
{
public:
	explicit GetLidarData_M10();
	~GetLidarData_M10();

	/*
	 参数1：false为网口解析，true为串口解析
	 参数2：目的IP
	 参数3：数据端口
	 参数4：雷达IP
	 参数5：雷达端口
	 提示：如果使用串口连接M10雷达，则参数2为串口名，参数3为波特率，M10-P雷达没有串口解析
	*/
	void init(bool SerialP, std::string computer_ip, int data_ip, std::string lidar_ip, int lidar_port);
	bool isFrameOK = false;                                         	//是否可以获取雷达解析的数据标志，true为可以获取
	bool isMotorRuning = false; 

	void LidarStar();                                                   //开始接收雷达数据
	void LidarStop();                                                   //停止雷达数据接收                                   
	std::vector<MuchLidarData> getLidarPerFrameDate();                  //获取解析的雷达数据的结构体

	bool setLidarRotateState(int StateValue);							//设置雷达0:旋转,1:静止
	bool setLidarFilter(int nFilter);									//设置雷达滤波0:不滤波,1:正常滤波,2:三米内滤波

private:
	int m_sock;
#ifdef LINUX
	int fd_;
#else
	WORD wVerRequest;
	WSADATA wsaData;
#endif


	void LidarRun();
	std::mutex m_mutex;													//锁																	 
	std::vector<MuchLidarData> all_lidardata, all_lidardata_d;
	bool isQuit = false;
	int Model = 0;
	Fun* callback = NULL;
	std::vector<MuchLidarData> LidarPerFrameDate{ 0 };

	MuchLidarData m_DataT[300];
	std::string cpt_ip = "192.168.1.102";								//目的IP
	int dataPort = 2368;												//数据端口
	std::string cpt_ip_1 = "192.168.1.200";								//雷达IP
	int lidarPort = 2369;												//雷达端口
	// 打开串口,成功返回true，失败返回false
	// portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
	// baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200
	// parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验
	// databit(数据位): 4-8，通常为8位
	// stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
	// synchronizable(同步、异步): 0为异步，1为同步
	// M10:460800 M10-P:512000
	bool openSerial(const char* portname, int baudrate = 460800, char parity = 0, char databit = 8, char stopbit = 1, char synchronizeflag = 0);

	//发送数据或写数据,成功返字节数，失败返0
	int send(char* dat, int datalong);

	//接受数据或读数据，成功返回读取实际数据的长度，失败返回0
	std::string receive();

	int pHandle[16];
	char synchronizeflag;
	bool m_SerialP = false;                                         //串口连接标志
	int m_subsize = 0;                                           	//串口数据的字节数
	std::string comName;                                         	//串口名
	int comPort;                                                 	//波特率
	bool isQuit_com = false;                                        //关闭串口标志
	void closeserial();                                             //关闭串口

	int IDistanceACC_M10;
	double endFrameAngle = 0;										//结束一帧的的角度（也是开始的角度）
	double fLastAzimuth = 0;                                      	//上一个点的角度
	std::vector<float>BlockAngle;									//每个块的角度

	void handleSingleEcho(unsigned char* data);						//M10数据处理
	void handleSingleEcho_P(unsigned char* data);					//M10-P数据处理
	void handleSingleEcho_GPS(unsigned char* data);					//M10_GPS数据处理
	m_PointXYZ XYZ_calculate(double, double);

	void sendLidarData();
	void deleteLidarData();                                         //释放除对象外的其他指针对象 

};


