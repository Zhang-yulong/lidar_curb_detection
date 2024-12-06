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


#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
typedef uint64_t u_int64;

#define PI 3.1415926
#define ConversionUnit	100.0			//单位转换 厘米转换成 米 

typedef struct _Point_XYZ
{
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
}m_PointXYZ;

#if 0
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
typedef struct MuchLidarData {
        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;
        int ID = 0;
        float H_angle = 0.0;
        float V_angle = 0.0;
        float Distance = 0.0;
        int Intensity = 0;
        u_int64 Mtimestamp_nsce = 0;
} MuchLidarData;

typedef struct _UTC_Time
{
	int year = 2000;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
}UTC_Time;

typedef struct Firing {
	double vertical_angle;
	//  int vertical_line;
	double azimuth;
	double distance;
	float intensity;
	double time;
	int channel_number;
	double prism_angle[4];
} Firing;

typedef struct LslidarLSPacket {
	//# Raw Leishen LIDAR packet.
	double stamp;
	float prism_angle[4];       // prism angle
	uint8_t data[1206];        //  packet contents
} LslidarLSPacket;

typedef std::function<void(std::vector<MuchLidarData>, int)> Fun;

#define ConversionDistance_C_v4_0 	3.61 / 100.0 		//光学中心转结构中心的 距离 （3.61 单位为 厘米） 转成  米为单位
#define ConversionAngle_C_v4_0	    20.25 				//光学中心转结构中心的 角度  单位为 度
#define IDistanceACC_C_v4_0			0.4 / 100.0			//距离精度转换	 0.4cm	转换成 1m		雷达传上来的数据1代表 0.4cm 

class GetLidarData
{
public:
	GetLidarData();
	~GetLidarData();

	void setCallbackFunction(Fun*);			//设置点云数据回调函数
	// void GetLidarData::setCallback(std::function<void(std::vector<MuchLidarData>, int)>fun);
	
	bool setEcho_SingleOrDouble(int value);														//设置雷达的单双回波



	void LidarStar();
	void LidarStop();
	virtual void LidarRun() = 0;
 	void CollectionDataArrive(void * pData, uint16_t len);										//把UDP的数据传入到此处
 	int iDataPort;
 	bool isSetOriAngle;
protected:
	bool isQuit = false;																		//雷达退出的标记
	int Model = 0;																				//判断输出：0是单双回波数据； 1是单回波数据； 2是单双回波数据
	std::queue<unsigned char *> allDataValue;													//数据缓存队列
	std::mutex m_mutex;																			//锁

	u_int64 timestamp_nsce;																		//包内秒以下时间戳（单位ns）
	u_int64 allTimestamp;																		//包内的时间戳保存
	u_int64 lastAllTimestamp = 0;																//保存上一包的时间戳
	double PointIntervalT_ns;																	//包内每个点之间的时间间隔	单位为纳秒

	std::vector<MuchLidarData> all_lidardata, all_lidardata_d;									//获取每一帧的数据
	UTC_Time m_UTC_Time;																		//获取设备包的GPS信息
	void clearQueue(std::queue<unsigned char *>& m_queue);										//清空队列
	void sendLidarData();																		//数据打包发送
	Fun *callback = NULL;																		//回调函数指针 -回调数据包
    std::string time_service_mode = "gps";                                                     //授时方式

};
