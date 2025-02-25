#include <iostream>
#include <string> 
#include <memory>
#include "LeiShenDrive.h"

#include "ulog_api.h"

#define CvLane_MC_PORT 9139

#define LOG_CFG_FILE_PATH "/home/zyl/echiev_lidar_curb_detection/config/ulog.cfg"
// #define LOG_CFG_FILE_PATH "/etc/ulog/curb/config/ulog.cfg"

using namespace Lidar_Curb_Dedection;

std::string Yaml_Path = "/home/zyl/echiev_lidar_curb_detection/config/debug_config.yaml";
// std::string Yaml_Path = "/etc/echiev/bin/debug_config.yaml";


STR_LIDAR_CONFIG *strToMainLidarConfig = (STR_LIDAR_CONFIG *)malloc(sizeof(STR_LIDAR_CONFIG));
STR_LIDAR_CONFIG *strToCarConfig = (STR_LIDAR_CONFIG *)malloc(sizeof(STR_LIDAR_CONFIG));

STR_FUSIONLOC g_strFusionLocation;
pthread_mutex_t g_SpeedMutex;
pthread_mutex_t g_ThreadMutex;
float  g_fSpeed = 0.0;



void setSpeedInfo(float fSpeed)
{
	pthread_mutex_lock( &g_SpeedMutex);
	g_fSpeed = fSpeed;
	pthread_mutex_unlock( &g_SpeedMutex );
}
void speed_set(unsigned char *pData, int Len)
{
    STR_CAN_SPEED canSpeed;
	memset(&canSpeed, 0, sizeof(STR_CAN_SPEED));
	memcpy(&canSpeed, pData, Len);
	setSpeedInfo(canSpeed.fCanSpeed);
}

void SetFusionLocation (STR_FUSIONLOC*  pstrFusionLocation )
{
/*
	API_LOG (LOG_ERR, "rev fusionLoc data, long %.15lf %.15lf \n",
	pstrFusionLocation->strImu.dLongitude, pstrFusionLocation->strImu.dLatitude);
	*/
	pthread_mutex_lock( &g_ThreadMutex );
	memcpy(&g_strFusionLocation,pstrFusionLocation, sizeof(STR_FUSIONLOC));
	pthread_mutex_unlock( &g_ThreadMutex );
}
int GetFusionLocation (STR_FUSIONLOC*  pstrFusionLocation)
{
	pthread_mutex_lock( &g_ThreadMutex );
	memcpy(pstrFusionLocation,&g_strFusionLocation,sizeof(STR_FUSIONLOC) );
	pthread_mutex_unlock( &g_ThreadMutex );
	return 0;
}
void fusionLoc_set(unsigned char *pData, int Len)
{
	//API_LOG (LOG_ERR, "rev fusionLoc data, len=%d \n", Len);

	if ( Len >= sizeof(STR_FUSIONLOC) )
	{
		STR_FUSIONLOC* pstrFusionLocation = (STR_FUSIONLOC*)pData;
		SetFusionLocation( pstrFusionLocation );
		//printf ("cccccccc rev fusionLoc data, len ok =%d \n", Len);
		//++g_fusionlocrev_times;
		//store_loc ((STR_FUSION_LOCATION*)pData);
	} else{
		printf ("cccccccc rev fusionLoc data, len not ok =%d \n", Len);
	}
}



int CommInit(const STR_CONFIG &config)
{
	int iRet_1, iRet_2;

	if(NULL == strToMainLidarConfig){
		LOG_RAW("strToMainLidarConfig malloc error\n");
		return -1;
	}
	if(NULL == strToCarConfig){
		LOG_RAW("strToCarConfig malloc error\n");
		return -1;
	}


	if(config.pathTolidarTransformConfig == 0){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_lidar_curb_detection/config/小车11上的雷达配置文件/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_lidar_curb_detection/config/小车11上的雷达配置文件/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig, PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig, PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 1){
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig, PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig, PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 2){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_lidar_curb_detection/config/佛山T1上的雷达配置文件/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_lidar_curb_detection/config/佛山T1上的雷达配置文件/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig, PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig, PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 3) {
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig, PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig, PathToCarConfig);
	}
	else{
		LOG_RAW("**** 2 **** pathTolidarTransformConfig 配置错误 \n");
		return -1;
	}	

	if(1 == iRet_1){
		LOG_RAW("**** 3 **** read ToMainLidar Config File error\n");
		return -1;
	}
	else{
		LOG_RAW("read ToMainLidar Config File successful\n");
	}

	if(1 == iRet_2){
		LOG_RAW("**** 4 **** read ToCar Config File error\n");
		return -1;
	}
	else{
		LOG_RAW("read ToCar Config File successful\n");
	}


	/*
	if(OpenTimeMCClient(TIME_MC_PORT, 0) < 0) 
	{
		printf("open Time client fail\n");
	}
	*/

	pthread_mutex_init(&g_SpeedMutex, NULL);
	if(OpenVSpdMCClient(VSPD_MC_PORT, 0) < 0){
		LOG_RAW("**** 5 **** OpenVSpdMCClient fail.\n");
		return -1;
	}
	else{
		LOG_RAW("OpenVSpdMCClient successful.\n");
	}
	vehicle_speed_set_callback(speed_set);

	/*
	if(OpenLidarMCClient(LIDAR_MC_PORT, 0) < 0)  //读：雷达接收端口，9106
	{
		printf("OpenLidarMCClient failed\n");
		return -1;
	}
	Lidar_set_callback(Lidar_set);	
	*/
	
	pthread_mutex_init(&g_ThreadMutex, NULL);
	if(OpenfusionLocMCClient(FUSIONLOC_MC_PORT, 0) < 0){
		LOG_RAW("**** 6 **** OpenfusionLocMCClient fail.\n");
		return -1;
	}
	else{
		LOG_RAW("OpenfusionLocMCClient successful.\n");
	}
	fusionLoc_set_callback(fusionLoc_set);


	if(OpenCameraLaneMCServer(CvLane_MC_PORT, CvLane_MC_INTERVAL, 0X01) < 0){ 
		LOG_RAW("**** 7 **** OpenCameraLaneMCServer fail.\n");
		return -1;
	}
	else{
		LOG_RAW("OpenCameraLaneMCServer successful.\n");
	}

	return 0;
}

void Comm_Exit(void)
{
	// CloseTimeMCClient();
	CloseVSpdMCClient();
	CloseLidarMCClient();
	ClosefusionLocMCClient();
	CloseCameraLaneMCServer();
	ulog_deinit();
}


STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO GetAllTransformConfigInfo(const STR_LIDAR_CONFIG *toMainLidar, const STR_LIDAR_CONFIG *toCar){

	STR_LIDAR_TRANSFORM_CONFIG_INFO toMainLidarInfo, toCarInfo;

	float fAngle_ToMainLidar = toMainLidar->flidar2imu_angle;
	float fRad_ToMainLidar = (fAngle_ToMainLidar * M_PI) / 180;
	Eigen::Matrix<float,4,4> R_ToMainLidar;
	R_ToMainLidar <<	toMainLidar->fComBinePara[0], toMainLidar->fComBinePara[1], toMainLidar->fComBinePara[2], toMainLidar->fComBinePara[3],
					 	toMainLidar->fComBinePara[4], toMainLidar->fComBinePara[5], toMainLidar->fComBinePara[6], toMainLidar->fComBinePara[7],
					 	toMainLidar->fComBinePara[8], toMainLidar->fComBinePara[9],	toMainLidar->fComBinePara[10], toMainLidar->fComBinePara[11],
					 	0, 0, 0, 1;
	
	// Eigen::Matrix<float,4,4> R_ToMainLidar = Eigen::Matrix4f::Identity();
	// R_ToMainLidar(0,0) = cos(fRad_ToMainLidar);
	// R_ToMainLidar(0,1) = sin(fRad_ToMainLidar);
	// R_ToMainLidar(0,3) = toMainLidar->fComBinePara[3];

	// R_ToMainLidar(1,0) = -sin(fRad_ToMainLidar);
	// R_ToMainLidar(1,1) = cos(fRad_ToMainLidar);
	// R_ToMainLidar(1,3) = toMainLidar->fComBinePara[7];
	
	// R_ToMainLidar(2,3) = toMainLidar->fComBinePara[11];
	
	std::cout<<"R_ToMainLidar = "<<R_ToMainLidar<<std::endl;
	toMainLidarInfo.transformMartix = R_ToMainLidar;
	toMainLidarInfo.rad = fRad_ToMainLidar;


	float fAngle_ToCar = toCar->flidar2imu_angle;
	float fRad_ToCar = (fAngle_ToCar * M_PI) / 180;
	Eigen::Matrix<float,4,4> R_ToCar = Eigen::Matrix4f::Identity();
	R_ToCar(0,0) = cos(fRad_ToCar);
	R_ToCar(0,1) = sin(fRad_ToCar);
	R_ToCar(0,3) = toCar->fComBinePara[3];

	R_ToCar(1,0) = -sin(fRad_ToCar);
	R_ToCar(1,1) = cos(fRad_ToCar);
	R_ToCar(1,3) = toCar->fComBinePara[7];
	
	R_ToCar(2,3) = toCar->fComBinePara[11];

	std::cout<<"R_ToCar = "<<R_ToCar<<std::endl; 
	toCarInfo.transformMartix = R_ToCar;
	toCarInfo.rad = fRad_ToCar;

	return {toMainLidarInfo, toCarInfo};
}





int main()
{
	ulog_init(LOG_CFG_FILE_PATH);

	STR_CONFIG strOpencvConfig;
	YamlReader *pYamlReader = new YamlReader(Yaml_Path);
    if (!pYamlReader->LoadConfig(strOpencvConfig)) {
		LOG_RAW("**** 1 **** debug.yaml加载失败\n");
        return -1; // 如果加载失败，则退出
    }

	CommInit(strOpencvConfig);


	STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO strAllLidarTransformInfo = GetAllTransformConfigInfo(strToMainLidarConfig, strToCarConfig);

   	LeiShenDrive lsDrive(strOpencvConfig, strAllLidarTransformInfo);
	lsDrive.Init();
    lsDrive.Start();
    

    while(1){
		
    }

    lsDrive.Free();
	Comm_Exit();

    return 0;
}
