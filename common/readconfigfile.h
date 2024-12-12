#ifndef _CONFIG_H_
#define _CONFIG_H_

typedef struct _STR_LIDAR16_CONIFG
{
	int iLidarNum;
	int iLidarType;
	int iPacketNumOfFrame;
	int iRsLidarHeadPort;
	int iRsLidarTailPort;
	
	double fLidarHeight;
	double fImuHeight;
	double fLeftCurb[8];
	double fRightCurb[8];
	double fLidarRMatrix[12];
	double flidar2imu_angle;
	double flidar2imu_X;
	double flidar2imu_Y;
	
	int ilidar_type;
	double fComBinePara[12];
}STR_LIDAR_CONFIG;


int readConfigFile(STR_LIDAR_CONFIG *strLidarConfig, const char* strConfigPath);

#endif
