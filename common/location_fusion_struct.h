#ifndef __LOCATION_FUSION_STRUCT_H__
#define __LOCATION_FUSION_STRUCT_H__

//#define MAX_TARGET_COUNTER 64
#define MAX_BUFFER_SIZE      1258    //1248+10


#include "type_define.h"
#include "struct_typedef.h"
#include <vector>

// #include "../3rdparty/comm_2.7.1/include/type_define.h"

#pragma pack(push, 1)


typedef struct _STR_IMU_OLD
{
	char reserve;
	double dLatitude;
	double dLongitude;
	float fAltitude;
	float fLateralSpeed;
	float fLinearSpeed;
	float fVerticalSpeed;
	float fRollAngle;
	float fPitchAngle;
	float fHeadingAngle;
	float fLinearAcceleration;
	float fLateralAcceleration;
	float fVerticalAcceleration;
	float fRollSpeed;
	float fPitchSpeed;
	float fHeadingSpeed;
	unsigned long long ullSystemTime;
	char cSystemStatus;
	char GPS_Status;
	float fOriLinearSpeed;
	float fSteeringAngle;
	float fSteeringSpeed;
} STR_IMU_OLD, *PSTR_IMU_OLD;


typedef struct _strLidarPoint
{
	int iSeq;
	unsigned long long ullTimestamp;
    float fX;            
	float fY;            
	float fZ;            
	float fI;     
}strLidarPoint;

typedef struct _strPacketPoints
{
	unsigned long long ullTimestamp;
    std::vector<strLidarPoint> vPacketPoints;
}strPacketPoints;

typedef struct _strLidarPacket
{
	unsigned long long ullTimestamp;
	unsigned char data[MAX_BUFFER_SIZE];
	int iPositionFlag;//1: represent head lidar 0:represent tail lidar
}strLidarPacket;



#pragma pack(pop)


#endif
