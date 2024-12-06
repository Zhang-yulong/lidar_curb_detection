#include <unistd.h>
#include <string.h>
#include "messageDeal.h"
#define Imu_QUEUE_SIZE 1

threadsafe_queue<STR_IMU> g_ImuObjectQueue;
threadsafe_queue<STR_CAN_SPEED> g_CanSpeedQueue;

int iSetImuPacket(STR_IMU *pstrImu)
{
	STR_IMU imuObject = *pstrImu;
	STR_IMU objectTmp;
	while(g_ImuObjectQueue.try_pop(objectTmp))
	{
		usleep(10);
	}
	g_ImuObjectQueue.push(imuObject);	

	return 0;
}

STR_IMU* getImuPacketWait()
{
	STR_IMU imuObject;
	g_ImuObjectQueue.wait_and_pop(imuObject);
	STR_IMU* pstrImu = (STR_IMU*)malloc(sizeof(STR_IMU));
	if(pstrImu != NULL)
	{
        memcpy(pstrImu, &imuObject, sizeof(STR_IMU));
	}
	
	return pstrImu;
}

STR_IMU* getImuPacketTry()
{
	STR_IMU imuObject;
	bool bGetImu =  g_ImuObjectQueue.try_pop(imuObject);
	STR_IMU* pstrImu = NULL;
    if(bGetImu)
	{
        pstrImu = (STR_IMU*)malloc(sizeof(STR_IMU));
		if(pstrImu != NULL)
		{
			memcpy(pstrImu, &imuObject, sizeof(STR_IMU));
		}
	}
	
	return pstrImu;
}

int iSetCanSpeed(STR_CAN_SPEED *strCanSpeed)
{
	//printf("~~~time = %lld, can speed = %f\n", strCanSpeed->ullTimestamp, strCanSpeed->fCanSpeed);
	STR_CAN_SPEED strCanSpeedObject = *strCanSpeed;
	STR_CAN_SPEED strTmpSpeed;
	while(g_CanSpeedQueue.try_pop(strTmpSpeed))
	{
		usleep(10);
	}
	g_CanSpeedQueue.push(strCanSpeedObject);

	return 0;
}

STR_CAN_SPEED* getCanSpeedWait()
{
	STR_CAN_SPEED strCanSpeedObject;
	g_CanSpeedQueue.wait_and_pop(strCanSpeedObject);
	STR_CAN_SPEED* strCanSpeed = (STR_CAN_SPEED*)malloc(sizeof(STR_CAN_SPEED));
	if(strCanSpeed != NULL)
	{
        memcpy(strCanSpeed, &strCanSpeedObject, sizeof(STR_CAN_SPEED));
	}

	return strCanSpeed;
}

STR_CAN_SPEED* getCanSpeedTry()
{
	STR_CAN_SPEED strCanSpeedObject;
	bool bGetSpeed =  g_CanSpeedQueue.try_pop(strCanSpeedObject);
	STR_CAN_SPEED* strCanSpeed = NULL;
    if(bGetSpeed)
	{
    	strCanSpeed = (STR_CAN_SPEED*)malloc(sizeof(STR_CAN_SPEED));
		if(strCanSpeed != NULL)
		{
			memcpy(strCanSpeed, &strCanSpeedObject, sizeof(STR_CAN_SPEED));
		}
	}

	return strCanSpeed;
}

int iSendLocationResult(STR_FUSIONLOC strLoc)
{
	int iRet = fusionLocMCServerSend((unsigned char *)&strLoc, sizeof(STR_FUSIONLOC));
	return iRet;
}



