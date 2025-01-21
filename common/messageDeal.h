
#ifndef __MESSAGE_DEAL_H__
#define __MESSAGE_DEAL_H__


#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "location_fusion_struct.h"
#include "communication.h"
#include "threadsafe_queue.h"
// #include "../3rdparty/comm_2.7.1/include/communication.h"

int iSetImuPacket(STR_IMU *pstrImu);
STR_IMU* getImuPacketWait();
STR_IMU* getImuPacketTry();

int iSetCanSpeed(STR_CAN_SPEED *strCanSpeed);
STR_CAN_SPEED* getCanSpeedWait();
STR_CAN_SPEED* getCanSpeedTry();

int iSendLocationResult(STR_FUSIONLOC strLoc);
int GetFusionLocation (STR_FUSIONLOC*  pstrFusionLocation);
// unsigned long long getLidarPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &vecRes);


// 删除数组  
template <typename T>  
inline void safe_delete(T *&target) 
{  
    if (nullptr != target) 
	{  
        delete target;  
        target = nullptr;  
    }  
}  
  
// 删除数组指针  
template <typename T>  
inline void safe_delete_arr(T *&target) 
{  
    if (nullptr != target) 
	{  
        delete[] target;  
        target = nullptr;  
    }  
} 

#endif

