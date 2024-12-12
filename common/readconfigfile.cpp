#include "readconfigfile.h"
#include <stdlib.h>
#include <string.h>
#define MAX_LEN 128
#define LOG_TAG "ecv.loc.readconfigfile"

#include "libconfig.h"
#include "ulog_api.h"

// #include "../3rdparty/libconfig/include/libconfig.h"
// #include "../3rdparty/ulog/include/ulog_api.h"

int readConfigFile(STR_LIDAR_CONFIG *strLidarConfig,const char* strConfigPath)
{
	//loadConfigFile();
    config_t strCfg;
    config_setting_t *pSetting;
    config_init(&strCfg);

    LOG_I("strConfigPath:%s\n", strConfigPath);
    int iRet = 0;
    iRet = config_read_file(&strCfg, strConfigPath);
    if(1 != iRet)
    {
    	LOG_E("config read file error iRet:%d\n", iRet);
		return 1;
    }
	
    pSetting = config_lookup(&strCfg, "application.lidar");
    if(NULL == pSetting)
    {
    	LOG_E("config lookup lidar error\n");
		return 1;
    }

    // iRet = config_setting_lookup_int(pSetting, "iLidarNum", &(strLidarConfig->iLidarNum));
    // if(1 != iRet)
    // {
    // 	LOG_E("Can not find application.lidar iLidarNum value\n");
	// 	return 1;
    // }   

    // iRet = config_setting_lookup_int(pSetting, "iLidarType", &(strLidarConfig->iLidarType));
    // if(1 != iRet)
    // {
    // 	LOG_E("Can not find application.lidar iLidarType value\n");
	// 	return 1;
    // }

    // iRet = config_setting_lookup_int(pSetting, "iPacketNumOfFrame", &(strLidarConfig->iPacketNumOfFrame));
	// if(1 != iRet)
	// {
	// 	LOG_E("Can not find application.lidar iPacketNumOfFrame value\n");
	// 	return 1;
	// }

    // iRet = config_setting_lookup_int(pSetting, "iRsLidarHeadPort", &(strLidarConfig->iRsLidarHeadPort));
    // if(1 != iRet)
    // {
    // 	LOG_E("Can not find application.lidar iRsLidarHeadPort value\n");
	// 	return 1;
    // }   

    // iRet = config_setting_lookup_int(pSetting, "iRsLidarTailPort", &(strLidarConfig->iRsLidarTailPort));
    // if(1 != iRet)
    // {
    // 	LOG_E("Can not find application.lidar iRsLidarTailPort value\n");
	// 	return 1;
    // }
	
    // iRet = config_setting_lookup_float(pSetting, "fLidarHeight", &(strLidarConfig->fLidarHeight));
    // if(1 != iRet)
    // {
    // 	LOG_E("Can not find application.lidar fLidarHeight value\n");
	// 	return 1;
    // }

    // iRet = config_setting_lookup_float(pSetting, "fImuHeight", &(strLidarConfig->fImuHeight));
	// if(1 != iRet)
	// {
	// 	LOG_E("Can not find application.lidar fImuHeight value\n");
	// 	return 1;
	// }

    iRet = config_setting_lookup_float(pSetting, "flidar2imu_angle", &(strLidarConfig->flidar2imu_angle));
    if(1 != iRet)
    {
    	LOG_E("Can not find application.lidar flidar2imu_angle value\n");
        return 1;
    }
    iRet = config_setting_lookup_float(pSetting, "flidar2imu_X", &(strLidarConfig->flidar2imu_X));
    if(1 != iRet)
    {
    	LOG_E("Can not find application.lidar flidar2imu_X value\n");
        return 1;
    }
    iRet = config_setting_lookup_float(pSetting, "flidar2imu_Y", &(strLidarConfig->flidar2imu_Y));
    if(1 != iRet)
    {
    	LOG_E("Can not find application.lidar flidar2imu_Y value\n");
        return 1;
    }


	 config_setting_t * array_setting;
	// array_setting = config_lookup(&strCfg, "application.lidar.fLeftCurb");
	// if(NULL == array_setting)
    // {
	// 	LOG_E("config lookup lidar fLeftCurb error\n");
	// 	return 1;
    // }
	// int iLeftCurbCount = config_setting_length(array_setting);
   	// for(int i = 0; i < iLeftCurbCount; ++i)
    // {
	// 	//LOG_RAW("This is the %dth time  num is %d  \r\n", i, config_setting_get_int_elem(array_setting, i));
	// 	strLidarConfig->fLeftCurb[i] = config_setting_get_float_elem(array_setting, i);
	// }
	

	// array_setting = config_lookup(&strCfg, "application.lidar.fRightCurb");
	// if(NULL == array_setting)
    // {
	// 	LOG_E("config lookup lidar fRightCurb error\n");
	// 	return 1;
    // }
	// int iRightCurbCount = config_setting_length(array_setting);
   	// for(int i = 0; i < iRightCurbCount; ++i)
    // {
	// 	//LOG_RAW("This is the %dth time  num is %d  \r\n", i, config_setting_get_int_elem(array_setting, i));
	// 	strLidarConfig->fRightCurb[i] = config_setting_get_float_elem(array_setting, i);
	// }	

	array_setting = config_lookup(&strCfg, "application.lidar.fComBinePara");
	if(NULL == array_setting)
    {
		LOG_E("config lookup lidar fComBinePara error\n");
		return 1;
    }
	int iLidarRMatrixCount = config_setting_length(array_setting);
   	for(int i = 0; i < iLidarRMatrixCount; ++i)
    {
		//LOG_RAW("This is the %dth time  num is %d  \r\n", i, config_setting_get_int_elem(array_setting, i));
		strLidarConfig->fComBinePara[i] = config_setting_get_float_elem(array_setting, i);
	}		

    
   	// LOG_RAW("iLidarNum:%d\n", strLidarConfig->iLidarNum);
   	// LOG_RAW("iLidarType:%d\n", strLidarConfig->iLidarType);
   	// LOG_RAW("iPacketNumOfFrame:%d\n", strLidarConfig->iPacketNumOfFrame);
	// LOG_RAW("iRsLidarHeadPort:%d\n", strLidarConfig->iRsLidarHeadPort);
	// LOG_RAW("iRsLidarTailPort:%d\n", strLidarConfig->iRsLidarTailPort);

	// LOG_RAW("fLidarHeight:%f\n", strLidarConfig->fLidarHeight);
	// LOG_RAW("fImuHeight:%f\n", strLidarConfig->fImuHeight);
	// LOG_RAW("fLeftCurb:");
	// for(int i=0; i<iLeftCurbCount; i++)
	// {
	// 	LOG_RAW("%3.1f ", strLidarConfig->fLeftCurb[i]);
	// }
	// LOG_RAW("\n");
	// LOG_RAW("fRightCurb:");
	// for(int i=0; i<iRightCurbCount; i++)
	// {
	// 	LOG_RAW("%3.1f ", strLidarConfig->fRightCurb[i]);
	// }
	// LOG_RAW("\n");

	// LOG_RAW("fLidarRMatrix:");
	// for(int i=0; i<iLidarRMatrixCount; i++)
	// {
	// 	LOG_RAW("%7.5f ", strLidarConfig->fLidarRMatrix[i]);
	// }
	// LOG_RAW("\n");
	// LOG_RAW("flidar2imu_angle:%f\n", strLidarConfig->flidar2imu_angle);
	// LOG_RAW("flidar2imu_X:%f\n", strLidarConfig->flidar2imu_X);
	// LOG_RAW("flidar2imu_Y:%f\n", strLidarConfig->flidar2imu_Y);
	// LOG_RAW("\n");

 
	return 0;
}
