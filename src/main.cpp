#include <iostream>
#include <string> 

#include "LeiShenDrive.h"

#include "ulog_api.h"

#define CvLane_MC_PORT 9139

#define LOG_CFG_FILE_PATH "/etc/ulog/curb/config/ulog.cfg"


std::string Yaml_Path = "/etc/echiev/bin/debug_config.yaml";

// #include "ConnectTools.h"
// using namespace pcl::visualization;

STR_LIDAR_CONFIG *strLidarConfig_ToMainLidar = (STR_LIDAR_CONFIG *)malloc(sizeof(STR_LIDAR_CONFIG));
STR_LIDAR_CONFIG *strLidarConfig_ToCar = (STR_LIDAR_CONFIG *)malloc(sizeof(STR_LIDAR_CONFIG));

STR_FUSIONLOC g_strFusionLocation;
pthread_mutex_t g_SpeedMutex;
pthread_mutex_t g_ThreadMutex;
float  g_fSpeed = 0.0;


class YamlReader;





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



int Comm_Init(void)
{
	const char *strConfigPath_ToMainLidar = "/etc/echiev/lidar/lsCH64w_front/lidar.cfg";
	if(NULL == strLidarConfig_ToMainLidar)
	{
		printf("strLidarConfig_ToMainLidar malloc error\n");
		return -1;
	}
	int iRet_1 = readConfigFile(strLidarConfig_ToMainLidar, strConfigPath_ToMainLidar);
	if(1 == iRet_1)
	{
		printf("read ToMainLidar Config File error\n");
		return -1;
	}
	else{
		printf("read ToMainLidar Config File successful\n");
	}


	const char *strConfigPath_ToCar = "/etc/echiev/lidar/lidar.cfg";
	if(NULL == strLidarConfig_ToCar)
	{
		printf("strConfigPath_ToCar malloc error\n");
		return -1;
	}
	int iRet_2 = readConfigFile(strLidarConfig_ToCar, strConfigPath_ToCar);
	if(1 == iRet_2)
	{
		printf("read ToCar Config File error\n");
		return -1;
	}
	else{
		printf("read ToCar Config File successful\n");
	}


	/*
	if(OpenTimeMCClient(TIME_MC_PORT, 0) < 0) 
	{
		printf("open Time client fail\n");
	}
	*/

	pthread_mutex_init(&g_SpeedMutex, NULL);
	if(OpenVSpdMCClient(VSPD_MC_PORT, 0) < 0)
	{
		printf("OpenVSpdMCClient fail.\n");
		return -1;
	}
	else
		{printf("OpenVSpdMCClient successful.\n");}
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
	if(OpenfusionLocMCClient(FUSIONLOC_MC_PORT, 0) < 0)
	{
		printf("OpenfusionLocMCClient fail.\n");
		return -1;
	}
	else
		{printf("OpenfusionLocMCClient successful.\n");}
	fusionLoc_set_callback(fusionLoc_set);


	if(OpenCameraLaneMCServer(CvLane_MC_PORT, CvLane_MC_INTERVAL, 0X01) < 0) 
	{
		printf("OpenCameraLaneMCServer fail.\n");
		return -1;
	}
	else
		{printf("OpenCameraLaneMCServer successful.\n");}

	
	return 0;
}

void Comm_Exit(void)
{
	// CloseTimeMCClient();
	CloseVSpdMCClient();
	CloseLidarMCClient();
	ClosefusionLocMCClient();
	CloseCameraLaneMCServer();
	// ulog_deinit();
}






/* 配合ConnectTools.cpp使用
// GetLidarData *m_GetLidarData;
// std::mutex m_mutex;
// std::vector<MuchLidarData> m_LidarData;	
// std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
*/
int main()
{
	
	ulog_init(LOG_CFG_FILE_PATH);

	Comm_Init();

	Config stOpencvConfig;
	YamlReader *pYamlReader = new YamlReader(Yaml_Path);
    if (!pYamlReader->LoadConfig(stOpencvConfig)) {
		LOG_RAW("debug.yaml加载失败\n");
        return -1; // 如果加载失败，则退出
    }

    // std::string sSelfComputerIP = "192.168.86.70";
    // int iMsopPort = 2368;
    // int iDifopPort = 2369;
    // std::string sLeiShenType = "CH64w";

    LeiShenDrive lsDrive(stOpencvConfig);
	lsDrive.Init();
    // lsDrive.Init(sSelfComputerIP, iMsopPort, iDifopPort, sLeiShenType, 0);
    lsDrive.Start();
    
/*	配合ConnectTools.cpp使用
    m_GetLidarData = new GetLidarData_CH64w;

    A m_a;
	Fun fun = std::bind(&A::callbackFunction, m_a, std::placeholders::_1, std::placeholders::_2);

	m_GetLidarData->setCallbackFunction(&fun);				//设置回调函数
	m_GetLidarData->LidarStar();							//开始解析雷达数据

    std::thread m_DataSockT(getDataSock, sSelfComputerIP ,iMsopPort, iDifopPort);
	m_DataSockT.detach();

	std::thread m_DevSockT(getDevSock, sSelfComputerIP ,iMsopPort, iDifopPort);
	m_DevSockT.detach();

    //pcd展示
	// pcl_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("LSPointCloudViewer");
	// pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
	// pcl_viewer->addCoordinateSystem(1.0);
	// pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "lslidar");	// 显示点云，
	// pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "lslidar"); // 设置点云大小
*/

    while(1){
		/*配合ConnectTools.cpp使用
        printf("m_LidarData size: %ld \n", m_LidarData.size());
		if (!m_LidarData.empty())
		{
			m_mutex.lock();
			std::vector<MuchLidarData> m_LidarData_temp;
			m_LidarData_temp = m_LidarData;
			m_LidarData.clear();
			m_mutex.unlock();
	
			pcl_pointcloud->points.clear();

			cloud->points.clear();

			for (u_int m_FF = 0; m_FF < m_LidarData_temp.size(); m_FF++)
			{
				pcl::PointXYZI tmppoint;
				
				tmppoint.x = m_LidarData_temp[m_FF].X;
				tmppoint.y = m_LidarData_temp[m_FF].Y;
				tmppoint.z = m_LidarData_temp[m_FF].Z;
				tmppoint.intensity = m_LidarData_temp[m_FF].Intensity;
				cloud->points.push_back(tmppoint);
				
				pcl_pointcloud->points.push_back(tmppoint);
            }
            ShowPointCloud(cloud);

            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pcl_pointcloud, "intensity"); 
			pcl_viewer->updatePointCloud<pcl::PointXYZI>(pcl_pointcloud, point_color_handle, "lslidar");
			pcl_viewer->spinOnce(); 

        }
        else
		{
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		*/

		
    }

    lsDrive.Free();
	Comm_Exit();
/*	配合ConnectTools.cpp使用
	
    m_GetLidarData->LidarStop();
*/
    return 0;
}
