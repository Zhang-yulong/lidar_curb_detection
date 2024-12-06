// #ifndef CONNECTTOOLS_H
// #define CONNECTTOOLS_H

// #include "../3rdparty/ls_driver/Include/GetLidarData_CH64w.h"
// #include "../3rdparty/ls_driver/Include/GetLidarData_LS128.h"

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h> 

	

// void getDataSock(std::string ComputerIP, int MsopPort, int DifopPort);	//创建Sock 获取雷达数据
// void getDevSock(std::string ComputerIP, int MsopPort, int DifopPort);	//创建Sock 获取雷达数据 设备包数据

// void ShowPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud);

// void SavePcd(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud);

// class A
// {
// public:
// 		void callbackFunction(std::vector<MuchLidarData> LidarDataValue, int);	//回调函数，获取每一帧的数据
// 		Fun fun;

// 		// Fun* bind()
// 		// {
// 		// 	fun = std::bind(&A::callbackFunction, this, std::placeholders::_1, std::placeholders::_2);
// 		// 	return &fun;
// 		// }
// };

// #endif