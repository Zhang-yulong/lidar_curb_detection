// #include "ConnectTools.h"


// extern std::mutex m_mutex;
// extern std::vector<MuchLidarData> m_LidarData;	
// extern GetLidarData *m_GetLidarData;

// void A::callbackFunction(std::vector<MuchLidarData> LidarDataValue, int Type)
// {
// 	m_mutex.lock();
// 	printf("回调LidarDataValue size: %d\n",LidarDataValue.size());
// 	m_LidarData = LidarDataValue;
// 	m_mutex.unlock();
// }



// pcl::visualization::PCLVisualizer viewer("3d viewer");
// void ShowPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud)
// {
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(PointCloud, 255, 255, 255);
//     viewer.removeAllPointClouds();
//     viewer.addPointCloud<pcl::PointXYZI> (PointCloud, color, "Cloud"); 
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
//     viewer.addCoordinateSystem (1.0);      
//     viewer.spinOnce(1);
// }

// void SavePcd(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud)
// {

//   	pcl_pointcloud->height = pcl_pointcloud->points.size();
//   	pcl_pointcloud->width = 1;
//   	pcl_pointcloud->is_dense = false;
// 	static int i = 0;
// /*	每两次保存一张pcd*/
// 	i++;
// 	if(i % 2 != 0)
// 	{
// 		return;
// 	}

// 	struct timeval tNow;
// 	gettimeofday(&tNow, NULL);
// 	unsigned long long ullTimestamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
// 	printf("pop ullTimestamp:%ld\n", ullTimestamp);
// 	//gettimeofday(&tStart, NULL);
// 	char pchFileName[128];
// 	bzero(pchFileName, sizeof (pchFileName));
// 	sprintf(pchFileName, "../pcd/%lld.pcd", ullTimestamp);
// 	pcl::io::savePCDFileASCII (pchFileName, *pcl_pointcloud);

// 	// std::string pchFileName = sSaveDepthAddress + sTime + ".pcd";
// 	// // pcl::io::savePCDFileASCII (pchFileName, *pcl_pointcloud); //保存ASCII，文件太大了
// 	// pcl::io::savePCDFileBinary(pchFileName, *pcl_pointcloud);
	
// 	// printf("save pcd time: %s\n", sTime.c_str());
// }




// //获取数据包的端口号
// void getDataSock(std::string ComputerIP, int MsopPort, int DifopPort)
// {
// // #ifdef LINUX
// // #else
// //     WORD wVerRequest = MAKEWORD(1, 1);
// // 	WSADATA wsaData;
// // 	WSAStartup(wVerRequest, &wsaData);
// // #endif

// 	//创建socket
// 	int m_sock = socket(2, 2, 0);			//构建sock
// 	//******************UDP通信初始化**************************//
// 	//创建socket
// 	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 	if( m_sock < 0 ){
// 		perror("create getData socket error\n");
//         exit(0);
// 	}
// 	else{
//         printf("create getData socket successful\n");
//     }

// 	//定义地址
// 	struct sockaddr_in sockAddr;
// 	sockAddr.sin_family = AF_INET;
// 	sockAddr.sin_port = htons(MsopPort);
// 	sockAddr.sin_addr.s_addr = inet_addr(ComputerIP.c_str());

// 	//绑定套接字
// 	int retVal = bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));
	
// 	if( retVal == -1){
//  		printf("bind getData socket error\n");
//         exit(0);
//     }
//     else{
//         printf("bind getData socket successful\n");
//     }

// 	struct sockaddr_in addrFrom;


// // #ifdef LINUX
//     socklen_t len = sizeof(sockaddr_in);
// // #else
// //     int len = sizeof(sockaddr_in);
// // #endif
// 	//接收数据
// 	char recvBuf[1212] = { 0 };
// 	int recvLen;

// 	while (true)
// 	{
//         //获取套接字接收内容
// // #ifdef LINUX
//         recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
// 		// printf("recvLen: %d \n", recvLen);
// // #else
// //         recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (SOCKADDR*)&addrFrom, &len);
// // #endif
// 		if (recvLen > 0)
// 		{
// 			u_char data[1212] = { 0 };
// 			memcpy(data, recvBuf, recvLen);
// 			m_GetLidarData->CollectionDataArrive(data, recvLen);												//把数据闯入到类内
// 		}
// 	}
// }





// //获取设备包的端口号
// void getDevSock(std::string ComputerIP, int MsopPort, int DifopPort)
// {
// // #ifdef LINUX
// // #else
// //     WORD wVerRequest = MAKEWORD(1, 1);
// // 	WSADATA wsaData;
// // 	WSAStartup(wVerRequest, &wsaData);
// // #endif
// 	//创建socket
// 	int m_sock = socket(2, 2, 0);			//构建sock
// 	//******************UDP通信初始化**************************//
// 	//创建socket
// 	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 	if( m_sock < 0 ){
// 		perror("create Dev socket error\n");
//         exit(0);
// 	}
// 	else{
//         printf("create Dev socket successful\n");
//     }


// 	//定义地址
// 	struct sockaddr_in sockAddr;
// 	sockAddr.sin_family = AF_INET;
// 	sockAddr.sin_port = htons(DifopPort);											//获取设备包的端口号
// 	sockAddr.sin_addr.s_addr = inet_addr(ComputerIP.c_str());

// 	//绑定套接字
// 	int retVal = bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));
	
// 	if( retVal == -1){
//  		printf("bind Dev socket error\n");
//         exit(0);
//     }
//     else{
//         printf("bind Dev socket successful\n");
//     }

	
// 	struct sockaddr_in addrFrom;
// // #ifdef LINUX
//     socklen_t len = sizeof(sockaddr_in);
// // #else
// //     int len = sizeof(sockaddr_in);
// // #endif
// 	//接收数据
// 	char recvBuf[1212] = { 0 };
// 	int recvLen;

// 	while (true)
// 	{
//         //获取套接字接收内容
// // #ifdef LINUX
//         recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
// // #else
// //         recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (SOCKADDR*)&addrFrom, &len);
// // #endif
// 		if (recvLen > 0)
// 		{
// 			u_char data[1212] = { 0 };
// 			memcpy(data, recvBuf, recvLen);
// 			m_GetLidarData->CollectionDataArrive(data, recvLen);												//把数据闯入到类内
// 		}
// 	}
// }
