#include "LeiShenDrive.h"

std::string CH64w; 
std::string LS128;


// boost::shared_ptr<pcl::visualization::PCLVisualizer> plane_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

#if 0
LeiShenDrive::LeiShenDrive(){

    m_pGetLidarData = nullptr;
	m_pLidarCurbDectection = nullptr;
}
#endif

LeiShenDrive::LeiShenDrive(Config & config){

    m_pGetLidarData = nullptr;
	m_pLidarCurbDectection = nullptr;
	m_stLSConfig = config;
	
}


LeiShenDrive::~LeiShenDrive(){
	m_pGetLidarData->LidarStop();
}

#if 1
void LeiShenDrive::Init(std::string ComputerIP, int MsopPort, int DifopPort, const std::string LidarType, int Number){

	m_sComputerIP = ComputerIP;
    m_iMsopPort = MsopPort;
    m_iDifopPort = DifopPort;
    m_sLeiShenType = LidarType;


    if(m_sLeiShenType == "LS128")
    	m_pGetLidarData = new GetLidarData_LS128;
    else if(m_sLeiShenType == "CH64w"){
		m_pGetLidarData = new GetLidarData_CH64w();
		m_pLidarCurbDectection = new LidarCurbDectection(0.05, -25, 15, 128);
	}
        
    
    if(m_pGetLidarData == nullptr){
        printf("m_pGetLidarData = nullptr \r\n");
        return;
    }

// Fun fun = std::bind(&LeiShenDrive::callBackFunction, this, std::placeholders::_1, std::placeholders::_2);  //这种错误
    fun = std::bind(&LeiShenDrive::CallBackFunction, this, std::placeholders::_1, std::placeholders::_2);

    m_pGetLidarData->setCallbackFunction(&fun);

    m_pGetLidarData->LidarStar();

    // std::thread m_DataSockT(&getDataSock); //不能够将类成员变量函数作为普通函数指针使用。
	std::thread m_DataSockT(&LeiShenDrive::GetDataSock,this);
	m_DataSockT.detach();

	// std::thread m_DevSockT(&getDevSock); //不能够将类成员变量函数作为普通函数指针使用。
	std::thread m_DevSockT(&LeiShenDrive::GetDevSock,this);
	m_DevSockT.detach();


}
#endif


void LeiShenDrive::Init(){


	m_sComputerIP = m_stLSConfig.selfComputerIP;
    m_iMsopPort = m_stLSConfig.msopPort;
    m_iDifopPort = m_stLSConfig.difopPort;
    m_sLeiShenType = m_stLSConfig.lidarType;


   	if(m_sLeiShenType == "CH64w"){

		m_pGetLidarData = new GetLidarData_CH64w();

		// m_pLidarCurbDectection = new LidarCurbDectection(0.05, -25, 15, 128);
		m_pLidarCurbDectection = new LidarCurbDectection(m_stLSConfig);
	}
        
    
    if(m_pGetLidarData == nullptr){
        printf("m_pGetLidarData = nullptr \r\n");
        return ;
    }

// Fun fun = std::bind(&LeiShenDrive::callBackFunction, this, std::placeholders::_1, std::placeholders::_2);  //这种错误
    fun = std::bind(&LeiShenDrive::CallBackFunction, this, std::placeholders::_1, std::placeholders::_2);

    m_pGetLidarData->setCallbackFunction(&fun);

    m_pGetLidarData->LidarStar();

    // std::thread m_DataSockT(&getDataSock); //不能够将类成员变量函数作为普通函数指针使用。
	std::thread m_DataSockT(&LeiShenDrive::GetDataSock,this);
	m_DataSockT.detach();

	// std::thread m_DevSockT(&getDevSock); //不能够将类成员变量函数作为普通函数指针使用。
	std::thread m_DevSockT(&LeiShenDrive::GetDevSock,this);
	m_DevSockT.detach();


}

void LeiShenDrive::CallBackFunction(std::vector<MuchLidarData> LidarDataValue, int)
{
	struct timeval tNow;
	gettimeofday(&tNow, NULL);

	// {
	// 	std::lock_guard<std::mutex> lock(m_DataMutex);
	// 	m_vLidarData.clear();
    // 	m_vLidarData = LidarDataValue;
   	// 	m_ullCatchTimeStamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	// }

	m_DataMutex.lock();
	m_vLidarData.clear();
    m_vLidarData = LidarDataValue;
   	m_ullCatchTimeStamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
    m_DataMutex.unlock();
	// printf("(In leishen callback function) time = %lld, size = %d\n", m_ullCatchTimeStamp, LidarDataValue.size());
}


void LeiShenDrive::SavePcd(const PointCloud2Intensity::Ptr &InputCloud, const unsigned long long &ullTime)
{

  	InputCloud->height = InputCloud->points.size();
  	InputCloud->width = 1;
  	InputCloud->is_dense = false;
	static int i = 0;
	// i++;
	// if(i % 2 != 0)
	// {
	// 	return;
	// }
	// struct timeval tNow;
	// gettimeofday(&tNow, NULL);
	// unsigned long long ullTimestamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	// printf("pop ullTimestamp:%ld\n", ullTimestamp);
	//gettimeofday(&tStart, NULL);
	char pchFileName[128];
	bzero(pchFileName, sizeof (pchFileName));
	sprintf(pchFileName, "../log/pcd/%lld.pcd", ullTime);
	pcl::io::savePCDFileASCII (pchFileName, *InputCloud);
}



void LeiShenDrive::GetLidarType(const std::string LidarType){

    return ;
}


void LeiShenDrive::startGetDevSock(void *pth )
{
	((LeiShenDrive*)pth)->GetDevSock();
}

void LeiShenDrive::startGetDataSock(void *pth )
{
	((LeiShenDrive*)pth)->GetDataSock();
}


//获取数据包的端口号
void LeiShenDrive::GetDataSock()
{
	//创建socket
	int m_sock = socket(2, 2, 0);			//构建sock
	//******************UDP通信初始化**************************//
	//创建socket
	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_sock < 0 ){
		perror("create getData socket error\n");
        exit(0);
	}
	else{
        printf("create getData socket successful\n");
    }

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iMsopPort);
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	int retVal = bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));
	
	if( retVal == -1){
 		printf("bind getData socket error\n");
        exit(0);
    }
    else{
        printf("bind getData socket successful\n");
    }

	struct sockaddr_in addrFrom;

    socklen_t len = sizeof(sockaddr_in);

	//接收数据
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (true)
	{
        //获取套接字接收内容

        recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
		// printf("recvLen: %d \n", recvLen);

		if (recvLen > 0)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			m_pGetLidarData->CollectionDataArrive(data, recvLen);	//把数据闯入到类内
			// printf("数据放入data\n");
		}
	}
	// close(m_sock);
}

//获取设备包的端口号
void LeiShenDrive::GetDevSock()
{

	//创建socket
	int m_sock = socket(2, 2, 0);			//构建sock
	//******************UDP通信初始化**************************//
	//创建socket
	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_sock < 0 ){
		perror("create Dev socket error\n");
        exit(0);
	}
	else{
        printf("create Dev socket successful\n");
    }


	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iDifopPort);											//获取设备包的端口号
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	int retVal = bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));
	
	if( retVal == -1){
 		printf("bind Dev socket error\n");
        exit(0);
    }
    else{
        printf("bind Dev socket successful\n");
    }

	
	struct sockaddr_in addrFrom;

    socklen_t len = sizeof(sockaddr_in);

	//接收数据
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (true)
	{
        //获取套接字接收内容

        recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);

		if (recvLen > 0)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			m_pGetLidarData->CollectionDataArrive(data, recvLen);//把数据传入到类内
		}
	}
}


#if 0
pcl::visualization::PCLVisualizer viewer("3d viewer");
void LeiShenDrive::ShowPointCloud(const PointCloud2Intensity::Ptr &PointCloud)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(PointCloud, 255, 255, 255);
    viewer.removeAllPointClouds();
    viewer.addPointCloud<pcl::PointXYZI> (PointCloud, color, "Cloud"); 
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    viewer.addCoordinateSystem (1.0);      
    viewer.spinOnce(1);
}
#endif


void LeiShenDrive::VoxelGridProcess(PointCloud2Intensity::Ptr &pOutputCloud){

	pcl::VoxelGrid<pcl::PointXYZI> sor;           	//创建滤波对象
	sor.setInputCloud (pOutputCloud);    			//设置需要过滤的点云给滤波对象
	sor.setLeafSize (0.05, 0.05, 0.05);             //设置滤波时创建的体素体积，单位m，因为保留点云大体形状的方式是保留每个体素的中心点，所以体素体积越大那么过滤掉的点云就越多
	sor.filter (*pOutputCloud);          			//执行滤波处理
	// std::cout<<"下采样后点云数量："<<pOutputCloud->points.size()<<std::endl;
}




void LeiShenDrive::PointCloudTransform(std::vector<MuchLidarData> vTmpLidarData, PointCloud2Intensity::Ptr &pOutputCloud)
{
	float fAngle_ToMainLidar = strLidarConfig_ToMainLidar->flidar2imu_angle;
	float fRad_ToMainLidar = (fAngle_ToMainLidar * M_PI) / 180;

	Eigen::Matrix4f R_ToMainLidar;
	R_ToMainLidar << strLidarConfig_ToMainLidar->fComBinePara[0], strLidarConfig_ToMainLidar->fComBinePara[1], strLidarConfig_ToMainLidar->fComBinePara[2], strLidarConfig_ToMainLidar->fComBinePara[3],
			strLidarConfig_ToMainLidar->fComBinePara[4], strLidarConfig_ToMainLidar->fComBinePara[5], strLidarConfig_ToMainLidar->fComBinePara[6], strLidarConfig_ToMainLidar->fComBinePara[7],
			strLidarConfig_ToMainLidar->fComBinePara[8], strLidarConfig_ToMainLidar->fComBinePara[9],	strLidarConfig_ToMainLidar->fComBinePara[10], strLidarConfig_ToMainLidar->fComBinePara[11],
			0, 0, 0, 1;


	float fAngle_ToCar = strLidarConfig_ToCar->flidar2imu_angle;
	float fRad_ToCar = (fAngle_ToCar * M_PI) / 180;
	float fX_ToCar = strLidarConfig_ToCar->flidar2imu_X;
	float fY_ToCar = strLidarConfig_ToCar->flidar2imu_Y;

	Eigen::Matrix4f R_ToCar;
	R_ToCar << strLidarConfig_ToCar->fComBinePara[0], strLidarConfig_ToCar->fComBinePara[1], strLidarConfig_ToCar->fComBinePara[2], strLidarConfig_ToCar->fComBinePara[3],
			strLidarConfig_ToCar->fComBinePara[4], strLidarConfig_ToCar->fComBinePara[5], strLidarConfig_ToCar->fComBinePara[6], strLidarConfig_ToCar->fComBinePara[7],
			strLidarConfig_ToCar->fComBinePara[8], strLidarConfig_ToCar->fComBinePara[9], strLidarConfig_ToCar->fComBinePara[10], strLidarConfig_ToCar->fComBinePara[11],
			strLidarConfig_ToCar->fComBinePara[12], strLidarConfig_ToCar->fComBinePara[13], strLidarConfig_ToCar->fComBinePara[14], strLidarConfig_ToCar->fComBinePara[15];



	for(int i = 0; i < vTmpLidarData.size(); i++)
	{
		Eigen::Matrix<float,4,1> x1_Init, x2_ToMainLidar, x3_ToCar, x4_Final;
		x1_Init << vTmpLidarData[i].X, vTmpLidarData[i].Y, vTmpLidarData[i].Z, 1;
		x2_ToMainLidar = R_ToMainLidar * x1_Init;

		pcl::PointXYZI tmppoint_ToMainLidar;
		tmppoint_ToMainLidar.x =  cos(fRad_ToMainLidar) * x2_ToMainLidar(0) + sin(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.y = -sin(fRad_ToMainLidar) * x2_ToMainLidar(0) + cos(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.z = x2_ToMainLidar(2) ;
		tmppoint_ToMainLidar.intensity = vTmpLidarData[i].Intensity;

		x3_ToCar << tmppoint_ToMainLidar.x, tmppoint_ToMainLidar.y, tmppoint_ToMainLidar.z, 1;
		x4_Final = R_ToCar * x3_ToCar;

		pcl::PointXYZI tmppoint_ToCar;
		tmppoint_ToCar.x =  cos(fRad_ToCar) * x4_Final(0) + sin(fRad_ToCar) * x4_Final(1) ;
		tmppoint_ToCar.y = -sin(fRad_ToCar) * x4_Final(0) + cos(fRad_ToCar) * x4_Final(1) ;
		tmppoint_ToCar.z = x4_Final(2) ;
		tmppoint_ToCar.intensity = vTmpLidarData[i].Intensity;

		pOutputCloud->points.push_back(tmppoint_ToMainLidar);
	}

	
	pOutputCloud->width    = pOutputCloud->points.size();
	pOutputCloud->height   = 1;
	pOutputCloud->is_dense = false;
	//LOG_RAW("!!!!!point size = %d\n", pOutputCloud->points.size());
	//pcl::io::savePCDFileASCII ("1.pcd", *pOutputCloud);
}


unsigned long long LeiShenDrive::GetPointCloudFromOnline(PointCloud2Intensity::Ptr &pOutputCloud)
{
	unsigned long long ullTimestamp = 0;
	std::vector<MuchLidarData> vTmpLidarData;

	// {
	// 	std::lock_guard<std::mutex> lock(m_DataMutex);
	// 	vTmpLidarData = m_vLidarData;
	// 	ullTimestamp = m_ullCatchTimeStamp;
	// }
	m_DataMutex.lock();
	vTmpLidarData = m_vLidarData;
	ullTimestamp = m_ullCatchTimeStamp;
	m_DataMutex.unlock();
	PointCloudTransform(vTmpLidarData, pOutputCloud);
	
	return ullTimestamp;
}


unsigned long long LeiShenDrive::GetPointCloudFromPcd(PointCloud2Intensity::Ptr &pOutputCloud){

	PointCloud2Intensity::Ptr cloud(new PointCloud2Intensity);
	std::string sPcdPath = "/home/zyl/echiev_lidar_curb_detection/log/pcd/" + m_stLSConfig.pcdPath + ".pcd";

	unsigned long long ullTimestamp = stoull(m_stLSConfig.pcdPath);

	if(pcl::io::loadPCDFile(sPcdPath, *cloud) == -1){
		std::cerr << "Couldn't read the PCD file: " << sPcdPath << std::endl;
        return -1;
	}
	else{
		std::cout<<"read the PCD file: "<<sPcdPath<<std::endl;
	}

	for (int i = 0; i < cloud->points.size(); ++i) {
		pcl::PointXYZI temp;
		temp.x = cloud->points[i].x;
		temp.y = cloud->points[i].y;
		temp.z = cloud->points[i].z;
		temp.intensity = cloud->points[i].intensity;

        // cout << "Point " << i << ": "
        //      << cloud->points[i].x << " "
        //      << cloud->points[i].y << " "
        //      << cloud->points[i].z << endl;

		pOutputCloud->points.push_back(temp);
    }

	return ullTimestamp;

}



void LeiShenDrive::Start(){

    std::thread thread1( [=]() {  //lamda表达式创建线程？？？

		bool bOnlineModel 	= m_stLSConfig.onlineModel;
		bool bPcapModel 	= m_stLSConfig.pcapRunningModel;
		bool bPcdModel 		= m_stLSConfig.pcdRunningModel;
		bool bSavePcdFlag 	= m_stLSConfig.savePcd;

		PointCloud2Intensity::Ptr pPointCloud(new PointCloud2Intensity);

//		pcl_viewer = std::make_shared< pcl::visualization::PCLVisualizer>("LSPointCloudViewer");	//标题栏定义一个名称"LSPointCloudViewer"
//		pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);	//黑色背景
//		pcl_viewer->addCoordinateSystem(1.0);	//显示坐标系统方向，可以通过使用X（红色）、Y（绿色）、Z（蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小通过scale参数控制
//		pcl_viewer->addPointCloud<pcl::PointXYZI>(pPointCloud, "lslidar");	// 显示点云，
//		pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lslidar"); // 显示自定义颜色数据，每个点云的RGB颜色字段

		if(bPcdModel)
		{
			LOG_RAW("读取pcd模式\n");
			/*读一张pcd*/
			m_ullUseTimeStamp = GetPointCloudFromPcd(pPointCloud);

//			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
//			pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
			// // pcl_viewer->spinOnce();
	
			VoxelGridProcess(pPointCloud);
			
			m_pLidarCurbDectection->GroudSegmentationStart(pPointCloud, m_ullUseTimeStamp);
		}	

        while (1)
        {
			if(!m_vLidarData.empty())
			{	
				printf("------------\n");//一帧点云，有m_vLidarData_temp.size()个点
				printf("进来的m_vLidarData size: %d \n", m_vLidarData.size());
				pPointCloud->points.clear();

				/*	没有加入周洋代码前的正常运行代码，pcap回放*/
				if(bPcapModel){
					
					// std::vector<MuchLidarData> m_vLidarData_temp;
					// {
					// 	std::lock_guard<std::mutex> lock(m_DataMutex);
					// 	m_vLidarData_temp = m_vLidarData;
					// 	m_ullUseTimeStamp = m_ullCatchTimeStamp;
					// }

					m_DataMutex.lock();
					std::vector<MuchLidarData> m_vLidarData_temp;
					m_vLidarData_temp = m_vLidarData;
					m_ullUseTimeStamp = m_ullCatchTimeStamp;
					m_DataMutex.unlock();

					//	去掉点云中无效点
					for(u_int m_FF = 0; m_FF < m_vLidarData_temp.size(); m_FF++)
					{
						// skip zero valued points
						if( (m_vLidarData_temp[m_FF].X * m_vLidarData_temp[m_FF].X) +
							(m_vLidarData_temp[m_FF].Y * m_vLidarData_temp[m_FF].Y) +
							(m_vLidarData_temp[m_FF].Z * m_vLidarData_temp[m_FF].Z) < 0.0001)
								{continue;}
							
						pcl::PointXYZI tmppoint;
						tmppoint.x = m_vLidarData_temp[m_FF].X;
						tmppoint.y = m_vLidarData_temp[m_FF].Y;
						tmppoint.z = m_vLidarData_temp[m_FF].Z;
						tmppoint.intensity = m_vLidarData_temp[m_FF].Intensity;
						
						if(m_FF>0){

						// std::cout<<"第"<<m_FF<<"个"<< ", x: "<< tmppoint.x<<", y: "<<tmppoint.y<<", z: "<<tmppoint.z<<", tmppoint.intensity: "<<tmppoint.intensity<<std::endl;
						// std::cout<<"前一个点"<<m_FF-1<<"个"<< ", x: "<< m_vLidarData_temp[m_FF-1].X<<", y: "<<m_vLidarData_temp[m_FF-1].Y<<", z: "<<m_vLidarData_temp[m_FF-1].Z<<std::endl;
						}
						pPointCloud->points.push_back(tmppoint);
					}
				}
				
				/*	现场小车跑*/
				if(bOnlineModel){
					
					m_ullUseTimeStamp = GetPointCloudFromOnline(pPointCloud); //涉及数据接收 + 雷达点云坐标转换, 这里pPointCloud是输出
				}	

				/*	保存点云*/
				if(bSavePcdFlag){
					LOG_RAW("保存点云\n");
					SavePcd(pPointCloud, m_ullUseTimeStamp); 
				}
				

//				pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
//				pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
//				pcl_viewer->spinOnce();
				
		
				/*先使用体素滤波向下采样处理点云*/
				VoxelGridProcess(pPointCloud);
				
				m_pLidarCurbDectection->GroudSegmentationStart(pPointCloud, m_ullUseTimeStamp);

				/*	法线估计
				struct timeval tNow;
				gettimeofday(&tNow, NULL);
				unsigned long long ullTimestamp1 = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	
				pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
				pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
				ne.setInputCloud(pPointCloud);
				ne.setSearchMethod(tree);
				ne.setKSearch(50);
				ne.compute(*cloud_normals);
				unsigned long long ullTimestamp2 = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
				printf(" ullTimestamp:%ld\n", (ullTimestamp2 - ullTimestamp1));
				*/				

			}
			else{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			
        }
        
    });
    thread1.detach();
}




void LeiShenDrive::Free(){

    m_pGetLidarData->LidarStop();
}
