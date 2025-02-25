#include "LeiShenDrive.h"
#include "pcl/filters/radius_outlier_removal.h"

namespace Lidar_Curb_Dedection
{
std::string CH64w; 
std::string LS128;


LeiShenDrive::LeiShenDrive(){

    m_pGetLidarData = nullptr;
	m_pLidarCurbDectection = nullptr;
}


LeiShenDrive::LeiShenDrive(const STR_CONFIG &config, const STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO &allLidarTransInfo){

    m_pGetLidarData = nullptr;
	m_pLidarCurbDectection = nullptr;
	m_stLSConfig = config;
	m_strAllLidarTransfromInfo = allLidarTransInfo;
}


LeiShenDrive::~LeiShenDrive(){

	m_pGetLidarData->LidarStop();
	delete(m_pLidarCurbDectection);
}


void LeiShenDrive::Free(){

    m_pGetLidarData->LidarStop();
}


void LeiShenDrive::Init(){

	m_sComputerIP = m_stLSConfig.selfComputerIP;
    m_iMsopPort = m_stLSConfig.msopPort;
    m_iDifopPort = m_stLSConfig.difopPort;
    m_sLeiShenType = m_stLSConfig.lidarType;


   	if(m_sLeiShenType == "CH64w"){

		m_pGetLidarData = new GetLidarData_CH64w();
		// m_pGetLidarData = GetLidarData_CH64w*;
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
	i++;
	if(i % 2 != 0)
	{
		return;
	}
	// struct timeval tNow;
	// gettimeofday(&tNow, NULL);
	// unsigned long long ullTimestamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	// printf("pop ullTimestamp:%ld\n", ullTimestamp);
	// // gettimeofday(&tStart, NULL);
	char pchFileName[128];
	bzero(pchFileName, sizeof (pchFileName));
	sprintf(pchFileName, "/home/zyl/echiev_lidar_curb_detection/log/pcd/%lld.pcd", ullTime);
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
	//******************UDP通信初始化**************************//
	//创建socket
	int m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_sock < 0 ){
		LOG_RAW("create getData socket error\n");
        exit(0);
	}
	else{
		LOG_RAW("create getData socket successful\n");
    }


	//端口复用
	int reuse = 1;

	if( setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0){
		LOG_RAW("set getData socket error : reuse address\n");
		close(m_sock);
		exit(0);
	}
	else{
		LOG_RAW("set getData socket successful : reuse address\n");
	}

	if( setsockopt(m_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0){
		LOG_RAW("set getData socket error : reuse port\n");
		close(m_sock);
		exit(0);
	}
	else{
		LOG_RAW("set getData socket successful : reuse port\n");
	}

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iMsopPort);
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	if( bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0){
 		LOG_RAW("bind getData socket error\n");
 		close(m_sock);
        exit(0);
    }
    else{
    	LOG_RAW("bind getData socket successful\n");
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
			m_pGetLidarData->CollectionDataArrive(data, recvLen);	//把数据传入到类内
			// printf("数据放入data\n");
		}
	}
	close(m_sock);
}

//获取设备包的端口号
void LeiShenDrive::GetDevSock()
{
	//******************UDP通信初始化**************************//
	//创建socket
	int m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_sock < 0 ){
		LOG_RAW("create Dev socket error\n");
		close(m_sock);
        exit(0);
	}
	else{
		LOG_RAW("create Dev socket successful\n");
    }


	//端口复用
	int reuse = 1;

	if( setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
	{
		LOG_RAW("set Dev socket error : reuse address\n");
		close(m_sock);
		exit(0);
	}
	else{
		LOG_RAW("set Dev socket successful : reuse address\n");
	}

	if( setsockopt(m_sock, SOL_SOCKET, SO_REUSEPORT, (const char *)&reuse, sizeof(reuse)) < 0)
	{
		LOG_RAW("set Dev socket error : reuse port\n");
		close(m_sock);
		exit(0);
	}
	else{
		LOG_RAW("set Dev socket successful : reuse port\n");
	}

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iDifopPort);		//获取设备包的端口号
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	if( bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0)
	{
		LOG_RAW("bind Dev socket error\n");
		close(m_sock);
		exit(0);
	}
	else{
		LOG_RAW("bind Dev socket successful\n");
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
	close(m_sock);
}

void LeiShenDrive::VoxelGridProcess(PointCloud2Intensity::Ptr &pOutputCloud){

	pcl::VoxelGrid<pcl::PointXYZI> sor;           	//创建滤波对象
	sor.setInputCloud (pOutputCloud);    			//设置需要过滤的点云给滤波对象
	sor.setLeafSize (0.05, 0.05, 0.05);             //设置滤波时创建的体素体积，单位m，因为保留点云大体形状的方式是保留每个体素的中心点，所以体素体积越大那么过滤掉的点云就越多
	sor.filter (*pOutputCloud);          			//执行滤波处理
	// std::cout<<"下采样后点云数量："<<pOutputCloud->points.size()<<std::endl;
}




void LeiShenDrive::PointCloudTransform(std::vector<MuchLidarData> vTmpLidarData, PointCloud2Intensity::Ptr &pOutputCloud)
{
	Eigen::Matrix4f R_ToMainLidar = m_strAllLidarTransfromInfo.toMainLidarInfo.transformMartix;
	float fRad_ToMainLidar = m_strAllLidarTransfromInfo.toMainLidarInfo.rad;
	float fcos_ToMainLidar = cos(fRad_ToMainLidar);
	float fsin_ToMainLidar = sin(fRad_ToMainLidar);

	Eigen::Matrix4f R_ToCar = m_strAllLidarTransfromInfo.toCarInfo.transformMartix; 
	float fRad_ToCar = m_strAllLidarTransfromInfo.toCarInfo.rad;

	for(int i = 0; i < vTmpLidarData.size(); i++)
	{
		// std::cout<<"-------------"<<std::endl;
		Eigen::Matrix<float,4,1> x1_Init, x2_ToMainLidar, x3_ToCar, x4_Final;
		x1_Init << vTmpLidarData[i].X, vTmpLidarData[i].Y, vTmpLidarData[i].Z, 1;
		x2_ToMainLidar = R_ToMainLidar * x1_Init;
		pcl::PointXYZI tmppoint_ToMainLidar;
		tmppoint_ToMainLidar.x =  cos(fRad_ToMainLidar) * x2_ToMainLidar(0) + sin(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.y = -sin(fRad_ToMainLidar) * x2_ToMainLidar(0) + cos(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.z = x2_ToMainLidar(2) ;
		tmppoint_ToMainLidar.intensity = vTmpLidarData[i].Intensity;

		// pcl::PointXYZI tmppoint_ToMainLidar;
		// tmppoint_ToMainLidar.x =  cos(fRad_ToMainLidar) * x1_Init(0) + sin(fRad_ToMainLidar) * x1_Init(1) +  R_ToMainLidar();
		// tmppoint_ToMainLidar.y = -sin(fRad_ToMainLidar) * x1_Init(0) + cos(fRad_ToMainLidar) * x1_Init(1) +  ;
		// tmppoint_ToMainLidar.z = x1_Init(2) ;
		// tmppoint_ToMainLidar.intensity = vTmpLidarData[i].Intensity;




		x3_ToCar << tmppoint_ToMainLidar.x, tmppoint_ToMainLidar.y, tmppoint_ToMainLidar.z, 1;
		x4_Final = R_ToCar * x3_ToCar;


		// x4_Final = R_ToCar * R_ToMainLidar * x1_Init;
		// x4_Final = R_ToMainLidar * x1_Init;
		
		pcl::PointXYZI tmppoint_ToCar;
		tmppoint_ToCar.x = x4_Final(0);
		tmppoint_ToCar.y = x4_Final(1);
		tmppoint_ToCar.z = x4_Final(2);
		tmppoint_ToCar.intensity = vTmpLidarData[i].Intensity;

		pOutputCloud->points.push_back(tmppoint_ToCar);
	}

	pOutputCloud->width    = pOutputCloud->points.size();
	pOutputCloud->height   = 1;
	pOutputCloud->is_dense = false;

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

	// ApplyRadiusOutlierFilter(pOutputCloud);

	return ullTimestamp;

}


void LeiShenDrive::ApplyRadiusOutlierFilter(PointCloud2Intensity::Ptr &InputCloud){

	pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierFilter;
	
	radiusOutlierFilter.setInputCloud(InputCloud);
	radiusOutlierFilter.setRadiusSearch(0.05);
	radiusOutlierFilter.setMinNeighborsInRadius(8);
	radiusOutlierFilter.filter(*InputCloud);
}


void LeiShenDrive::Start(){

    std::thread thread1( [=]() {  //lamda表达式创建线程？？？

		bool bOnlineModel 	= m_stLSConfig.onlineModel;
		bool bPcapModel 	= m_stLSConfig.pcapRunningModel;
		bool bPcdModel 		= m_stLSConfig.pcdRunningModel;
		bool bSavePcdFlag 	= m_stLSConfig.savePcd;

		PointCloud2Intensity::Ptr pPointCloud(new PointCloud2Intensity);

		if(bOnlineModel && !bPcapModel && !bPcdModel){
			LOG_RAW("在线模式\n");
		}
		else if(bPcapModel && !bOnlineModel && !bPcdModel){
			LOG_RAW("pcap读取模式\n");
			pcl_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("LSPointCloudViewer");	//标题栏定义一个名称"LSPointCloudViewer"
			pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);	//黑色背景
			pcl_viewer->addCoordinateSystem(1.0);	//显示坐标系统方向，可以通过使用X（红色）、Y（绿色）、Z（蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小通过scale参数控制
			pcl_viewer->addPointCloud<pcl::PointXYZI>(pPointCloud, "lslidar");	// 显示点云，
			pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lslidar"); // 显示自定义颜色数据，每个点云的RGB颜色字段

		}
		else if(bPcdModel && !bPcapModel && !bOnlineModel){
			LOG_RAW("pcd读取模式\n");
		}
		else{
			LOG_RAW("模式未选择\n");
		}


		// pcl_viewer = std::make_shared< pcl::visualization::PCLVisualizer>("LSPointCloudViewer");	//标题栏定义一个名称"LSPointCloudViewer"
		// pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);	//黑色背景
		// pcl_viewer->addCoordinateSystem(1.0);	//显示坐标系统方向，可以通过使用X（红色）、Y（绿色）、Z（蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小通过scale参数控制
		// pcl_viewer->addPointCloud<pcl::PointXYZI>(pPointCloud, "lslidar");	// 显示点云，
		// pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lslidar"); // 显示自定义颜色数据，每个点云的RGB颜色字段


		if(bPcdModel)
		{
			LOG_RAW("读取pcd模式\n");
			/*读一张pcd*/
			m_ullUseTimeStamp = GetPointCloudFromPcd(pPointCloud);

//			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
//			pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
			// // pcl_viewer->spinOnce();
	
			// VoxelGridProcess(pPointCloud);
			
			m_pLidarCurbDectection->GroudSegmentationStart(pPointCloud, m_ullUseTimeStamp);
		}	



        while (1)
        {
			if(!m_vLidarData.empty())
			{	

				LOG_RAW("------------time: %lld , m_vLidarData size: %ld \n", m_ullCatchTimeStamp, m_vLidarData.size());
				pPointCloud->points.clear();

				/*	没有加入周洋代码前的正常运行代码，pcap回放*/
				if(bPcapModel){
					
					// std::vector<MuchLidarData> m_vLidarData_offline;
					// {
					// 	std::lock_guard<std::mutex> lock(m_DataMutex);
					// 	m_vLidarData_offline = m_vLidarData;
					// 	m_ullUseTimeStamp = m_ullCatchTimeStamp;
					// }

					m_DataMutex.lock();
					std::vector<MuchLidarData> m_vLidarData_offline;
					m_vLidarData_offline = m_vLidarData;
					m_ullUseTimeStamp = m_ullCatchTimeStamp;
					m_DataMutex.unlock();

					PointCloudTransform(m_vLidarData_offline, pPointCloud);

					// ApplyRadiusOutlierFilter(pPointCloud);

					pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
					pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
					pcl_viewer->spinOnce();

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
				

				// pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
				// pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
				// pcl_viewer->spinOnce();
				
		
				/*先使用体素滤波向下采样处理点云*/
				// VoxelGridProcess(pPointCloud);
				
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
				// LOG_RAW("m_vLidarData empty!!!\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			
        }
    });
    thread1.detach();
}





}