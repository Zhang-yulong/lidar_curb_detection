#ifndef LEISHENDRIVE_H
#define LEISHENDRIVE_H

#include <iostream>
#include <stdlib.h>

#include <pcl/filters//radius_outlier_removal.h>

#include "BaseDrive.h"
#include "LidarCurbDectection.h"



#include "GetLidarData_CH64w.h"
#include "GetLidarData_LS128.h"
#include "readconfigfile.h"
#include "ReadYamlFile.h"


// enum eLeishen_LidarType = {s_LS_CH64w, s_LS_128}


# define pcl_isfinite(x) std::isfinite(x)


namespace Lidar_Curb_Dedection
{


class LeiShenDrive : public BaseDrive
{
public:
    LeiShenDrive();
    LeiShenDrive(const STR_CONFIG &config, const STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO &allLidarTransInfo);
    ~LeiShenDrive();

    void Start();
    
    void Init();
    
    void Free(); 

    void CallBackFunction(std::vector<MuchLidarData> LidarDataValue, int);	//回调函数，获取每一帧的数据
    void GetLidarType(const std::string LidarType);

    static void startGetDevSock(void *pth );  
    static void startGetDataSock(void *pth );

    unsigned long long GetPointCloudFromOnline(PointCloud2Intensity::Ptr &pOutputCloud);

private: 
    void InitSocket(); //把获取设备包的端口号getDevSock()和获取数据包的端口号getDataSock() 包装在一起 (没写)

    void GetDevSock();
    void GetDataSock();

    void SavePcd(const PointCloud2Intensity::Ptr &InputCloud, const unsigned long long &ullTime);

    void PointCloudTransform(std::vector<MuchLidarData> vTmpLidarData, PointCloud2Intensity::Ptr &pOutputCloud);

    unsigned long long GetPointCloudFromPcd(PointCloud2Intensity::Ptr &pOutputCloud);

    void VoxelGridProcess(PointCloud2Intensity::Ptr &pOutputCloud);

    void ApplyRadiusOutlierFilter( PointCloud2Intensity::Ptr &InputCloud);

private: 
    unsigned long long m_ullCatchTimeStamp;
    unsigned long long m_ullUseTimeStamp;

    int m_Sock;
    std::string m_sComputerIP;
    int m_iMsopPort; //数据端口
    int m_iDifopPort; //设备端口
    std::string m_sLeiShenType;

    Fun fun;
    // GetLidarData *m_pGetLidarData;  //没有命名空间时使用
    GetLidarData_CH64w *m_pGetLidarData;    //使用了命名空间时使用
    LidarCurbDectection *m_pLidarCurbDectection;
    STR_CONFIG m_stLSConfig;
    STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO m_strAllLidarTransfromInfo;
    
    std::vector<MuchLidarData> m_vLidarData;
    std::mutex m_DataMutex;

    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;

    // std::shared_ptr<pcl::visualization::PCLVisualizer> after_ground_viewer;
    

};
}

#endif