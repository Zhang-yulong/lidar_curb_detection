#ifndef TYPE_H
#define TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 各种点云数据类型
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>   // 包含了用于可视化点云的函数和类，用于在3D视窗中现实点云数据

#include <Eigen/Core>

namespace Lidar_Curb_Dedection
{  

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud2Intensity;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud2RGB;



struct STR_LIDAR_TRANSFORM_CONFIG_INFO{
	Eigen::Matrix4f transformMartix;
	float rad;
};


struct STR_ALL_LIDAR_TRANSFORM_CONFIG_INFO{
	STR_LIDAR_TRANSFORM_CONFIG_INFO toMainLidarInfo;
	STR_LIDAR_TRANSFORM_CONFIG_INFO toCarInfo;
};


}
#endif