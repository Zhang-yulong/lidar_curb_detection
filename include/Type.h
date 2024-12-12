#ifndef TYPE_H
#define TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 各种点云数据类型
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>   // 包含了用于可视化点云的函数和类，用于在3D视窗中现实点云数据


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud2Intensity;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud2RGB;


#endif