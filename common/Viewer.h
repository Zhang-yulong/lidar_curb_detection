#ifndef VIEWER_H
#define VIEWER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "Type.h"
#include "ReadYamlFile.h"

using namespace cv;

namespace Lidar_Curb_Dedection
{
class Viewer
{
public:
    // 构造函数，初始化图像大小、比例和偏移量
    Viewer(int width, int height, double scale, cv::Point2d offset);
    Viewer(const STR_CONFIG & config);
    ~Viewer();
    void ProjectPointCloud( const PointCloud2Intensity::Ptr &pAllInit, 
                            const PointCloud2Intensity::Ptr &pAllNoGroud, 
                            const PointCloud2RGB::Ptr &pAllCluster, 
                            const PointCloud2RGB::Ptr &pAllCurve);
    
    void ProjectPointCloud(  
                            const PointCloud2Intensity::Ptr &pAllNoGroud, 
                            const PointCloud2RGB::Ptr &pAllCluster);
    
    void Display();
    cv::Mat Display() const;
    
    void SaveImage(unsigned long long ullTime);
    void SaveImage(std::string folderPath, unsigned long long ullTime);
    
private:
    
    int m_iWidth;           // 图像宽度
    int m_iHeight;          // 图像高度          
    double m_dScale;        // 真实环境1米对应的像素数        
    cv::Point2d m_offset;   // 偏移量，定义原点在图像中的位置
    cv::Mat m_image;        // 用于显示点云的图像

    void DrawAxes();
    void DrawEquidistantLines();
    void DrawHorizontalEquidistantLines();

    
};
}
#endif
