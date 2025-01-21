#include "Viewer.h"

namespace Lidar_Curb_Dedection
{

std::string Image_folderPath= "/home/zyl/echiev_lidar_curb_detection/log/image/";

Viewer::Viewer(int width, int height, double scale, Point2d offset)
    : m_iWidth(width), m_iHeight(height), m_dScale(scale), m_offset(offset) 
{
    m_image = Mat::zeros(height, width, CV_8UC3);  // 创建图像并初始化为黑色
}

Viewer::Viewer(const STR_CONFIG & config)
{

    m_iWidth = config.viewer_width;
    m_iHeight = config.viewer_height;
    m_dScale = config.viewer_scale;
    m_offset.x = config.viewer_offsetX;
    m_offset.y = config.viewer_offsetY;
    m_image = Mat::zeros(m_iHeight, m_iWidth, CV_8UC3);  // 创建图像并初始化为黑色
}


Viewer::~Viewer(){

}

// #if 0
// //以下图像都是Y轴正方向向右，x轴正方向向下


// // 绘制图像坐标轴
// // Y轴向右，x轴向下
// void Viewer::DrawAxes() {
//     // 绘制y轴（水平向右）
//     line(m_image, Point(0, m_offset.x), Point(m_iWidth, m_offset.x), Scalar(255, 0, 0), 1);  // 蓝色线
//     putText(m_image, "Y", Point(m_iWidth - 50, m_offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);

//     // 绘制x轴（垂直向下）
//     line(m_image, Point(m_offset.y, 0), Point(m_offset.y, m_iHeight), Scalar(0, 0, 255), 1); // 红色线
//     putText(m_image, "X", Point(m_offset.y + 10, m_iHeight - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    
// }


// // 绘制与x轴平行的等距线（x=interval的倍数）
// void Viewer::DrawEquidistantLines() 
// {
//     // 等距线间隔，单位为米
//     double interval = 5.0;

//     // 计算等距线在图像中的位置并绘制
//     for (double x = -50; x <= 50; x += interval) {
//         int y = static_cast<int>(x * m_dScale + m_offset.y); // 计算等距线在图像中的位置
//         if(x == 0)
//             continue;
//         else{
//             if (y >= 0 && y < m_iWidth) {  // 确保线在图像边界内
//                 // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
//                 putText(m_image, std::to_string(static_cast<int>(x)) + "m", Point(y + 5, m_offset.x - 10), 
//                         FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
//             }
//         }   
//     }

// }


// // 绘制与y轴平行的等距线（y=interval的倍数）
// void Viewer::DrawHorizontalEquidistantLines() 
// {
//     // 等距线间隔，单位为米
//     double interval = 1.0;

//     // 计算等距线在图像中的位置并绘制
//     for (double y = -50; y <= 50; y += interval) {
//         int x = static_cast<int>(y * m_dScale + m_offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
//         if (x >= 0 && x < m_iHeight) {  // 确保线在图像边界内
//             line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
//             putText(m_image, std::to_string(static_cast<int>(y)) + "m", Point(m_offset.y + 10, x - 5), 
//                     FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
//         }
//     }
// }


// void Viewer::ProjectPointCloud(const PointCloud2RGB::Ptr &pAllCluster, const PointCloud2RGB::Ptr &pAllCurve)
// {
    
//     // 清空图像
//     m_image.setTo(Scalar(0, 0, 0));   //黑色

//     DrawAxes();
//     DrawEquidistantLines();
//     DrawHorizontalEquidistantLines();

//     // 遍历所有聚类簇点云
//     for (const auto& pt : pAllCluster->points) {
        
//         // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
//         int x = static_cast<int>(pt.x * m_dScale + m_offset.x);
//         int y = static_cast<int>(pt.y * m_dScale + m_offset.y);

//         // 检查是否在图像边界内
//         if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
//             circle(m_image, Point(y, x), 3, Scalar(0, 255, 0), -1); // 绘制绿色小圆点
//         else
//             printf("聚类簇点云不在图像内 (%d , %d)\n",x,y);
         
//     }

//     // 遍历所有拟合直线点云
//     for (const auto& pt : pAllCurve->points) {
//         // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
//         int x = static_cast<int>(pt.x * m_dScale + m_offset.x);
//         int y = static_cast<int>(pt.y * m_dScale + m_offset.y);

//         // 检查是否在图像边界内
//         if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
//             circle(m_image, Point(y, x), 3, Scalar(255, 255, 255), -1); // 绘制白色小圆点
//         else
//             printf("拟合直线点云不在图像内 (%d , %d)\n",x,y);

//     }



// }
// #endif


// 显示投影后的图像
void Viewer::Display()
{
    imshow("Projected Point Cloud", m_image);
    
}


// 显示投影后的图像
cv::Mat Viewer::Display() const
{
    return m_image;
    
}

void Viewer::SaveImage(unsigned long long ullTime)
{
    std::string sFilePath = Image_folderPath + std::to_string(ullTime) + ".png";
    imwrite(sFilePath, m_image);
}


void Viewer::SaveImage(std::string folderPath, unsigned long long ullTime)
{
    std::string sFilePath = folderPath + std::to_string(ullTime) + ".png";
    imwrite(sFilePath, m_image);
}



#if 1
// 与点云同样方向

// 绘制图像坐标轴
// 这里车辆中心点会提高一点，要查原因
void Viewer::DrawAxes() {
    // 绘制x轴
    line(m_image, Point(0, m_offset.x), Point(m_iWidth, m_offset.x), Scalar(255, 0, 0), 1);  // 蓝色线
    putText(m_image, "X", Point(m_iWidth - 50, m_offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);

    // 绘制x轴
    line(m_image, Point(m_offset.y, 0), Point(m_offset.y, m_iHeight), Scalar(255, 0, 0), 1); // 蓝色线
    putText(m_image, "Y", Point(m_offset.y + 10,  20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    
}


// 绘制与y轴平行的等距线（y=interval的倍数）
void Viewer::DrawEquidistantLines() 
{
    // 等距线间隔，单位为米
    double interval = 1.0;

    // 计算等距线在图像中的位置并绘制
    for (double x = -50; x <= 50; x += interval) {
        int y = static_cast<int>(x * m_dScale + m_offset.y); // 计算等距线在图像中的位置
        if(x == 0)
            continue;
        else{
            if (y >= 0 && y < m_iWidth) {  // 确保线在图像边界内
                // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
                putText(m_image, std::to_string(static_cast<int>(x)), Point(y, m_offset.x + 20), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
        }   
    }

}


// 绘制与x轴平行的等距线（x=interval的倍数）
void Viewer::DrawHorizontalEquidistantLines() 
{
    // 等距线间隔，单位为米
    double interval = 5.0;

    // 计算等距线在图像中的位置并绘制
    for (double y = -50; y <= 50; y += interval) {
        int x = static_cast<int>(-y * m_dScale + m_offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
        if (x >= 0 && x < m_iHeight) {  // 确保线在图像边界内
            // line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
            putText(m_image, std::to_string(static_cast<int>(y)) + "m", Point(m_offset.y + 10, x - 5), 
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
        }
    }
}


void Viewer::ProjectPointCloud(const PointCloud2Intensity::Ptr &pAllInit, const PointCloud2Intensity::Ptr &pAllNoGroud, const PointCloud2RGB::Ptr &pAllCluster, const PointCloud2RGB::Ptr &pAllCurve)
{
    
    // 清空图像
    m_image.setTo(Scalar(0, 0, 0));   //黑色

    DrawAxes();
    DrawEquidistantLines();
    DrawHorizontalEquidistantLines();

    if(!pAllInit->points.empty()){
        // 遍历原始点云
        for (const auto& pt : pAllInit->points) {
            
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 1, Scalar(255, 255, 255), -1); // 绘制白色小圆点
            // else
            //     printf("原始点云不在图像内 (%d , %d)\n",x,y);
        
        }
    }

    if(!pAllNoGroud->points.empty())
    {    
        // 遍历所有非地面点点云
        for (const auto& pt : pAllNoGroud->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 1, Scalar(0, 0, 255), -1); // 绘制红色小圆点
            // else
            //     printf("非地面点点云不在图像内 (%d , %d)\n",x,y);

        }
    }

    if(!pAllCluster->points.empty())
    {
        // 遍历所有聚类簇点云
        for (const auto& pt : pAllCluster->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight){
                // Scalar color = pt.x > 0 ? Scalar(0, 255, 0) : Scalar(0, 0, 255); // 绿色(x轴>0)或红色(x轴<0)
                // circle(m_image, Point(y, x), 1, color, -1);
                circle(m_image, Point(y, x), 7, Scalar(0, 255, 0), -1); // 绘制绿色小圆点
            }
            else
                printf("聚类簇点云不在图像内 (%d , %d)\n",x,y);
        }
    }

    if(!pAllCurve->points.empty())
    {
        // 遍历所有拟合直线点云
        for (const auto& pt : pAllCurve->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 10, Scalar(255, 0, 0), -1); // 绘制蓝色小圆点
            else
                printf("拟合直线点云不在图像内 (%d , %d)\n",x,y);

        }
    }
}

#if 0 
void Viewer::ProjectPointCloud(const PointCloud2Intensity::Ptr &pAllNoGroud, const PointCloud2RGB::Ptr &pAllCluster)
{
    
    // 清空图像
    m_image.setTo(Scalar(0, 0, 0));   //黑色

    DrawAxes();
    DrawEquidistantLines();
    DrawHorizontalEquidistantLines();

    if(!pAllNoGroud->points.empty()){
        // 遍历所有非地面点点云
        for (const auto& pt : pAllNoGroud->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 2, Scalar(0, 0, 255), -1); // 绘制红色小圆点
            else
                printf("非地面点点云不在图像内 (%d , %d)\n",x,y);

        }
    }    

    if(!pAllCluster->points.empty())
    {
        // 遍历所有聚类簇点云
        for (const auto& pt : pAllCluster->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight){
                // Scalar color = pt.x > 0 ? Scalar(0, 255, 0) : Scalar(0, 0, 255); // 绿色(x轴>0)或红色(x轴<0)
                // circle(m_image, Point(y, x), 1, color, -1);
                circle(m_image, Point(y, x), 1, Scalar(0, 255, 0), -1); // 绘制绿色小圆点
            }
            else
                printf("聚类簇点云不在图像内 (%d , %d)\n",x,y);
        }
    }

    
}
#endif





}
#endif