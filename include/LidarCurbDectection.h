#ifndef LIDARCURBDECTECTION_H
#define LIDARCURBDECTECTION_H

#include <pcl/common/common.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>  // 模型系数的定义
#include <pcl/sample_consensus/method_types.h> // 包含用于采样一致性算法的不同方法的定义，如RANSAC、MSAC等
#include <pcl/sample_consensus/model_types.h> // 包含用于采样一致性算法的不同模型的定义，如平面、球体、圆柱体
#include <pcl/segmentation/sac_segmentation.h>  // 包含用于分割点云的采样一致性算法（SACSegmentation）的定义，用于识别点云的几何模型
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>    // 包含用于从点云中提取特定索引的函数和类，用于根据索引提取点云中的子集
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <time.h>
#include <condition_variable>
#include <queue>
#include <thread>
#include <chrono>

#include "Type.h"
#include "CurveFitting.h"
#include "Viewer.h"

#include "struct_typedef.h"
#include "communication.h"

namespace Lidar_Curb_Dedection
{

class LidarCurbDectection;

struct LineInfo {
    cv::Vec4f lineParams; // 直线参数 (vx, vy, x0, y0)
    double length;        // 直线长度
    double avg_x;
};

struct PolarLine{
    double rho;     //距原点的距离
    double theta;   //法向量与x轴的夹角
};

struct LineRunningInfo{
    long runningValue;   //程序运行次数
    bool existFlag;
    PointCloud2Intensity::Ptr linePointCloud;
};

// 计算直线的斜率和截距
struct HoughSPLineInfo {
    double slope;  // 斜率
    double intercept;  // 截距
    cv::Vec4i line;  // 原始直线段
};

struct Line {
    cv::Point2f start;
    cv::Point2f end;
    // float slope;
    float angle;
    float length;

    // 构造函数
    Line(const cv::Vec4i& line) {
        // std::cout<<"Line 构造"<<std::endl;
        start = cv::Point2f(line[0], line[1]);
        end = cv::Point2f(line[2], line[3]);
       
        // slope = (end.x - start.x) != 0 ? (end.y - start.y) / (end.x - start.x) : std::numeric_limits<float>::infinity();
        // 使用 atan2 计算角度，避免斜率除以零的问题
        angle = std::abs(std::atan2(end.y - start.y, end.x - start.x))* 180 /M_PI;
        length = std::sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
    }
};



class LidarCurbDectection{

public:

    LidarCurbDectection(const STR_CONFIG & config);
    
    ~LidarCurbDectection();

    const int &GetValueOfScanRings();

    int GroudSegmentationStart(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime);
    
   
private:   

    void CalculateScanID(PointCloud2Intensity::Ptr pInCloud, PointCloud2Intensity::Ptr &pOutCloud);

    void GroudFilterFromSAC(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints);

    void GroudFilterFromAPMF(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints);

    void ExtractGroundFromSAC( PointCloud2Intensity::Ptr &pIncloud,
                        PointCloud2Intensity::Ptr &pOutcloud,
                        pcl::PointIndices::Ptr pIndices,
                        bool bSetNeg);

    void ExtractGroundFromPMF( PointCloud2Intensity::Ptr &pIncloud,
                        PointCloud2Intensity::Ptr &pOutcloud,
                        pcl::PointIndicesPtr pIndices,
                        bool bSetNeg);

    void ExtractPointsByHeightDifference(PointCloud2Intensity::Ptr &pInCloud);
    
    bool IsClustering(  const float PointsDistance, 
                        const float MaximumClusterThreshold, 
                        const float MimimumClusterThreshold, 
                        PointCloud2Intensity::Ptr pInputCloud,
                        std::vector<pcl::PointIndices> &vClusterIndices);

    bool IsClustering(  const float PointsDistance, 
                        const float MaximumClusterThreshold, 
                        const float MimimumClusterThreshold, 
                        PointCloud2Intensity::Ptr pInputCloud, 
                        std::vector<PointCloud2Intensity::Ptr> &vClusterPtr);

    double euclideanDistance(const cv::Point& p1, const cv::Point& p2);
    double calculateAngle(const cv::Vec4f& line1, const cv::Vec4f& line2);
    double computeXDistance(const cv::Point2f& center1, const cv::Point2f& center2);
    bool isDirectionSimilar(const cv::Vec4f& line1, const cv::Vec4f& line2, double angleThreshold);
    bool isBoundingBoxOverlapping(const cv::Rect& bbox1, const cv::Rect& bbox2);
    double computeYDistance(const cv::Point2f& center1, const cv::Point2f& center2);
    double pointToLineDistance(const cv::Point& pt, const cv::Vec4f& line);
    bool isLineAngleValid(const cv::Vec4f& line, double maxAngle);
    void normalizeLineABC(double &A, double &B, double &C);    

    std::vector<std::vector<Line>> clusterLinesBySlope(const std::vector<Line>& lines, float slope_threshold);
    std::vector<Line> findLongestCluster(const std::vector<std::vector<Line>>& clusters);
    double calculateSlope(const cv::Vec4i& line);
    double calculateIntercept(const cv::Vec4i& line, double slope);
    std::vector<std::vector<HoughSPLineInfo>> clusterLines(const std::vector<HoughSPLineInfo>& lines, double slope_threshold, double distance_threshold);
    cv::Vec4f fitLineToCluster(const std::vector<HoughSPLineInfo>& cluster);
    float calculateLength(const cv::Point& start, const cv::Point& end);
    

    cv::Point2f computeCentroid(const std::vector<cv::Point>& contour);
    void splitContoursByCentroid(const std::vector<cv::Point>& contour, 
                             std::vector<cv::Point>& finalContour, 
                             int iSideFlag, 
                             cv::Mat &image);
    void ransacFitLine(const std::vector<cv::Point>& points, cv::Vec4f& bestLine, std::vector<cv::Point>& inliers, 
                   int maxIterations , double threshold , double maxAngle , double inlierRatio );
    
    void findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image);
    void findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image, int iSideFlag);
    

    bool IsClustering_hough(  
                        PointCloud2Intensity::Ptr pInputCloud, 
                        std::vector<PointCloud2Intensity::Ptr> &vClusterPtr,
                        unsigned long long ullTime,
                        int iSideFlag);                    
    
    void EdgeClusteringProcess( PointCloud2Intensity::Ptr pAfterVoxelGrid, 
                                PointCloud2Intensity::Ptr pNoGroudPoints, 
                                unsigned long long ullTime);

    PointCloud2Intensity::Ptr SlideWindowProcess(std::queue<LineRunningInfo> &qFinalClusterInfo);

    // void CurveFitting(PointCloud2Intensity::Ptr pInCloud, std::vector<pcl::PointIndices> &vClusterIndices);  //尝试写的另一个文件里
    
    PointCloud2Intensity::Ptr getCloserRightCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters);
    PointCloud2Intensity::Ptr getCloserLeftCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters);


private:  

    float m_fGroudSegmentationThreshold;
    float m_fLowerBound;    //雷达的最低垂直视场角 
    float m_fUpperBound;
    int m_iScanRings;       //可以把雷达分成多少线
    float m_fFactor;
    
    Eigen::Matrix<float,3,3> R_ToLeftProject;
    Eigen::Matrix<float,3,3> R_ToRightProject;
    Eigen::Matrix<float,3,3> R_Inv_ToLeftProject;
    Eigen::Matrix<float,3,3> R_Inv_ToRightProject;

    std::vector<PointCloud2Intensity::Ptr> m_vCloudPtrList;

    std::shared_ptr<pcl::visualization::PCLVisualizer> ground_viewer;
    std::shared_ptr<pcl::visualization::PCLVisualizer> no_ground_viewer;
    std::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer;

    PointCloud2RGB::Ptr m_pAll_deal_cluster_cloud;
    PointCloud2RGB::Ptr m_pAll_output_curve_cloud;

    // PointCloud2Intensity::Ptr m_pGroundPoints;
    // PointCloud2Intensity::Ptr m_pNoGroundPoints;

    CurveFitting *m_pCurveFitting;
    Viewer *m_pViewer;
    STR_CONFIG m_strLCDConfig;
    
    long m_iSystemRunCount;   //记录程序运行次数
    std::queue<LineRunningInfo> m_qLeftLineInfo;
    std::queue<LineRunningInfo> m_qRightLineInfo;
};
}
#endif
