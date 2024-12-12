#ifndef LIDARCURBDECTECTION_H
#define LIDARCURBDECTECTION_H

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
// #include "ConnectTools.h"
#include "Type.h"
#include "CurveFitting.h"
#include "Viewer.h"

#include "struct_typedef.h"
#include "communication.h"

class CurveFitting;
class Viewer;


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


class LidarCurbDectection{

public:

    LidarCurbDectection(const float fGroudSegmentationThreshold, const float fLidarVerticalLowerAngle, const float fLidarVerticalUpperAngle,
                        const int iLidarScanRings);

    LidarCurbDectection(Config & config);
    
    ~LidarCurbDectection();

    const int &GetValueOfScanRings();

    void GroudSegmentationStart(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime);
    
   
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

    bool FinalSend(int iLeftId, int iRightId);

private:  

    float m_fGroudSegmentationThreshold;
    float m_fLowerBound;    //雷达的最低垂直视场角 
    float m_fUpperBound;
    int m_iScanRings;       //可以把雷达分成多少线
    float m_fFactor;
    
    int m_iLeftId;
    int m_iRightId;

    std::vector<PointCloud2Intensity::Ptr> m_vCloudPtrList;

    PointCloud2RGB::Ptr m_pAll_deal_cluster_cloud;
    PointCloud2RGB::Ptr m_pAll_output_curve_cloud;

    // PointCloud2Intensity::Ptr m_pGroundPoints;
    // PointCloud2Intensity::Ptr m_pNoGroundPoints;

    CurveFitting *m_pCurveFitting;
    Viewer *m_pViewer;
    Config m_stLCDConfig;
    
    long m_iSystemRunCount;   //记录程序运行次数
    std::queue<LineRunningInfo> m_qLeftLineInfo;
    std::queue<LineRunningInfo> m_qRightLineInfo;
};

#endif
