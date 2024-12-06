#ifndef CUREFITTING_H
#define CUREFITTING_H

#include <Eigen/Core>
#include <pcl/PointIndices.h>

#include "Type.h"
#include "ReadYamlFile.h"
#include "kalman.h"

class LineKalmanFilter;

// 极坐标直线
struct LinePolar {
    double theta; // 角度 [0, π)
    double d;     // 到原点的距离 d >= 0
};

//直线截距式
struct LineKBFunction{
    double k;
    double b;
};

class CurveFitting
{

public:
    CurveFitting(const float fDistinguishRoadSideThreshold);
    CurveFitting(Config & config);

    ~CurveFitting();


    PointCloud2Intensity::Ptr CurveFittingStart(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime, int iSideFlag);


private:
   
    Eigen::VectorXd PolynomialCurveFit(const std::vector<Eigen::Vector2d>& points, int order);
    
    PointCloud2Intensity::Ptr GenerateCurve(const Eigen::VectorXd& Coefficients, double start_x, double end_x, double step);
    
    PointCloud2RGB::Ptr GenerateCurveRGB(const Eigen::VectorXd& Coefficients, double start_x, double end_x, double step);

    // double pointToLineDistance(const Point& p, double a, double b, double c);
    double calculateSlope(const Point& p1, const Point& p2);
    LinePolar fitLinePolar(const Point& p1, const Point& p2);
    LinePolar normalizePolar(double theta, double d);
    double pointToLineDistance(const cv::Point& point, const LinePolar& line);
    LineKBFunction polarToCartesian(const LinePolar& line);


    void computeCovarianceMatrix(const std::vector<cv::Point>& points, double S[3][3]);
    void computeSmallestEigenVector(double S[3][3], double eigenVector[3]);
    void fitLineImplicit(const std::vector<cv::Point>& points, double& A, double& B, double& C, double epsilon);
    PointCloud2RGB::Ptr GenerateCurveRGB(PointCloud2Intensity::Ptr pInCloud, float yMin, float yMax, unsigned long long ullTime);

    // PointCloud2Intensity::Ptr GetLargestCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters);

    float m_fDistinguishRoadSideThreshold; //用于区分左右簇
    
    
    Config m_stCFConfig;
    LineKalmanFilter *m_pKF;
};





#endif