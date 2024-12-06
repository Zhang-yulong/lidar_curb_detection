#ifndef READYAMLFILE_H
#define READYAMLFILE_H

#include <opencv/cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>

using namespace cv;

struct Config {
    

    // Lidar
    std::string selfComputerIP; 
    int msopPort;
    int difopPort;
    std::string lidarType;
    int verticalUpperAngle;
    int verticalLowerAngle;
    int scanRings;

    // Road
    float leftXMin;
    float leftXMax;
    float leftYMax;
    float rightXMin;
    float rightXMax;
    float rightYMax;
    float heightUpper;
    float heightLower;

    // GroudSegmentationFromSAC:
    float groudSegmentationThreshold;
    
    // GroudSegmentationFromPMF:
    int maxWindowSize;
    float slope;
    float initialDistance;
    float maxDistance;
    int cellSize;
    int base;

    // Clustering:
    float leftMinClusterThreshold;
    float leftMaxClusterThreshold;
    float leftPointsDistance;
    float rightMinClusterThreshold;
    float rightMaxClusterThreshold;
    float rightPointsDistance;

    //HoughLinesP:
    int hough_width;
    int hough_height;
    float hough_scale;
    float hough_offsetX;
    float hough_left_offsetY;
    float hough_right_offsetY;
    float hough_rho;
    float hough_theta;
    float hough_threshold;
    float hough_minLineLength;
    float hough_maxLineGap;

    // CurveFitting:
    float distinguishRoadSideThreshold;

    // RANSAC:
    int interations;
    float simga;
    float kMin;
    float kMax;

    // Model
    int pcapModel;
    int onlyPcd;
    std::string pcdPath;

    // Viewer
    int projection;
    int savePicture;
    
    int viewer_width;
    int viewer_height;
    float viewer_scale;
    float viewer_offsetX;
    float viewer_offsetY;

    //Pcd
    int savePcd;
};

class YamlReader {
public:
    // 构造函数，传入配置文件路径
    YamlReader(const std::string& configFilePath);
      
    // 从配置文件中加载参数
    bool LoadConfig(Config& config);

private:
    std::string m_sConfigFilePath; // 配置文件路径
};


#endif