#include "ReadYamlFile.h"



YamlReader::YamlReader(const std::string& configFilePath) 
    : m_sConfigFilePath(configFilePath) 
{

}


bool YamlReader::LoadConfig(Config& config) 
{
    FileStorage fs(m_sConfigFilePath, FileStorage::READ); // 打开文件以读取

    if (!fs.isOpened()) {
        std::cerr << "Error: Unable to open config file: " << m_sConfigFilePath << std::endl;
        return false;
    }

    // 读取lidar
    config.selfComputerIP       = (std::string)fs["Lidar"]["selfComputerIP"];
    config.msopPort             = (int)fs["Lidar"]["msopPort"];
    config.difopPort            = (int)fs["Lidar"]["difopPort"];
    config.lidarType            = (std::string)fs["Lidar"]["lidarType"];
    config.verticalUpperAngle   = (int)fs["Lidar"]["verticalUpperAngle"];
    config.verticalLowerAngle   = (int)fs["Lidar"]["verticalLowerAngle"];
    config.scanRings            = (int)fs["Lidar"]["scanRings"];

    // 读取道路
    config.leftXMin             = (float)fs["Road"]["leftXMin"];
    config.leftXMax             = (float)fs["Road"]["leftXMax"];
    config.leftYMax             = (float)fs["Road"]["leftYMax"];
    config.rightXMin            = (float)fs["Road"]["rightXMin"];
    config.rightXMax            = (float)fs["Road"]["rightXMax"];
    config.rightYMax            = (float)fs["Road"]["rightYMax"];
    config.heightLower          = (float)fs["Road"]["heightLower"];
    config.heightUpper          = (float)fs["Road"]["heightUpper"];
    config.isDetectionLeftRoad  = (int)fs["Road"]["isDetectionLeftRoad"];
    config.isDetectionRightRoad = (int)fs["Road"]["isDetectionRightRoad"];

    // 读取地面分割SAC算法
    config.groudSegmentationThreshold  = (float)fs["GroudSegmentationFromSAC"]["groudSegmentationThreshold"];
    
    // 读取形态学分割算法
    config.maxWindowSize            = (int)fs["GroudSegmentationFromPMF"]["maxWindowSize"];
    config.slope                    = (float)fs["GroudSegmentationFromPMF"]["slope"];
    config.initialDistance          = (float)fs["GroudSegmentationFromPMF"]["initialDistance"];
    config.maxDistance              = (float)fs["GroudSegmentationFromPMF"]["maxDistance"];
    config.cellSize                 = (int)fs["GroudSegmentationFromPMF"]["cellSize"];
    config.base                     = (int)fs["GroudSegmentationFromPMF"]["base"];

    // 读取聚类参数(欧式聚类)
    config.leftMinClusterThreshold  = (float)fs["Clustering"]["leftMinClusterThreshold"];
    config.leftMaxClusterThreshold  = (float)fs["Clustering"]["leftMaxClusterThreshold"];
    config.leftPointsDistance       = (float)fs["Clustering"]["leftPointsDistance"];
    config.rightMinClusterThreshold = (float)fs["Clustering"]["rightMinClusterThreshold"];
    config.rightMaxClusterThreshold = (float)fs["Clustering"]["rightMaxClusterThreshold"];
    config.rightPointsDistance      = (float)fs["Clustering"]["rightPointsDistance"];

    // 读取霍夫变换
    config.hough_width = (int)fs["HoughLinesP"]["width"];
    config.hough_height = (int)fs["HoughLinesP"]["height"];
    config.hough_scale = (float)fs["HoughLinesP"]["scale"];
    config.hough_offsetX = (float)fs["HoughLinesP"]["x"];
    config.hough_left_offsetY = (float)fs["HoughLinesP"]["left_y"];
    config.hough_right_offsetY = (float)fs["HoughLinesP"]["right_y"];
    config.hough_rho = (float)fs["HoughLinesP"]["rho"];
    config.hough_theta = (float)fs["HoughLinesP"]["theta"];
    config.hough_threshold = (float)fs["HoughLinesP"]["threshold"];
    config.hough_minLineLength = (float)fs["HoughLinesP"]["minLineLength"];
    config.hough_maxLineGap = (float)fs["HoughLinesP"]["maxLineGap"];

    // 读取曲线拟合
    config.distinguishRoadSideThreshold = (float)fs["CurveFitting"]["distinguishRoadSideThreshold"];

    // 读取RANSAC
    config.interations  = (int)fs["RANSAC"]["interations"];
    config.simga        = (float)fs["RANSAC"]["sigma"];
    config.kMin         = (float)fs["RANSAC"]["kMin"];
    config.kMax         = (float)fs["RANSAC"]["kMax"];

    // 读取滑窗滤波
    config.slideWindowCount = (int)fs["SlideWindow"]["slideWindowCount"];

    // 读取模式
    config.onlineModel      = (int)fs["Model"]["onlineModel"];
    config.pcapRunningModel = (int)fs["Model"]["pcapRunningModel"];
    config.pcdRunningModel  = (int)fs["Model"]["pcdRunningModel"];
    config.pcdPath          = (std::string)fs["Model"]["pcdPath"];

    // 读取Viewer
    config.projectionModel  = (int)fs["Viewer"]["projectionModel"];
    config.savePictureModel = (int)fs["Viewer"]["savePictureModel"];
    config.viewer_width     = (int)fs["Viewer"]["width"];
    config.viewer_height    = (int)fs["Viewer"]["height"];
    config.viewer_scale     = (float)fs["Viewer"]["scale"];
    config.viewer_offsetX   = (float)fs["Viewer"]["x"];
    config.viewer_offsetY   = (float)fs["Viewer"]["y"];

    // 读取pcd保存
    config.savePcd      = (int)fs["Pcd"]["savePcd"];

    fs.release(); // 释放文件
    return true;
}