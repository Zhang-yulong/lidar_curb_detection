#include "LidarCurbDectection.h" 
#include <memory>

namespace Lidar_Curb_Dedection
{


// pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Euclidean Clustering"));
// extern pcl::visualization::PCLVisualizer::Ptr CurveViewer;

static BYTE byAlgNum = 0;

struct CloudData 
    {
        PointCloud2Intensity::Ptr cloud;
        double avg_x;
        size_t point_count;
    };


// 选出左右两侧的最大簇
auto getLargestCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters) {
    return *std::max_element(clusters.begin(), clusters.end(),
                                [](const PointCloud2Intensity::Ptr& a, const PointCloud2Intensity::Ptr& b) {
                                    return a->size() < b->size();
                                });
};

auto getCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters){
    return clusters.front();
};


// auto getCloserRightCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters){

//     PointCloud2Intensity::Ptr min_cloud(new PointCloud2Intensity());
//     double xMin = 1000; 

//     for(const auto &cloud : clusters){

//         double sum_x = 0.0;
//         for(const auto &point : cloud->points){
//             sum_x += point.x;
//         }
//         double avg_x = sum_x/ cloud->points.size();
        
//         if (xMin > avg_x) {
//             xMin = avg_x;
//             min_cloud = cloud;
//         }
//     }
//     return min_cloud;
// };


// auto getCloserLeftCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters){

//     PointCloud2Intensity::Ptr max_cloud(new PointCloud2Intensity());
//     double xMax = -1000; 

//     for(const auto &cloud : clusters){

//         double sum_x = 0.0;
//         for(const auto &point : cloud->points){
//             sum_x += point.x;
//         }
//         double avg_x = sum_x/ cloud->points.size();
        
//         if (xMax < avg_x) {
//             xMax = avg_x;
//             max_cloud = cloud;
//         }
//     }
//     return max_cloud;
// };

PointCloud2Intensity::Ptr LidarCurbDectection::getCloserRightCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters){
    
    struct CloudData1 
    {
        PointCloud2Intensity::Ptr cloud;
        double avg_x;
        size_t point_count;
    };


    std::vector<CloudData1> cloud_data;

    for(const auto &cloud : clusters){

        double sum_x = 0.0;
        double yMax = -1000;
        double yMin = 1000;
        for(const auto &point : cloud->points){
            sum_x += point.x;
            // if(point.y > yMax)
            //     yMax = point.y;
            
            // if(point.y < yMin)
            //     yMin = point.y;
        }
        
        double avg_x = sum_x / cloud->points.size();
        // double length_y = std::abs(yMax-yMin);

        // if(length_y > 0.5){ //直线长度
        
        // }

        cloud_data.push_back({cloud, avg_x, cloud->points.size() });
    }

    if (cloud_data.size() < 2) {
        std::cerr << "Error: Not enough valid point clouds!" << std::endl;
        return nullptr;
    }

    
    /*正常 */
    // 按平均x值升序排序
    std::sort(cloud_data.begin(), cloud_data.end(), [](const CloudData1& a, const CloudData1& b) {
        return a.avg_x < b.avg_x;
    });
    // 最小平均x值和第二小平均x值的点云
    const auto& first = cloud_data[0];
    const auto& second = cloud_data[1];

    // 判断数量条件
    if (first.point_count >= second.point_count * 0.25) {
        return first.cloud;
    } else {
        return second.cloud;
    }
   

}

PointCloud2Intensity::Ptr LidarCurbDectection::getCloserLeftCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters){
    
    struct CloudData2 
    {
        PointCloud2Intensity::Ptr cloud;
        double avg_x;
        size_t point_count;
    };

    std::vector<CloudData2> cloud_data;

    for(const auto &cloud : clusters){

        double sum_x = 0.0;
        for(const auto &point : cloud->points){
            sum_x += point.x;

            // if(point.y > yMax)
            //     yMax = point.y;
            
            // if(point.y < yMin)
            //     yMin = point.y;
        }

        double avg_x = sum_x / cloud->points.size();
        // double length_y = std::abs(yMax-yMin);

        cloud_data.push_back({cloud, avg_x, cloud->points.size()});
    }
    
    if (cloud_data.size() < 2) {
        std::cerr << "Error: Not enough valid point clouds!" << std::endl;
        return nullptr;
    }

    /*正常*/
    // 按平均x值降序排序
    std::sort(cloud_data.begin(), cloud_data.end(), [](const CloudData2& a, const CloudData2& b) {
        return a.avg_x > b.avg_x;
    });

    // 最大平均x值和第二大平均x值的点云
    const auto& first = cloud_data[0];
    const auto& second = cloud_data[1];

    // 判断数量条件
    if (first.point_count >= second.point_count * 0.25) {
        return first.cloud;
    } else {
        return second.cloud;
    }
    
}



/*使用配置文件调参的构造函数*/
LidarCurbDectection::LidarCurbDectection(const STR_CONFIG & config)
    : m_strLCDConfig(config)
{
    m_fUpperBound = m_strLCDConfig.verticalUpperAngle;
    m_fLowerBound = m_strLCDConfig.verticalLowerAngle;
    m_fGroudSegmentationThreshold = m_strLCDConfig.groudSegmentationThreshold;
    m_iScanRings = m_strLCDConfig.scanRings;
    m_fFactor = (m_fUpperBound - m_fLowerBound)/ (m_iScanRings - 1);


    // int viewer_width = m_strLCDConfig.viewer_width;
    // int viewer_height = m_strLCDConfig.viewer_height;
    // float viewer_scale = m_strLCDConfig.viewer_scale;
    // cv::Point2d viewer_offset(m_strLCDConfig.viewer_offsetX, m_strLCDConfig.viewer_offsetY);

    if(m_strLCDConfig.openGroudViewer){
        ground_viewer = std::make_shared<pcl::visualization::PCLVisualizer> ("groud Viewer");
        no_ground_viewer = std::make_shared<pcl::visualization::PCLVisualizer> ("no groud Viewer");
    }

    if(m_strLCDConfig.openClusterViewer){
        cluster_viewer = std::make_shared<pcl::visualization::PCLVisualizer> ("cluster Viewer");
    }

    m_pCurveFitting = new CurveFitting(m_strLCDConfig);
    // m_pViewer = new Viewer(viewer_width, viewer_height, viewer_scale, viewer_offset); 
    m_pViewer = new Viewer(m_strLCDConfig);

    m_pAll_deal_cluster_cloud = boost::make_shared<PointCloud2RGB>();
    m_pAll_output_curve_cloud = boost::make_shared<PointCloud2RGB>();

    m_iSystemRunCount = 0;


    R_ToLeftProject <<  0, -m_strLCDConfig.hough_scale, m_strLCDConfig.hough_offsetX,
                        m_strLCDConfig.hough_scale, 0, m_strLCDConfig.hough_left_offsetY,
                        0, 0, 1;

    R_ToRightProject << 0, -m_strLCDConfig.hough_scale, m_strLCDConfig.hough_offsetX,
                        m_strLCDConfig.hough_scale, 0, m_strLCDConfig.hough_right_offsetY,
                        0, 0, 1;

    R_Inv_ToLeftProject = R_ToLeftProject.inverse();
    R_Inv_ToRightProject = R_ToRightProject.inverse();

    std::cout<<"R_Inv_ToLeftProject = "<<"\n"<< R_Inv_ToLeftProject <<std::endl;
    std::cout<<"R_Inv_ToRightProject = "<<"\n"<< R_Inv_ToRightProject <<std::endl;
}




LidarCurbDectection::~LidarCurbDectection()
{
    delete(m_pViewer);
    delete(m_pCurveFitting);
}




//用 const 修饰返回的指针或引用，保护指针或引用的内容不被修改
const int &LidarCurbDectection::GetValueOfScanRings(){
    return m_iScanRings;
}



void LidarCurbDectection::CalculateScanID(PointCloud2Intensity::Ptr pInCloud, PointCloud2Intensity::Ptr &pOutCloud){
    
    size_t cloudSize_1 = pInCloud->points.size();
    int iScanID = 0;
    std::vector<PointCloud2Intensity> vLidarCloudScan(GetValueOfScanRings());
    
    for(int i = 0; i < cloudSize_1; i++){
        //垂直夹角
        float fRadians = std::atan(pInCloud->points[i].z / std::sqrt(pInCloud->points[i].x * pInCloud->points[i].x
                                                            + pInCloud->points[i].y * pInCloud->points[i].y));

        //确定是哪条线
        int iId = int(  ((fRadians * 180 / M_PI) - m_fLowerBound) * m_fFactor + 0.5  );

        if(iId >= GetValueOfScanRings() || iId < 0)
            continue;

        vLidarCloudScan[iId].push_back(pInCloud->points[i]);
    }

    for(int j = 0; j < GetValueOfScanRings(); j++ ){
        
    }

}


void LidarCurbDectection::ExtractPointsByHeightDifference(PointCloud2Intensity::Ptr &pInCloud)
{
    float fHeightLower = m_strLCDConfig.heightLower;
    float fHeightUpper = m_strLCDConfig.heightUpper;

    // std::cout<<"进高度差函数的点数: "<<pInCloud->points.size()<<std::endl;

    PointCloud2Intensity::Ptr temp(new PointCloud2Intensity);
    int count = 0;
    for(int i = 0; i < pInCloud->points.size(); i++ ){
        pcl::PointXYZI tempPoint;
        if( (pInCloud->points[i].z < fHeightUpper && pInCloud->points[i].z > fHeightLower)  ){    //（-0.2，-0.30） (-0.75, 0.81)
            
            tempPoint.x = pInCloud->points[i].x;
            tempPoint.y = pInCloud->points[i].y;
            tempPoint.z = pInCloud->points[i].z;
            tempPoint.intensity = pInCloud->points[i].intensity;
            count++ ;
            temp->points.push_back(tempPoint);
            
        }
    }

    *pInCloud = *temp;

    LOG_RAW("高度差筛选后，剩下的点： %d\n",temp->points.size());

}


//SAC算法
void LidarCurbDectection::ExtractGroundFromSAC(PointCloud2Intensity::Ptr &pIncloud,
                                        PointCloud2Intensity::Ptr &pOutcloud,
                                        pcl::PointIndices::Ptr pIndices,
                                        bool bSetNeg){

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(bSetNeg);   //设置提取内点
    extract.setIndices(pIndices);   //设置分割后的内点为需要提取的点集
    extract.setInputCloud(pIncloud);
    extract.filter(*pOutcloud);  //开始分割
}


//PMF算法
void LidarCurbDectection::ExtractGroundFromPMF(PointCloud2Intensity::Ptr &pIncloud,
                                        PointCloud2Intensity::Ptr &pOutcloud,
                                        pcl::PointIndicesPtr pIndices,
                                        bool bSetNeg){

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(bSetNeg);   //设置提取内点
    extract.setIndices(pIndices);   //设置分割后的内点为需要提取的点集
    extract.setInputCloud(pIncloud);
    extract.filter(*pOutcloud);  //开始分割
}


//使用SACSegmentation方法分割地面
void LidarCurbDectection::GroudFilterFromSAC(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints){
    
    for(int i = 0; i < m_vCloudPtrList.size(); i++)
    {
        PointCloud2Intensity::Ptr pGround_i(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr pGround_no_i(new PointCloud2Intensity);
        pcl::ModelCoefficients::Ptr pCoefficients(new pcl::ModelCoefficients);      //分割时所需的模型系数对象
        pcl::PointIndices::Ptr pInliers(new pcl::PointIndices);                     //存储内点的点索引集合对象
    
        pcl::SACSegmentation<pcl::PointXYZI> segmentation;
        segmentation.setInputCloud(m_vCloudPtrList[i]);
        segmentation.setOptimizeCoefficients(true);                                 //设置对估计的模型参数进行优化处理
        segmentation.setModelType(pcl::SACMODEL_PLANE);                             //模型类型
        segmentation.setMethodType(pcl::SAC_RANSAC);                                //所用的随机参数估计方法
        segmentation.setDistanceThreshold(m_fGroudSegmentationThreshold);           //距离阈值
        segmentation.setMaxIterations(800);
        segmentation.segment(*pInliers, *pCoefficients);
        
        // std::cout<<"第 "<<i<<" ,提取的: 内点(pInliers)数量："<<pInliers->indices.size()<<std::endl;

        ExtractGroundFromSAC(m_vCloudPtrList[i], pGround_i, pInliers, false);
        ExtractGroundFromSAC(m_vCloudPtrList[i], pGround_no_i, pInliers, true);
        
        *pGroundPoints += *pGround_i;  // pcl重载了 += 运算符
        *pNoGroundPoints += *pGround_no_i;

    
    }
}

//使用渐进式形态学滤波分割地面
void LidarCurbDectection::GroudFilterFromAPMF(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints){
    
    int iMaxWindowSize = m_strLCDConfig.maxWindowSize;
    float fSlope = m_strLCDConfig.slope;
    float fInitialDistance = m_strLCDConfig.initialDistance;
    float fMaxDistance = m_strLCDConfig.maxDistance;
    int iCellSize = m_strLCDConfig.cellSize;
    int iBase = m_strLCDConfig.base;


    for(int i = 0; i < m_vCloudPtrList.size(); i++)
    {   
        PointCloud2Intensity::Ptr pGround_i(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr pGround_no_i(new PointCloud2Intensity);
        
        pcl::PointIndicesPtr pInliers(new pcl::PointIndices);             // 存储地面点索引

        // 渐进式形态学滤波
        pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZI> segmentation;
        segmentation.setInputCloud(m_vCloudPtrList[i]);						// 待处理点云
        segmentation.setMaxWindowSize(iMaxWindowSize);						// 最大窗口大小
        segmentation.setSlope(fSlope);										// 地形坡度参数
        segmentation.setInitialDistance(fInitialDistance);					// 初始高差阈值
        segmentation.setMaxDistance(fMaxDistance);							// 最大高差阈值
        segmentation.setCellSize(iCellSize);								// 设置窗口的大小
        segmentation.setBase(iBase);										// 设置计算渐进窗口大小时使用的基数
        segmentation.setExponential(true);									// 设置是否以指数方式增加窗口大小
        segmentation.extract(pInliers->indices);
    
        ExtractGroundFromPMF(m_vCloudPtrList[i], pGround_i, pInliers, false);
        ExtractGroundFromPMF(m_vCloudPtrList[i], pGround_no_i, pInliers, true);

        *pGroundPoints += *pGround_i;  
        *pNoGroundPoints += *pGround_no_i;
    
    }

    if(m_strLCDConfig.pcdRunningModel){

        pGroundPoints->height = pGroundPoints->points.size();
        pGroundPoints->width = 1;
        pGroundPoints->is_dense = false;
        char pchFileName_1[128];
        bzero(pchFileName_1, sizeof (pchFileName_1));
        sprintf(pchFileName_1, "/home/zyl/echiev_lidar_curb_detection/log/pcdRunningModel/Ground.pcd");
        pcl::io::savePCDFileASCII (pchFileName_1, *pGroundPoints);

        pNoGroundPoints->height = pNoGroundPoints->points.size();
        pNoGroundPoints->width = 1;
        pNoGroundPoints->is_dense = false;
        char pchFileName_2[128];
        bzero(pchFileName_2, sizeof (pchFileName_2));
        sprintf(pchFileName_2, "/home/zyl/echiev_lidar_curb_detection/log/pcdRunningModel/noGround.pcd");
        pcl::io::savePCDFileASCII (pchFileName_2, *pNoGroundPoints);
    }

}


//调用pcl里面的聚类方法, 输出 Indices
bool LidarCurbDectection::IsClustering(const float PointsDistance, const float MaximumClusterThreshold, const float MimimumClusterThreshold, PointCloud2Intensity::Ptr pInputCloud, std::vector<pcl::PointIndices> &vClusterIndices){

    // 创建一个KD树
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(pInputCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;   //欧式聚类

    ec.setClusterTolerance(PointsDistance);    // 邻域距离阈值，决定了簇之间的最小距离
    ec.setMinClusterSize(MimimumClusterThreshold);       // 簇的最小点数
    ec.setMaxClusterSize(MaximumClusterThreshold);      // 簇的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(pInputCloud);
    ec.extract(vClusterIndices);

    if(vClusterIndices.empty()){
        std::cout<<"  聚类完簇数: "<<vClusterIndices.size()<<" ，空 "<<std::endl;
        return false;
    }
    else{
        std::cout<<"  聚类完簇数: "<<vClusterIndices.size()<<" ，不为空 "<<std::endl;
        return true;
    }
    
}


//调用pcl里面的聚类方法, 输出 ptr
bool LidarCurbDectection::IsClustering(const float PointsDistance, const float MaximumClusterThreshold, const float MimimumClusterThreshold, PointCloud2Intensity::Ptr pInputCloud, std::vector<PointCloud2Intensity::Ptr> &vClusterPtr){

    // //把3d点压缩成2d点
    // PointCloud2Intensity::Ptr pDeal_InputCloud(new PointCloud2Intensity);
    // for(const auto& point : pInputCloud->points ){
    //     pcl::PointXYZI temp;
    //     temp.x = point.x; 
    //     temp.y = point.y;
    //     temp.z = 0; 
    //     temp.intensity = point.intensity;
    //     pDeal_InputCloud->points.push_back(temp);
    // }
    
    
    std::vector<pcl::PointIndices> vClusterIndices;

    // 创建一个KD树
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(pInputCloud);
    // tree->setInputCloud(pDeal_InputCloud);
    

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;     // 欧式聚类
    ec.setClusterTolerance(PointsDistance);                 // 邻域距离阈值，决定了簇之间的最小距离
    ec.setMinClusterSize(MimimumClusterThreshold);          // 簇的最小点数
    ec.setMaxClusterSize(MaximumClusterThreshold);          // 簇的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(pInputCloud);
    // ec.setInputCloud(pDeal_InputCloud);
    ec.extract(vClusterIndices);

    for (const auto& indices : vClusterIndices) {
        PointCloud2Intensity::Ptr cluster(new PointCloud2Intensity());
        for (int idx : indices.indices) {
            cluster->push_back((*pInputCloud)[idx]);
            // cluster->push_back((*pDeal_InputCloud)[idx]);
        }
        vClusterPtr.push_back(cluster);
    
    }
    
    if(vClusterPtr.empty()){
        std::cout<<"  聚类完簇数: "<<vClusterPtr.size()<<" ，空 "<<std::endl;
        return false;
    }
    else{
        std::cout<<"  聚类完簇数: "<<vClusterPtr.size()<<" ，不为空 "<<std::endl;
        for(int i = 0; i<vClusterPtr.size();i++){
            for(int j = 0; j < vClusterPtr[i]->points.size(); j++){

                pcl::PointXYZRGB colored_point;
                colored_point.x = vClusterPtr[i]->points[j].x;
                colored_point.y = vClusterPtr[i]->points[j].y;
                colored_point.z = vClusterPtr[i]->points[j].z;
                colored_point.r = 0;
                colored_point.g = 255;  //绿色
                colored_point.b = 0;
                
                m_pAll_deal_cluster_cloud->points.push_back(colored_point);
            }    
        }
        
        return true;
    }
    
}

double LidarCurbDectection::euclideanDistance(const cv::Point& p1, const cv::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 计算两个向量的夹角（弧度）
double LidarCurbDectection::calculateAngle(const cv::Vec4f& line1, const cv::Vec4f& line2) {
    double dotProduct = line1[0] * line2[0] + line1[1] * line2[1];
    double magnitude1 = std::sqrt(line1[0] * line1[0] + line1[1] * line1[1]);
    double magnitude2 = std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);
    return std::acos(dotProduct / (magnitude1 * magnitude2));
}

// 计算两条向量的夹角是否小于阈值
bool LidarCurbDectection::isDirectionSimilar(const cv::Vec4f& line1, const cv::Vec4f& line2, double angleThreshold) {
    cv::Vec2f dir1(line1[0], line1[1]); // line1 的方向向量
    cv::Vec2f dir2(line2[0], line2[1]); // line2 的方向向量

    double dotProduct = dir1.dot(dir2); //反映了两个向量在方向上的相似度，结果越大越相似
    double magnitude1 = cv::norm(dir1);
    double magnitude2 = cv::norm(dir2);
    double angle = std::acos(dotProduct / (magnitude1 * magnitude2)) * 180.0 / CV_PI; // 转为角度

    return angle < angleThreshold;
}

// 计算两个区域质心间的水平距离
double LidarCurbDectection::computeXDistance(const cv::Point2f& center1, const cv::Point2f& center2) {
    return std::abs(center1.x - center2.x);
}

// 计算两区域质心间的垂直距离
double LidarCurbDectection::computeYDistance(const cv::Point2f& center1, const cv::Point2f& center2) {
    return std::abs(center1.y - center2.y);
}

// 判断两个区域的包围盒是否有重叠
bool LidarCurbDectection::isBoundingBoxOverlapping(const cv::Rect& bbox1, const cv::Rect& bbox2) {
    return (bbox1 & bbox2).area() > 0; // 交集面积是否大于0
}

double LidarCurbDectection::pointToLineDistance(const cv::Point& pt, const cv::Vec4f& line){
    // 计算点到直线的距离
    float vx = line[0], vy = line[1];
    float x0 = line[2], y0 = line[3];
    return std::abs(vx * (pt.y - y0) - vy * (pt.x - x0)) / std::sqrt(vx * vx + vy * vy);

}

// 检查直线斜率是否满足角度限制
bool LidarCurbDectection::isLineAngleValid(const cv::Vec4f& line, double maxAngle){
    
    float vx = line[0], vy = line[1];
    double angle = std::atan2(std::abs(vy), std::abs(vx)) * 180.0 / CV_PI; // 角度转换为绝对值
    return angle > maxAngle;
}

// 使用 RANSAC 拟合直线，限制斜率角度
void LidarCurbDectection::ransacFitLine(const std::vector<cv::Point>& points, cv::Vec4f& bestLine, std::vector<cv::Point>& inliers, 
                   int maxIterations , double threshold , double maxAngle , double inlierRatio ) {
    int bestInlierCount = 0;
    cv::RNG rng(cv::getTickCount()); // 随机数生成器

    for (int i = 0; i < maxIterations; ++i) {
        // 随机选择两个点
        int idx1 = rng.uniform(0, static_cast<int>(points.size()));
        int idx2 = rng.uniform(0, static_cast<int>(points.size()));

        if (idx1 == idx2) continue;

        // 构造直线模型
        cv::Point p1 = points[idx1];
        cv::Point p2 = points[idx2];
        cv::Vec4f candidateLine;
        float vx = p2.x - p1.x;
        float vy = p2.y - p1.y;
        float norm = std::sqrt(vx * vx + vy * vy);
        candidateLine = cv::Vec4f(vx / norm, vy / norm, p1.x, p1.y);

        // 检查斜率是否满足角度限制
        if (!isLineAngleValid(candidateLine, maxAngle)) {
            continue;
        }

        // 统计内点
        std::vector<cv::Point> currentInliers;
        for (const auto& pt : points) {
            if (pointToLineDistance(pt, candidateLine) < threshold) {
                currentInliers.push_back(pt);
            }
        }

        // 如果当前内点数更优，则更新最佳模型
        if (currentInliers.size() > bestInlierCount) {
            bestLine = candidateLine;
            inliers = currentInliers;
            bestInlierCount = static_cast<int>(currentInliers.size());

            // 提前终止条件：如果内点比例足够高
            if (static_cast<double>(bestInlierCount) / points.size() > inlierRatio) {
                break;
            }
        }
    }
    // std::cout<<"结束"<<std::endl;
}


void LidarCurbDectection::normalizeLineABC(double &A, double &B, double &C){
    
    //计算归一化系数
    double norm = std::sqrt(A*A + B*B);

    if(norm < 1e-6) {
        std::cerr<<"Line normalization failed"<<std::endl;
        return;
    }

    A /= norm;
    B /= norm;
    C /= norm; 
}



//正常findOnlyFitCluster
#if 1
// void LidarCurbDectection::findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, const int sideFlag){
void LidarCurbDectection::findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image){ 
    
    std::vector<std::vector<cv::Point>> vWithProcessedContours;
    std::vector<LineInfo> vLines;  //存储检测到直线,里面是起始点和终点

    // 计算图像边界上的两个点
    int width = image.cols, height = image.rows;

    for(int i = 0; i < InitContours.size(); i++){

        if(InitContours[i].size() <= 25 ){
            // std::cout<<" 剔除轮廓区域点数少与25, 点数： "<<InitContours[i].size()<<std::endl;
            
            cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(255,0,0), -1); //应该是蓝色
           
            continue;
        }
            
        // 拟合直线
        cv::Vec4f lineParams;
        cv::fitLine(InitContours[i], lineParams, CV_DIST_L2, 0, 0.01, 0.01); //该函数会把轮廓区域内所有的点

        // 提取拟合直线的方向向量 (vx, vy)，直线上一点(x0, y0)
        float vx = lineParams[0];
        float vy = lineParams[1];
        float x0 = lineParams[2];
        float y0 = lineParams[3];

        // 假设延展到图像左、右边界
        cv::Point pt1, pt2;
        pt1.x = 0;
        pt1.y = cvRound(y0 - (x0 * vy / vx));
        pt2.x = width - 1;
        pt2.y = cvRound(y0 + ((width - 1 - x0) * vy / vx));
        cv::line(image, pt1, pt2, cv::Scalar(0, 255, 255), 2); //黄色

        // 计算方向向量与 x 轴的夹角
        double angle = std::atan2(vy, vx) * 180.0 / CV_PI; // 转为角度
        angle = std::abs(angle); // 只关心绝对值 [-180,180]

        //剔除与水平夹角小于60度的线
        if(angle < 40){
            // std::cout <<" 该簇估计方向小于60度 , 实际角度： "<<angle<<std::endl;
            
            cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(0,0,255), -1); //应该是红色
            
            continue;
        }
        
        // 找到点集的范围
        int minX = std::numeric_limits<int>::max();
        int maxX = std::numeric_limits<int>::lowest();
        int minY = std::numeric_limits<int>::max();
        int maxY = std::numeric_limits<int>::lowest();
        float length = 0.0;
        for(int j = 0; j < InitContours[i].size(); j++ ){
            minX = std::min(minX, InitContours[i][j].x);
            maxX = std::max(maxX, InitContours[i][j].x);
            minY = std::min(minY, InitContours[i][j].y);
            maxY = std::max(maxY, InitContours[i][j].y);
        }
        // std::cout<<"(xmin,xmax): "<<minX<<" , "<<maxX<<" , (ymin,ymax): "<<minY<<" , "<<maxY<<std::endl;
        // 判断直线的方向
        // const float epsilon = 1e-6;
        // if (std::fabs(vx) < epsilon) {
        //     // 接近垂直的直线，用 y 的范围计算长度
        //     length =  maxY - minY;
        // } 
        // else {
        //     // 一般情况，用 x 的范围延展计算端点
        //     float y1 = y0 + (minX - x0) * (vy / vx);
        //     float y2 = y0 + (maxX - x0) * (vy / vx);
        //     length =  std::sqrt((maxX - minX) * (maxX - minX) + (y2 - y1) * (y2 - y1));
        // }


        length =  maxY - minY;
        //剔除直线
        // if(length <= 75) {//单位：像素，应该与缩放大小有关 1米5以下就踢掉
        //     // std::cout<<"轮廓区域长度: "<<length<<std::endl;
        //     continue;
        // }
        cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(0,255,0), -1); //应该是绿色
       
        LOG_RAW("轮廓点数: %d, 轮廓方向: %f, 轮廓长度:%f\n", InitContours[i].size(), angle, length);

        // std::vector<float> projections;
        // for (const auto& point : contour) {
        //     float t = vx * (point.x - x0) + vy * (point.y - y0);  //向量投影公式
        //     projections.push_back(t);
        // }

        // // 找到投影范围
        // float tMin = *std::min_element(projections.begin(), projections.end());
        // float tMax = *std::max_element(projections.begin(), projections.end());

        // // 实际长度
        // float lineLength = tMax - tMin;

        // if(lineLength < 3){
        //     std::cout <<" 该簇投影距离小于3 " <<std::endl;
        //     continue;
        // }

        // // 绘制拟合直线
        // cv::Point point1 = cv::Point(x0 + vx * tMin, y0 + vy * tMin);
        // cv::Point point2 = cv::Point(x0 + vx * tMax, y0 + vy * tMax);
        // cv::line(resultImage, point1, point2, cv::Scalar(0, 255, 0), 2);
        
        // vLines.push_back({lineParams,lineLength});
        vWithProcessedContours.push_back(InitContours[i]);
    }

    // if(sideFlag == 0){
    //     cv::Mat mask = cv::Mat::zeros(temp_image.size(), CV_8UC1);
    //     mergedContours.push_back(contours[i]);
    //     // // 在掩码上绘制当前轮廓
    //     cv::drawContours(mask, vWithProcessedContours, static_cast<int>(i), cv::Scalar(255), -1);

    //     // 提取轮廓内mergedContours的非零点
    //     std::vector<cv::Point> points;
    //     cv::findNonZero(mask, points);

    // }
    // else{

    // }

    int n = vWithProcessedContours.size();
    InitContours.resize(n);
    InitContours = vWithProcessedContours;
}
#endif

cv::Point2f LidarCurbDectection::computeCentroid(const std::vector<cv::Point>& contour) {
    cv::Point2f centroid(0, 0);
    for (const auto& pt : contour) {
        centroid.x += pt.x;
        centroid.y += pt.y;
    }
    centroid.x /= contour.size();
    centroid.y /= contour.size();
    return centroid;
}


void LidarCurbDectection::splitContoursByCentroid(const std::vector<cv::Point>& contour, 
                             std::vector<cv::Point>& finalContour, 
                             int iSideFlag,
                             cv::Mat &image) 
{
    // 计算轮廓的重心
    cv::Point2f centroid = computeCentroid(contour);
    cv::circle(image, centroid, 2, cv::Scalar(0,0,255), -1);//应该是红色

    std::vector<cv::Point> leftContour, rightContour;
    
    for (const auto& pt : contour) {
        if (pt.x < centroid.x) {
            leftContour.push_back(pt);  // 左轮廓
        } 
        else {
            rightContour.push_back(pt); // 右轮廓
        }
    }

    if(iSideFlag == 0) {//右
        finalContour.resize(leftContour.size()); 
        finalContour = leftContour;
    }
    else{
        finalContour.resize(rightContour.size()); 
        finalContour = rightContour;
    }

}

//修改，效果差
#if 0
void LidarCurbDectection::findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image, int iSideFlag){ 
    
    std::vector<std::vector<cv::Point>> vWithProcessedContours;
    std::vector<LineInfo> vLines;  //存储检测到直线,里面是起始点和终点

    // 计算图像边界上的两个点
    int width = image.cols, height = image.rows;

    for(int i = 0; i < InitContours.size(); i++){

        if(InitContours[i].size() < 25 ){
            std::cout<<" 剔除区域轮廓点数少于25, 点数： "<<InitContours[i].size()<<std::endl;
            
            cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(255,0,0), -1); //应该是蓝色
           
            continue;
        }
        
        std::vector<cv::Point> dealContour;
        splitContoursByCentroid(InitContours[i],dealContour,iSideFlag, image);

        
            
        //加入Ransac
        // 使用 RANSAC 拟合直线
        cv::Vec4f bestLine;
        std::vector<cv::Point> inliers;
        ransacFitLine(dealContour, bestLine, inliers, 500, 1.0, 80.0, 0.8 );
        float vx = bestLine[0];
        float vy = bestLine[1];
        float x0 = bestLine[2];
        float y0 = bestLine[3];


        // // 拟合直线
        // cv::Vec4f lineParams;
        // cv::fitLine(InitContours[i], lineParams, cv::DIST_L2, 0, 0.01, 0.01); //该函数会把轮廓区域内所有的点计算进去，如果是 “L”字型会把短的边也算进来

        // // 提取拟合直线的方向向量 (vx, vy)，直线上一点(x0, y0)
        // float vx = lineParams[0];
        // float vy = lineParams[1];
        // float x0 = lineParams[2];
        // float y0 = lineParams[3];

        // 假设延展到图像左、右边界
        cv::Point pt1, pt2;
        pt1.x = 0;
        pt1.y = cvRound(y0 - (x0 * vy / vx));
        pt2.x = width - 1;
        pt2.y = cvRound(y0 + ((width - 1 - x0) * vy / vx));
        cv::line(image, pt1, pt2, cv::Scalar(0, 255, 255), 2); //黄色

        // 计算方向向量与 x 轴的夹角
        double angle = std::atan2(vy, vx) * 180.0 / CV_PI; // 转为角度
        angle = std::abs(angle); // 只关心绝对值 [-180,180]

        //剔除与水平夹角小于80度的线
        if(angle < 80){
            std::cout <<" 该簇估计方向小于80度 , 角度： "<<angle<<std::endl;
            
            // cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(0,0,255), -1); //应该是红色
        
            // 显示内点
            for (const auto& pt : inliers) {
                cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1);//应该是红色
            }

            continue;
        }
        
        
        // cv::drawContours(image, InitContours, static_cast<int>(i), cv::Scalar(0,255,0), -1); //应该是绿色
        
        // 显示内点
        for (const auto& pt : inliers) {
            cv::circle(image, pt, 1, cv::Scalar(0,255,0), -1);//应该是绿色
        }
        
        // std::cout<<"点数: "<<InitContours[i].size()<<" , 方向: "<<angle<<std::endl;
        std::cout<<"处理完外轮廓点数: "<<inliers.size()<<" , 方向: "<<angle<<std::endl;

        // std::vector<float> projections;
        // for (const auto& point : contour) {
        //     float t = vx * (point.x - x0) + vy * (point.y - y0);  //向量投影公式
        //     projections.push_back(t);
        // }

        // // 找到投影范围
        // float tMin = *std::min_element(projections.begin(), projections.end());
        // float tMax = *std::max_element(projections.begin(), projections.end());

        // // 实际长度
        // float lineLength = tMax - tMin;

        // if(lineLength < 3){
        //     std::cout <<" 该簇投影距离小于3 " <<std::endl;
        //     continue;
        // }

        // // 绘制拟合直线
        // cv::Point point1 = cv::Point(x0 + vx * tMin, y0 + vy * tMin);
        // cv::Point point2 = cv::Point(x0 + vx * tMax, y0 + vy * tMax);
        // cv::line(resultImage, point1, point2, cv::Scalar(0, 255, 0), 2);
        
        // vLines.push_back({lineParams,lineLength});
        vWithProcessedContours.push_back(inliers);
    }

    // if(sideFlag == 0){
    //     cv::Mat mask = cv::Mat::zeros(temp_image.size(), CV_8UC1);
    //     mergedContours.push_back(contours[i]);
    //     // // 在掩码上绘制当前轮廓
    //     cv::drawContours(mask, vWithProcessedContours, static_cast<int>(i), cv::Scalar(255), -1);

    //     // 提取轮廓内mergedContours的非零点
    //     std::vector<cv::Point> points;
    //     cv::findNonZero(mask, points);

    // }
    // else{

    // }

    int n = vWithProcessedContours.size();
    InitContours.resize(n);
    InitContours = vWithProcessedContours;
}
#endif



double LidarCurbDectection::calculateSlope(const cv::Vec4i& line) {
    double dx = line[2] - line[0];
    double dy = line[3] - line[1];
    return (dx == 0) ? std::numeric_limits<double>::infinity() : dy / dx;
}

double LidarCurbDectection::calculateIntercept(const cv::Vec4i& line, double slope) {
    return line[1] - slope * line[0];
}

// 聚类直线段
std::vector<std::vector<HoughSPLineInfo>> LidarCurbDectection::clusterLines(const std::vector<HoughSPLineInfo>& lines, double slope_threshold, double distance_threshold) {
    std::vector<std::vector<HoughSPLineInfo>> clusters;

    for (const auto& line : lines) {
        bool added = false;
        for (auto& cluster : clusters) {
            if (std::abs(line.slope - cluster[0].slope) < slope_threshold) {
                cluster.push_back(line);
                added = true;
                break;
            }
        }
        if (!added) {
            clusters.push_back({line});
        }
    }

    return clusters;
}

// 拟合一条新的直线
cv::Vec4f LidarCurbDectection::fitLineToCluster(const std::vector<HoughSPLineInfo>& cluster) {
    std::vector<cv::Point2f> points;
    for (const auto& line : cluster) {
        points.emplace_back(line.line[0], line.line[1]);
        points.emplace_back(line.line[2], line.line[3]);

    }

    cv::Vec4f fitted_line;
    cv::fitLine(points, fitted_line, CV_DIST_L2, 0, 0.01, 0.01);
    return fitted_line;
}

// 聚类直线（仅考虑斜率）
std::vector<std::vector<Line>> LidarCurbDectection::clusterLinesBySlope(const std::vector<Line>& lines, float slope_threshold) {
    std::vector<std::vector<Line>> clusters;

    for (const auto& line : lines) {
        if(line.angle < 60){
        
            continue;
        }
            
        bool found_cluster = false;

        // 尝试加入现有簇
        for (auto& cluster : clusters) {
            
            // // std::cout<<"角度: "<<line.angle<<std::endl;
            // if (std::abs(cluster.front().angle - line.angle) < slope_threshold) {
            //     cluster.push_back(line);
            //     found_cluster = true;
            //     break;
            // }

            const Line& cluster_line = cluster.front();

            // 计算当前直线和簇内第一个直线的角度差异
            if (std::abs(cluster_line.angle - line.angle) < slope_threshold) {
                // 检查当前直线的中点横坐标与簇内第一个直线的中点横坐标的差异
                float cluster_x_midpoint = (cluster_line.start.x + cluster_line.end.x) / 2.0f;
                float line_x_midpoint = (line.start.x + line.end.x) / 2.0f;

                if (std::abs(cluster_x_midpoint - line_x_midpoint) < 10) {
                    // 如果符合条件，将当前直线加入到该簇中
                    cluster.push_back(line);
                    found_cluster = true;
                    break;
                }
            }

        }

        // 如果没有找到匹配的簇，创建一个新簇
        if (!found_cluster) {
            clusters.push_back({line});
        }
    }

    return clusters;
}

// 从聚类中找到最长的簇
std::vector<Line> LidarCurbDectection::findLongestCluster(const std::vector<std::vector<Line>>& clusters) {
    std::vector<Line> longest_cluster;
    float max_length = 0.0;

    for (const auto& cluster : clusters) {
        float cluster_length = std::accumulate(cluster.begin(), cluster.end(), 0.0f,
                                               [](float sum, const Line& line) { return sum + line.length; });
        if (cluster_length > max_length) {
            max_length = cluster_length;
            longest_cluster = cluster;
        }
    }

    return longest_cluster;
}

// 计算两点之间的欧几里得距离，即线段的长度
float LidarCurbDectection::calculateLength(const cv::Point& start, const cv::Point& end) {
    return std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
}

//修改
#if 1
// //调用聚类方法, 输出 ptr , 用来加入霍夫变换或直线分割
bool LidarCurbDectection::IsClustering_hough(PointCloud2Intensity::Ptr pInputCloud, std::vector<PointCloud2Intensity::Ptr> &vClusterPtr, unsigned long long ullTime, int iSideFlag)
{
    cv::Point2d offset;    
    if(iSideFlag == 0){ //右
        offset.x = m_strLCDConfig.hough_offsetX;
        offset.y = m_strLCDConfig.hough_right_offsetY;
    }
    else{               //左
        offset.x = m_strLCDConfig.hough_offsetX;
        offset.y = m_strLCDConfig.hough_left_offsetY;
    }


    int iWidth = m_strLCDConfig.hough_width;
    int iHeight = m_strLCDConfig.hough_height;
    double dScale = m_strLCDConfig.hough_scale; //默认是50个像素表示1米
    
    cv::Mat temp_image = cv::Mat::zeros(iHeight, iWidth, CV_8UC1); //单通道黑色
    cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);

    //把3d点压缩成2d点
    // 遍历原始点云
    for (const auto& pt : pInputCloud->points) {
       
        // 图像坐标 u 轴向右 为正（与点云x轴相同）（列），图像坐标 v 轴向下为正（与点云y轴相反）（行）。
        // // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
        int x = static_cast<int>(-pt.y * dScale + offset.x);
        int y = static_cast<int>(pt.x * dScale + offset.y);

        // 检查是否在图像边界内
        if (x >= 0 && x < iWidth && y >= 0 && y < iHeight){
            temp_image.at<uchar>(x,y) = 255;    //将点设置为白色
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

/**/ // // 进行膨胀操作连接断点  
    int dilationSize = 1; // 膨胀核大小
    //cv::MORPH_RECT 函数返回矩形卷积核（形状），MORPH_ELLIPSE（椭圆核）；Size：卷积核大小； Point：表示卷积核有x行，y列
    //3*3：最小有效核； 5*5更平滑的连接效果
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                cv::Point(dilationSize, dilationSize));
    cv::dilate(temp_image, temp_image, element);  //膨胀
    

    // 使用 HoughLinesP 检测直线段
    std::vector<cv::Vec4i> lines_p;
    cv::HoughLinesP(temp_image, lines_p, 1, CV_PI / 180, 50, 37, 20);

    std::vector<Line> lines;
    for (int i = 0; i < lines_p.size(); i++){
        cv::Point start(lines_p[i][0], lines_p[i][1]);
        cv::Point end(lines_p[i][2], lines_p[i][3]);

        // 计算当前直线段的长度
        float length = calculateLength(start, end);

        if(length >= m_strLCDConfig.hough_scale){
             cv::Scalar random_color(rand() % 256, rand() % 256, rand() % 256);
             cv::line(result, start, end, random_color, 3);
            lines.emplace_back(lines_p[i]);
        }    
    }

    // 聚类直线（仅考虑斜率）
    float slope_threshold = 2; // 斜率差异阈值
    auto clusters = clusterLinesBySlope(lines, slope_threshold);

    // 找到最长的簇
    std::vector<Line> longest_cluster = findLongestCluster(clusters);
    std::cout<<"最长的簇 : "<<longest_cluster.size()<<std::endl;
    std::vector<cv::Point2f> all_longest_cluster_points;
    if(!longest_cluster.empty()){
        
        for (const auto& line : longest_cluster) {
            cv::line(result, line.start, line.end, cv::Scalar(0, 255, 0), 2);
            all_longest_cluster_points.push_back(line.start);
            all_longest_cluster_points.push_back(line.end);
        }
    }

    
    
    
    if(!all_longest_cluster_points.empty()){
       
    
        
        if(iSideFlag == 0) {     //右
            PointCloud2Intensity::Ptr pReflectionLidar_right(new PointCloud2Intensity);
            for (int i = 0; i < all_longest_cluster_points.size(); i++ ) {
        

                pcl::PointXYZI point;
                pcl::PointXYZRGB colored_point;
                

                float x0 = R_Inv_ToRightProject(0,1) * all_longest_cluster_points[i].x + R_Inv_ToRightProject(0,2);
                float y0 = R_Inv_ToRightProject(1,0) * all_longest_cluster_points[i].y + R_Inv_ToRightProject(1,2);

                point.x = x0;
                point.y = y0;
                point.z = 1;
                point.intensity = 0;
                pReflectionLidar_right->points.push_back(point);
                

                colored_point.x = x0;
                colored_point.y = y0;
                colored_point.z = 1;
                colored_point.r = 0;
                colored_point.g = 255;  //绿色
                colored_point.b = 0;
                m_pAll_deal_cluster_cloud->points.push_back(colored_point);
            }
            vClusterPtr.push_back(pReflectionLidar_right);

            
        }
        else if (iSideFlag == 1){   //左
            PointCloud2Intensity::Ptr pReflectionLidar_left(new PointCloud2Intensity);
            for (int i = 0; i < all_longest_cluster_points.size(); i++ ) {
        
                pcl::PointXYZI point;
                pcl::PointXYZRGB colored_point;
            
                float x1 = R_Inv_ToLeftProject(0,1) * all_longest_cluster_points[i].x + R_Inv_ToLeftProject(0,2);
                float y1 = R_Inv_ToLeftProject(1,0) * all_longest_cluster_points[i].y + R_Inv_ToLeftProject(1,2);
        
                point.x = x1;
                point.y = y1;
                point.z = 1;
                point.intensity = 0;
                pReflectionLidar_left->points.push_back(point);
                
                colored_point.x = x1;
                colored_point.y = y1;
                colored_point.z = 1;
                colored_point.r = 0;
                colored_point.g = 255;  //绿色
                colored_point.b = 0;
                m_pAll_deal_cluster_cloud->points.push_back(colored_point);
            }
            vClusterPtr.push_back(pReflectionLidar_left);
        }


    }
    std::cout<<"聚类完点云数："<<m_pAll_deal_cluster_cloud->points.size()<<std::endl;
    

    

    // ////修改
    // if(iSideFlag == 0)  //右
    //     findOnlyFitCluster(contours, result, 0);
    // else
    //     findOnlyFitCluster(contours, result, 1);
    
    // std::vector<std::vector<cv::Point>> cv_clusters;// 存储簇的容器
    // std::cout<<"轮廓数： "<<contours.size()<<std::endl;
    

    // cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    // for (size_t i = 0; i < mergedContours.size(); i++) {
    //     cv::Scalar color(rand() % 256, rand() % 256, rand() % 256); // 随机颜色

    //     cv::drawContours(result, mergedContours, static_cast<int>(i), color, -1);
    // }


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> ulldif = (end-start) ;
    auto time = ulldif.count() * 1000;
    std::cout<<"耗时： "<<time <<" ms"<<std::endl;

    //图片下面加注释
    std::string white = "white: All contours";
    std::string blue = "blue: contour less than 25";
    std::string red = "red: direction less than 60 degrees";
    std::string green = "green: result";
    // line(result, Point(0, offset.x -10), Point(iWidth, offset.x -10), Scalar(255, 255, 255), 1);  // 白色线
    
    line(result, Point(0, offset.x ), Point(iWidth, offset.x ), Scalar(255, 255, 255), 1);  // 白色线
    line(result, Point(offset.y, 0), Point(offset.y, iHeight), Scalar(255, 255, 255), 1); // 白色线
    putText(result, "Y", Point(offset.y + 10,  20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    putText(result, "X", Point(iWidth - 50, offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);
    
    circle(result, Point(offset.y, offset.x), 5, Scalar(0, 0, 255), -1); // 绘制红色小圆点
    // 等距线间隔，单位为米
    double interval_x = 5.0;
    // 计算等距线在图像中的位置并绘制
    for (double y = -50; y <= 50; y += interval_x) {
        int x = static_cast<int>(-y * dScale + offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
        if (x >= 0 && x < iHeight) {  // 确保线在图像边界内
            // line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
            putText(result, std::to_string(static_cast<int>(y)) + "m", Point(offset.y + 10, x - 5), 
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
        }
    }

    // 等距线间隔，单位为米
    double interval_y= 1.0;
    // 计算等距线在图像中的位置并绘制
    for (double x = -50; x <= 50; x += interval_y) {
        int y = static_cast<int>(x * dScale + offset.y); // 计算等距线在图像中的位置
        if(x == 0)
            continue;
        else{
            if (y >= 0 && y < iWidth) {  // 确保线在图像边界内
                // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
                putText(result, std::to_string(static_cast<int>(x)), Point(y, offset.x + 15), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
        }   
    }



    putText(result, white, Point(0, offset.x + 35 ), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(result, blue, Point(0, offset.x + 55), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);

    putText(result, red, Point(0, offset.x + 75), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(result, green, Point(0, offset.x + 90), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    // putText(result, green, Point(offset.y-80, offset.x + 10), 
    //                 FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 1);

    std::string sFilePath;
    if(iSideFlag == 0){
        sFilePath = "/home/zyl/echiev_lidar_curb_detection/log/temp_right_side_image/" + std::to_string(ullTime) +"_"+std::to_string(iSideFlag) +".png";
    }
    else if(iSideFlag == 1){
        sFilePath = "/home/zyl/echiev_lidar_curb_detection/log/temp_left_side_image/" + std::to_string(ullTime) +"_"+std::to_string(iSideFlag) +".png";
    }
    
    cv::imwrite(sFilePath,result);
    //  cv::imwrite(sFilePath,temp_image);

    // return false; 
    


    
    if(vClusterPtr.empty()){
        std::cout<<" hough 聚类完簇数: "<<vClusterPtr.size()<<" ，空 "<<std::endl;
        return false;
    }
    else{
        std::cout<<" hough 聚类完簇数: "<<vClusterPtr.size()<<" ，不为空 "<<std::endl;
        return true;
    }

}  

#endif


//正常使用
#if 0
// //调用聚类方法, 输出 ptr , 用来加入霍夫变换或直线分割
bool LidarCurbDectection::IsClustering_hough(PointCloud2Intensity::Ptr pInputCloud, std::vector<PointCloud2Intensity::Ptr> &vClusterPtr, unsigned long long ullTime, int iSideFlag)
{
    cv::Point2d offset;    
    if(iSideFlag == 0){ //右
        offset.x = m_strLCDConfig.hough_offsetX;
        offset.y = m_strLCDConfig.hough_right_offsetY;
    }
    else{               //左
        offset.x = m_strLCDConfig.hough_offsetX;
        offset.y = m_strLCDConfig.hough_left_offsetY;
    }


    int iWidth = m_strLCDConfig.hough_width;
    int iHeight = m_strLCDConfig.hough_height;
    double dScale = m_strLCDConfig.hough_scale; //默认是50个像素表示1米
    
    cv::Mat temp_image = cv::Mat::zeros(iHeight, iWidth, CV_8UC1); //单通道黑色
    
    //把3d点压缩成2d点
    // 遍历原始点云
    for (const auto& pt : pInputCloud->points) {
       
        // 图像坐标 u 轴向右 为正（与点云x轴相同）（列），图像坐标 v 轴向下为正（与点云y轴相反）（行）。
        // // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
        int x = static_cast<int>(-pt.y * dScale + offset.x);
        int y = static_cast<int>(pt.x * dScale + offset.y);

        // 检查是否在图像边界内
        if (x >= 0 && x < iWidth && y >= 0 && y < iHeight){
            temp_image.at<uchar>(x,y) = 255;    //将点设置为白色
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

/**/    // // 进行膨胀操作连接断点  
    int dilationSize = 1; // 膨胀核大小
    //cv::MORPH_RECT 函数返回矩形卷积核（形状），MORPH_ELLIPSE（椭圆核），cv::MORPH_CROSS（十字核）；Size：卷积核大小； Point：表示卷积核有x行，y列
    //3*3：最小有效核； 5*5更平滑的连接效果
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                cv::Point(dilationSize, dilationSize));
                                                
    cv::dilate(temp_image, temp_image, element);  //膨胀
 

    // morphologyEx(temp_image, temp_image, MORPH_OPEN, element);//形态学开运算

/*单纯进行轮廓检测*/
    // 检查图像是否是二值图
    // cv::threshold(temp_image, temp_image, 127, 255, cv::THRESH_BINARY);  //大于第三个值的点重新设置为255
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> mergedContours;
    //使用 findContours 替代( connectedComponents : opencv 3以上)
    cv::findContours(temp_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //cv::RETR_EXTERNAL：只检测最外层轮廓
    
    cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(result, contours, static_cast<int>(i), cv::Scalar(255,255,255), 3);
    }

    findOnlyFitCluster(contours, result);
    
    std::vector<std::vector<cv::Point>> cv_clusters;// 存储簇的容器
    std::cout<<"最终合适的轮廓个数： "<<contours.size()<<std::endl;
    
//正常    
#if 1    
    for (int i = 0; i < contours.size(); i++) {
          
        
        // std::cout<<"   区域点数 "<<contours[i].size()<<std::endl;
        cv::Mat mask = cv::Mat::zeros(temp_image.size(), CV_8UC1);
        mergedContours.push_back(contours[i]);
        // // 在掩码上绘制当前轮廓
        cv::drawContours(mask, contours, static_cast<int>(i), cv::Scalar(255), -1);

        // 提取轮廓内mergedContours的非零点
        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);


        // 将提取的点存入簇容器
        cv_clusters.push_back(points);
        
        if(iSideFlag == 0 && !points.empty()) {             //右

            PointCloud2Intensity::Ptr pReflectionLidar_right(new PointCloud2Intensity);
            
            for(int j = 0; j < points.size(); j++) {

                pcl::PointXYZI point;
                pcl::PointXYZRGB colored_point;
                // float x0 = 0.02 * points[j].x - 2;      //以后计算完逆矩阵后需要把x和y对调
                // float y0 = -0.02 * points[j].y + 11;

                float x0 = R_Inv_ToRightProject(0,1) * points[j].x + R_Inv_ToRightProject(0,2);
                float y0 = R_Inv_ToRightProject(1,0) * points[j].y + R_Inv_ToRightProject(1,2);

                point.x = x0;
                point.y = y0;
                point.z = 1;
                point.intensity = 0;
                pReflectionLidar_right->points.push_back(point);
                

                colored_point.x = x0;
                colored_point.y = y0;
                colored_point.z = 1;
                colored_point.r = 0;
                colored_point.g = 255;  //绿色
                colored_point.b = 0;
                m_pAll_deal_cluster_cloud->points.push_back(colored_point);
            }
            vClusterPtr.push_back(pReflectionLidar_right);
        }
        else if(iSideFlag == 1 && !points.empty()) {        //左

            PointCloud2Intensity::Ptr pReflectionLidar_left(new PointCloud2Intensity);
            
            for(int j = 0; j < points.size(); j++) {

                pcl::PointXYZI point;
                pcl::PointXYZRGB colored_point;
                // float x1 = 0.02 * points[j].x - 8;      //以后计算完逆矩阵后需要把x和y对调
                // float y1 = -0.02 * points[j].y + 11;

                float x1 = R_Inv_ToLeftProject(0,1) * points[j].x + R_Inv_ToLeftProject(0,2);
                float y1 = R_Inv_ToLeftProject(1,0) * points[j].y + R_Inv_ToLeftProject(1,2);
      
                point.x = x1;
                point.y = y1;
                point.z = 1;
                point.intensity = 0;
                pReflectionLidar_left->points.push_back(point);
                
                colored_point.x = x1;
                colored_point.y = y1;
                colored_point.z = 1;
                colored_point.r = 0;
                colored_point.g = 255;  //绿色
                colored_point.b = 0;
                m_pAll_deal_cluster_cloud->points.push_back(colored_point);
            }
            vClusterPtr.push_back(pReflectionLidar_left);
        }
            
    }
#endif

//试试正交二乘法用极坐标的形式，直接用轮廓点了，不进行轮廓投影
    for (int i = 0; i < contours.size(); i++) {
        
        Eigen::Vector2d mean(0, 0);
        for(int j = 0; j < contours[i].size(); j++){
            mean.x() += contours[i][j].x;
            mean.y() += contours[i][j].y;
        }

        mean.x() /= contours[i].size();
        mean.y() /= contours[i].size();

        // 构建中心化矩阵
        Eigen::MatrixXd centered(contours[i].size(), 2);
        for (int k = 0; k < contours[i].size(); ++k) {
            centered(k, 0) = contours[i][k].x - mean.x();
            centered(k, 1) = contours[i][k].y - mean.y();
        }

        // SVD 分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeFullV);
        Eigen::Vector2d normal = svd.matrixV().col(1);  // 最小奇异值对应的右奇异向量

        // 提取 rho 和 theta
        double theta = atan2(normal.y(), normal.x());
        double rho = mean.x() * normal.x() + mean.y() * normal.y();

        // 保证 rho 为正
        if (rho < 0) {
            rho = -rho;
            theta += M_PI;
        }

        // 归一化 theta 到 [0, π)
        theta = fmod(theta, M_PI);  //取余操作

        double A = cos(theta);
        double B = sin(theta);
        double C = -rho;

        normalizeLineABC(A, B, C);

    }

    // cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    // for (size_t i = 0; i < mergedContours.size(); i++) {
    //     cv::Scalar color(rand() % 256, rand() % 256, rand() % 256); // 随机颜色

    //     cv::drawContours(result, mergedContours, static_cast<int>(i), color, -1);
    // }


/* 耗时长10+毫秒   // 拟合每个连通区域的方向
    
    std::vector<cv::Vec4f> fittedLines(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        cv::fitLine(contours[i], fittedLines[i], 2, 0, 0.01, 0.01);
    }

    // 合并方向一致且相距较近的区域
    double angleThreshold = CV_PI / 36; // 表示5度对应的弧度值,夹角小于 5° (弧度)
    double distanceThreshold = 20.0;   // 距离阈值（像素）

    std::vector<std::vector<cv::Point>> mergedContours;
    std::vector<bool> visited(contours.size(), false);

    for (size_t i = 0; i < contours.size(); i++) {
        if (visited[i]) continue;

        // 当前聚类的点集
        std::vector<cv::Point> currentCluster = contours[i];
        visited[i] = true;

        for (size_t j = i + 1; j < contours.size(); j++) {
            if (visited[j]) 
                continue;

            // 判断方向一致性
            double angle = calculateAngle(fittedLines[i], fittedLines[j]);
            if (angle > angleThreshold) continue;

            // 判断距离
            bool isNearby = false;
            for (const auto& p1 : contours[i]) {
                for (const auto& p2 : contours[j]) {
                    if (euclideanDistance(p1, p2) < distanceThreshold) {
                        isNearby = true;
                        break;
                    }
                }
                if (isNearby) break;
            }

            // 如果方向一致且距离较近，则合并
            if (isNearby) {
                currentCluster.insert(currentCluster.end(), contours[j].begin(), contours[j].end());
                visited[j] = true;
            }
        }

        // 保存合并后的点集
        mergedContours.push_back(currentCluster);
    }

    printf("检测到区域数： %d\n", mergedContours.size());
    // 绘制结果
    cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    for (size_t i = 0; i < mergedContours.size(); i++) {
        cv::Scalar color(rand() % 256, rand() % 256, rand() % 256); // 随机颜色
        cv::drawContours(result, mergedContours, static_cast<int>(i), color, -1);
    }
*/

/*//直接由距离判断得到

    // 合并相邻区域
    double distanceThreshold = 20.0; // 距离阈值（像素）
    std::vector<std::vector<cv::Point>> mergedContours;
    std::vector<bool> visited(contours.size(), false);

    for (size_t i = 0; i < contours.size(); i++) {
        if (visited[i]) 
            continue;

        // 当前区域的点集
        std::vector<cv::Point> currentCluster = contours[i];
        visited[i] = true;

        for (size_t j = i + 1; j < contours.size(); j++) {
            if (visited[j]) continue;

            // 判断两个区域是否靠近
            bool isNearby = false;
            for (const auto& p1 : currentCluster) {
                for (const auto& p2 : contours[j]) {
                    if (euclideanDistance(p1, p2) < distanceThreshold) {
                        isNearby = true;
                        break;
                    }
                }
                if (isNearby) 
                    break;
            }
        
            // 合并区域
            if (isNearby) {
                currentCluster.insert(currentCluster.end(), contours[j].begin(), contours[j].end());
                visited[j] = true;
            }
        }

        // 保存合并后的区域
        mergedContours.push_back(currentCluster);
    }
     printf("检测到区域数： %d\n", mergedContours.size());
    // 绘制结果
    cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    for (size_t i = 0; i < mergedContours.size(); i++) {
        cv::Scalar color(rand() % 256, rand() % 256, rand() % 256); // 随机颜色
        cv::drawContours(result, mergedContours, static_cast<int>(i), color, -1);
    }
*/

/**///可以使用网格划分或 KD-Tree 结构加速区域间的搜索和判断
    
/*
    //使用 findContours 替代 connectedComponents
    std::vector<std::vector<cv::Point>> contours;
    //cv::RETR_EXTERNAL：只检测最外层轮廓
    cv::findContours(temp_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    printf("检测到区域数： %d\n", contours.size());


    // 创建掩膜图像，与输入图像大小一致
    // cv::Mat mask = cv::Mat::zeros(temp_image.size(), CV_8UC1);

    // // 选择某个轮廓（例如第一个轮廓）
    // for(int i = 0; i < contours.size(); i++){

    //     // 绘制该轮廓到掩膜图像上
    //     cv::drawContours(mask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    // }
    // // 提取掩膜图中非零像素的位置
    // std::vector<cv::Point> pointsInside;
    // cv::findNonZero(mask, pointsInside);

    // // 输出轮廓内的原始点
    // std::cout << "Number of points inside the contour: " << pointsInside.size() << std::endl;

    // // 可视化结果
    // cv::Mat result;
    // cv::cvtColor(temp_image, result, cv::COLOR_GRAY2BGR);
    // for (const auto& point : pointsInside) {
    //     result.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255); // 红色标记原始点
    // }
    
    std::vector<std::vector<cv::Point>> mergedContours;
    // 存储每个连通区域的方向和质心和包围盒
    std::vector<cv::Vec4f> directions;
    std::vector<cv::Point2f> centers;
    std::vector<cv::Rect> boundingBoxes;

    for (const auto& contour : contours) {
        if (contour.size() < 20) 
        {   
            double area = cv::contourArea(contour);
            continue; // 忽略太小的轮廓
            std::cout<<"   区域点数少，面积为 "<<area<<std::endl;
        }
        else{
            double area = cv::contourArea(contour);
            // mergedContours.push_back(contour);
            std::cout<<"   区域面积为 "<<area<<std::endl;
        }
            
        // 计算方向
        cv::Vec4f line;
        cv::fitLine(contour, line, 2, 0, 0.01, 0.01);
        directions.push_back(line);

        // 计算质心
        cv::Moments moments = cv::moments(contour);
        cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
        centers.push_back(center);

        // 计算包围盒
        boundingBoxes.push_back(cv::boundingRect(contour));
    }

    // 合并方向和距离接近的区域
    double angleThreshold = 10.0;  // 方向夹角最大容许值（单位：度）
    double xdistanceThreshold = 20.0; // 水平方向最大容许距离（单位：像素）
    double yDistanceThreshold = 10.0; // 垂直方向最大容许距离

    for (size_t i = 0; i < contours.size(); ++i) {
        bool merged = false;
        for (size_t j = i + 1; j < contours.size(); ++j) {
            if (isDirectionSimilar(directions[i], directions[j], angleThreshold) &&
                computeYDistance(centers[i], centers[j]) < yDistanceThreshold &&
                isBoundingBoxOverlapping(boundingBoxes[i], boundingBoxes[j])) 
            {
                
                // 合并两个区域
                std::vector<cv::Point> combined = contours[i];
                combined.insert(combined.end(), contours[j].begin(), contours[j].end());
                mergedContours.push_back(combined);
                merged = true;
            }
        }
        if (!merged) {
            mergedContours.push_back(contours[i]); // 独立区域
        }
    }
     printf("修改后区域数： %d\n", mergedContours.size());

    // 创建彩色图像以显示结果
    cv::Mat result = cv::Mat::zeros(temp_image.size(), CV_8UC3);
    // 为每个连通区域分配颜色
    for (size_t i = 0; i < mergedContours.size(); i++) {
        cv::Scalar color(rand() % 256, rand() % 256, rand() % 256); // 随机颜色
        cv::drawContours(result, mergedContours, static_cast<int>(i), color, -1); // 填充区域
    }
*/
/* 霍夫变换检测直线
    //边缘检测
    // cv::Mat edges;
    // cv::Canny(temp_image, edges, 50, 200);

    // 霍夫变换检测直线
    std::vector<cv::Vec4i> lines;
    //1：ρ的分辨率（像素）（累加器的距离分辨率）； 生成极坐标时候的像素扫描步长，一般取值为 1 ，不要大于图像尺寸的一半
    // CV_PI / 180：θ的分辨率（）（累加器的角度分辨率）；生成极坐标时候的角度步长
    //50：累加器的阈值；50：最短线段长度；10：最大间隔
    cv::HoughLinesP(temp_image, lines, m_strLCDConfig.hough_rho, m_strLCDConfig.hough_theta, 
                    m_strLCDConfig.hough_threshold, m_strLCDConfig.hough_minLineLength, m_strLCDConfig.hough_maxLineGap);
   
    //绘制结果
    cv::Mat result;
    cv::cvtColor(temp_image, result, cv::COLOR_GRAY2BGR); // 转换为彩色图像
    if(!lines.empty())
    {
        printf("检测到直线数目： %d\n", lines.size());
        for (const auto& line : lines) {
            cv::line(result, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                    cv::Scalar(0, 0, 255), 2); // 红色直线
            // PointCloud2Intensity::Ptr pReflectionLine(new PointCloud2Intensity);
            // pcl::PointXYZI point1, point2;
            // float x1 = 0.02 * line[1] - 10;
            // float y1 = -0.02 * line[0] + 11;
            // float x2 = 0.02 * line[3] - 10;
            // float y2 = -0.02 * line[2] + 11;

            // point1.x = x1;
            // point1.y = y1;
            // point1.z = 0;
            // point1.intensity = 0;
            // pReflectionLine->points.push_back(point1);

            // point2.x = x2;
            // point2.y = y2;
            // point2.z = 0;
            // point2.intensity = 0;
            // pReflectionLine->points.push_back(point2);

            // vClusterPtr.push_back(pReflectionLine);
        }
    }
    else{
        printf("没有检测到直线\n");
    }
*/

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> ulldif = (end-start) ;
    auto time = ulldif.count() * 1000;
    std::cout<<"耗时： "<<time <<" ms"<<std::endl;

    //图片下面加注释
    std::string white = "white: All contours";
    std::string blue = "blue: contour less than 25";
    std::string red = "red: direction less than 40 degrees";
    std::string green = "green: result";
    // line(result, Point(0, offset.x -10), Point(iWidth, offset.x -10), Scalar(255, 255, 255), 1);  // 白色线
    
    line(result, Point(0, offset.x ), Point(iWidth, offset.x ), Scalar(255, 255, 255), 1);  // 白色线
    line(result, Point(offset.y, 0), Point(offset.y, iHeight), Scalar(255, 255, 255), 1); // 白色线
    putText(result, "Y", Point(offset.y + 10,  20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    putText(result, "X", Point(iWidth - 50, offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);
    
    circle(result, Point(offset.y, offset.x), 5, Scalar(0, 0, 255), -1); // 绘制红色小圆点
    // 等距线间隔，单位为米
    double interval_x = 5.0;
    // 计算等距线在图像中的位置并绘制
    for (double y = -50; y <= 50; y += interval_x) {
        int x = static_cast<int>(-y * dScale + offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
        if (x >= 0 && x < iHeight) {  // 确保线在图像边界内
            // line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
            putText(result, std::to_string(static_cast<int>(y)) + "m", Point(offset.y + 10, x - 5), 
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
        }
    }

    // 等距线间隔，单位为米
    double interval_y= 1.0;
    // 计算等距线在图像中的位置并绘制
    for (double x = -50; x <= 50; x += interval_y) {
        int y = static_cast<int>(x * dScale + offset.y); // 计算等距线在图像中的位置
        if(x == 0)
            continue;
        else{
            if (y >= 0 && y < iWidth) {  // 确保线在图像边界内
                // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
                putText(result, std::to_string(static_cast<int>(x)), Point(y, offset.x + 15), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
        }   
    }



    putText(result, white, Point(0, offset.x + 35 ), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(result, blue, Point(0, offset.x + 55), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);

    putText(result, red, Point(0, offset.x + 75), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(result, green, Point(0, offset.x + 90), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    // putText(result, green, Point(offset.y-80, offset.x + 10), 
    //                 FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 1);

    std::string sFilePath;
    if(iSideFlag == 0){
        sFilePath = "/home/zyl/echiev_lidar_curb_detection/log/temp_right_side_image/" + std::to_string(ullTime) +"_"+std::to_string(iSideFlag) +".png";
    }
    else if(iSideFlag == 1){
        sFilePath = "/home/zyl/echiev_lidar_curb_detection/log/temp_left_side_image/" + std::to_string(ullTime) +"_"+std::to_string(iSideFlag) +".png";
    }
    
    //  cv::imwrite(sFilePath,result);
    cv::imwrite(sFilePath,temp_image);

    // return false; 
    


    // double min_x = 100.0;
    // double max_x = -100.0;
    // double min_y = 100.0;
    // double max_y = -100.0;
    // if(!pLastestLeftCluster->points.empty()){
    //     for(int i = 0; i<pLastestLeftCluster->points.size();i++){
    //         pcl::PointXYZRGB colored_point;
    //         colored_point.x = pLastestLeftCluster->points[i].x;
    //         colored_point.y = pLastestLeftCluster->points[i].y;
    //         colored_point.z = pLastestLeftCluster->points[i].z;
    //         colored_point.r = 0;
    //         colored_point.g = 255;  //绿色
    //         colored_point.b = 0;
    //         m_pAll_deal_cluster_cloud->points.push_back(colored_point);
    //         // std::cout<<"左侧聚类簇内点: x = "<<colored_point.x<<" , y = "<<colored_point.y<<std::endl;
        
    //         if(pLastestLeftCluster->points[i].x < min_x)
    //             min_x = pLastestLeftCluster->points[i].x;

    //         if(pLastestLeftCluster->points[i].x > min_x)
    //             max_x = pLastestLeftCluster->points[i].x;

    //         if(pLastestLeftCluster->points[i].y < min_y)
    //             min_y = pLastestLeftCluster->points[i].y;

    //         if(pLastestLeftCluster->points[i].y > max_y)
    //             max_y = pLastestLeftCluster->points[i].y;
    
    //     }
    //     std::cout<<"左侧聚类簇内 x的范围: ("<<min_x<<" , "<<max_x<<")"<<std::endl;
    // }

/**/
    
    if(vClusterPtr.empty()){
        std::cout<<" hough 聚类完簇数: "<<vClusterPtr.size()<<" ，空 "<<std::endl;
        return false;
    }
    else{
        std::cout<<" hough 聚类完簇数: "<<vClusterPtr.size()<<" ，不为空 "<<std::endl;
        return true;
    }

    // temp_viewer.ProjectPointCloud(pProjectedClouds, m_pAll_deal_cluster_cloud);
    // temp_viewer.SaveImage("/home/zyl/echiev_lidar_curb_detection/log/temp_image/",ullTime);

}
#endif




void LidarCurbDectection::EdgeClusteringProcess(PointCloud2Intensity::Ptr pAfterVoxelGrid, PointCloud2Intensity::Ptr pNoGroudPoints, unsigned long long ullTime){
    
    bool bProjectionModel = m_strLCDConfig.projectionModel;
    bool bSavePictureModel = m_strLCDConfig.savePictureModel;

    float fLeftMinClusterThreshold = m_strLCDConfig.leftMinClusterThreshold;
    float fLeftMaxClusterThreshold = m_strLCDConfig.leftMaxClusterThreshold;
    float fLeftPointsDistance = m_strLCDConfig.leftPointsDistance;
    float fRightMinClusterThreshold = m_strLCDConfig.rightMinClusterThreshold;
    float fRightMaxClusterThreshold = m_strLCDConfig.rightMaxClusterThreshold;
    float fRightPointsDistance = m_strLCDConfig.rightPointsDistance;

    m_pAll_deal_cluster_cloud->points.clear();
    m_pAll_output_curve_cloud->points.clear();

    // CurveViewer->setBackgroundColor(0, 0, 0);
    // CurveViewer->addCoordinateSystem(1.0);
    // CurveViewer->removeAllPointClouds();

    std::vector<pcl::PointIndices> vClusterIndices;

    //想想粗提取这个地方就是感兴趣区域提取（ROI）
    // 粗提取把道路分两边
    std::vector<PointCloud2Intensity::Ptr> vCrudeExtractTwoEdgeCloudPtrList;
    vCrudeExtractTwoEdgeCloudPtrList.resize(2);  
    for(int i = 0; i < vCrudeExtractTwoEdgeCloudPtrList.size(); i++){
        vCrudeExtractTwoEdgeCloudPtrList[i].reset(new PointCloud2Intensity);
    }
    for(int i = 0; i < pNoGroudPoints->points.size(); i++){
        // if(pNoGroudPoints->points[i].x > 5 && pNoGroudPoints->points[i].y < 10)  //右侧;  这里把右侧道路的y也限制，不然点云数量太多，影响欧式聚类
        //     vCrudeExtractTwoEdgeCloudPtrList[0]->points.push_back(pNoGroudPoints->points[i]);
        // else if(pNoGroudPoints->points[i].x < -5.5 & pNoGroudPoints->points[i].x > -5.75 && pNoGroudPoints->points[i].y < 10)    //左侧 && pNoGroudPoints->points[i].x > -5.15  -4.8 |  x < -2.5 y < 1.5
        //     vCrudeExtractTwoEdgeCloudPtrList[1]->points.push_back(pNoGroudPoints->points[i]);
        // else
        //     continue;

        if(pNoGroudPoints->points[i].x > 0 && pNoGroudPoints->points[i].y < m_strLCDConfig.rightYMax)  //右侧;  这里把右侧道路的y也限制，不然点云数量太多，影响欧式聚类
            vCrudeExtractTwoEdgeCloudPtrList[0]->points.push_back(pNoGroudPoints->points[i]);
        else if(pNoGroudPoints->points[i].x < 0 && pNoGroudPoints->points[i].y < m_strLCDConfig.leftYMax)    //左侧 && pNoGroudPoints->points[i].x > -5.15  -4.8 |  x < -2.5 y < 1.5
            vCrudeExtractTwoEdgeCloudPtrList[1]->points.push_back(pNoGroudPoints->points[i]);
        else
            continue;
        
    }
    
    // 存储聚类结果
    std::vector<pcl::PointIndices> clusters_indices_right;
    std::vector<pcl::PointIndices> clusters_indices_left;

    std::vector<PointCloud2Intensity::Ptr> clusters_ptr_right;
    std::vector<PointCloud2Intensity::Ptr> clusters_ptr_left;

    PointCloud2Intensity::Ptr pLastestLeftCluster(new PointCloud2Intensity());
    PointCloud2Intensity::Ptr pLastestRightCluster(new PointCloud2Intensity());

    PointCloud2Intensity::Ptr pResult(new PointCloud2Intensity());
    
    bool right_flag;
    bool left_flag;

    LineRunningInfo left_line_running_info;
    LineRunningInfo right_line_running_info;

    //发送
    STR_CV_LANE_DATA strCurbs;
    strCurbs.byAlgNum = byAlgNum;
    strCurbs.ullTimestamp = ullTime;
    strCurbs.ullTimestampModule = ullTime;
    strCurbs.strAlmInfo.byConfidence = 1;
    strCurbs.emTypeSensor = 10;
    strCurbs.iCounts = 0;

    std::ofstream file_left("/home/zyl/echiev_lidar_curb_detection/log/left_side_points.csv", ios::app);
    std::ofstream file_right("/home/zyl/echiev_lidar_curb_detection/log/right_side_points.csv", ios::app);
    // if (!file.is_open()) {
    //     cerr << "无法打开文件" << endl;
    //     return 0;
    // }

//右侧无滤波
#if 1
    if( m_strLCDConfig.isDetectionRightRoad && 
        !m_strLCDConfig.isRunningFilter &&
        !vCrudeExtractTwoEdgeCloudPtrList[0]->points.empty())     
    {    
        pResult->points.clear();

        //计算簇的平均x坐标
        double x_sum = 0;
        double x_right_avg = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[0]->points) {
            x_sum += point.x;
        }
        x_right_avg = x_sum / vCrudeExtractTwoEdgeCloudPtrList[0]->size();

        //计算簇的平均y坐标
        double y_sum = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[0]->points) {
            y_sum += point.y;
        }
        double y_avg = y_sum / vCrudeExtractTwoEdgeCloudPtrList[0]->size();

        //计算簇的平均z坐标
        double z_sum = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[0]->points) {
            z_sum += point.z;
        }
        double z_avg = z_sum / vCrudeExtractTwoEdgeCloudPtrList[0]->size();

        LOG_RAW("***道路右侧输入的点数: %d , 平均x: %f , 平均y: %f , 平均z: %f\n", vCrudeExtractTwoEdgeCloudPtrList[0]->points.size(), x_right_avg, y_avg, z_avg);
        right_flag = IsClustering_hough(vCrudeExtractTwoEdgeCloudPtrList[0], clusters_ptr_right, ullTime, 0);
        // right_flag = IsClustering(fRightPointsDistance, fRightMaxClusterThreshold, fRightMinClusterThreshold, vCrudeExtractTwoEdgeCloudPtrList[0], clusters_ptr_right);
        // right_flag = IsClustering(0.5, 200, 20, vCrudeExtractTwoEdgeCloudPtrList[0], clusters_indices_right);
        // // if(right_flag)
        // //     vClusterIndices.insert(vClusterIndices.end(),cluster_indices_right.begin(),cluster_indices_right.end());
       
        if(right_flag){

            if(clusters_ptr_right.size() >= 2){
                // pLastestRightCluster = getLargestCluster(clusters_ptr_right);
                pLastestRightCluster = getCloserRightCluster(clusters_ptr_right);
                // vClusters.push_back(pLastestRightCluster);
                LOG_RAW("右侧大于2个簇, 点云数量： %d\n", pLastestRightCluster->points.size());
            }
            else{
                pLastestRightCluster = getCluster(clusters_ptr_right);
                // vClusters.push_back(pLastestRightCluster);
                LOG_RAW("右侧只有1簇, 点云数量： %d\n", pLastestRightCluster->points.size());
            }

            //曲线拟合计算
            pResult = m_pCurveFitting->CurveFittingStart(pLastestRightCluster,ullTime,0); 
            
           
            file_right<<ullTime<<","<<pResult->points[0].x<<","<<pResult->points[0].y<<","<<pResult->points[1].x<<","<<pResult->points[1].y<<endl;
            file_right.close();
            
            strCurbs.iCounts += 1;
            strCurbs.pstrCVLane[0].byType = 1;
            for(int i = 0; i < pResult->points.size(); i++){
                float x = pResult->points[i].x;
                float y = pResult->points[i].y;
                strCurbs.pstrCVLane[0].pstrPoint2fBirdEyeView[i].fX = x;
                strCurbs.pstrCVLane[0].pstrPoint2fBirdEyeView[i].fY = y;
                strCurbs.pstrCVLane[0].pstrPoint2fCamera[i].fX = x;
                strCurbs.pstrCVLane[0].pstrPoint2fCamera[i].fY = y; 
                LOG_RAW("   最后结果点数 = %d, i = %d, x = %f, y = %f\n", pResult->points.size(), i, x, y);

                pcl::PointXYZRGB tempRGB;
                tempRGB.x = x;
                tempRGB.y = y;
                tempRGB.z = 0;
                tempRGB.r = 255;
                tempRGB.g = 255;
                tempRGB.b = 255;
                m_pAll_output_curve_cloud->points.push_back(tempRGB);
            }
            
        }
    }
#endif

//左侧无滤波
#if 1
    double x_left_avg = 0;
    if( m_strLCDConfig.isDetectionLeftRoad && 
        !m_strLCDConfig.isRunningFilter &&
        !vCrudeExtractTwoEdgeCloudPtrList[1]->points.empty())     
    {    
        pResult->points.clear();

        //计算簇的平均x坐标
        double x_sum = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[1]->points) {
            x_sum += point.x;
        }
        x_left_avg = x_sum / vCrudeExtractTwoEdgeCloudPtrList[1]->size();
        
        //计算簇的平均y坐标
        double y_sum = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[1]->points) {
            y_sum += point.y;
        }
        double y_avg = y_sum / vCrudeExtractTwoEdgeCloudPtrList[1]->size();

        //计算簇的平均z坐标
        double z_sum = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[1]->points) {
            z_sum += point.z;
        }
        double z_avg = z_sum / vCrudeExtractTwoEdgeCloudPtrList[1]->size();
        
        LOG_RAW("***道路左侧输入的点数: %d , 平均x: %f , 平均y: %f , 平均z: %f\n", vCrudeExtractTwoEdgeCloudPtrList[1]->points.size(), x_left_avg, y_avg, z_avg);
        left_flag = IsClustering_hough(vCrudeExtractTwoEdgeCloudPtrList[1], clusters_ptr_left, ullTime, 1);
        // left_flag = IsClustering(fLeftPointsDistance, fLeftMaxClusterThreshold, fLeftMinClusterThreshold, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_ptr_left);
        // left_flag = IsClustering(1, 200, 10, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_indices_left);
        
        // //if(left_flag)
        // //    vClusterIndices.insert(vClusterIndices.end(),cluster_indices_left.begin(),cluster_indices_left.end());
        
        if(left_flag){

            if(clusters_ptr_left.size() >= 2){
                // pLastestLeftCluster = getLargestCluster(clusters_ptr_left);
                pLastestLeftCluster = getCloserLeftCluster(clusters_ptr_left);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("左侧大于2个簇, 点云数量： %d\n", pLastestLeftCluster->points.size());
            }
            else{
                pLastestLeftCluster = getCluster(clusters_ptr_left);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("左侧只有1个簇, 点云数量： %d\n", pLastestLeftCluster->points.size());
            }

            //曲线拟合计算
            pResult = m_pCurveFitting->CurveFittingStart(pLastestLeftCluster,ullTime,1);

            file_left<<ullTime<<","<<pResult->points[0].x<<","<<pResult->points[0].y<<","<<pResult->points[1].x<<","<<pResult->points[1].y<<endl;
            file_left.close();

            strCurbs.iCounts += 1;
            strCurbs.pstrCVLane[0].byType = 1;
            for(int i = 0; i < pResult->points.size(); i++){
                float x = pResult->points[i].x;
                float y = pResult->points[i].y;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fY = y;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fY = y; 
                LOG_RAW("   最后结果点数 = %d, i = %d, x = %f, y = %f\n", pResult->points.size(), i, x, y);

                pcl::PointXYZRGB tempRGB;
                tempRGB.x = x;
                tempRGB.y = y;
                tempRGB.z = 0;
                tempRGB.r = 255;
                tempRGB.g = 255;
                tempRGB.b = 255;
                m_pAll_output_curve_cloud->points.push_back(tempRGB);

            }
            
        }
    
    }
#endif


//右侧滑窗滤波版本
#if 1
    if( m_strLCDConfig.isDetectionRightRoad &&
        m_strLCDConfig.isRunningFilter && 
        !vCrudeExtractTwoEdgeCloudPtrList[0]->points.empty())     
    {     
        pResult->points.clear();

        //计算簇的平均x坐标
        double x_sum = 0;
        double x_right_avg = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[0]->points) {
            x_sum += point.x;
        }
        x_right_avg = x_sum / vCrudeExtractTwoEdgeCloudPtrList[0]->size();
        
        LOG_RAW("***道路右侧输入的点数: %d , 平均x: %f\n", vCrudeExtractTwoEdgeCloudPtrList[0]->points.size(), x_right_avg);
        right_flag = IsClustering_hough(vCrudeExtractTwoEdgeCloudPtrList[0], clusters_ptr_right, ullTime, 0);
        // left_flag = IsClustering(fLeftPointsDistance, fLeftMaxClusterThreshold, fLeftMinClusterThreshold, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_ptr_left);
        // left_flag = IsClustering(1, 200, 10, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_indices_left);
        
        // //if(left_flag)
        // //    vClusterIndices.insert(vClusterIndices.end(),cluster_indices_left.begin(),cluster_indices_left.end());
        
        if(right_flag){

            if(clusters_ptr_right.size() >= 2){
                // pLastestRightCluster = getLargestCluster(clusters_ptr_left);
                pLastestRightCluster = getCloserRightCluster(clusters_ptr_right);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("右侧大于2个簇, 点云数量： %d\n", pLastestRightCluster->points.size());
            }
            else{
                pLastestRightCluster = getCluster(clusters_ptr_right);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("右侧只有1个簇, 点云数量： %d\n", pLastestRightCluster->points.size());
            }

           
            //曲线拟合计算
            pResult = m_pCurveFitting->CurveFittingStart(pLastestRightCluster,ullTime,0);

            right_line_running_info.runningValue = m_iSystemRunCount;
            right_line_running_info.existFlag = true;
            right_line_running_info.linePointCloud = pResult;
            m_qRightLineInfo.push(right_line_running_info);
          
        }    
        else{
           
            right_line_running_info.runningValue = m_iSystemRunCount;
            right_line_running_info.existFlag = false;
            PointCloud2Intensity::Ptr pInvalidPointCloud(new PointCloud2Intensity());
            pcl::PointXYZI invaliData;
            invaliData.x = 0;
            invaliData.y = 0;
            invaliData.z = 0;
            invaliData.intensity = 125;
            pInvalidPointCloud->points.push_back(invaliData);
            right_line_running_info.linePointCloud = pInvalidPointCloud;    
            m_qRightLineInfo.push(right_line_running_info);
           
        }  

        // m_pAll_output_curve_cloud->points.clear();
    
        //滑窗操作
        if(fmod(m_iSystemRunCount, m_strLCDConfig.slideWindowCount) == 0){
            pResult = SlideWindowProcess(m_qRightLineInfo);
            // std::cout<<"滑窗结束后个数： "<<m_qLeftLineInfo.size()<<std::endl;

            strCurbs.iCounts += 1;
            strCurbs.pstrCVLane[0].byType = 1;
            for(int i = 0; i < pResult->points.size(); i++){
                float x = pResult->points[i].x;
                float y = pResult->points[i].y;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fY = y;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fY = y; 

                LOG_RAW("   滑窗操作最后结果点数 = %d, i = %d, x = %f, y = %f \n", pResult->points.size(), i, x, y);

                pcl::PointXYZRGB tempRGB;
                tempRGB.x = x;
                tempRGB.y = y;
                tempRGB.z = 0;
                tempRGB.r = 255;
                tempRGB.g = 255;
                tempRGB.b = 255;
                m_pAll_output_curve_cloud->points.push_back(tempRGB);

            }
        
        
            file_right<<ullTime<<","<<pResult->points[0].x<<","<<pResult->points[0].y<<","<<pResult->points[1].x<<","<<pResult->points[1].y<<endl;
            file_right.close();

            byAlgNum++;
            int iRet = CameraLaneMCServerSend((unsigned char *)&strCurbs, sizeof(STR_CV_LANE_DATA));

            //点云转图像
            if(bProjectionModel){
                m_pViewer->ProjectPointCloud(pAfterVoxelGrid, pNoGroudPoints, m_pAll_deal_cluster_cloud, m_pAll_output_curve_cloud);
                // m_pViewer->Display();
                if(bSavePictureModel)
                    m_pViewer->SaveImage(ullTime);
            }    
        }
    }

#endif

//左侧滑窗滤波版本
#if 1
    if( m_strLCDConfig.isDetectionLeftRoad &&
        m_strLCDConfig.isRunningFilter && 
        !vCrudeExtractTwoEdgeCloudPtrList[1]->points.empty())     
    {     
        pResult->points.clear();

        //计算簇的平均x坐标
        double x_sum = 0;
        double x_left_avg = 0;
        for (const auto& point : vCrudeExtractTwoEdgeCloudPtrList[1]->points) {
            x_sum += point.x;
        }
        x_left_avg = x_sum / vCrudeExtractTwoEdgeCloudPtrList[1]->size();
        
        LOG_RAW("***道路左侧输入的点数: %d , 平均x: %f\n", vCrudeExtractTwoEdgeCloudPtrList[1]->points.size(), x_left_avg);
        left_flag = IsClustering_hough(vCrudeExtractTwoEdgeCloudPtrList[1], clusters_ptr_left, ullTime, 1);
        // left_flag = IsClustering(fLeftPointsDistance, fLeftMaxClusterThreshold, fLeftMinClusterThreshold, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_ptr_left);
        // left_flag = IsClustering(1, 200, 10, vCrudeExtractTwoEdgeCloudPtrList[1], clusters_indices_left);
        
        // //if(left_flag)
        // //    vClusterIndices.insert(vClusterIndices.end(),cluster_indices_left.begin(),cluster_indices_left.end());
        
        if(left_flag){

            if(clusters_ptr_left.size() >= 2){
                // pLastestLeftCluster = getLargestCluster(clusters_ptr_left);
                pLastestLeftCluster = getCloserLeftCluster(clusters_ptr_left);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("左侧大于2个簇, 点云数量： %d\n", pLastestLeftCluster->points.size());
            }
            else{
                pLastestLeftCluster = getCluster(clusters_ptr_left);
                // vClusters.push_back(pLastestLeftCluster);
                LOG_RAW("左侧只有1个簇, 点云数量： %d\n", pLastestLeftCluster->points.size());
            }

           
            //曲线拟合计算
            pResult = m_pCurveFitting->CurveFittingStart(pLastestLeftCluster,ullTime,1);

            left_line_running_info.runningValue = m_iSystemRunCount;
            left_line_running_info.existFlag = true;
            left_line_running_info.linePointCloud = pResult;
            m_qLeftLineInfo.push(left_line_running_info);
        }    
        else{
           
            left_line_running_info.runningValue = m_iSystemRunCount;
            left_line_running_info.existFlag = false;
            PointCloud2Intensity::Ptr pInvalidPointCloud(new PointCloud2Intensity());
            pcl::PointXYZI invaliData;
            invaliData.x = 0;
            invaliData.y = 0;
            invaliData.z = 0;
            invaliData.intensity = 125;
            pInvalidPointCloud->points.push_back(invaliData);
            left_line_running_info.linePointCloud = pInvalidPointCloud;    
            m_qLeftLineInfo.push(left_line_running_info);
           
        }  
    
        // m_pAll_output_curve_cloud->points.clear();

        //滑窗操作
        if(fmod(m_iSystemRunCount, m_strLCDConfig.slideWindowCount) == 0){
            pResult = SlideWindowProcess(m_qLeftLineInfo);
            // std::cout<<"滑窗结束后个数： "<<m_qLeftLineInfo.size()<<std::endl;

            strCurbs.iCounts += 1;
            strCurbs.pstrCVLane[0].byType = 1;
            for(int i = 0; i < pResult->points.size(); i++){
                float x = pResult->points[i].x;
                float y = pResult->points[i].y;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fBirdEyeView[i].fY = y;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fX = x;
                strCurbs.pstrCVLane[1].pstrPoint2fCamera[i].fY = y; 

                LOG_RAW("   滑窗操作最后结果点数 = %d, i = %d, x = %f, y = %f \n", pResult->points.size(), i, x, y);

                pcl::PointXYZRGB tempRGB;
                tempRGB.x = x;
                tempRGB.y = y;
                tempRGB.z = 0;
                tempRGB.r = 255;
                tempRGB.g = 255;
                tempRGB.b = 255;
                m_pAll_output_curve_cloud->points.push_back(tempRGB);

            }
        
        
            file_left<<ullTime<<","<<pResult->points[0].x<<","<<pResult->points[0].y<<","<<pResult->points[1].x<<","<<pResult->points[1].y<<endl;
            file_left.close();

            byAlgNum++;
            int iRet = CameraLaneMCServerSend((unsigned char *)&strCurbs, sizeof(STR_CV_LANE_DATA));

            //点云转图像
            if(bProjectionModel){
                m_pViewer->ProjectPointCloud(pAfterVoxelGrid, pNoGroudPoints, m_pAll_deal_cluster_cloud, m_pAll_output_curve_cloud);
                // m_pViewer->Display();
                if(bSavePictureModel)
                    m_pViewer->SaveImage(ullTime);
            }    
        }
    }

#endif

    if(!m_strLCDConfig.isRunningFilter){
        byAlgNum++;
        int iRet = CameraLaneMCServerSend((unsigned char *)&strCurbs, sizeof(STR_CV_LANE_DATA));

        //点云转图像
        if(bProjectionModel){
            m_pViewer->ProjectPointCloud(pAfterVoxelGrid, pNoGroudPoints, m_pAll_deal_cluster_cloud, m_pAll_output_curve_cloud);
            // m_pViewer->Display();
            if(bSavePictureModel)
                m_pViewer->SaveImage(ullTime);
        }    
    }
    
    

    // std::cout<<"****** 总数 : "<<vClusterIndices.size()<<std::endl;
    
    // CurveViewer->addPointCloud<pcl::PointXYZRGB>(m_pAll_deal_cluster_cloud, "All_deal_cluster_cloud");
    // CurveViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "All_deal_cluster_cloud");
    
    // CurveViewer->addPointCloud<pcl::PointXYZRGB>(m_pAll_output_curve_cloud, "curve_cloud");
    // CurveViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "curve_cloud");

    // CurveViewer->spinOnce();
 //-----------------------------------------   

#if 0 
    // 可视化
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->removeAllPointClouds();

    
    // 用于可视化的聚类点云
    PointCloud2RGB::Ptr colored_cloud(new PointCloud2RGB());
    // int cluster_id = 0;
#endif

#if 0    
    for (const auto& indices : clusters_indices_left) {
        PointCloud2Intensity::Ptr cluster(new PointCloud2Intensity());
        for (int idx : indices.indices) {
            cluster->push_back((*m_vCrudeExtractTwoEdgeCloudPtrList[1])[idx]);
        }

        // 计算簇的平均 x 坐标
        double x_sum = 0;
        for (const auto& point : cluster->points) {
            x_sum += point.x;
        }
        double x_avg = x_sum / cluster->points.size();

        uint8_t r, g, b;
        if (x_avg < 0.0) { // 左侧簇
            r = 255; g = 0; b = 0; // 红色
        } 

        if(x_avg > 0.0) {  // 右侧簇
            r = 0; g = 255; b = 0; // 绿色
        }

        for (const auto& point : cluster->points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            colored_cloud->points.push_back(colored_point);
        }
    
        // cluster_id++;
    }
#endif

#if 0

    for (const auto& indices : clusters_indices_right) {
        PointCloud2Intensity::Ptr cluster(new PointCloud2Intensity());
        for (int idx : indices.indices) {
            cluster->push_back((*m_vCrudeExtractTwoEdgeCloudPtrList[0])[idx]);
        }

        // 计算簇的平均 x 坐标
        double x_sum = 0;
        for (const auto& point : cluster->points) {
            x_sum += point.x;
        }
        double x_avg = x_sum / cluster->size();

        uint8_t r, g, b;
        if (x_avg < 0.0) { // 左侧簇
            r = 255; g = 0; b = 0; // 红色
        } 

        if(x_avg > 0.0) {  // 右侧簇
            r = 0; g = 255; b = 0; // 绿色
        }

        for (const auto& point : cluster->points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            colored_cloud->points.push_back(colored_point);
        }
    
    //     // cluster_id++;
    }

#endif

#if 0

    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "clustered_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "clustered_cloud");
    viewer->spinOnce();
#endif

//-----------------------------------------  

}


PointCloud2Intensity::Ptr LidarCurbDectection::SlideWindowProcess(std::queue<LineRunningInfo> &qFinalClusterInfo){
    
    std::queue<LineRunningInfo> tempQueue = qFinalClusterInfo;
    // std::cout<<"qFinalClusterInfo的长度: "<<qFinalClusterInfo.size()<<std::endl;
    PointCloud2Intensity::Ptr pOutput(new PointCloud2Intensity());
    cv::Point2f point_front_sum, point_back_sum;
    int validDataCount = 0;
    // std::cout<<"当簇为0时: 3"<<std::endl;
    for(int i = 0; i < m_strLCDConfig.slideWindowCount; i++){

        LineRunningInfo tempLineInfo = tempQueue.front();
        tempQueue.pop();
        // std::cout<<"tempQueue.pop完的长度: "<<tempQueue.size()<<std::endl;
        if(tempLineInfo.existFlag){
            
            validDataCount++;
           
            for(int j = 0; j < tempLineInfo.linePointCloud->points.size(); ++j){
                if(j == 0){
                    point_front_sum.x += tempLineInfo.linePointCloud->points[j].x;
                    point_front_sum.y += tempLineInfo.linePointCloud->points[j].y;
                    std::cout<<"y=0时: x = "<<tempLineInfo.linePointCloud->points[j].x<<" , y = "<<tempLineInfo.linePointCloud->points[j].y<<std::endl;
                }
                else{
                    point_back_sum.x += tempLineInfo.linePointCloud->points[j].x;
                    point_back_sum.y += tempLineInfo.linePointCloud->points[j].y;
                    std::cout<<"y=3时: x = "<<tempLineInfo.linePointCloud->points[j].x<<" , y = "<<tempLineInfo.linePointCloud->points[j].y<<std::endl;
                }
            }
        
        }
    }

    //  std::cout<<"有效值个数: "<<validDataCount<<std::endl;

    pcl::PointXYZI front, back;
    if(validDataCount == 0){
        front.x = 0;
        front.y = 0;
        front.z = 0;        //z轴先写死为0
        front.intensity = 255;
        pOutput->points.push_back(front);

        back.x = 0;
        back.y = 0;
        back.z = 0;        //z轴先写死为0
        back.intensity = 255;
        pOutput->points.push_back(back);
    }    
    else{
        
        front.x = point_front_sum.x / validDataCount;
        front.y = point_front_sum.y / validDataCount;
        front.z = 0;        //z轴先写死为0
        front.intensity = 255;
        pOutput->points.push_back(front);
        // std::cout<<"y=0时: x = "<<front.x<<" , y = "<<front.y<<std::endl;

        back.x = point_back_sum.x / validDataCount;
        back.y = point_back_sum.y / validDataCount;
        back.z = 0;        //z轴先写死为0
        back.intensity = 255;
        pOutput->points.push_back(back);
        // std::cout<<"y=3时: x = "<<back.x<<" , y = "<<back.y<<std::endl;
    }
    
    int initQueueCount = qFinalClusterInfo.size();
    for(int i = initQueueCount; i > 1; i--)
        qFinalClusterInfo.pop();
    
    return pOutput;
}



void LidarCurbDectection::GroudSegmentationStart(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime)
{
    m_iSystemRunCount++;

    m_vCloudPtrList.resize(2);  //2
    for(int i = 0; i < m_vCloudPtrList.size(); i++){
        m_vCloudPtrList[i].reset(new PointCloud2Intensity);
    }

    PointCloud2Intensity::Ptr m_pGroundPoints(new PointCloud2Intensity);
    PointCloud2Intensity::Ptr m_pNoGroundPoints(new PointCloud2Intensity);


    // int count = 0;
    // float max_LessAndEqual = 0.0;
    // float max_angle = 0.0; 
    // float min_lessAndEqual = 100.0;
    // float min_x = 0.0;
    // int index = 0;

    float fLeftXMin = m_strLCDConfig.leftXMin;
    float fLeftXMax = m_strLCDConfig.leftXMax;
    float fLeftYMax = m_strLCDConfig.leftYMax;
    float fRightXMin = m_strLCDConfig.rightXMin;
    float fRightXMax = m_strLCDConfig.rightXMax;
    float fRightYMax = m_strLCDConfig.rightYMax;
    

    //分段进行平面分割 
    //公司镭神y轴朝前，x轴朝右（根据x轴切两段）
#if 1
    //右+
    for(int i = 0; i < pInCloud->points.size(); i++){
        
        if( pInCloud->points[i].x <= fRightXMax && 
            pInCloud->points[i].x >= fRightXMin && 
            std::abs(pInCloud->points[i].y) < fLeftYMax){
        
            
            m_vCloudPtrList[0]->points.push_back(pInCloud->points[i]);

            
            // 水平夹角
            // float radians = std::atan2(pIncloud->points[i].y, pIncloud->points[i].x);

            //从这段代码计算出10度，最大的x距离就是2.3左右，路沿大概是2.5，所以扫描线可以用64线
            // float angle =  radians * 180 / M_PI;                           
            // if(angle<=10){
            //     count++;
            //     if(max_angle < angle){
            //         max_angle = angle;
            //         index = i;
            //     } 

                
            // }
                
        }

        //左-
        if( pInCloud->points[i].x >= fLeftXMin && 
            pInCloud->points[i].x < fLeftXMax && 
            std::abs(pInCloud->points[i].y) < fRightYMax){
            
            m_vCloudPtrList[1]->points.push_back(pInCloud->points[i]);
        
            // 水平夹角
            // float radians = std::atan2(pIncloud->points[i].y, pIncloud->points[i].x);

            // float angle =  radians * 180 / M_PI;                           
            // if(angle<=10){
            //     count++;
            //     if(min_lessAndEqual > pIncloud->points[i].x)
            //         min_lessAndEqual = pIncloud->points[i].x;
            // }

            // if(min_lessAndEqual > pIncloud->points[i].x)
            // {   
            //     min_lessAndEqual = pIncloud->points[i].x;
            //     index=i;
            // } 
        
        }
        
    }

    // std::cout<<"x轴左边(-): "<<m_vCloudPtrList[1]->points.size()<<" , x轴右边(+): "<<m_vCloudPtrList[0]->points.size()<<" , 水平夹角<=10°: "<<count<<" , 且x(+)最大值: "<<max_LessAndEqual<<std::endl;
    // std::cout<<"x轴左边(-): "<<m_vCloudPtrList[1]->points.size()<<" , x轴右边(+): "<<m_vCloudPtrList[0]->points.size()<<" , 且x(-)最大值: "<<"最大角度： "<<max_angle<<" , ("<<pIncloud->points[index].x<<", "<<pIncloud->points[index].y<<", "<<pIncloud->points[index].z<<")"<<std::endl;
    // std::cout<<"x轴左边(-): "<<m_vCloudPtrList[1]->points.size()<<" , 水平夹角<=10°: "<<count<<" , 且x(-)最大值: "<<"("<<pIncloud->points[index].x<<", "<<pIncloud->points[index].y<<", "<<pIncloud->points[index].z<<")"<<" , x轴右边(+): "<<m_vCloudPtrList[0]->points.size()<<std::endl;
#endif
    std::cout<<"x轴左边(-): "<<m_vCloudPtrList[1]->points.size()<<" , x轴右边(+): "<<m_vCloudPtrList[0]->points.size()<<std::endl;
    
    //地面点过滤
    // GroudFilterFromSAC(m_pGroundPoints, m_pNoGroundPoints);
    GroudFilterFromAPMF(m_pGroundPoints, m_pNoGroundPoints);

    if(m_pNoGroundPoints->points.empty()){
        std::cout<<"地面上的点为0,  时间： "<<ullTime<<std::endl;
    }

    printf("时间: %lld , 地面点：%ld, 非地面点: %ld, 加起来：%ld \n", ullTime, m_pGroundPoints->points.size(), m_pNoGroundPoints->points.size(),
                                (m_pGroundPoints->points.size()+m_pNoGroundPoints->points.size()));

    //根据高度提取过滤点
    ExtractPointsByHeightDifference(m_pNoGroundPoints);

    EdgeClusteringProcess(pInCloud, m_pNoGroundPoints, ullTime);

    
/*下面当在pcd读取模式时，需要换成spin()，但是两个窗口只会有一个窗口有数据，应该取决于哪个窗口代码排前面(即被堵塞)
在pcap模式时，需要改成spinOnce()
*/

    if(!m_strLCDConfig.onlineModel && m_strLCDConfig.pcapRunningModel && m_strLCDConfig.openGroudViewer){
        
        
        ground_viewer->setBackgroundColor(0, 0, 0);
        ground_viewer->addCoordinateSystem(1.0);
        ground_viewer->removeAllPointClouds();
        ground_viewer->addPointCloud<pcl::PointXYZI>(m_pGroundPoints, "GroundPoints");
        ground_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "GroundPoints");
        ground_viewer->spinOnce();

        // ground_viewer->spin();
        //  ground_viewer->spinOnce(); //这样视图会无法拖动
        // ground_viewer->addPlane(*pCoefficients, "plane");
    }
    

    if(!m_strLCDConfig.onlineModel && m_strLCDConfig.pcapRunningModel && m_strLCDConfig.openGroudViewer){
        
        
    //       // 动态调整视图范围
    // pcl::PointXYZI min_point, max_point;
    // pcl::getMinMax3D(*m_pNoGroundPoints, min_point, max_point);
    // double x_center = (min_point.x + max_point.x) / 4.0;
    // double y_center = (min_point.y + max_point.y) / 4.0;
    // double z_center = (min_point.z + max_point.z) / 4.0;
    // double max_extent = std::max({max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z});
    // double camera_distance = max_extent * 2.0; // 2倍最大范围
        
    // std::cout << "Min point: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ")\n";
    // std::cout << "Max point: (" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")\n";


        no_ground_viewer->setBackgroundColor(0, 0, 0);
        no_ground_viewer->addCoordinateSystem(1.0);
        no_ground_viewer->removeAllPointClouds();
        no_ground_viewer->addPointCloud<pcl::PointXYZI>(m_pNoGroundPoints, "NoGroundPoints");
        no_ground_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "NoGroundPoints");
        
// no_ground_viewer->setCameraPosition(0, 0, 10, 0, 0, 0);

// // 设置相机
//     no_ground_viewer->setCameraPosition(
//         x_center, y_center, z_center + camera_distance,
//         x_center, y_center, z_center,
//         0, 1, 0
//     );
//     no_ground_viewer->setCameraClipDistances(0.1, camera_distance * 4.0);

         no_ground_viewer->spinOnce();
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // no_ground_viewer->spin();
        // no_ground_viewer->spinOnce(); //只读取pcd这样视图会无法拖动
        // no_ground_viewer->addPlane(*pCoefficients, "plane");
    }



}


}