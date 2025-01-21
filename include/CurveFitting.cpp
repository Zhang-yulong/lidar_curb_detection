#include "CurveFitting.h"


// pcl::visualization::PCLVisualizer::Ptr CurveViewer(new pcl::visualization::PCLVisualizer("CurveFitting"));
namespace Lidar_Curb_Dedection
{

// 选出左右两侧的最大簇
auto getLargestCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters) {
    return *std::max_element(clusters.begin(), clusters.end(),
                                [](const PointCloud2Intensity::Ptr& a, const PointCloud2Intensity::Ptr& b) {
                                    return a->size() < b->size();
                                });
};


auto getCluster = [](const std::vector<PointCloud2Intensity::Ptr>& clusters){
    return clusters[0];
};


CurveFitting::CurveFitting(const STR_CONFIG & config)
    : m_stCFConfig(config)
{
    m_fDistinguishRoadSideThreshold = m_stCFConfig.distinguishRoadSideThreshold;
    m_pKF = new LineKalmanFilter(-80.0, -10.0);
}


CurveFitting::~CurveFitting(){

}

// 多项式拟合函数，拟合结果为多项式系数
Eigen::VectorXd CurveFitting::PolynomialCurveFit(const std::vector<Eigen::Vector2d>& vPoints, int iOrder){

    int n = vPoints.size();
    Eigen::MatrixXd X(n, iOrder + 1);
    Eigen::VectorXd Y(n);

    for (int i = 0; i < n; ++i) {
        double xi = vPoints[i].x();
        Y(i) = vPoints[i].y();
        for (int j = 0; j <= iOrder; ++j) {
            X(i, j) = pow(xi, j);
        }
    }

    // 最小二乘求解
    Eigen::VectorXd coefficients = X.colPivHouseholderQr().solve(Y);  //QR分解
    return coefficients;
}

// 用于可视化的点生成函数
PointCloud2Intensity::Ptr CurveFitting::GenerateCurve(const Eigen::VectorXd& Coefficients, double start_x, double end_x, double step) {
    PointCloud2Intensity::Ptr curve(new PointCloud2Intensity());
    // PointCloud2RGB::Ptr curve(new PointCloud2RGB());
    for (double x = start_x; x <= end_x; x += step) {
        double y = 0.0;
        for (int i = 0; i < Coefficients.size(); ++i) {
            y += Coefficients[i] * pow(x, i);
        }
        // pcl::PointXYZRGB point;
        // point.x = x;
        // point.y = y;
        // point.z = 0; //假设平面上
        // point.r = 255;
        // point.g = 0;
        // point.b = 0;
        // curve->push_back(point);
        curve->push_back((x, y, 0, 0));
    }
    return curve;
}



PointCloud2RGB::Ptr CurveFitting::GenerateCurveRGB(const Eigen::VectorXd& Coefficients, double start_x, double end_x, double step)
{
   
    PointCloud2RGB::Ptr curve(new PointCloud2RGB());
    for (double x = start_x; x <= end_x; x += step) {
        double y = 0.0;
        for (int i = 0; i < Coefficients.size(); ++i) {
            y += Coefficients[i] * pow(x, i);
        }
        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = 0; //假设平面上
        point.r = 255;
        point.g = 255;
        point.b = 255;
        curve->push_back(point);
        
    }
    return curve;
}

// // 计算点到直线的距离 ax + by + c = 0
// double CurveFitting::pointToLineDistance(const Point& p, double a, double b, double c) {
//     return std::fabs(a * p.x + b * p.y + c) / std::sqrt(a * a + b * b);
// }

double CurveFitting::calculateSlope(const Point& p1, const Point& p2) {
    if (p1.x == p2.x) {
        return std::numeric_limits<double>::infinity(); // 垂直线的斜率
    }
    return (p2.y - p1.y) / (p2.x - p1.x);
}

LinePolar CurveFitting::normalizePolar(double theta, double d){
    
    theta = fmod(theta, M_PI); // 限制到 [0, π)
    if (theta < 0) {
        theta += M_PI;
    }
    if (d < 0) {
        theta = fmod(theta + M_PI, M_PI); // 调整角度
        d = -d;
    }
    return {theta, d};
}

LinePolar CurveFitting::fitLinePolar(const Point& p1, const Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double theta = atan2(dy, dx) + M_PI_2; // 垂直方向
    double d = p1.x * cos(theta) + p1.y * sin(theta);
    return normalizePolar(theta, d);
}

// 转换为笛卡尔坐标系的一般形式
LineKBFunction CurveFitting::polarToCartesian(const LinePolar& line) {
    
    LineKBFunction out = {0,0};

    double A = cos(line.theta);
    double B = sin(line.theta);
    double C = -line.d;

    // 检查 A 是否接近于 0（垂直直线情况）
    if (fabs(A) < 1e-10) {
        A = 0;      // 设置 A 为 0
        B = 1;      // 垂直直线，B 固定为 1
        C = -line.d;

        double k = 10000;
        double b = -C;
        out={k,b};

    }
    else{
        double k = -A/B;
        double b = -C/B;
        out={k,b};
    }



    // if(B == 0){
    //     double k = 1000;
    //     double b = -C;
    //     out={k,b};
    // }
    // else{
    //     double k = -A/B;
    //     double b = -C/B;
    //     out={k,b};
    // }
        

    std::cout << "Cartesian Form: " << A << "x + " << B << "y + " << C << " = 0" << std::endl;
    
    return out;
}

// 计算协方差矩阵 S
void CurveFitting::computeCovarianceMatrix(const std::vector<cv::Point>& points, double S[3][3]) {
    // 初始化协方差矩阵
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = 0.0;

    // 累加每个点的信息
    for (const auto& p : points) {
        double D[3] = {p.x, p.y, 1.0};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                S[i][j] += D[i] * D[j];
            }
        }
    }
}

// 求解 3x3 矩阵的最小特征值对应的特征向量
void CurveFitting::computeSmallestEigenVector(double S[3][3], double eigenVector[3]) {
    // 使用幂迭代法（Power Iteration）来求解最小特征值对应的特征向量

    // 初始化一个随机向量
    double v[3] = {1.0, 1.0, 1.0};
    double lambda = 0.0;  // 特征值
    const int maxIter = 1000;
    const double epsilon = 1e-9;

    for (int iter = 0; iter < maxIter; ++iter) {
        // 矩阵-向量乘法：w = S * v
        double w[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                w[i] += S[i][j] * v[j];
            }
        }

        // 归一化向量 w
        double norm = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        for (int i = 0; i < 3; ++i)
            w[i] /= norm;

        // 检查收敛
        double diff = sqrt((w[0] - v[0]) * (w[0] - v[0]) +
                           (w[1] - v[1]) * (w[1] - v[1]) +
                           (w[2] - v[2]) * (w[2] - v[2]));
        if (diff < epsilon)
            break;

        // 更新 v
        for (int i = 0; i < 3; ++i)
            v[i] = w[i];
    }

    // 将结果赋值给特征向量
    for (int i = 0; i < 3; ++i)
        eigenVector[i] = v[i];
}

void CurveFitting::fitLineImplicit(const std::vector<cv::Point>& points, double& A, double& B, double& C, double epsilon){
    double S[3][3];         // 协方差矩阵
    double eigenVector[3];  // 最小特征值对应的特征向量

    // 计算协方差矩阵
    computeCovarianceMatrix(points, S);

    // 求解最小特征值对应的特征向量
    computeSmallestEigenVector(S, eigenVector);

    // 提取拟合结果
    A = eigenVector[0];
    B = eigenVector[1];
    C = eigenVector[2];

    // 如果 B 接近 0，则认为直线接近垂直于 x 轴
    if (fabs(B) < epsilon) {
        A = 1.0;  // 直线垂直于 x 轴
        B = 0.0;  // y 的系数为 0
        C = -eigenVector[0];  // x = constant 形式，constant = -C / A
    }
}


// 点到直线的距离
double CurveFitting::pointToLineDistance(const cv::Point& point, const LinePolar& line) {
    return fabs(point.x * cos(line.theta) + point.y * sin(line.theta) - line.d);
}

PointCloud2RGB::Ptr CurveFitting::GenerateCurveRGB(PointCloud2Intensity::Ptr pInCloud, float yMin, float yMax, unsigned long long ullTime){

    LOG_RAW("进入Curve计算 , size: %d\n",pInCloud->points.size() );
    PointCloud2RGB::Ptr curve(new PointCloud2RGB());

    int n = pInCloud->points.size();

    double min_x = 0.0;
    double min_y = 100.0;
    double max_y = -100.0;

/*   最小二乘法算的
    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for(int i = 0; i < n; i++){
        sum_x += pInCloud->points[i].x;
        sum_y += pInCloud->points[i].y;
        sum_xx += (pInCloud->points[i].x) * (pInCloud->points[i].x);
        sum_xy += (pInCloud->points[i].x) * (pInCloud->points[i].y);

        if(pInCloud->points[i].x < min_x)
            min_x = pInCloud->points[i].x;

        if(pInCloud->points[i].y < min_y)
            min_y = pInCloud->points[i].y;

        if(pInCloud->points[i].y > max_y)
            max_y = pInCloud->points[i].y;
    } 

    double x1 = sum_x / n;
    
    double k = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    double b = (sum_y - k * sum_x) / n;

    LOG_RAW("输出(k: %f , b: %f)\n", k, b);

    // for(double y = yMin; y <= yMax; y += 0.1){
    for(double y = 0; y <= 3; y += 0.01){
        double x = (y-b) / k;

        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = 0; //假设平面上
        point.r = 255;
        point.g = 255;
        point.b = 255;
        curve->push_back(point);
    }  
*/

/*正交最小二乘法*/
    double mean_x = 0, mean_y = 0;
    for(int i = 0; i < n; i++){
        mean_x += pInCloud->points[i].x;
        mean_y += pInCloud->points[i].y;

        // std::cout<<"("<<pInCloud->points[i].x<<","<<pInCloud->points[i].y<<")"<<std::endl;
        // if(pInCloud->points[i].x < min_x)
        //     min_x = pInCloud->points[i].x;

        // if(pInCloud->points[i].y < min_y)
        //     min_y = pInCloud->points[i].y;

        // if(pInCloud->points[i].y > max_y)
        //     max_y = pInCloud->points[i].y;
    } 

    mean_x /= n;
    mean_y /= n;

    // 2. 计算去中心化后的协方差矩阵
    double Sxx = 0, Syy = 0, Sxy = 0;
    for (const auto& point : pInCloud->points) {
        double dx = point.x - mean_x;
        double dy = point.y - mean_y;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
    }

    // 3. 求解协方差矩阵的特征值和特征向量
    double theta = 0.5 * atan2(2 * Sxy, Sxx - Syy);  // 计算方向角
    double eigvec_x = cos(theta);  // 特征向量的x分量
    double eigvec_y = sin(theta);  // 特征向量的y分量

    // std::cout<<"特征向量的x分量: "<<eigvec_x<<std::endl;

    // 4. 计算拟合直线的斜率和截距
    double k = eigvec_y / eigvec_x;    // 斜率
    double b = mean_y - k * mean_x;    // 截距
    
    Vector2d temp2d(k,b);
//    std::cout<<"进入之前k,b: (" <<k<<" , "<<b<<")"<<std::endl;
    LOG_RAW("输出(k: %f , b: %f)\n", k, b);

    // m_pKF->setTimeStep(0.2); //时间步长先设定为0.2s
    // m_pKF->predict();
    // m_pKF->update(temp2d);

    // Vector4d final_state= m_pKF->getState();
    // std::cout<<"kf结束后k,b: (" <<final_state[0]<<" , "<<final_state[1]<<")"<<std::endl;
    
    for(double y = 0; y <= 5; y += 0.01)
    // for(double y = min_y; y <= max_y; y += 0.01)
    {
        double x = (y-b) / k;  //原来正常的
        
        // double x = (y-final_state[1]) / final_state[0];

        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = 0; //假设平面上
        point.r = 255;
        point.g = 255;
        point.b = 255;
        curve->push_back(point);
    }  


/*通用形式正交最小二乘法
    std::vector<cv::Point> points;
    for(int i = 0; i < n; i++){
        cv::Point point;
        point.x = pInCloud->points[i].x;
        point.y = pInCloud->points[i].y;
        points.emplace_back(point);
    }
    // 拟合直线参数
    double A, B, C;
    fitLineImplicit(points, A, B, C, 1e-6);

    for(double y = 0; y <= 5; y += 0.01)
    // for(double y = min_y; y <= max_y; y += 0.01)
    {

        if(B == 0.0){
            

            pcl::PointXYZRGB point;
            point.x = -C/A;
            point.y = y;
            point.z = 0; //假设平面上
            point.r = 255;
            point.g = 255;
            point.b = 255;
            curve->push_back(point);
        }   
        else{
            

            pcl::PointXYZRGB point;
            point.x = (-C/A)-(B/A)*y;
            point.y = y;
            point.z = 0; //假设平面上
            point.r = 255;
            point.g = 255;
            point.b = 255;
            curve->push_back(point);
        }
    }  
*/
/*RANSAC
    std::srand(static_cast<unsigned int>(std::time(0))); // 随机种子
    std::vector<cv::Point2f> points;
    for(int i = 0; i < n; i++){
        cv::Point2f point;
        point.x = pInCloud->points[i].x;
        point.y = pInCloud->points[i].y;
        points.emplace_back(point);
    }
    int iInterations = m_stCFConfig.interations;
    float fSigma = m_stCFConfig.simga;
    float fKMin = m_stCFConfig.kMin;
    float fKMax = m_stCFConfig.kMax;

    double dBestScore = -1;
    
    int bestInliersCount = 0;
    double bestA = 0, bestB = 0, bestC = 0;
    for(int i = 0; i < iInterations; i++){

        int index_1 = std::rand() % n;
        int index_2 = std::rand() % n;
        while(index_1 == index_2)
            index_2 = std::rand() % n;

        const cv::Point2f &p1 = points[index_1];
        const cv::Point2f &p2 = points[index_2];

        // 计算斜率并检查是否满足约束
        // double slope = calculateSlope(p1, p2);
        // if (std::fabs(slope - targetSlope) > slopeTolerance) {
        //     continue; // 不满足斜率约束，跳过
        // }

        // 计算直线参数 ax + by + c = 0
        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = p2.x * p1.y - p1.x * p2.y;

        // 计算内点数
        std::vector<cv::Point2f> currentInliers;
        for (const auto& p : points) {
            double dist = pointToLineDistance(p, a, b, c);
            if (dist < fSigma) {
                currentInliers.push_back(p);
            }
        }

        // 更新最佳模型
        if (currentInliers.size() > bestInliersCount) {
            bestInliersCount = currentInliers.size();
            bestA = a;
            bestB = b;
            bestC = c;

        }
    }
    
    //当斜率接近90°，值很大
    // if(bestB==){
    //     double b =
    // }
    // else{

    
        // 4. 计算拟合直线的斜率和截距
        double k = -bestA / bestB;    // 斜率
        double b = -bestC / bestB;    // 截距

        for(double y = 0; y <= 5; y += 0.01){
            double x = (y-b) / k;

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = 0; //假设平面上
            point.r = 255;
            point.g = 255;
            point.b = 255;
            curve->push_back(point);
        }  
    // }
*/

/*极坐标形式的Ransac

    LinePolar bestLine = {0, 0};
    int bestInlierCount = 0;
    double minDistanceToX0 = 1000; // 内点中直线离 x = 0 的最近距离
    int iInterations = m_stCFConfig.interations;
    double threshold = 0.5;

    std::vector<cv::Point> points;
    for(int i = 0; i < n; i++){
        cv::Point point;
        point.x = pInCloud->points[i].x;
        point.y = pInCloud->points[i].y;
        points.emplace_back(point);
    }

    std::srand(static_cast<unsigned int>(std::time(0))); // 随机种子
    for(int i = 0; i < iInterations; i++){

        int index_1 = std::rand() % n;
        int index_2 = std::rand() % n;
        while(index_1 == index_2)
            index_2 = std::rand() % n;

        // const cv::Point2f &p1 = points[index_1];
        // const cv::Point2f &p2 = points[index_2];

        // 拟合直线
        LinePolar line = fitLinePolar(points[index_1], points[index_2]);

        // 计算内点数和距离
        int inlierCount = 0;
        double avgDistanceToX0 = 0.0; // 平均离 x = 0 的距离
        for (const auto& point : points) {
            if (pointToLineDistance(point, line) < threshold) {
                ++inlierCount;
                avgDistanceToX0 += fabs(line.d); // 累加直线距离 x = 0 的距离
            }
        }

        avgDistanceToX0 /= (inlierCount > 0 ? inlierCount : 1); // 防止除零
    
        // 更新最佳直线（内点数优先，其次靠近 x = 0）
        if (inlierCount > bestInlierCount || (inlierCount == bestInlierCount && avgDistanceToX0 < minDistanceToX0)) {
            bestInlierCount = inlierCount;
            bestLine = line;
            minDistanceToX0 = avgDistanceToX0;
        }

    }

    LineKBFunction finalLine = polarToCartesian(bestLine);

    for(double y = 0; y <= 5; y += 0.01)
    // for(double y = min_y; y <= max_y; y += 0.01)
    {

        if(finalLine.k == 10000){
            

            pcl::PointXYZRGB point;
            point.x = finalLine.b;
            point.y = y;
            point.z = 0; //假设平面上
            point.r = 255;
            point.g = 255;
            point.b = 255;
            curve->push_back(point);
        }   
        else{
            double x = (y-finalLine.b) / finalLine.k;

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = 0; //假设平面上
            point.r = 255;
            point.g = 255;
            point.b = 255;
            curve->push_back(point);
        }
    }  
   */
     std::cout<<"拟合直线斜率 = "<< k <<std::endl;

    // if(B==0.0)
    //     std::cout<<"拟合直线斜率 =  " << 10000000000 <<std::endl;
    // else
    //     std::cout<<"拟合直线斜率 =  " << (-A/B) <<std::endl;
    return curve;
}




PointCloud2Intensity::Ptr CurveFitting::CurveFittingStart(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime, int iSideFlag )
{
   
    
    // 用于可视化的聚类点云
    PointCloud2RGB::Ptr colored_cloud(new PointCloud2RGB());

    std::vector<Eigen::Vector2d> points;
    for (const auto& point : pInCloud->points) {
        points.emplace_back(point.x, point.y);

        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 0;
        colored_point.g = 255;  //绿色
        colored_point.b = 0;
        colored_cloud->points.push_back(colored_point);

    }

    // //方式一：
    // // 设置拟合阶数，比如2阶表示二次曲线
    // int polynomial_order = 1;
    // Eigen::VectorXd coefficients = PolynomialCurveFit(points, polynomial_order);
    // //生成拟合直线的点
    // PointCloud2RGB::Ptr pCurveRGB = GenerateCurveRGB(coefficients, -3.0 , 0, 0.01);
    
    // 方式二： 
    float y_min = 0;
    float y_max = 2.0; 
    PointCloud2RGB::Ptr pCurveRGB = GenerateCurveRGB(pInCloud, y_min, y_max, ullTime);



    PointCloud2Intensity::Ptr pResultCurve(new PointCloud2Intensity());
    PointCloud2RGB::Ptr pResultCurveRGB(new PointCloud2RGB());

    
    //取第一个点和最后一个点
    // std::cout<<"拟合出来的最后点数"<<pCurveRGB->points.size()<<std::endl;
    for(int i = 0; i < pCurveRGB->points.size(); i++ ){
        if(i == 0)
        {
            pcl::PointXYZI temp;
            temp.x = pCurveRGB->points[i].x;
            temp.y = pCurveRGB->points[i].y;
            temp.z = pCurveRGB->points[i].z;
            temp.intensity = 255;
            pResultCurve->points.push_back(temp);
            // std::cout<<"第一个点("<<temp.x<<","<<temp.y<<")"<<std::endl;
           
        }
        
        if(i == pCurveRGB->points.size() - 1)
        {
            pcl::PointXYZI temp;
            temp.x = pCurveRGB->points[i].x;
            temp.y = pCurveRGB->points[i].y;
            temp.z = pCurveRGB->points[i].z;
            temp.intensity = 255;
            pResultCurve->points.push_back(temp);
            // std::cout<<"最后一个点("<<temp.x<<","<<temp.y<<")"<<std::endl;
           
        }

        // pcl::PointXYZI temp;
        //     temp.x = pCurveRGB->points[i].x;
        //     temp.y = pCurveRGB->points[i].y;
        //     temp.z = pCurveRGB->points[i].z;
        //     temp.intensity = 1;
        //     pResultCurve->points.push_back(temp);
       
        
    }


   return pResultCurve;
}
}