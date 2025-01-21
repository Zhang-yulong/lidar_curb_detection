
#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "time.h"
#include <chrono>

// using namespace std;
using namespace Eigen;

namespace Lidar_Curb_Dedection
{
class LineKalmanFilter {
public:
    LineKalmanFilter(double k_min,double k_max);
        

    // 设置动态时间步长 dt (根据外部数据调整)
    void setTimeStep(double dt);

    // 预测步骤
    void predict();

    // 更新步骤
    void update(const Vector2d& z);

    Vector4d getState();

    void updateTimeStep(unsigned long long currTime);

private:
    Vector4d m_x;  // 状态向量 [k, b, dot_k, dot_b]
    Matrix4d m_A;  // 状态转移矩阵
    MatrixXd m_H;  // 观测矩阵
    Matrix4d m_P;  // 协方差矩阵
    Matrix4d m_Q;  // 过程噪声矩阵
    Matrix2d m_R;  // 测量噪声矩阵

    double m_kmin;   // 负斜率的最小绝对值斜率
    double m_kmax;   // 负斜率的最大绝对值斜率

    unsigned long long m_LasterUllTime;
    // unsigned long long m_CurrentUllTime;
};
}
#endif