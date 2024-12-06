#include "kalman.h"

LineKalmanFilter::LineKalmanFilter(double k_min, double k_max) : m_kmin(k_min),m_kmax(k_max) 
{
    // 初始化状态向量 [k, b, dot_k, dot_b]
    m_x = Vector4d(0, 0, 0, 0);

    // 状态转移矩阵 A
    m_A = Matrix4d::Identity();
    m_A(0, 2) = 1.0; // k的变化率
    m_A(1, 3) = 1.0; // b的变化率

    // 观测矩阵 H (假设观测的是直线的两个点)
    m_H = MatrixXd(2, 4);
    m_H << 1, 0, 0, 0,  // y = kx + b
            0, 1, 0, 0;  // 观测截距 b

    // 初始协方差矩阵 P
    m_P = Matrix4d::Identity() * 1000;

    // 过程噪声矩阵 Q
    m_Q = Matrix4d::Identity() * 0.001;

    // 测量噪声矩阵 R
    m_R = Matrix2d::Identity() * 1;
}

// 设置动态时间步长 dt (根据外部数据调整)
void LineKalmanFilter::setTimeStep(double dt) {
    // 更新状态转移矩阵 A 中与时间相关的项
    m_A(0, 2) = dt; // k的变化率
    m_A(1, 3) = dt; // b的变化率
}

// 预测步骤
void LineKalmanFilter::predict() {
    // 预测状态
    m_x = m_A * m_x;

    // 预测协方差
    m_P = m_A * m_P * m_A.transpose() + m_Q;
}



// void LineKalmanFilter::updateTimeStep(unsigned long long currTime) {
//     // 计算当前帧与上一帧的时间差
//     // auto t_curr = std::chrono::steady_clock::now();
//     std::chrono::duration<double> elapsed = t_curr - m_LasterUllTime;

    
//     m_LasterUllTime = t_curr; // 更新上一帧时间

//     // double dt = elapsed.count(); // 时间步长（秒）
//     if (dt <= 0.0) {
//         dt = 0.001; // 防止时间步长为零或负值
//     }

//     // 更新状态转移矩阵 A 中与时间相关的项
//     m_A(0, 2) = dt; // k的变化率
//     m_A(1, 3) = dt; // b的变化率

// }


// 更新步骤
void LineKalmanFilter::update(const Vector2d& z) {
    // 计算卡尔曼增益
    MatrixXd K = m_P * m_H.transpose() * (m_H * m_P * m_H.transpose() + m_R).inverse();

    // 更新状态
    m_x = m_x + K * (z - m_H * m_x);

    // 更新协方差
    m_P = (Matrix4d::Identity() - K * m_H) * m_P;

    // 限制斜率 k 的范围
    if (m_x(0) < m_kmin) {
        m_x(0) = m_kmin;
    } 
    else if (m_x(0) > m_kmax) {
        m_x(0) = m_kmax;
    }
}

Vector4d LineKalmanFilter::getState() {
    return m_x;
}