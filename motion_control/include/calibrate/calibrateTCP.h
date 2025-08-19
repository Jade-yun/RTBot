#ifndef CALIBRATE_TCP_H
#define CALIBRATE_TCP_H

#include <array>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "kinematics/Classic6dofKine.h"

/**
 * @brief TCP标定器类
 * 
 * 该类实现了工具坐标系(TCP)的四点标定算法，用于计算工具相对于机器人法兰的位置和姿态偏移。
 * 四点标定法通过让TCP尖端接触同一个固定参考点的四个不同姿态来求解TCP偏移量。
 */
class TCPCalibrator
{
public:
    /**
     * @brief 构造函数
     */
    TCPCalibrator();
    
    /**
     * @brief 析构函数
     */
    ~TCPCalibrator();
    
    /**
     * @brief 开始TCP标定流程
     * 
     * 初始化标定过程，清除之前的标定数据，并提示用户操作步骤
     */
    void startCalibration();
    
    /**
     * @brief 添加标定点
     * @param pose 当前法兰位姿 [X, Y, Z, A, B, C]
     * @return true 如果已收集足够的标定点
     * @return false 如果还需要更多标定点
     */
    bool addCalibrationPoint(const std::array<float, 6>& pose);
    
    /**
     * @brief 手动添加标定点（用于测试）
     * @param x 位置X坐标
     * @param y 位置Y坐标
     * @param z 位置Z坐标
     * @param a 姿态A角度
     * @param b 姿态B角度
     * @param c 姿态C角度
     * @return true 如果已收集足够的标定点
     * @return false 如果还需要更多标定点
     */
    bool addManualCalibrationPoint(float x, float y, float z, float a, float b, float c);
    
    /**
     * @brief 计算TCP位置和姿态偏移
     * 
     * 使用四点法算法计算TCP相对于法兰坐标系的位置偏移
     * @return true 标定成功
     * @return false 标定失败
     */
    bool calculateTCP();
    
    /**
     * @brief 清除标定数据
     * 
     * 清除所有已收集的标定点和计算结果
     */
    void clearCalibrationData();
    
    /**
     * @brief 检查是否准备好进行标定
     * @return true 如果已收集足够的标定点（>=4个）
     * @return false 如果标定点不足
     */
    bool isCalibrationReady() const;
    
    /**
     * @brief 获取TCP位置偏移量
     * @return std::array<float, 3> TCP相对于法兰的位置偏移 [X, Y, Z]
     */
    std::array<float, 3> getTCPOffset() const;
    
    /**
     * @brief 获取TCP姿态偏移量
     * @return std::array<float, 3> TCP相对于法兰的姿态偏移 [A, B, C]
     */
    std::array<float, 3> getTCPRotation() const;
    
    /**
     * @brief 计算TCP在基坐标系下的位姿
     * @param current_joints 当前关节角度
     * @return std::array<float, 6> TCP在基坐标系下的位姿 [X, Y, Z, A, B, C]
     */
    std::array<float, 6> getTCPPoseInBase(const std::array<float, 6>& current_joints) const;
    
    /**
     * @brief 检查TCP是否已标定
     * @return true TCP已标定
     * @return false TCP未标定
     */
    bool isCalibrated() const;
    
    /**
     * @brief 获取标定点数量
     * @return size_t 当前已收集的标定点数量
     */
    size_t getCalibrationPointCount() const;
    
    /**
     * @brief 获取标定精度（标准差）
     * @return float 标定精度，单位：mm
     */
    float getCalibrationAccuracy() const;
    
    /**
     * @brief TCP标定测试程序
     * 
     * 提供交互式界面用于测试TCP标定功能
     */
    void runTestProgram();
    
private:
    std::vector<std::array<float, 6>> m_calibrationPoses;  ///< 存储标定点的位姿 (X,Y,Z,A,B,C)
    std::array<float, 3> m_tcpOffset;                      ///< TCP位置偏移量 (相对于法兰中心)
    std::array<float, 3> m_tcpRotation;                    ///< TCP姿态偏移量 (相对于法兰坐标系)
    bool m_isCalibrated;                                   ///< TCP是否已标定
    float m_calibrationAccuracy;                           ///< 标定精度（标准差）
    
    /**
     * @brief 构建旋转矩阵
     * @param euler_angles 欧拉角 [A, B, C]
     * @return Eigen::Matrix3f 旋转矩阵
     */
    Eigen::Matrix3f buildRotationMatrix(const std::array<float, 3>& euler_angles) const;
    
    /**
     * @brief 计算标定精度
     * @param tcp_local TCP在法兰坐标系下的位置
     * @return float 标定精度（标准差），单位：mm
     */
    float calculateAccuracy(const Eigen::Vector3f& tcp_local) const;
};

#endif // CALIBRATE_TCP_H