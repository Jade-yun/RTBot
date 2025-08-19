#include "calibrate/calibrateTCP.h"
#include <cmath>
#include <stdexcept>
#include <iomanip>

TCPCalibrator::TCPCalibrator()
    : m_tcpOffset({0.0f, 0.0f, 0.0f})
    , m_tcpRotation({0.0f, 0.0f, 0.0f})
    , m_isCalibrated(false)
    , m_calibrationAccuracy(0.0f)
{
    m_calibrationPoses.clear();
}

TCPCalibrator::~TCPCalibrator()
{
}

void TCPCalibrator::startCalibration()
{
    std::cout << "\n=== TCP四点标定开始 ===" << std::endl;
    std::cout << "TCP标定需要4个不同姿态的标定点" << std::endl;
    std::cout << "操作步骤:" << std::endl;
    std::cout << "  1. 将TCP尖端精确接触固定参考点" << std::endl;
    std::cout << "  2. 调用addCalibrationPoint()添加当前位姿" << std::endl;
    std::cout << "  3. 改变机器人姿态，重复步骤1-2，直到收集4个点" << std::endl;
    std::cout << "  4. 调用calculateTCP()计算TCP偏移量" << std::endl;
    std::cout << "  注意: 四个姿态应该尽可能不同，以提高标定精度" << std::endl;
    std::cout << "请按顺序调用addCalibrationPoint()添加标定点" << std::endl;

    // 清除之前的标定数据
    clearCalibrationData();
}

bool TCPCalibrator::addCalibrationPoint(const std::array<float, 6>& pose)
{
    // 将当前法兰位姿添加到标定点列表
    m_calibrationPoses.push_back(pose);

    std::cout << "添加标定点 " << m_calibrationPoses.size()
              << ": 位置(" << pose[0] << ", " << pose[1] << ", " << pose[2] << ") "
              << "姿态(" << pose[3] << ", " << pose[4] << ", " << pose[5] << ")" << std::endl;

    // 如果已经有足够的点,提示可以进行标定
    if (m_calibrationPoses.size() >= 4) {
        std::cout << "已收集到 " << m_calibrationPoses.size() << " 个标定点,可以调用calculateTCP()进行标定" << std::endl;
        return true;
    } else {
        std::cout << "还需要 " << (4 - m_calibrationPoses.size()) << " 个标定点" << std::endl;
        return false;
    }
}

bool TCPCalibrator::addManualCalibrationPoint(float x, float y, float z, float a, float b, float c)
{
    // 手动添加标定点（用于测试）
    std::array<float, 6> pose = {x, y, z, a, b, c};
    return addCalibrationPoint(pose);
}

bool TCPCalibrator::calculateTCP()
{
    if (m_calibrationPoses.size() < 4) {
        std::cerr << "标定点不足,需要4个点,当前只有 " << m_calibrationPoses.size() << " 个点" << std::endl;
        return false;
    }

    // 四点法TCP标定算法
    // 四个不同姿态下TCP尖端接触同一个固定参考点
    // 通过求解超定方程组得到TCP在法兰坐标系下的位置偏移

    try {
        // 构建线性方程组 A * tcp_pos = b
        // 对于每两个姿态,有约束: flange_pos_i + R_i * tcp_local = flange_pos_j + R_j * tcp_local
        // 即: (R_i - R_j) * tcp_local = flange_pos_j - flange_pos_i
        
        int num_equations = 0;
        // 计算方程数量: C(4,2) * 3 = 6 * 3 = 18个方程
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                num_equations += 3;
            }
        }
        
        Eigen::MatrixXf A(num_equations, 3);
        Eigen::VectorXf b(num_equations);
        
        int eq_idx = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                // 构建旋转矩阵
                std::array<float, 3> euler_i = {m_calibrationPoses[i][3], m_calibrationPoses[i][4], m_calibrationPoses[i][5]};
                std::array<float, 3> euler_j = {m_calibrationPoses[j][3], m_calibrationPoses[j][4], m_calibrationPoses[j][5]};
                
                Eigen::Matrix3f R_i = buildRotationMatrix(euler_i);
                Eigen::Matrix3f R_j = buildRotationMatrix(euler_j);
                
                Eigen::Vector3f flange_pos_i(m_calibrationPoses[i][0],
                                             m_calibrationPoses[i][1],
                                             m_calibrationPoses[i][2]);
                
                Eigen::Vector3f flange_pos_j(m_calibrationPoses[j][0],
                                             m_calibrationPoses[j][1],
                                             m_calibrationPoses[j][2]);
                
                // (R_i - R_j) * tcp_local = flange_pos_j - flange_pos_i
                Eigen::Matrix3f diff_R = R_i - R_j;
                Eigen::Vector3f diff_pos = flange_pos_j - flange_pos_i;
                
                A.block<3,3>(eq_idx, 0) = diff_R;
                b.segment<3>(eq_idx) = diff_pos;
                eq_idx += 3;
            }
        }
        
        // 使用最小二乘法求解TCP在法兰坐标系下的位置
        Eigen::Vector3f tcp_local = A.colPivHouseholderQr().solve(b);
        
        m_tcpOffset[0] = tcp_local[0];
        m_tcpOffset[1] = tcp_local[1];
        m_tcpOffset[2] = tcp_local[2];
        
        // 四点法只标定位置偏移,姿态偏移设为零
        m_tcpRotation[0] = 0.0f;
        m_tcpRotation[1] = 0.0f;
        m_tcpRotation[2] = 0.0f;
        
        // 计算标定精度
        m_calibrationAccuracy = calculateAccuracy(tcp_local);
        
        m_isCalibrated = true;
        
        std::cout << "TCP四点法标定完成!" << std::endl;
        std::cout << "TCP位置偏移: (" << m_tcpOffset[0] << ", " << m_tcpOffset[1] << ", " << m_tcpOffset[2] << ") mm" << std::endl;
        std::cout << "TCP姿态偏移: (" << m_tcpRotation[0] << ", " << m_tcpRotation[1] << ", " << m_tcpRotation[2] << ") rad" << std::endl;
        std::cout << "标定精度 (标准差): " << m_calibrationAccuracy << " mm" << std::endl;
        
        if (m_calibrationAccuracy > 1.0f) {
            std::cout << "警告: 标定精度较低,建议重新标定或检查标定点质量" << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "TCP标定过程中发生错误: " << e.what() << std::endl;
        return false;
    }
}

void TCPCalibrator::clearCalibrationData()
{
    m_calibrationPoses.clear();
    m_tcpOffset = {0.0f, 0.0f, 0.0f};
    m_tcpRotation = {0.0f, 0.0f, 0.0f};
    m_isCalibrated = false;
    m_calibrationAccuracy = 0.0f;
    std::cout << "TCP标定数据已清除" << std::endl;
}

bool TCPCalibrator::isCalibrationReady() const
{
    return m_calibrationPoses.size() >= 4;
}

std::array<float, 3> TCPCalibrator::getTCPOffset() const
{
    return m_tcpOffset;
}

std::array<float, 3> TCPCalibrator::getTCPRotation() const
{
    return m_tcpRotation;
}

std::array<float, 6> TCPCalibrator::getTCPPoseInBase(const std::array<float, 6>& current_joints) const
{
    std::array<float, 6> tcp_pose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    if (!m_isCalibrated) {
        std::cerr << "警告: TCP未标定,返回法兰位姿" << std::endl;
        // 如果TCP未标定,返回法兰位姿
        Kine6d flange_pose;
        std::array<float, 6> joints_copy = current_joints;
        classic6dofForKine(joints_copy.data(), &flange_pose);
        tcp_pose[0] = flange_pose.X;
        tcp_pose[1] = flange_pose.Y;
        tcp_pose[2] = flange_pose.Z;
        tcp_pose[3] = flange_pose.A;
        tcp_pose[4] = flange_pose.B;
        tcp_pose[5] = flange_pose.C;
        return tcp_pose;
    }

    // 获取当前法兰位姿
    Kine6d flange_pose;
    std::array<float, 6> joints_copy = current_joints;
    classic6dofForKine(joints_copy.data(), &flange_pose);

    // 构建法兰到基坐标系的变换矩阵
    Eigen::Matrix4f T_base_flange = Eigen::Matrix4f::Identity();

    // 从欧拉角构建旋转矩阵 (ZYX顺序)
    std::array<float, 3> euler = {flange_pose.A, flange_pose.B, flange_pose.C};
    Eigen::Matrix3f R_base_flange = buildRotationMatrix(euler);

    T_base_flange.block<3,3>(0,0) = R_base_flange;
    T_base_flange.block<3,1>(0,3) = Eigen::Vector3f(flange_pose.X, flange_pose.Y, flange_pose.Z);

    // 四点标定法只标定TCP位置，不标定姿态
    // 因此TCP坐标系与法兰坐标系姿态相同，只有位置偏移
    Eigen::Vector3f tcp_offset(m_tcpOffset[0], m_tcpOffset[1], m_tcpOffset[2]);
    
    // 计算TCP在基坐标系下的位置
    Eigen::Vector3f tcp_position_base = T_base_flange.block<3,1>(0,3) + R_base_flange * tcp_offset;
    
    // TCP位置
    tcp_pose[0] = tcp_position_base[0];
    tcp_pose[1] = tcp_position_base[1];
    tcp_pose[2] = tcp_position_base[2];
    
    // TCP姿态与法兰姿态相同（四点标定法不标定姿态）
    tcp_pose[3] = flange_pose.A;
    tcp_pose[4] = flange_pose.B;
    tcp_pose[5] = flange_pose.C;

    return tcp_pose;
}

bool TCPCalibrator::isCalibrated() const
{
    return m_isCalibrated;
}

size_t TCPCalibrator::getCalibrationPointCount() const
{
    return m_calibrationPoses.size();
}

float TCPCalibrator::getCalibrationAccuracy() const
{
    return m_calibrationAccuracy;
}

void TCPCalibrator::runTestProgram()
{
    int choice;
    bool running = true;
    
    std::cout << "\n=== TCP四点标定测试程序 ===" << std::endl;
    
    while (running) {
        std::cout << "\n请选择操作：" << std::endl;
        std::cout << "1. 开始TCP标定 (startCalibration)" << std::endl;
        std::cout << "2. 添加标定点 (addCalibrationPoint)" << std::endl;
        std::cout << "3. 计算工具坐标系 (calculateTCP)" << std::endl;
        std::cout << "4. 清除标定数据 (clearCalibrationData)" << std::endl;
        std::cout << "5. 输出TCP位置偏移 (getTCPOffset)" << std::endl;
        std::cout << "6. 输出标定精度 (getCalibrationAccuracy)" << std::endl;
        std::cout << "0. 退出测试" << std::endl;
        std::cout << "请输入选择 (0-6): ";
        
        std::cin >> choice;
        
        switch (choice) {
            case 1: {
                std::cout << "\n=== 开始TCP标定 ===" << std::endl;
                startCalibration();
                break;
            }
            
            case 2: {
                std::cout << "\n=== 添加标定点 ===" << std::endl;
                std::cout << "当前已有 " << m_calibrationPoses.size() << " 个标定点" << std::endl;
                
                if (m_calibrationPoses.size() >= 4) {
                    std::cout << "已有足够的标定点，无需继续添加" << std::endl;
                    break;
                }
                
                std::cout << "请输入第 " << (m_calibrationPoses.size() + 1) << " 个标定点的法兰位姿：" << std::endl;
                std::cout << "X Y Z A B C: ";
                
                float x, y, z, a, b, c;
                std::cin >> x >> y >> z >> a >> b >> c;
                
                addManualCalibrationPoint(x, y, z, a, b, c);
                break;
            }
            
            case 3: {
                std::cout << "\n=== 计算工具坐标系 ===" << std::endl;
                if (calculateTCP()) {
                    std::cout << "TCP标定计算成功！" << std::endl;
                } else {
                    std::cout << "TCP标定计算失败！" << std::endl;
                }
                break;
            }
            
            case 4: {
                std::cout << "\n=== 清除标定数据 ===" << std::endl;
                clearCalibrationData();
                break;
            }
            
            case 5: {
                std::cout << "\n=== TCP位置偏移 ===" << std::endl;
                if (m_isCalibrated) {
                    std::array<float, 3> offset = getTCPOffset();
                    std::cout << "TCP相对于法兰坐标系的位置偏移：" << std::endl;
                    std::cout << "X: " << std::fixed << std::setprecision(3) << offset[0] << " mm" << std::endl;
                    std::cout << "Y: " << std::fixed << std::setprecision(3) << offset[1] << " mm" << std::endl;
                    std::cout << "Z: " << std::fixed << std::setprecision(3) << offset[2] << " mm" << std::endl;
                } else {
                    std::cout << "TCP未标定，无法获取偏移量" << std::endl;
                }
                break;
            }
            
            case 6: {
                std::cout << "\n=== 标定精度 ===" << std::endl;
                if (m_isCalibrated) {
                    std::cout << "标定精度（标准差）: " << std::fixed << std::setprecision(3) 
                              << getCalibrationAccuracy() << " mm" << std::endl;
                } else {
                    std::cout << "TCP未标定，无法获取精度信息" << std::endl;
                }
                break;
            }
            
            case 0: {
                std::cout << "退出TCP标定测试程序" << std::endl;
                running = false;
                break;
            }
            
            default: {
                std::cout << "无效选择，请输入0-6之间的数字" << std::endl;
                break;
            }
        }
    }
}

Eigen::Matrix3f TCPCalibrator::buildRotationMatrix(const std::array<float, 3>& euler_angles) const
{
    return (Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX())).toRotationMatrix();
}

float TCPCalibrator::calculateAccuracy(const Eigen::Vector3f& tcp_local) const
{
    std::vector<float> errors;
    
    // 计算参考点位置（使用第一个姿态）
    std::array<float, 3> euler_0 = {m_calibrationPoses[0][3], m_calibrationPoses[0][4], m_calibrationPoses[0][5]};
    Eigen::Matrix3f R_0 = buildRotationMatrix(euler_0);
    
    Eigen::Vector3f flange_pos_0(m_calibrationPoses[0][0],
                                 m_calibrationPoses[0][1],
                                 m_calibrationPoses[0][2]);
    
    Eigen::Vector3f reference_point = flange_pos_0 + R_0 * tcp_local;
    
    for (size_t i = 0; i < 4; i++) {
        std::array<float, 3> euler_i = {m_calibrationPoses[i][3], m_calibrationPoses[i][4], m_calibrationPoses[i][5]};
        Eigen::Matrix3f R_i = buildRotationMatrix(euler_i);
        
        Eigen::Vector3f flange_pos_i(m_calibrationPoses[i][0],
                                     m_calibrationPoses[i][1],
                                     m_calibrationPoses[i][2]);
        
        Eigen::Vector3f tcp_world_i = flange_pos_i + R_i * tcp_local;
        float error = (tcp_world_i - reference_point).norm();
        errors.push_back(error);
    }
    
    // 计算标准差
    float mean_error = 0.0f;
    for (float error : errors) {
        mean_error += error;
    }
    mean_error /= errors.size();
    
    float std_dev = 0.0f;
    for (float error : errors) {
        std_dev += std::pow(error - mean_error, 2);
    }
    std_dev = std::sqrt(std_dev / errors.size());
    
    return std_dev;
}