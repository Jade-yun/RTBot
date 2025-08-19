#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <array>
#include <vector>
#include "Parameters/SharedDataType.h"
#include "Utilities/SharedMemoryManager.h"
//#include "kinematics/DOF6Kinematic.h"
#include "kinematics/Classic6dofKine.h"
#include <math.h>
#include "velocityplanner/velocityplanner.h"  // 添加VelocityPlanner头文件
#include "calibrate/calibrateTCP.h"  // 添加TCP标定类头文件

#define PI (3.1415926)

class Robot
{
public:
    Robot();
    ~Robot();

    void init();

    void setEnable(bool _enabled);
    bool isEnabled() const;
    void emergecyStop();
    void homing();
    void pause();
    void resume();
    void stop();

     // TCP标定相关方法 - 通过TCPCalibrator实现
    void calibrationTCP();          // 执行TCP标定流程
    bool addTCPCalibrationPoint();  // 添加标定点
    bool calculateTCP();            // 计算TCP位置
    void clearTCPCalibrationData(); // 清除标定数据
    bool isTCPCalibrationReady();   // 检查是否可以进行标定
    std::array<float, 3> getTCPOffset() const; // 获取TCP位置偏移量
    std::array<float, 3> getTCPRotation() const; // 获取TCP姿态偏移量
    std::array<float, 6> getTCPPoseInBase() const; // 获取TCP相对于基坐标系的位姿
    bool isTCPCalibrated() const;   // 检查TCP是否已标定
    void testTCPCalibration();
    bool addManualTCPCalibrationPoint(float x, float y, float z, float a, float b, float c);

    //
    void moveJ(const std::array<float, NUM_JOINTS>& _joint_pos, float _speed, float _start_speed, float _end_speed);
    void moveL(std::array<float, NUM_JOINTS> _pose, float _speed, float _start_speed, float _end_speed);
    void moveC(std::array<float, NUM_JOINTS> mid_pose, std::array<float, NUM_JOINTS> end_pose, float speed, float start_speed, float end_speed);
    void moveCF(std::array<float, NUM_JOINTS> pose1, std::array<float, NUM_JOINTS> pose2, float speed);
    void moveJoints(const std::array<float, NUM_JOINTS>& _joints);
    void jogJ(int _mode, int _joint_index, int _direction);
    void jogL(int _mode, int _axis, int _direction);


    // B样条平滑
    // 奇异点规避
    void twoMoveL_BSplineTransition(std::array<float, NUM_JOINTS>& first_pose,std::array<float, NUM_JOINTS>& second_pose,const std::array<float, 2>& _speed, const std::array<float, 2>& _start_speed, const std::array<float, 2>& _end_speed);
    void moveBSpline( float _speed, float _start_speed, float _end_speed);
    void moveL_Avoid_points(std::array<float, NUM_JOINTS> tar_pose,  float _speed, float _start_speed, float _end_speed);
    void moveL_handle(std::array<float,6> tar_tmp, float _speed, float _start_speed, float _end_speed);
    void avoid_moveJ(const std::array<float, NUM_JOINTS>& _joint_pos, float _speed, float _start_speed, float _end_speed);
    void avoid_moveL(std::array<float, NUM_JOINTS> _pose, float _speed, float _start_speed, float _end_speed);

    // 设置速度
    void setSpeed(float _speed);
    // 调用运动学正解，更新位姿
    void updatePose();
    void updateJointStates();
    // void updateJointStatesCallback();
    bool isPoseInWorkspace(const std::array<float, 6>& pose);


    // void handleHMICommand(const HighLevelCommand& _cmd);


    void controlLoop();

//    CommandHandler m_commandHandler = CommandHandler(this);
private:
    void handleHighPriorityCommand(const HighLevelCommand &_cmd);
    void handleNormalCommand(const HighLevelCommand& cmd);
    void handleParameterOrder(const HighLevelCommand& _cmd);
    

public:
    // const std::array<float, NUM_JOINTS> REST_JOINT = {M_PI_2, 0, 0, 0, 0, 0};
    const std::array<float, NUM_JOINTS> REST_JOINT = {0, 0, M_PI_2, 0, M_PI_2, 0};
    // const std::array<float, NUM_JOINTS> REST_JOINT = {0, 0, 0, 0, 0, 0};
    // const std::array<float, NUM_JOINTS> REST_JOINT = {150.0f / 180.0f * M_PI, 45.0f / 180.0f * M_PI, 37.5f / 180.0f * M_PI, 0, 97.5f / 180.0f * M_PI, -30.0f / 180.0f * M_PI};


    const float DEFAULT_JOINT_SPEED = 20;
    // 当前关节位置
    std::array<float, NUM_JOINTS> m_curJoints = REST_JOINT;
    // std::array<float, NUM_JOINTS> m_targetJoints = REST_JOINT; 
//    std::array<float, NUM_JOINTS> m_initPose = REST_JOINT;
    std::array<float, NUM_JOINTS> m_currentPose = {0};
    
    // 当前速度、力矩
    std::array<float, NUM_JOINTS> m_curVelocity = {0};
    std::array<float, NUM_JOINTS> m_curTorque = {0};

    // 参数设置
    // 关节位置
    std::array<float, NUM_JOINTS> m_angleLimitMax;
    std::array<float, NUM_JOINTS> m_angleLimitMin;
    // 缩减关节位置
    std::array<float, NUM_JOINTS> m_ReducedJointPosMax;
    std::array<float, NUM_JOINTS> m_ReducedJointPosMin;
    // 关节速度
    std::array<float, NUM_JOINTS> m_JointSpeedMax;
//    std::array<float, NUM_JOINTS> m_speedMin;
    // 缩减关节速度
    std::array<float, NUM_JOINTS> m_ReducedJointSpeedMax;
    // 关节力矩
    std::array<float, NUM_JOINTS> m_JointTorqueMax;
    std::array<float, NUM_JOINTS> m_ReducedJointTorqueMax;
    // 关节功率
    std::array<float, NUM_JOINTS> m_JointPowerMax;
    std::array<float, NUM_JOINTS> m_ReducedJointPowerMax;

    // 轴加速度
    std::array<float, NUM_JOINTS> m_AxisAccMax;
    // 轴加加速度
    std::array<float, NUM_JOINTS> m_AxisAccAccMax;
    // 关节柔性系数
    std::array<float, NUM_JOINTS> m_JointCompliance;

    //工作空间限制
    float m_workspaceLimitMaxX = 620.0f;
    float m_workspaceLimitMinX = -620.0f;
    float m_workspaceLimitMaxY = 620.0f;
    float m_workspaceLimitMinY = -620.0f;
    float m_workspaceLimitMaxZ = 950.0f;
    float m_workspaceLimitMinZ = -290.0f;

    // 加速度倍率
    float m_AccRatio = 1;
    // 起步加速度上升时间
    float m_AccRampUpTime = 0.1f;
    // 到位减速加速度上升时间
    float m_DecRampUpTime = 0.15f;
    // 速度平滑系数
    float m_SpeedSmoothingFactor;
    // Jog模式加速度上升时间
    float m_JogAccRampUpTime = 0.2f;

    // 前瞻系数
    float m_LookAheadFactor;

    // 全局速度缩放
    float m_SpeedRatio = 1;

    // 每个关节的减速比
    std::array<float, NUM_JOINTS> m_GearRatio = {76, 76, 76, 60, 50, 50};
    // 编码器位数
    std::array<float, NUM_JOINTS> m_Encoderbit = {17, 17, 17, 17, 17,17};

    // TCP标定相关变量
    std::vector<std::array<float, 6>> m_tcpCalibrationPoses; // 存储标定点的位姿 (X,Y,Z,A,B,C)
    std::array<float, 3> m_tcpOffset = {0.0f, 0.0f, 0.0f};   // TCP位置偏移量 (相对于法兰中心)
    std::array<float, 3> m_tcpRotation = {0.0f, 0.0f, 0.0f}; // TCP姿态偏移量 (相对于法兰坐标系)
    bool m_tcpCalibrated = false;                             // TCP是否已标定

private:
    bool enabled = false;

    float m_jointSpeed = DEFAULT_JOINT_SPEED;

    // 7段S型速度规划器实例
    VelocityPlanner m_velocityPlanner;  // 主速度规划器，用于关节空间运动规划
    // TCP标定器实例
    TCPCalibrator m_tcpCalibrator;
};


#endif



