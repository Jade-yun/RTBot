#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <array>
#include "Parameters/SharedDataType.h"
#include "Utilities/SharedMemoryManager.h"
//#include "kinematics/DOF6Kinematic.h"
#include "kinematics/Classic6dofKine.h"

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
    
    // 
    //void planMoveJ(const std::array<float, NUM_JOINTS>& _joint_pos);
    void moveJ(const std::array<float, NUM_JOINTS>& _joint_pos, float _speed);
    void moveL(std::array<float, NUM_JOINTS> _pose, float _speed);
    void moveC(std::array<float, NUM_JOINTS> mid_pose, std::array<float, NUM_JOINTS> end_pose, float speed);
    void moveCF(std::array<float, NUM_JOINTS> pose1, std::array<float, NUM_JOINTS> pose2, float speed);
    void moveJoints(const std::array<float, NUM_JOINTS>& _joints);
    void jogJ(int _mode, int _joint_index, int _direction);
    void jogL(int _mode, int _axis, int _direction);
    // 设置速度
    // void setSpeed(float _speed);
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
    const std::array<float, NUM_JOINTS> REST_JOINT = {0, 0, 0, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 10;
    // 当前关节位置
    std::array<float, NUM_JOINTS> m_curJoints = REST_JOINT;
    std::array<float, NUM_JOINTS> m_targetJoints = REST_JOINT;
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
    float m_workspaceLimitMaxX = 850.0f;
    float m_workspaceLimitMinX = -850.0f;
    float m_workspaceLimitMaxY = 850.0f;
    float m_workspaceLimitMinY = -850.0f;
    float m_workspaceLimitMaxZ = 900.0f;
    float m_workspaceLimitMinZ = -700.0f;

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
    std::array<float, NUM_JOINTS> m_GearRatio = {10, 10, 10, 10, 10, 10};
    // 编码器位数
    std::array<float, NUM_JOINTS> m_Encoderbit;

private:
    bool enabled = false;

    float m_jointSpeed = DEFAULT_JOINT_SPEED;
    //float m_cartesianSpeed = 10.0f; 
//    std::array<float, NUM_JOINTS> m_dynamicJointSpeeds = {0};

//    DOF6Kinematic* dof6Solver;

};


#endif
