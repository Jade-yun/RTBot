#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <array>
#include "Parameters/SharedDataType.h"
#include "Utilities/SharedMemoryManager.h"
#include "CommandHandler.h"

#define PI (3.1415926)

class Robot
{
public:
    void init();

    void setEnable(bool _enabled);
    bool isEnabled() const;
    bool isMoving();
    void emergecyStop();

    // 
    void planMoveJ(const std::array<float, NUM_JOINTS>& _joint_pos, float _arg_vel);
    void moveJ(const std::array<float, NUM_JOINTS>& _joint_pos);
    void moveL(std::array<float, NUM_JOINTS> _pose);

    // 设置速度
    void setSpeed(float _speed);
    // 调用运动学正解，更新位姿
    void updatePose();
    void updateJointStates();

    // void handleHMICommand(const HighLevelCommand& _cmd);


    void controlLoop();

//    CommandHandler m_commandHandler = CommandHandler(this);


public:
    const std::array<float, NUM_JOINTS> REST_POSE = {0, -73, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 30;
    // 当前关节位置
    std::array<float, NUM_JOINTS> m_curJoints = REST_POSE;
    std::array<float, NUM_JOINTS> m_targetJoints = REST_POSE;
//    std::array<float, NUM_JOINTS> m_initPose = REST_POSE;
    std::array<float, NUM_JOINTS> m_currentPose = {0};

private:
    bool enabled = false;

    float m_jointSpeed = DEFAULT_JOINT_SPEED;
    float m_jointSpeedRatio = 1;
    std::array<float, NUM_JOINTS> m_dynamicJointSpeeds = {0};

};


#endif
