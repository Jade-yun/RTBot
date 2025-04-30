#ifndef _GLOBAL_PARAMETERS_H
#define _GLOBAL_PARAMETERS_H

#include <atomic>
#include <array>
#include <boost/lockfree/spsc_queue.hpp>
#include "SharedDataType.h"

class GlobalParams
{
public:

enum class MotorMode : uint8_t {
    CSP = 0,
    CSV,
    CST
};

struct LowLevelCommand { 
    MotorMode mode;
    union 
    {
        float motor_pos[NUM_JOINTS];  // 每个关节的目标位置
        float motor_velocity[NUM_JOINTS]; // 每个关节的目标速度
        float motor_torque[NUM_JOINTS]; // 每个关节的目标力矩
    };
};
struct LowLevelState { 
    float position;      // 当前关节位置
    float velocity;      // 当前速度
    float torque;        // 电机反馈转矩 [Nm]
    uint16_t ecat_status_word; // EtherCAT 状态字
};

public:

    // 缓存插补轨迹
    static boost::lockfree::spsc_queue<TrajectoryPoint,
        boost::lockfree::capacity<MAX_TRAJECTORY_POINTS>> traj_buffer;

    // 控制器发送 ethercat读取发给电机驱动器
    static boost::lockfree::spsc_queue<LowLevelCommand,
        boost::lockfree::capacity<256>> requst_cmd;
    // 关节参数
    static std::array<std::atomic<LowLevelState>, NUM_JOINTS> joint_state;

};

#endif