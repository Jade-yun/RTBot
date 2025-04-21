// shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <cstdint>
#include <atomic>

#define NUM_JOINTS 6
#define MAX_TRAJECTORY_POINTS 500  // 支持的最大轨迹点数

// 控制命令类型
enum class ControlMode : uint8_t {
    IDLE = 0,
    JOG = 1,
    MOVE_JOINT = 2,
    MOVE_CARTESIAN = 3,
    PLAY_TRAJECTORY = 4,
    STOP = 5,
    RESET_FAULT = 6,
};

// 控制器状态
enum class ControllerState : uint8_t {
    INIT = 0,
    READY = 1,
    RUNNING = 2,
    ERROR = 3,
    ESTOP = 4,
};

// 关节状态反馈（周期更新，HMI 读取）
struct JointState {
    double position;   // 当前关节位置 [rad]
    double velocity;   // 当前速度 [rad/s]
    double torque;     // 电机反馈转矩 [Nm]
    uint16_t status_word; // EtherCAT 状态字
};

// 单个轨迹点结构
struct TrajectoryPoint {
    double joint_pos[NUM_JOINTS];  // 每个关节的位置
    double duration;               // 从上一个点到这个点的时间（秒）
};

// 控制器状态反馈（周期更新，HMI 读取）
struct RtState {
    JointState joints[NUM_JOINTS];
    double tcp_pose[6];  // 末端位姿 XYZ+RPY
    ControllerState controller_state;
    ControlMode current_mode;
    bool trajectory_running;
    uint32_t heartbeat_counter;  // 控制器周期+1，Qt定时读取用于判断控制模块是否活着
    char message[128];           // 当前状态信息（可用于报警或提示）
};

// 控制命令结构体（HMI 写入，motion_rt 读取）
struct RtCommand {
    std::atomic<bool> new_cmd;  // HMI 写入后置 true，motion_rt 处理后置 false

    ControlMode mode;           // 控制模式（Jog/Move等）
    double target_joint_pos[NUM_JOINTS];  // 目标关节位置
    double target_tcp_pose[6];            // 目标末端位姿（XYZ+RPY）
    double jog_step[NUM_JOINTS];          // Jog 步长
    double velocity_scale;                // 速度缩放
    double acceleration_scale;            // 加速度缩放

    // 多段轨迹命令缓冲区（适用于轨迹播放）
    uint32_t trajectory_point_count;               // 轨迹点数量（由 Qt 写入）
    TrajectoryPoint trajectory_buffer[MAX_TRAJECTORY_POINTS];  // 轨迹数据
};

// 总体共享数据结构
struct SharedData {
    RtState state;     // 实时状态（由 motion_rt 周期更新）
    RtCommand command; // 控制命令（由 HMI 写入）
};

#endif
