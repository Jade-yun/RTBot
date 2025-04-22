// shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <cstdint>
#include <atomic>

#define NUM_JOINTS 6
#define MAX_TRAJECTORY_POINTS 500  // 支持的最大轨迹点数
#define MAX_TRAJECTORY_LIST 8
#define MAX_CMD_QUEUE_LEN 32
#define MAX_TRAJECTORY_SEGMENT 50 // 最大的连续轨迹段数

// 控制命令类型
enum class ControlMode : uint8_t {
    STOP = 0,
    IDLE,
    JOG,                // 手动移动
    MOVE_JOINT,         // 关节空间PTP
    MOVE_CARTESIAN,     // 笛卡尔轨迹
    MULTI_TRAJECTORY,   // 多段笛卡尔轨迹
    PLAY_TRAJECTORY,    // 关节空间轨迹拟合
    RESET_FAULT,
};

// 控制器状态
enum class ControllerState : uint8_t {
    INIT = 0,
    READY = 1,
    RUNNING = 2,
    ERROR = 3,
    ESTOP = 4,
};

//笛卡尔轨迹类型定义
enum CartesianTrajectoryType {
    LINEAR_MOTION = 1,
    CIRCULAR_MOTION,
    CURVILINEAR_MOTION,
};

// 机械结构参数
struct STCartesian {
    double length_A[NUM_JOINTS];  // 连杆长度
    double length_D[NUM_JOINTS];  // 连杆偏距
};

//关节参数
struct JointParameter {
    double Joint_limit[NUM_JOINTS];
    double Reduction_ratio[NUM_JOINTS]; //减速比
    double Rated_speed[NUM_JOINTS]; //额定转速
    double Max_acceleration[NUM_JOINTS]; //最大加速度
    double Max_deceleration[NUM_JOINTS]; //最大减速度
    uint8_t Encoder_bit[NUM_JOINTS]; //轴编码器位数
};

//笛卡尔运行参数
struct CartesianParameter {
    double Cartesian_Max_Speed;
    double Cartesian_acceleration;
    double Cartesian_deceleration;
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

// 轨迹拟合数据点
struct Trajectory {
    uint32_t point_count;
    TrajectoryPoint points[MAX_TRAJECTORY_POINTS];
};

// 多段轨迹数据点
struct MultiTrajectory {
    uint16_t cartesion_segment_num; // 轨迹段数
    double traj_point[MAX_TRAJECTORY_SEGMENT * 3];            // 每段轨迹的所需位置
    CartesianTrajectoryType traj_type[MAX_TRAJECTORY_SEGMENT];   // 每段轨迹的类型
};

// 控制器状态反馈（周期更新，HMI 读取）
struct RtState {
    JointState joints[NUM_JOINTS];
    double tcp_pose[6];  // 末端位姿 XYZ+RPY
    ControllerState controller_state;
    ControlMode current_mode;
    uint32_t heartbeat_counter;  // 控制器周期+1，Qt定时读取用于判断控制模块是否活着
};

// 控制命令结构体（HMI 写入，motion_rt 读取）
struct RtCommand {
    uint32_t command_id;
    // uint32_t param_mask;
    ControlMode mode;                     // 控制模式（Jog/Move等）
    double target_joint_pos[NUM_JOINTS];  // 目标关节位置
    double target_tcp_pose[6];            // 目标末端位姿（XYZ+RPY）
    double velocity_scale;                // 速度缩放
    double acceleration_scale;            // 加速度缩放

    // 对 trajectory_pool / cartesian_trajectory 的索引
    int16_t trajectory_index = -1; // -1 表示无轨迹
};

// 控制命令队列（生产者：HMI，消费者：RT）
class CommandQueue {
    std::atomic<uint32_t> head{0}; // 写指针（HMI 写）
    std::atomic<uint32_t> tail{0}; // 读指针（RT 读）

public:
    RtCommand buffer[MAX_CMD_QUEUE_LEN];

    bool push(const RtCommand& cmd) {
        uint32_t h = head.load(std::memory_order_relaxed);
        uint32_t t = tail.load(std::memory_order_acquire);
        if ((h + 1) % MAX_CMD_QUEUE_LEN == t) {
            return false; // 队列满
        }
        buffer[h] = cmd;
        head.store((h + 1) % MAX_CMD_QUEUE_LEN, std::memory_order_release);
        return true;
    }

    bool pop(RtCommand& out) {
        uint32_t h = head.load(std::memory_order_acquire);
        uint32_t t = tail.load(std::memory_order_relaxed);
        if (t == h) {
            return false; // 队列空
        }
        out = buffer[t];
        tail.store((t + 1) % MAX_CMD_QUEUE_LEN, std::memory_order_release);
        return true;
    }
};

class StateBuffer {
    std::atomic<uint32_t> write_flag{0}; // 控制模块写入后更新
    std::atomic<uint32_t> read_flag{0};  // HMI 读取后更新

public:
    RtState buffer[2];

    // 控制模块写入
    void write(const RtState& s) {
        uint32_t idx = write_flag.load(std::memory_order_relaxed) % 2;
        buffer[idx] = s;

        write_flag.fetch_add(1, std::memory_order_release);
    }

    // HMI 读取
    bool read(RtState& s) {
        uint32_t w = write_flag.load(std::memory_order_acquire);
        if (read_flag.load(std::memory_order_relaxed) == w) {
            return false; // 无新数据
        }

        uint32_t idx = (w - 1) % 2;
        s = buffer[idx];
        read_flag.store(w, std::memory_order_relaxed);
        return true;
    }
};

// 总体共享数据结构
struct SharedData {
    CommandQueue cmd_queue; 
    StateBuffer state_buffer;

    // HMI 填写轨迹，RT 消费；可支持多个命令轨迹并发
    // 指向轨迹池中使用的哪一段轨迹（由命令指定）
    Trajectory trajectory_pool[MAX_TRAJECTORY_LIST];
    // 多连续段集合
    MultiTrajectory cartesian_trajectory[MAX_TRAJECTORY_LIST];
};

#endif
