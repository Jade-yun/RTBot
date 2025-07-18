// SharedData.h
#ifndef SHARED_DATATYPE_H
#define SHARED_DATATYPE_H

#include <cstdint>
#include <atomic>
#include <array>
#include <cstddef>

#define NUM_JOINTS 6
#define MAX_TRAJECTORY_POINTS 512 // 支持的最大轨迹点数
#define MAX_TRAJECTORY_LIST 8
#define MAX_CMD_QUEUE_LEN 32      // 命令队列长度 最好为 2 的幂
#define MAX_TRAJECTORY_SEGMENT 50 // 最大的连续轨迹段数


enum class HighLevelCommandType : uint8_t {
    Stop = 0, // 停止（减速停止）
    Homing,   // 回零（Homing）
    Pause,    // 暂停
    Resume,   // 继续
    MoveJ,    // 关节空间插补运动到目标关节角度（Joint Space Move）
    MoveL,    // 笛卡尔空间直线轨迹（Linear Move in Cartesian Space）
    MoveC,    // 圆弧插补移动（Circular Move）
    MoveP,    // 多点轨迹（Move Path），经过一系列途经点的关节轨迹运动
    PlayTraj, // 播放轨迹
    Hold,     // 保持当前位置（Hold Current Position）
    ServoJ,   // 关节伺服模式（Servo Joint）
    ServoL,   // 笛卡尔伺服模式（Servo Cartesian）
    SetIO,    // IO控制指令（Set IO）：设置数字量IO，例如打开夹爪
    SetParm,  // 设置参数
    Wait,     // 等待（Wait）
    JogJ,     // 关节点动（Jog Joint）
    JogL,     // 笛卡尔点动（Jog Linear）
};

// 机械结构参数
struct STCartesian
{
    double length_A[NUM_JOINTS]; // 连杆长度
    double length_D[NUM_JOINTS]; // 连杆偏距
};

// 关节参数
struct JointParameter
{
    double Joint_limit[NUM_JOINTS];
    double Reduction_ratio[NUM_JOINTS];  // 减速比
    double Rated_speed[NUM_JOINTS];      // 额定转速
    double Max_acceleration[NUM_JOINTS]; // 最大加速度
    double Max_deceleration[NUM_JOINTS]; // 最大减速度
    uint8_t Encoder_bit[NUM_JOINTS];     // 轴编码器位数
};

// 笛卡尔运行参数
struct CartesianParameter
{
    double Cartesian_Max_Speed;
    double Cartesian_acceleration;
    double Cartesian_deceleration;
};

// 单个轨迹点结构
struct TrajectoryPoint
{
    float joint_pos[NUM_JOINTS]; // 每个关节的位置
    float duration;              // 从上一个点到这个点的时间（秒）
};

// 轨迹拟合数据点
struct Trajectory
{
    uint32_t point_count;
    TrajectoryPoint points[MAX_TRAJECTORY_POINTS];
};

// 笛卡尔轨迹类型定义
enum CartesianTrajectoryType
{
    LINEAR_MOTION = 1,
    CIRCULAR_MOTION,
    CURVILINEAR_MOTION,
};
// 多段轨迹数据点
struct MultiTrajectory
{
    uint16_t cartesion_segment_num;                            // 轨迹段数
    float traj_point[MAX_TRAJECTORY_SEGMENT * 3];             // 每段轨迹的所需位置
    CartesianTrajectoryType traj_type[MAX_TRAJECTORY_SEGMENT]; // 每段轨迹的类型
};

// 控制命令结构体（HMI 写入，motion_rt 读取）
struct HighLevelCommand {
    HighLevelCommandType command_type;  // 指令类型
    uint8_t reserved[3];                // 对齐 padding，使结构体按4字节对齐
    uint32_t command_index; // 指令编号
    union {
        // 关节空间移动
        struct {
            float target_joint_pos[NUM_JOINTS]; // 各关节角度
            float velocity;
            float acceleration;
        } movej_params;

        // 笛卡尔空间移动
        struct {
            float target_pose[6]; // 末端位姿
            float velocity;
            float acceleration;
        } movel_params;

        // 圆弧插补移动
        struct {
            float via_pose[6]; // 中间点位姿
            float target_pose[6]; // 末端位姿
            float velocity;
            float acceleration;
        } movec_params;

        struct {
            float target_joint_pos[NUM_JOINTS];
            float gain;
        } servoj_params;

        struct {
            float target_pose[6];
            float gain;
        } servol_params;

        // 连续多段轨迹
        struct {
            uint8_t traj_index; // 用于指定轨迹
        } movep_params;
        // 轨迹播放
        struct {
            uint32_t traj_index;
        } play_traj_params;

        // io 控制
        struct {
            uint32_t channel;
            uint8_t value; // 0 
        } setio_params;
        // 设置参数
        struct {
            uint8_t mainType;       // 主类型（如 Motion、Control）
            uint8_t subType;        // 子类型（如 MaxSpeed、SafetyZone）
            uint8_t valueCount;     // 数组个数
            float values[NUM_JOINTS];
        } setparms;


        struct {
            uint32_t hold_time_ms;
        } hold_params;

        struct {
            uint32_t wait_time_ms;
        } wait_params;
        
        // 关节点动
        struct {
            int mode;          // 模式 '0' 连续点动, '1' 微动
            int joint_index;  // 关节索引 (0-5)
            int direction;       // 方向 ('1'正向 或 '0'负向)
        } jogj_params;

        // 笛卡尔点动
        struct {
            int mode;          // 模式 '0' 连续点动, '1' 微动
            int axis;
            int direction;
        } jogl_params;
    };
};

// 电机控制模式定义 
enum MotorMode {
    CSP = 0x08,   // Cyclic Synchronous Position
    CSV = 0x09,   // Cyclic Synchronous Velocity
    CST = 0x0A    // Cyclic Synchronous Torque
  };

// 关节命令
struct LowLevelCommand { 
    MotorMode mode;
    union 
    {
        signed int joint_pos[NUM_JOINTS];  // 每个关节的目标位置
        signed int joint_velocity[NUM_JOINTS]; // 每个关节的目标速度
        signed int joint_torque[NUM_JOINTS]; // 每个关节的目标力矩
    };
};

// 控制器状态
enum class ControllerState : uint8_t {
    IDLE = 0,
    STOPPED,
    RUNNING,
    ERROR,
    ESTOP,
};

// enum MotorState
// {
//     RUNNING,
//     FINISH,
//     STOP
// };
// 关节状态反馈
struct JointState
{
    float position;      // 当前关节位置
    float velocity;      // 当前速度
    float torque;        // 电机反馈转矩 [Nm]

    uint8_t motor_state; // 电机状态

    // uint32_t Cur_PulseTotalCounter;  //当前的电机脉冲
    uint16_t ecat_status_word; // EtherCAT 状态字
};

// 控制器状态反馈
struct RobotState {
    ControllerState controller_state; // 1字节
    uint8_t error_code;
    JointState joint_state[NUM_JOINTS];
    // float current_pose[6]; // 当前末端位姿
};

// 控制命令队列（生产者：HMI，消费者：RT）
template <class T, size_t LEN>
class SPSC_CmdQueue
{
    alignas(64) std::atomic<uint32_t> head{0}; // 写指针（HMI 写）
    alignas(64) std::atomic<uint32_t> tail{0}; // 读指针（RT 读）

public:
    T buffer[LEN];

    bool push(const T &cmd)
    {
        uint32_t h = head.load(std::memory_order_relaxed);
        uint32_t t = tail.load(std::memory_order_acquire);
        if ((h - t) >= LEN)
        {
            return false; // 队列满
        }
        buffer[h % LEN] = cmd;
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    bool pop(T &out)
    {
        uint32_t h = head.load(std::memory_order_acquire);
        uint32_t t = tail.load(std::memory_order_relaxed);
        if (t == h)
        {
            return false; // 队列空
        }
        out = buffer[t % LEN];
        tail.store((t + 1), std::memory_order_release);
        return true;
    }
    size_t pop_batch(T *out, size_t count)
    {
        const uint32_t t = tail.load(std::memory_order_relaxed);
        const uint32_t h = head.load(std::memory_order_acquire);
        const size_t avail = h - t;
        const size_t n = std::min(avail, count);

        for (size_t i = 0; i < n; ++i)
        {
            out[i] = buffer[(t + i) % LEN];
        }
        tail.store(t + n, std::memory_order_release);
        return n;
    }
};

#if 0
template <class T>
class DoubleBuffer
{
    alignas(64) std::atomic<uint32_t> write_flag{0}; // 控制模块写入后更新
    alignas(64) std::atomic<uint32_t> read_flag{0};  // HMI 读取后更新

public:
    alignas(64) T buffer[2];

    // 控制模块写入
    void write(const T &s)
    {
        uint32_t idx = write_flag.load(std::memory_order_relaxed) % 2;
        buffer[idx] = s;

        write_flag.fetch_add(1, std::memory_order_release);
    }

    // HMI 读取
    bool read(T &s)
    {
        uint32_t w = write_flag.load(std::memory_order_acquire);
        if (read_flag.load(std::memory_order_relaxed) == w)
        {
            return false; // 无新数据
        }

        uint32_t idx = (w - 1) % 2;
        s = buffer[idx];
//        read_flag.store(w, std::memory_order_relaxed);
        read_flag.fetch_add(1, std::memory_order_release);
        return true;
    }
};
#endif

template <typename T>
class DoubleBuffer {
    std::array<T, 2> buffer;
    std::atomic<uint32_t> version{0}; // 偶数：稳定，奇数：写入中

public:
    void write(const T& data) {
        uint32_t v = version.load(std::memory_order_relaxed);
        version.store(v + 1, std::memory_order_release);
        buffer[(v / 2) % 2] = data;
        version.store(v + 2, std::memory_order_release);
    }

    bool read(T& data) const {
        T result;
        while (true) {
            uint32_t v1 = version.load(std::memory_order_acquire);
            if (v1 & 1) continue; // 写入中，重试

            data = buffer[(v1 / 2) % 2];

            uint32_t v2 = version.load(std::memory_order_acquire);
            if (v1 == v2) return true;
        }
    }
};

// 总体共享数据结构
struct SharedMemoryData
{
    SPSC_CmdQueue<HighLevelCommand, 8> high_prio_cmd_queue;
    SPSC_CmdQueue<HighLevelCommand, MAX_CMD_QUEUE_LEN> cmd_queue;
    DoubleBuffer<RobotState> state_buffer;

    // 用于标识当前运行的指令指令编号
    std::atomic<uint32_t> cur_cmd_index;

    // HMI 填写轨迹，RT 消费；可支持多个命令轨迹并发
    // 指向轨迹池中使用的哪一段轨迹（由命令指定）
    Trajectory trajectory_pool[MAX_TRAJECTORY_LIST];
    // 多连续段集合
    MultiTrajectory cartesian_trajectory[MAX_TRAJECTORY_LIST];
};
#endif
