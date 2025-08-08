#ifndef VELOCITY_PLANNER_H
#define VELOCITY_PLANNER_H

#include <vector>
#include <array>

/**
 * @brief 7段S型速度规划器
 * 实现更灵活的速度规划，支持非零起始和结束速度，提供良好的封装性
 */
class VelocityPlanner {
public:
    /**
     * @brief 速度规划阶段枚举
     */
    enum class PlanningPhase {
        IDLE = 0,           // 未规划
        ACCEL_JERK = 1,     // 加速度增加阶段（正加加速度）
        ACCEL_CONST = 2,    // 恒定加速度阶段
        ACCEL_JERK_DEC = 3, // 加速度减少阶段（负加加速度）
        CONST_VELOCITY = 4, // 恒定速度阶段
        DECEL_JERK = 5,     // 减速度增加阶段（负加加速度）
        DECEL_CONST = 6,    // 恒定减速度阶段
        DECEL_JERK_DEC = 7  // 减速度减少阶段（正加加速度）
    };

    /**
     * @brief 速度规划参数结构体
     */
    struct PlanningParams {
        double maxVelocity;     // 系统最大速度 (m/s)
        double targetVelocity;  // 设定速度 (m/s) - 新增
        double maxAcceleration; // 最大加速度 (m/s²)
        double maxJerk;         // 最大加加速度 (m/s³)
        double startVelocity;   // 起始速度 (m/s)
        double endVelocity;     // 结束速度 (m/s)
        double totalDistance;   // 总距离 (m)
        double cycleTime;       // 插补周期 (s)
        
        PlanningParams() : 
            maxVelocity(1.0), targetVelocity(1.0), maxAcceleration(250.0), maxJerk(800.0),
            startVelocity(0.0), endVelocity(0.0), totalDistance(0.0),
            cycleTime(0.001) {}
    };

    /**
     * @brief 当前运动状态结构体
     */
    struct MotionState {
        double position;        // 当前位置 (m)
        double velocity;        // 当前速度 (m/s)
        double acceleration;    // 当前加速度 (m/s²)
        double jerk;           // 当前加加速度 (m/s³)
        double timeElapsed;    // 已用时间 (s)
        PlanningPhase phase;   // 当前阶段
        bool isCompleted;      // 是否完成
        
        MotionState() : 
            position(0.0), velocity(0.0), acceleration(0.0), jerk(0.0),
            timeElapsed(0.0), phase(PlanningPhase::IDLE), isCompleted(false) {}
    };

    /**
     * @brief 7段时间参数结构体
     */
    struct SevenSegmentTimes {
        double t1;  // 加速度增加时间
        double t2;  // 恒定加速度时间
        double t3;  // 加速度减少时间
        double t4;  // 恒定速度时间
        double t5;  // 减速度增加时间
        double t6;  // 恒定减速度时间
        double t7;  // 减速度减少时间
        
        SevenSegmentTimes() : t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0) {}
        
        double getTotalTime() const {
            return t1 + t2 + t3 + t4 + t5 + t6 + t7;
        }
    };

public:
    /**
     * @brief 构造函数
     */
    VelocityPlanner();
    
    /**
     * @brief 析构函数
     */
    ~VelocityPlanner();

    /**
     * @brief 初始化速度规划
     * @param params 规划参数
     * @return 是否初始化成功
     */
    bool initialize(const PlanningParams& params);

    /**
     * @brief 重置规划器状态
     */
    void reset();

    /**
     * @brief 计算下一个插补点的运动状态
     * @return 当前运动状态
     */
    MotionState getNextState();

    /**
     * @brief 获取当前运动状态
     * @return 当前运动状态
     */
    const MotionState& getCurrentState() const;

    /**
     * @brief 检查规划是否完成
     * @return 是否完成
     */
    bool isCompleted() const;

    /**
     * @brief 获取7段时间参数
     * @return 7段时间参数
     */
    const SevenSegmentTimes& getSegmentTimes() const;

    /**
     * @brief 获取规划参数
     * @return 规划参数
     */
    const PlanningParams& getPlanningParams() const;

    /**
     * @brief 设置新的目标速度（用于在线调整）
     * @param newMaxVelocity 新的最大速度
     * @return 是否设置成功
     */
    bool updateMaxVelocity(double newMaxVelocity);

    /**
     * @brief 紧急停止规划（以最大减速度停止）
     */
    void emergencyStop();

    /**
     * @brief 暂停规划（保持当前状态）
     */
    void pause();

    /**
     * @brief 恢复规划
     */
    void resume();

    /**
     * @brief 检查规划是否暂停
     * @return 是否暂停
     */
    bool isPaused() const;

    /**
     * @brief 获取剩余距离
     * @return 剩余距离 (m)
     */
    double getRemainingDistance() const;

    /**
     * @brief 获取剩余时间
     * @return 剩余时间 (s)
     */
    double getRemainingTime() const;

    /**
     * @brief 静态方法：验证规划参数的合理性
     * @param params 待验证的参数
     * @return 参数是否合理
     */
    static bool validateParams(const PlanningParams& params);

    /**
     * @brief 静态方法：计算给定参数下的最小规划距离
     * @param params 规划参数
     * @return 最小距离 (m)
     */
    static double calculateMinDistance(const PlanningParams& params);

    /**
     * @brief 更新目标速度
     * @param newTargetVelocity 新的目标速度
     * @return 是否更新成功
     */
    bool updateTargetVelocity(double newTargetVelocity);

private:
    /**
     * @brief 计算7段时间参数
     * @return 是否计算成功
     */
    bool calculateSegmentTimes();

    /**
     * @brief 计算指定阶段的运动状态
     * @param phase 规划阶段
     * @param phaseTime 阶段内时间
     * @return 运动状态
     */
    MotionState calculatePhaseState(PlanningPhase phase, double phaseTime);

    /**
     * @brief 判断当前应处于哪个阶段
     * @param totalTime 总时间
     * @return 当前阶段和阶段内时间
     */
    std::pair<PlanningPhase, double> determineCurrentPhase(double totalTime);

    /**
     * @brief 检查速度规划的可行性
     * @return 是否可行
     */
    bool checkFeasibility();

    /**
     * @brief 优化时间参数（处理退化情况）
     */
    void optimizeTimeParameters();

private:
    PlanningParams m_params;        // 规划参数
    MotionState m_currentState;     // 当前运动状态
    SevenSegmentTimes m_segmentTimes; // 7段时间参数
    bool m_initialized;             // 是否已初始化
    bool m_paused;                  // 是否暂停
    bool m_emergencyStop;           // 是否紧急停止
    double m_pauseTime;             // 暂停时的时间
    
    // 内部计算用的中间变量
    double m_maxReachableVelocity;  // 实际能达到的最大速度
    bool m_hasConstVelocityPhase;   // 是否有恒速阶段
    bool m_hasConstAccelPhase;      // 是否有恒加速阶段
    bool m_hasConstDecelPhase;      // 是否有恒减速阶段
    
    /**
     * @brief 计算加速阶段距离
     * @return 加速距离 (m)
     */
    double calculateAccelDistance();
    
    /**
     * @brief 计算减速阶段距离
     * @return 减速距离 (m)
     */
    double calculateDecelDistance();
    
    /**
     * @brief 无恒速段时重新计算时间参数
     * @return 是否计算成功
     */
    bool recalculateWithoutConstVelocity();
    
    /**
     * @brief 寻找最优峰值速度
     * @param v0 起始速度
     * @param v1 结束速度
     * @param s 总距离
     * @param amax 最大加速度
     * @param jmax 最大加加速度
     * @return 最优峰值速度
     */
    double findOptimalPeakVelocity(double v0, double v1, double s, double amax, double jmax);
    
    /**
     * @brief 计算给定峰值速度下的总距离
     * @param v0 起始速度
     * @param v1 结束速度
     * @param vpeak 峰值速度
     * @param amax 最大加速度
     * @param jmax 最大加加速度
     * @return 总距离
     */
    double calculateTotalDistanceForPeakVelocity(double v0, double v1, double vpeak, double amax, double jmax);
    
    /**
     * @brief 计算理论最大速度（基于距离约束）
     * @param v0 起始速度
     * @param v1 结束速度
     * @param s 总距离
     * @param amax 最大加速度
     * @param jmax 最大加加速度
     * @return 理论最大速度
     */
    double calculateTheoreticalMaxVelocity(double v0, double v1, double s, double amax, double jmax);
    
    /**
     * @brief 使用最优速度重新计算
     * @return 是否计算成功
     */
    bool recalculateWithOptimalVelocity();
    
    /**
     * @brief 处理紧急停止
     */
    void handleEmergencyStop();
    
};

#endif // VELOCITY_PLANNER_H