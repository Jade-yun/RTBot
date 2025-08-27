#include "velocityplanner/velocityplanner.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

VelocityPlanner::VelocityPlanner() 
    : m_initialized(false), m_paused(false), m_emergencyStop(false), 
      m_pauseTime(0.0), m_maxReachableVelocity(0.0), 
      m_hasConstVelocityPhase(false), m_hasConstAccelPhase(false), 
      m_hasConstDecelPhase(false) {
    // 平滑暂停/恢复初始化
    m_mainPlanFrozen = false;
    m_pauseRamping = false;
    m_resumeRamping = false;
    m_velocityBeforePause = 0.0;
    m_positionOffset = 0.0;
}

VelocityPlanner::~VelocityPlanner() {
}

bool VelocityPlanner::initialize(const PlanningParams& params) {
    // 验证参数
    if (!validateParams(params)) {
        return false;
    }
    
    m_params = params;
    
    // 重置状态
    reset();
    
    // 计算7段时间参数
    if (!calculateSegmentTimes()) {
        return false;
    }
    
    // 检查可行性
    if (!checkFeasibility()) {
        return false;
    }
    
    m_initialized = true;
    return true;
}

void VelocityPlanner::reset() {
    m_currentState = MotionState();
    m_currentState.velocity = m_params.startVelocity;
    m_segmentTimes = SevenSegmentTimes();
    m_paused = false;
    m_emergencyStop = false;
    m_pauseTime = 0.0;

    // 平滑暂停/恢复复位
    m_mainPlanFrozen = false;
    m_pauseRamping = false;
    m_resumeRamping = false;
    m_velocityBeforePause = 0.0;
    m_positionOffset = 0.0;
}

VelocityPlanner::MotionState VelocityPlanner::getNextState() {
    if (!m_initialized || m_currentState.isCompleted) {
        return m_currentState;
    }
    
    // 紧急停止优先
    if (m_emergencyStop) {
        // 紧急停止处理
        handleEmergencyStop();
        return m_currentState;
    }

    // 正在平滑减速到0（冻结主规划时间）
    if (m_pauseRamping) {
        bool done = updateRampTowards(0.0);
        if (done) {
            m_pauseRamping = false;
            m_paused = true;           // 到0后处于暂停保持
            m_mainPlanFrozen = true;   // 主规划时间保持冻结
            m_currentState.acceleration = 0.0;
            m_currentState.jerk = 0.0;
        }
        return m_currentState; // ramp阶段不推进主规划时间
    }

    // 正在平滑恢复到暂停前速度（冻结主规划时间）
    if (m_resumeRamping) {
        bool done = updateRampTowards(m_velocityBeforePause);
        if (done) {
            m_resumeRamping = false;
            m_paused = false;
            m_mainPlanFrozen = false;  // 结束恢复，解除冻结，继续主规划
            m_currentState.acceleration = 0.0;
            m_currentState.jerk = 0.0;
        }
        return m_currentState;
    }
    
    // 暂停保持：冻结主规划时间，保持当前状态
    if (m_paused || m_mainPlanFrozen) {
        return m_currentState;
    }
    
    // 正常主规划推进
    double currentTime = m_currentState.timeElapsed + m_params.cycleTime;
    
    // 确定当前阶段
    auto [phase, phaseTime] = determineCurrentPhase(currentTime);
    
    // 计算当前阶段的运动状态（主规划部分）
    m_currentState = calculatePhaseState(phase, phaseTime);
    // 修复：正确设置更新后的时间
    m_currentState.timeElapsed = currentTime;

    // 保存主规划的理论位置（用于完成判断）
    double theoreticalPosition = m_currentState.position;

    // 叠加暂停/恢复期间的位移偏置，使位置对齐
    m_currentState.position += m_positionOffset;
    
    // 检查是否完成 - 使用理论位置而不是实际位置
    if (m_currentState.timeElapsed >= m_segmentTimes.getTotalTime() || 
        theoreticalPosition >= m_params.totalDistance) {
        m_currentState.isCompleted = true;
        m_currentState.velocity = m_params.endVelocity;
        m_currentState.acceleration = 0.0;
        m_currentState.jerk = 0.0;
        m_currentState.position = m_params.totalDistance + m_positionOffset; // 保持位置连续性
    }
    
    return m_currentState;
}

bool VelocityPlanner::calculateSegmentTimes() {
    double v0 = m_params.startVelocity;
    double v1 = m_params.endVelocity;
    double vmax_system = m_params.maxVelocity;      // 系统最大速度
    double vtarget = m_params.targetVelocity;       // 设定速度
    double amax = m_params.maxAcceleration;
    double jmax = m_params.maxJerk;
    double s = m_params.totalDistance;
    
    // 处理退化情况：起始和结束速度相同，距离很小
    if (std::abs(v0 - v1) < 1e-6 && s < 1e-6) {
        m_maxReachableVelocity = v0;
        m_segmentTimes = SevenSegmentTimes(); // 全部为0
        m_hasConstVelocityPhase = false;
        m_hasConstAccelPhase = false;
        m_hasConstDecelPhase = false;
        return true;
    }
    
    // 处理零距离情况
    if (s <= 1e-9) {
        if (std::abs(v0 - v1) < 1e-6) {
            // 距离为0且速度相同，直接完成
            m_maxReachableVelocity = v0;
            m_segmentTimes = SevenSegmentTimes();
            return true;
        } else {
            // 距离为0但速度不同，无法规划
            return false;
        }
    }
    
    // 确定实际的目标速度：取设定速度和系统最大速度的较小值
    double vmax_actual = std::min(vtarget, vmax_system);

    // 检查在给定距离内，以目标速度运动是否可行
    double s_for_vmax_actual = calculateTotalDistanceForPeakVelocity(v0, v1, vmax_actual, amax, jmax);

    if (s_for_vmax_actual <= s) {
        // 目标速度可以达到，轨迹将包含匀速阶段
        m_maxReachableVelocity = vmax_actual;
        m_hasConstVelocityPhase = true;
    } else {
        // 目标速度无法达到，是距离受限的短行程
        // 计算此距离下能达到的最大速度
        m_maxReachableVelocity = calculateTheoreticalMaxVelocity(v0, v1, s, amax, jmax);
        if (m_maxReachableVelocity < 0) {
            return false; // 无法找到有效解
        }
        m_hasConstVelocityPhase = false;
    }

    // 确保最终的峰值速度不小于起始和结束速度
    m_maxReachableVelocity = std::max(m_maxReachableVelocity, std::max(v0, v1));
    
    // 计算加速阶段时间参数
    double dv_accel = m_maxReachableVelocity - v0;
    double dv_decel = m_maxReachableVelocity - v1;
    
    // 加速阶段计算
    if (dv_accel > 1e-9) {
        if (dv_accel <= amax*amax/jmax) {
            // 三角形加速度曲线
            m_segmentTimes.t1 = std::sqrt(dv_accel/jmax);
            m_segmentTimes.t2 = 0.0;
            m_segmentTimes.t3 = m_segmentTimes.t1;
            m_hasConstAccelPhase = false;
        } else {
            // 梯形加速度曲线
            m_segmentTimes.t1 = amax/jmax;
            m_segmentTimes.t2 = dv_accel/amax - amax/jmax;
            m_segmentTimes.t3 = amax/jmax;
            m_hasConstAccelPhase = true;
        }
    } else {
        m_segmentTimes.t1 = 0.0;
        m_segmentTimes.t2 = 0.0;
        m_segmentTimes.t3 = 0.0;
        m_hasConstAccelPhase = false;
    }
    
    // 减速阶段计算
    if (dv_decel > 1e-9) {
        if (dv_decel <= amax*amax/jmax) {
            // 三角形减速度曲线
            m_segmentTimes.t5 = std::sqrt(dv_decel/jmax);
            m_segmentTimes.t6 = 0.0;
            m_segmentTimes.t7 = m_segmentTimes.t5;
            m_hasConstDecelPhase = false;
        } else {
            // 梯形减速度曲线
            m_segmentTimes.t5 = amax/jmax;
            m_segmentTimes.t6 = dv_decel/amax - amax/jmax;
            m_segmentTimes.t7 = amax/jmax;
            m_hasConstDecelPhase = true;
        }
    } else {
        m_segmentTimes.t5 = 0.0;
        m_segmentTimes.t6 = 0.0;
        m_segmentTimes.t7 = 0.0;
        m_hasConstDecelPhase = false;
    }
    
    // 计算加速和减速阶段的距离
    double s_accel = calculateAccelDistance();
    double s_decel = calculateDecelDistance();
    
    // 计算恒速阶段时间
    if (m_hasConstVelocityPhase) {
        double s_const = s - s_accel - s_decel;
        m_segmentTimes.t4 = std::max(0.0, s_const / m_maxReachableVelocity);
    } else {
        m_segmentTimes.t4 = 0.0;
    }
    
    return true;
}

// 修复：计算理论最大速度（基于距离约束）
double VelocityPlanner::calculateTheoreticalMaxVelocity(double v0, double v1, double s, double amax, double jmax) {
    // 检查输入参数的有效性
    if (s <= 0 || amax <= 0 || jmax <= 0) {
        return -1.0;
    }
    
    // 计算最小可能的峰值速度
    double vmin = std::max(v0, v1);
    
    // 检查是否超出系统限制
    if (vmin > m_params.maxVelocity) {
        return -1.0; // 无解
    }
    
    // 设置合理的上限
    double vmax = std::min(m_params.maxVelocity * 1.5, vmin + std::sqrt(2 * amax * s));
    
    // 首先检查最小速度是否可行
    double s_min = calculateTotalDistanceForPeakVelocity(v0, v1, vmin, amax, jmax);
    if (s_min > s) {
        return -1.0; // 即使最小峰值速度也无法满足距离要求
    }
    
    double tolerance = 1e-6;
    int max_iterations = 100;
    int iteration = 0;
    
    while (vmax - vmin > tolerance && iteration < max_iterations) {
        double vmid = (vmin + vmax) / 2.0;
        double s_calc = calculateTotalDistanceForPeakVelocity(v0, v1, vmid, amax, jmax);
        
        if (s_calc < s) {
            vmin = vmid;
        } else {
            vmax = vmid;
        }
        iteration++;
    }
    
    return (vmin + vmax) / 2.0;
}

// 修复：重新计算最优速度
bool VelocityPlanner::recalculateWithOptimalVelocity() {
    double v0 = m_params.startVelocity;
    double v1 = m_params.endVelocity;
    double amax = m_params.maxAcceleration;
    double jmax = m_params.maxJerk;
    double s = m_params.totalDistance;
    
    // 计算在给定距离下的最优峰值速度
    double vmax_opt = calculateTheoreticalMaxVelocity(v0, v1, s, amax, jmax);
    
    if (vmax_opt < 0) {
        return false; // 无法找到有效解
    }
    
    m_maxReachableVelocity = vmax_opt;
    
    // 重新计算时间参数
    double dv_accel = vmax_opt - v0;
    double dv_decel = vmax_opt - v1;
    
    // 重新计算加速阶段
    if (dv_accel > 1e-9) {
        if (dv_accel <= amax*amax/jmax) {
            m_segmentTimes.t1 = std::sqrt(dv_accel/jmax);
            m_segmentTimes.t2 = 0.0;
            m_segmentTimes.t3 = m_segmentTimes.t1;
            m_hasConstAccelPhase = false;
        } else {
            m_segmentTimes.t1 = amax/jmax;
            m_segmentTimes.t2 = dv_accel/amax - amax/jmax;
            m_segmentTimes.t3 = amax/jmax;
            m_hasConstAccelPhase = true;
        }
    } else {
        m_segmentTimes.t1 = 0.0;
        m_segmentTimes.t2 = 0.0;
        m_segmentTimes.t3 = 0.0;
        m_hasConstAccelPhase = false;
    }
    
    // 重新计算减速阶段
    if (dv_decel > 1e-9) {
        if (dv_decel <= amax*amax/jmax) {
            m_segmentTimes.t5 = std::sqrt(dv_decel/jmax);
            m_segmentTimes.t6 = 0.0;
            m_segmentTimes.t7 = m_segmentTimes.t5;
            m_hasConstDecelPhase = false;
        } else {
            m_segmentTimes.t5 = amax/jmax;
            m_segmentTimes.t6 = dv_decel/amax - amax/jmax;
            m_segmentTimes.t7 = amax/jmax;
            m_hasConstDecelPhase = true;
        }
    } else {
        m_segmentTimes.t5 = 0.0;
        m_segmentTimes.t6 = 0.0;
        m_segmentTimes.t7 = 0.0;
        m_hasConstDecelPhase = false;
    }
    
    m_segmentTimes.t4 = 0.0;
    m_hasConstVelocityPhase = false;
    
    return true;
}

// 新增：更新设定速度的接口
bool VelocityPlanner::updateTargetVelocity(double newTargetVelocity) {
    if (newTargetVelocity <= 0) {
        return false;
    }
    
    m_params.targetVelocity = newTargetVelocity;
    
    // 重新计算时间参数
    return calculateSegmentTimes();
}

double VelocityPlanner::calculateAccelDistance() {
    double v0 = m_params.startVelocity;
    double t1 = m_segmentTimes.t1;
    double t2 = m_segmentTimes.t2;
    double t3 = m_segmentTimes.t3;
    double jmax = m_params.maxJerk;
    
    if (t1 <= 0 && t2 <= 0 && t3 <= 0) {
        return 0.0;
    }
    
    double s1 = v0*t1 + jmax*t1*t1*t1/6.0;
    double v1 = v0 + jmax*t1*t1/2.0;
    double a1 = jmax*t1;
    
    double s2 = v1*t2 + a1*t2*t2/2.0;
    double v2 = v1 + a1*t2;
    
    double s3 = v2*t3 + a1*t3*t3/2.0 - jmax*t3*t3*t3/6.0;
    
    return s1 + s2 + s3;
}

double VelocityPlanner::calculateDecelDistance() {
    double v_end = m_maxReachableVelocity;  // 减速开始速度
    double t5 = m_segmentTimes.t5;
    double t6 = m_segmentTimes.t6;
    double t7 = m_segmentTimes.t7;
    double jmax = m_params.maxJerk;
    
    if (t5 <= 0 && t6 <= 0 && t7 <= 0) {
        return 0.0;
    }
    
    // 第5段：减速度增加阶段
    double s5 = v_end*t5 - jmax*t5*t5*t5/6.0;
    double v5 = v_end - jmax*t5*t5/2.0;
    double a5 = -jmax*t5;
    
    // 第6段：恒定减速度阶段
    double s6 = v5*t6 + a5*t6*t6/2.0;
    double v6 = v5 + a5*t6;
    
    // 第7段：减速度减少阶段（精确公式）
    // 此处使用精确的运动学公式，而不是之前有问题的近似“修复”
    double s7 = v6*t7 + a5*t7*t7/2.0 + jmax*t7*t7*t7/6.0;
    
    return s5 + s6 + s7;
}

bool VelocityPlanner::recalculateWithoutConstVelocity() {
    // 当无法达到最大速度时，重新计算时间参数
    double v0 = m_params.startVelocity;
    double v1 = m_params.endVelocity;
    double amax = m_params.maxAcceleration;
    double jmax = m_params.maxJerk;
    double s = m_params.totalDistance;
    
    // 使用数值方法求解最优的峰值速度
    double vmax_opt = findOptimalPeakVelocity(v0, v1, s, amax, jmax);
    
    if (vmax_opt < 0) {
        return false;
    }
    
    m_maxReachableVelocity = vmax_opt;
    
    // 重新计算时间参数
    double dv_accel = vmax_opt - v0;
    double dv_decel = vmax_opt - v1;
    
    // 重新计算加速阶段
    if (dv_accel <= amax*amax/jmax) {
        m_segmentTimes.t1 = std::sqrt(dv_accel/jmax);
        m_segmentTimes.t2 = 0.0;
        m_segmentTimes.t3 = m_segmentTimes.t1;
    } else {
        m_segmentTimes.t1 = amax/jmax;
        m_segmentTimes.t2 = dv_accel/amax - amax/jmax;
        m_segmentTimes.t3 = amax/jmax;
    }
    
    // 重新计算减速阶段
    if (dv_decel <= amax*amax/jmax) {
        m_segmentTimes.t5 = std::sqrt(dv_decel/jmax);
        m_segmentTimes.t6 = 0.0;
        m_segmentTimes.t7 = m_segmentTimes.t5;
    } else {
        m_segmentTimes.t5 = amax/jmax;
        m_segmentTimes.t6 = dv_decel/amax - amax/jmax;
        m_segmentTimes.t7 = amax/jmax;
    }
    
    m_segmentTimes.t4 = 0.0;
    
    return true;
}

double VelocityPlanner::findOptimalPeakVelocity(double v0, double v1, double s, double amax, double jmax) {
    // 使用二分法求解最优峰值速度
    double vmin = std::max(v0, v1);
    double vmax = m_params.maxVelocity;
    
    // 检查边界条件
    if (vmin > vmax) {
        return -1.0; // 无解
    }
    
    double tolerance = 1e-6;
    int max_iterations = 100;
    int iteration = 0;
    
    while (vmax - vmin > tolerance && iteration < max_iterations) {
        double vmid = (vmin + vmax) / 2.0;
        double s_calc = calculateTotalDistanceForPeakVelocity(v0, v1, vmid, amax, jmax);
        
        if (s_calc < s) {
            vmin = vmid;
        } else {
            vmax = vmid;
        }
        iteration++;
    }
    
    return (vmin + vmax) / 2.0;
}

// 修复：计算给定峰值速度下的总距离
double VelocityPlanner::calculateTotalDistanceForPeakVelocity(double v0, double v1, double vpeak, double amax, double jmax) {
    // 计算给定峰值速度下的总距离
    double dv_accel = vpeak - v0;
    double dv_decel = vpeak - v1;
    
    // 保存现场，防止污染主规划状态
    SevenSegmentTimes original_times = m_segmentTimes;
    double original_v_reachable = m_maxReachableVelocity;
    double original_v_start = m_params.startVelocity;
    double original_v_end = m_params.endVelocity;

    // 临时设置参数以进行计算
    m_maxReachableVelocity = vpeak;
    m_params.startVelocity = v0;
    m_params.endVelocity = v1;

    // 计算加速阶段时间
    if (dv_accel > 1e-9) {
        if (dv_accel <= amax*amax/jmax) { // 三角形
            m_segmentTimes.t1 = std::sqrt(dv_accel/jmax);
            m_segmentTimes.t2 = 0.0;
            m_segmentTimes.t3 = m_segmentTimes.t1;
        } else { // 梯形
            m_segmentTimes.t1 = amax/jmax;
            m_segmentTimes.t2 = dv_accel/amax - amax/jmax;
            m_segmentTimes.t3 = amax/jmax;
        }
    } else {
        m_segmentTimes.t1 = 0.0; m_segmentTimes.t2 = 0.0; m_segmentTimes.t3 = 0.0;
    }

    // 计算减速阶段时间
    if (dv_decel > 1e-9) {
        if (dv_decel <= amax*amax/jmax) { // 三角形
            m_segmentTimes.t5 = std::sqrt(dv_decel/jmax);
            m_segmentTimes.t6 = 0.0;
            m_segmentTimes.t7 = m_segmentTimes.t5;
        } else { // 梯形
            m_segmentTimes.t5 = amax/jmax;
            m_segmentTimes.t6 = dv_decel/amax - amax/jmax;
            m_segmentTimes.t7 = amax/jmax;
        }
    } else {
        m_segmentTimes.t5 = 0.0; m_segmentTimes.t6 = 0.0; m_segmentTimes.t7 = 0.0;
    }
    
    m_segmentTimes.t4 = 0.0; // 此计算不考虑匀速阶段

    double s_accel = calculateAccelDistance();
    double s_decel = calculateDecelDistance();

    // 恢复现场
    m_segmentTimes = original_times;
    m_maxReachableVelocity = original_v_reachable;
    m_params.startVelocity = original_v_start;
    m_params.endVelocity = original_v_end;

    return s_accel + s_decel;
}

VelocityPlanner::MotionState VelocityPlanner::calculatePhaseState(PlanningPhase phase, double phaseTime) {
    MotionState state;
    state.phase = phase;
    
    double v0 = m_params.startVelocity;
    double jmax = m_params.maxJerk;
    double amax = m_params.maxAcceleration;
    
    // 计算各阶段结束时的累积值
    double t1 = m_segmentTimes.t1;
    double t2 = m_segmentTimes.t2;
    double t3 = m_segmentTimes.t3;
    double t4 = m_segmentTimes.t4;
    double t5 = m_segmentTimes.t5;
    double t6 = m_segmentTimes.t6;
    // 使用实际峰值加速度（若为三角形阶段，没有恒加速/恒减速，则峰值加速度小于等于 amax）
    double a_peak_accel = m_hasConstAccelPhase ? amax : (jmax * t1); // 加速段峰值加速度
    double a_peak_decel = m_hasConstDecelPhase ? amax : (jmax * t5); // 减速段峰值加速度（取正值，方向在公式里体现）
    
    switch (phase) {
        case PlanningPhase::ACCEL_JERK: {
            // 第1段：加速度增加阶段
            double t = phaseTime;
            state.jerk = jmax;
            state.acceleration = jmax * t;
            state.velocity = v0 + jmax * t * t / 2.0;
            state.position = v0 * t + jmax * t * t * t / 6.0;
            break;
        }
        
        case PlanningPhase::ACCEL_CONST: {
            // 第2段：恒定加速度阶段（如果是三角形，加速度峰值为 a_peak_accel，且 t2 = 0，不会进入此分支）
            double t = phaseTime;
            double v1 = v0 + jmax * t1 * t1 / 2.0;
            double s1 = v0 * t1 + jmax * t1 * t1 * t1 / 6.0;
            
            state.jerk = 0.0;
            state.acceleration = a_peak_accel;
            state.velocity = v1 + a_peak_accel * t;
            state.position = s1 + v1 * t + a_peak_accel * t * t / 2.0;
            break;
        }
        
        case PlanningPhase::ACCEL_JERK_DEC: {
            // 第3段：加速度减少阶段（若为三角加速，此阶段从 a_peak_accel 线性降到 0）
            double t = phaseTime;
            double v1 = v0 + jmax * t1 * t1 / 2.0;
            double s1 = v0 * t1 + jmax * t1 * t1 * t1 / 6.0;
            double v2 = v1 + a_peak_accel * t2;
            double s2 = s1 + v1 * t2 + a_peak_accel * t2 * t2 / 2.0;
            
            state.jerk = -jmax;
            state.acceleration = a_peak_accel - jmax * t;
            state.velocity = v2 + a_peak_accel * t - jmax * t * t / 2.0;
            state.position = s2 + v2 * t + a_peak_accel * t * t / 2.0 - jmax * t * t * t / 6.0;
            break;
        }
        
        case PlanningPhase::CONST_VELOCITY: {
            // 第4段：恒定速度阶段
            double t = phaseTime;
            double s_accel = calculateAccelDistance();
            
            state.jerk = 0.0;
            state.acceleration = 0.0;
            state.velocity = m_maxReachableVelocity;
            state.position = s_accel + m_maxReachableVelocity * t;
            break;
        }
        
        case PlanningPhase::DECEL_JERK: {
            // 第5段：减速度增加阶段（加速度从 0 线性到 -a_peak_decel）
            double t = phaseTime;
            double s_prev = calculateAccelDistance() + m_maxReachableVelocity * t4;
            
            state.jerk = -jmax;
            state.acceleration = -jmax * t;
            state.velocity = m_maxReachableVelocity - jmax * t * t / 2.0;
            state.position = s_prev + m_maxReachableVelocity * t - jmax * t * t * t / 6.0;
            break;
        }
        
        case PlanningPhase::DECEL_CONST: {
            // 第6段：恒定减速度阶段（若为三角减速，t6 = 0，不会进入此分支）
            double t = phaseTime;
            double s_prev = calculateAccelDistance() + m_maxReachableVelocity * t4;
            double v5 = m_maxReachableVelocity - jmax * t5 * t5 / 2.0;
            double s5 = s_prev + m_maxReachableVelocity * t5 - jmax * t5 * t5 * t5 / 6.0;
            
            state.jerk = 0.0;
            state.acceleration = -a_peak_decel;
            state.velocity = v5 - a_peak_decel * t;
            state.position = s5 + v5 * t - a_peak_decel * t * t / 2.0;
            break;
        }
        
        case PlanningPhase::DECEL_JERK_DEC: {
            // 第7段：减速度减少阶段（加速度从 -a_peak_decel 上升到 0）
            double t = phaseTime;
            double s_prev = calculateAccelDistance() + m_maxReachableVelocity * t4;
            double v5 = m_maxReachableVelocity - jmax * t5 * t5 / 2.0;
            double s5 = s_prev + m_maxReachableVelocity * t5 - jmax * t5 * t5 * t5 / 6.0;
            double v6 = v5 - a_peak_decel * t6;
            double s6 = s5 + v5 * t6 - a_peak_decel * t6 * t6 / 2.0;
            
            state.jerk = jmax;
            state.acceleration = -a_peak_decel + jmax * t;
            
            // 确保第7段结束时速度为目标结束速度
            double t7_total = m_segmentTimes.t7;
            if (t7_total > 0 && t >= t7_total) {
                state.velocity = m_params.endVelocity;
            } else {
                state.velocity = v6 - a_peak_decel * t + jmax * t * t / 2.0;
                if (state.velocity < m_params.endVelocity) {
                    state.velocity = m_params.endVelocity;
                }
            }
            
            state.position = s6 + v6 * t - a_peak_decel * t * t / 2.0 + jmax * t * t * t / 6.0;
            break;
        }
        
        default:
            state.jerk = 0.0;
            state.acceleration = 0.0;
            state.velocity = m_params.endVelocity;
            state.position = m_params.totalDistance;
            break;
    }
    
    return state;
}

std::pair<VelocityPlanner::PlanningPhase, double> VelocityPlanner::determineCurrentPhase(double totalTime) {
    double t1 = m_segmentTimes.t1;
    double t2 = m_segmentTimes.t2;
    double t3 = m_segmentTimes.t3;
    double t4 = m_segmentTimes.t4;
    double t5 = m_segmentTimes.t5;
    double t6 = m_segmentTimes.t6;
    double t7 = m_segmentTimes.t7;
    
    double cumTime = 0.0;
    
    if (t1 > 0 && totalTime <= (cumTime += t1)) {
        return {PlanningPhase::ACCEL_JERK, totalTime};
    }
    if (t2 > 0 && totalTime <= (cumTime += t2)) {
        return {PlanningPhase::ACCEL_CONST, totalTime - (cumTime - t2)};
    }
    if (t3 > 0 && totalTime <= (cumTime += t3)) {
        return {PlanningPhase::ACCEL_JERK_DEC, totalTime - (cumTime - t3)};
    }
    if (t4 > 0 && totalTime <= (cumTime += t4)) {
        return {PlanningPhase::CONST_VELOCITY, totalTime - (cumTime - t4)};
    }
    if (t5 > 0 && totalTime <= (cumTime += t5)) {
        return {PlanningPhase::DECEL_JERK, totalTime - (cumTime - t5)};
    }
    if (t6 > 0 && totalTime <= (cumTime += t6)) {
        return {PlanningPhase::DECEL_CONST, totalTime - (cumTime - t6)};
    }
    if (t7 > 0 && totalTime <= (cumTime += t7)) {
        return {PlanningPhase::DECEL_JERK_DEC, totalTime - (cumTime - t7)};
    }
    
    return {PlanningPhase::IDLE, 0.0};
}

bool VelocityPlanner::checkFeasibility() {
    // 检查时间参数是否合理
    if (m_segmentTimes.getTotalTime() <= 0) {
        return false;
    }
    
    // 检查距离是否匹配
    double calculatedDistance = calculateAccelDistance() + calculateDecelDistance() + 
                               m_maxReachableVelocity * m_segmentTimes.t4;
    
    double tolerance = 1e-3;
    if (std::abs(calculatedDistance - m_params.totalDistance) > tolerance) {
        return false;
    }
    
    return true;
}

void VelocityPlanner::handleEmergencyStop() {
    // 紧急停止：以最大减速度停止
    double currentV = m_currentState.velocity;
    if (currentV <= 0.001) {
        m_currentState.isCompleted = true;
        m_currentState.velocity = 0.0;
        m_currentState.acceleration = 0.0;
        m_currentState.jerk = 0.0;
        return;
    }
    
    // 计算紧急停止的减速度
    double emergencyDecel = -m_params.maxAcceleration;
    double stopTime = currentV / m_params.maxAcceleration;
    
    if (stopTime <= m_params.cycleTime) {
        m_currentState.velocity = 0.0;
        m_currentState.acceleration = 0.0;
        m_currentState.jerk = 0.0;
        m_currentState.isCompleted = true;
    } else {
        m_currentState.velocity += emergencyDecel * m_params.cycleTime;
        m_currentState.acceleration = emergencyDecel;
        m_currentState.jerk = 0.0;
        m_currentState.position += m_currentState.velocity * m_params.cycleTime;
    }
}

// 获取器方法
const VelocityPlanner::MotionState& VelocityPlanner::getCurrentState() const {
    return m_currentState;
}

bool VelocityPlanner::isCompleted() const {
    return m_currentState.isCompleted;
}

const VelocityPlanner::SevenSegmentTimes& VelocityPlanner::getSegmentTimes() const {
    return m_segmentTimes;
}

const VelocityPlanner::PlanningParams& VelocityPlanner::getPlanningParams() const {
    return m_params;
}

bool VelocityPlanner::updateMaxVelocity(double newMaxVelocity) {
    if (newMaxVelocity <= 0) {
        return false;
    }
    
    m_params.maxVelocity = newMaxVelocity;
    
    // 重新计算时间参数
    return calculateSegmentTimes();
}

void VelocityPlanner::emergencyStop() {
    m_emergencyStop = true;
}

void VelocityPlanner::pause() {
    // 已经在减速/恢复/暂停中，忽略
    if (m_pauseRamping || m_resumeRamping || m_paused) {
        return;
    }
    // 记录暂停前速度并启动平滑减速
    m_velocityBeforePause = std::max(0.0, m_currentState.velocity);
    startSmoothPause();
}

void VelocityPlanner::resume() {
    // 若正在减速，直接切换到恢复（从当前速度恢复到暂停前速度）
    if (m_pauseRamping) {
        m_pauseRamping = false;
        startSmoothResume();
        return;
    }
    // 已经在恢复中，忽略
    if (m_resumeRamping) {
        return;
    }
    // 从暂停保持状态恢复
    if (m_paused || m_mainPlanFrozen) {
        startSmoothResume();
    }
}

bool VelocityPlanner::isPaused() const {
    // ramp阶段也视为“暂停态”（对上层而言不可自由运行）
    return m_paused || m_pauseRamping || m_mainPlanFrozen;
}

double VelocityPlanner::getRemainingDistance() const {
    return m_params.totalDistance - m_currentState.position;
}

double VelocityPlanner::getRemainingTime() const {
    return m_segmentTimes.getTotalTime() - m_currentState.timeElapsed;
}

// 修复：完善参数验证
bool VelocityPlanner::validateParams(const PlanningParams& params) {
    // 基本参数检查
    if (params.maxVelocity <= 0 || params.maxAcceleration <= 0 || params.maxJerk <= 0) {
        return false;
    }
    
    if (params.totalDistance < 0) { // 允许零距离
        return false;
    }
    
    if (params.cycleTime <= 0) {
        return false;
    }
    
    if (params.startVelocity < 0 || params.endVelocity < 0) {
        return false;
    }
    
    if (params.startVelocity > params.maxVelocity || params.endVelocity > params.maxVelocity) {
        return false;
    }
    
    // 新增：targetVelocity 验证（放宽为仅检查正值，超过最大值将由计算阶段自动钳制）
    if (params.targetVelocity <= 0) {
        return false;
    }
    
    // 新增：距离可行性验证（仅当距离大于0时）
    if (params.totalDistance > 1e-9) {
        double minDistance = calculateMinDistance(params);
        if (minDistance < 0 || params.totalDistance < minDistance - 1e-6) {
            return false;
        }
    }
    
    return true;
}

// 修复：正确的最小距离计算
double VelocityPlanner::calculateMinDistance(const PlanningParams& params) {
    // 基本参数检查（避免递归调用validateParams）
    if (params.maxVelocity <= 0 || params.maxAcceleration <= 0 || params.maxJerk <= 0) {
        return -1.0;
    }
    
    if (params.startVelocity < 0 || params.endVelocity < 0) {
        return -1.0;
    }
    
    double v0 = params.startVelocity;
    double v1 = params.endVelocity;
    double amax = params.maxAcceleration;
    double jmax = params.maxJerk;
    
    // 如果起始和结束速度相同，最小距离为0
    if (std::abs(v0 - v1) < 1e-9) {
        return 0.0;
    }
    
    // 计算最小距离（直接从起始速度到结束速度）
    double dv = std::abs(v1 - v0);
    
    if (dv <= amax * amax / jmax) {
        // 三角形加速度曲线
        double t = std::sqrt(dv / jmax);
        if (v1 > v0) {
            // 加速情况：s = v0*t + (1/2)*a_avg*t^2, 其中a_avg = jmax*t/2
            return v0 * 2 * t + jmax * t * t * t / 3.0;
        } else {
            // 减速情况：s = v0*t - (1/2)*a_avg*t^2
            return v0 * 2 * t - jmax * t * t * t / 3.0;
        }
    } else {
        // 梯形加速度曲线
        double t1 = amax / jmax;  // 加加速度阶段时间
        double t2 = dv / amax - amax / jmax;  // 恒加速度阶段时间
        
        if (v1 > v0) {
            // 加速情况
            double s1 = v0 * 2 * t1 + jmax * t1 * t1 * t1 / 3.0;  // 加加速度阶段距离
            double v_mid = v0 + amax * t1;  // 恒加速度开始时的速度
            double s2 = v_mid * t2 + amax * t2 * t2 / 2.0;  // 恒加速度阶段距离
            return s1 + s2;
        } else {
            // 减速情况
            double s1 = v0 * 2 * t1 - jmax * t1 * t1 * t1 / 3.0;  // 减加速度阶段距离
            double v_mid = v0 - amax * t1;  // 恒减速度开始时的速度
            double s2 = v_mid * t2 - amax * t2 * t2 / 2.0;  // 恒减速度阶段距离
            return s1 + s2;
        }
    }
}

// ========== 新增：平滑暂停/恢复的内部方法 ==========
void VelocityPlanner::startSmoothPause() {
    m_mainPlanFrozen = true;  // 冻结主规划时间
    m_pauseRamping = true;
    m_resumeRamping = false;
    m_paused = false;         // ramp完成后再置为 true
}

void VelocityPlanner::startSmoothResume() {
    m_mainPlanFrozen = true;  // 恢复坡度期间仍冻结主规划
    m_resumeRamping = true;
    m_pauseRamping = false;
    m_paused = false;
}

// 按最大加速度限制，将当前速度逐步逼近 targetVelocity
// 说明：
// - 不推进主轨迹时间（timeElapsed 不变）
// - 会对 m_currentState.position 进行积分，并累计到 m_positionOffset
// - 无加加速度限制（若需加加速度限制，可按 jmax 细化加速度变化）
bool VelocityPlanner::updateRampTowards(double targetVelocity) {
    double dt = m_params.cycleTime;
    double amax = m_params.maxAcceleration;
    targetVelocity = std::clamp(targetVelocity, 0.0, m_params.maxVelocity);

    double v = std::max(0.0, m_currentState.velocity);
    double dv = targetVelocity - v;

    // 单周期能改变的最大速度
    double maxDeltaV = amax * dt;

    // 如果本周期能够到达目标速度，则直接到达并结束
    if (std::abs(dv) <= maxDeltaV + 1e-12) {
        double v_new = targetVelocity;
        double s_inc = (v + v_new) * 0.5 * dt;

        m_currentState.position += s_inc;
        m_positionOffset += s_inc;

        m_currentState.velocity = v_new;
        m_currentState.acceleration = 0.0;
        m_currentState.jerk = 0.0;
        // timeElapsed 不变（冻结主规划时间）
        return true;
    }

    // 否则按最大加速度做一小步
    double step = (dv > 0 ? maxDeltaV : -maxDeltaV);
    double v_new = v + step;
    v_new = std::clamp(v_new, 0.0, m_params.maxVelocity);

    double s_inc = (v + v_new) * 0.5 * dt;
    m_currentState.position += s_inc;
    m_positionOffset += s_inc;

    m_currentState.velocity = v_new;
    m_currentState.acceleration = step / dt;
    m_currentState.jerk = 0.0;
    // timeElapsed 不变（冻结主规划时间）
    return false;
}