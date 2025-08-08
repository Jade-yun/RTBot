# VelocityPlanner 使用说明

## 概述

VelocityPlanner是一个7段S型速度规划器，专为机器人运动控制设计。它支持非零起始和结束速度，能够实现多段轨迹的平滑过渡，有效解决传统速度规划中起始和结束速度只能为零的限制。

## 主要特性

- **7段S型规划**：加加速段、匀加速段、减加速段、匀速段、加减速段、匀减速段、减减速段
- **非零起始/结束速度**：支持任意起始和结束速度，实现轨迹间平滑过渡
- **高度封装**：清晰的接口设计，降低与其他模块的耦合度
- **实时控制**：支持周期性调用，适合实时控制系统
- **紧急停止**：提供安全的紧急停止功能
- **参数可配置**：最大速度、加速度、加加速度等参数可灵活设置

## 核心数据结构

### PlanningParams - 规划参数
```cpp
struct PlanningParams {
    double max_velocity;     // 最大速度 [m/s 或 rad/s]
    double max_acceleration; // 最大加速度 [m/s² 或 rad/s²]
    double max_jerk;        // 最大加加速度 [m/s³ 或 rad/s³]
    double cycle_time;      // 控制周期 [s]
};
```

### MotionState - 运动状态
```cpp
struct MotionState {
    double position;     // 位置 [m 或 rad]
    double velocity;     // 速度 [m/s 或 rad/s]
    double acceleration; // 加速度 [m/s² 或 rad/s²]
    double jerk;        // 加加速度 [m/s³ 或 rad/s³]
};
```

### TrajectorySegment - 轨迹段
```cpp
struct TrajectorySegment {
    double start_position;   // 起始位置
    double end_position;     // 结束位置
    double start_velocity;   // 起始速度
    double end_velocity;     // 结束速度
    double distance;         // 总距离
};
```

## 基本用法

### 1. 创建规划器实例
```cpp
#include "velocityplanner/velocityplanner.h"

VelocityPlanner planner;
```

### 2. 设置规划参数
```cpp
VelocityPlanner::PlanningParams params;
params.max_velocity = 2.0;      // 最大速度 2 m/s
params.max_acceleration = 5.0;  // 最大加速度 5 m/s²
params.max_jerk = 20.0;         // 最大加加速度 20 m/s³
params.cycle_time = 0.001;      // 控制周期 1ms

planner.setParams(params);
```

### 3. 开始轨迹规划
```cpp
// 定义轨迹段：从位置0到位置10，起始速度0.5 m/s，结束速度1.0 m/s
VelocityPlanner::TrajectorySegment segment(0.0, 10.0, 0.5, 1.0);

// 定义初始状态
VelocityPlanner::MotionState initial_state(0.0, 0.5, 0.0, 0.0);

// 开始规划
if (planner.startPlanning(segment, initial_state)) {
    std::cout << "规划成功启动" << std::endl;
} else {
    std::cout << "规划失败" << std::endl;
}
```

### 4. 实时获取运动状态
```cpp
VelocityPlanner::MotionState current_state;

// 在控制循环中调用
while (planner.getNextState(current_state)) {
    // 使用当前状态进行运动控制
    std::cout << "位置: " << current_state.position 
              << ", 速度: " << current_state.velocity 
              << ", 加速度: " << current_state.acceleration << std::endl;
    
    // 发送给运动控制器
    // motor_controller.setTarget(current_state);
    
    // 等待下一个控制周期
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

std::cout << "轨迹规划完成" << std::endl;
```

## 高级功能

### 紧急停止
```cpp
// 触发紧急停止
planner.emergencyStop();

// 检查是否处于紧急停止状态
if (planner.isEmergencyStop()) {
    std::cout << "系统处于紧急停止状态" << std::endl;
}
```

### 状态查询
```cpp
// 检查规划是否完成
if (planner.isFinished()) {
    std::cout << "规划已完成" << std::endl;
}

// 获取当前规划阶段
auto phase = planner.getCurrentPhase();
switch (phase) {
    case VelocityPlanner::PlanningPhase::ACCEL_JERK:
        std::cout << "当前阶段：加加速段" << std::endl;
        break;
    case VelocityPlanner::PlanningPhase::CONST_VEL:
        std::cout << "当前阶段：匀速段" << std::endl;
        break;
    // ... 其他阶段
}

// 获取总规划时间
double total_time = planner.getTotalTime();
std::cout << "总规划时间: " << total_time << " 秒" << std::endl;
```

### 重置规划器
```cpp
// 重置到初始状态
planner.reset();
```

## 完整示例

```cpp
#include "velocityplanner/velocityplanner.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 创建规划器
    VelocityPlanner planner;
    
    // 设置参数
    VelocityPlanner::PlanningParams params(2.0, 5.0, 20.0, 0.001);
    planner.setParams(params);
    
    // 定义轨迹：从0到10米，起始速度0.5 m/s，结束速度1.0 m/s
    VelocityPlanner::TrajectorySegment segment(0.0, 10.0, 0.5, 1.0);
    VelocityPlanner::MotionState initial_state(0.0, 0.5, 0.0, 0.0);
    
    // 开始规划
    if (!planner.startPlanning(segment, initial_state)) {
        std::cerr << "规划失败" << std::endl;
        return -1;
    }
    
    // 执行轨迹
    VelocityPlanner::MotionState current_state;
    int step = 0;
    
    while (planner.getNextState(current_state)) {
        // 每100步输出一次状态
        if (step % 100 == 0) {
            std::cout << "步数: " << step 
                      << ", 位置: " << current_state.position 
                      << ", 速度: " << current_state.velocity 
                      << ", 加速度: " << current_state.acceleration << std::endl;
        }
        
        // 模拟控制周期
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        step++;
    }
    
    std::cout << "轨迹执行完成，总步数: " << step << std::endl;
    return 0;
}
```

## 注意事项

1. **参数设置**：确保最大速度、加速度、加加速度参数符合实际硬件限制
2. **控制周期**：cycle_time应与实际控制系统的周期时间一致
3. **初始状态**：startPlanning时的初始状态应与实际系统状态匹配
4. **线程安全**：当前实现不是线程安全的，多线程使用时需要外部同步
5. **内存管理**：规划器会自动管理内部状态，无需手动释放资源

## 错误处理

- `startPlanning()` 返回false表示规划失败，可能原因：
  - 参数不合理（如距离为负、速度超限等）
  - 给定约束条件下无法完成规划
- `getNextState()` 返回false表示规划已完成或出现错误
- 使用前确保已正确设置规划参数

## 性能优化建议

1. 避免频繁调用`setParams()`，在初始化时设置一次即可
2. 合理设置控制周期，过小会增加计算负担
3. 对于连续轨迹，可以在前一段轨迹即将结束时提前规划下一段