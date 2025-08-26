#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include "ethercat/EtherCATInterface.h"
#include <thread>
#include <iostream>
#include <math.h>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <fstream>
#include <algorithm>
#include "bspline/BSplineSmoothing.h"
#include "bspline/TargetJointsDataManager.h"




extern SharedMemoryManager<SharedMemoryData> shm;

Robot::Robot()
{

}

Robot::~Robot()
{
}

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);

    m_angleLimitMax[0] = M_PI * 170.0f / 180.0f;
    m_angleLimitMin[0] = -M_PI * 170.0f / 180.0f;
    m_angleLimitMax[1] = M_PI * 130.0f / 180.0f;
    m_angleLimitMin[1] = -M_PI * 110.0f / 180.0f;
    m_angleLimitMax[2] = M_PI * 155.0f / 180.0f;
    m_angleLimitMin[2] = -M_PI * 65.0f / 180.0f;
    m_angleLimitMax[3] = M_PI * 180.0f / 180.0f;
    m_angleLimitMin[3] = -M_PI * 180.0f / 180.0f;
    m_angleLimitMax[4] = M_PI * 123.0f / 180.0f;
    m_angleLimitMin[4] = -M_PI * 123.0f / 180.0f;
    m_angleLimitMax[5] = M_PI * 455.0f / 180.0f;
    m_angleLimitMin[5] = -M_PI * 455.0f / 180.0f;

#if 0  //初始参数读取
    // 在初始化阶段执行“初始参数读取”，仅处理 SetParm，其他命令禁止执行
    using namespace std::chrono;
    std::cout << "开始参数同步，请稍候..." << std::endl;

    // 至少收到过一个 SetParm 以后，若超过该静默时间未再收到新的 SetParm，则判定同步完成
    const auto idle_timeout = std::chrono::milliseconds(1000);
    auto last_recv = std::chrono::steady_clock::now();
    bool got_any_setparm = false;

    while (true) {
        bool received_any = false;
        HighLevelCommand cmd;

        // 先处理高优先级队列，仅消费 SetParm
        while (shm().high_prio_cmd_queue.pop(cmd)) {
            received_any = true;
            if (cmd.command_type == HighLevelCommandType::SetParm) {
                handleParameterOrder(cmd);
                last_recv = std::chrono::steady_clock::now();
                got_any_setparm = true;
            } else {
                // 同步阶段禁止其他操作：忽略非参数指令
                std::cout << "[参数同步中] 忽略高优先级命令（非参数设置）" << std::endl;
            }
        }

        // 再处理普通队列，仅消费 SetParm
        while (shm().cmd_queue.pop(cmd)) {
            received_any = true;
            if (cmd.command_type == HighLevelCommandType::SetParm) {
                handleParameterOrder(cmd);
                last_recv = std::chrono::steady_clock::now();
                got_any_setparm = true;
            } else {
                // 同步阶段禁止其他操作：忽略非参数指令
                std::cout << "[参数同步中] 忽略普通命令（非参数设置）" << std::endl;
            }
        }

        auto now = std::chrono::steady_clock::now();
        // 完成条件：已至少接收过一次 SetParm，且超过静默时间未再收到新的 SetParm
        if (got_any_setparm &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_recv) > idle_timeout) {
            break;
        }

        // 若本轮没有收到任何命令，稍作等待，以避免忙等
        if (!received_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::cout << "参数同步完成" << std::endl;
#endif 
    std::cout << "机器人初始化完成" << std::endl;
}

void Robot::setEnable(bool _enabled)
{
    // motorJ[ALL]->SetEnable(_enable);
    // 控制器使能
//    ctrl->setEnable(_enabled);
    enabled = _enabled;
}

bool Robot::isEnabled() const
{
    return enabled;
}


void Robot::emergecyStop()
{
    // TO DO
    // context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
    //     context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    // context->MoveJoints(context->targetJoints);
    enabled = false;
    // clearFifo();
}


void Robot::moveJ(const std::array<float, NUM_JOINTS> &_joint_pos, float _speed, float _start_speed, float _end_speed)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
    // 检查目标关节位置是否在限制范围内
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (_joint_pos[i] > m_angleLimitMax[i] || _joint_pos[i] < m_angleLimitMin[i]) {
            std::cerr << "关节 " << i << " 目标位置超出限制范围!" << std::endl;
            return;
        }
    }
    
    // 计算每个关节的运动距离，找出最大距离
    std::array<double, NUM_JOINTS> jointDistances;
    double maxDistance = 0.0;
    int maxDistanceJoint = 0;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        jointDistances[i] = std::abs(_joint_pos[i] - m_curJoints[i]);
        if (jointDistances[i] > maxDistance) {
            maxDistance = jointDistances[i];
            maxDistanceJoint = i;
        }
    }
    
    // 如果距离太小，直接返回
    if (maxDistance < 1e-6) {
        std::cout << "目标位置与当前位置相同，无需移动" << std::endl;
        return;
    }
    
    // 设置主规划器参数（基于最长距离关节）
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 25 * M_PI / 180.0;  // 直接使用关节最大速度作为系统限制
    params.maxAcceleration = 50 * M_PI / 180.0;  // 不使用比例系数
    params.maxJerk = 200 * M_PI / 180.0;
    params.targetVelocity = _speed * M_PI / 180.0;  // 直接使用用户输入的设定速度
    params.startVelocity = _start_speed * M_PI / 180.0;  // 
    params.endVelocity = _end_speed * M_PI / 180.0;    // 
    params.totalDistance = maxDistance;  // 使用最大距离作为规划距离
    
    // 初始化主速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "关节空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    // 记录起始关节位置
    std::array<float, NUM_JOINTS> startJoints = m_curJoints;
    
    // 执行插补循环
    bool motionCompleted = false;
    int cycleCount = 0;

    while (!motionCompleted && !GlobalParams::isStop) {
        // 检查暂停/恢复（非阻塞，允许规划器内部平滑过程在 getNextState 中推进）
        if (GlobalParams::isPause) {
            // 每个周期都调用一次，内部有状态保护，不会重复生效
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            // 仅当规划器已经真正处于暂停态时，才启动平滑恢复
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态（必须每周期调用，以推进平滑暂停/恢复过程）
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插补比例系数
        double lambda = state.position / maxDistance;
        
        // 限制插补比例在[0,1]范围内
        lambda = std::max(0.0, std::min(1.0, lambda));

        cycleCount++;
        // 每100个周期打印一次状态信息
        if (cycleCount % 100 == 0) {
            std::cout << "周期: " << cycleCount 
                      << ", 位置: " << m_curJoints[maxDistanceJoint] * 180.0 / M_PI << "°"
                      << ", 速度: " << state.velocity * 180.0 / M_PI << "°/s"
                      << ", 加速度: " << state.acceleration * 180.0 / M_PI << "°/s²"
                      << ", 插补比例: " << lambda * 100.0 << "%" << std::endl;
        }
        
        // 根据插补比例计算所有关节的当前位置
        for (int i = 0; i < NUM_JOINTS; i++) {
            // 使用插补比例系数同步所有关节
            m_curJoints[i] = startJoints[i] + lambda * (_joint_pos[i] - startJoints[i]);
            
            // 计算关节速度（用于监控）
            if (jointDistances[i] > 1e-6) {
                double jointDirection = (_joint_pos[i] > startJoints[i]) ? 1.0 : -1.0;
                m_curVelocity[i] = jointDirection * state.velocity * (jointDistances[i] / maxDistance);
            } else {
                m_curVelocity[i] = 0.0;
            }
            
            // 限制关节位置在安全范围内
            m_curJoints[i] = std::max(m_angleLimitMin[i], 
                                     std::min(m_angleLimitMax[i], m_curJoints[i]));
        }
        
        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        
        // 发送关节位置到硬件
        moveJoints(m_curJoints);
        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "关节插补停止" << std::endl;
    } else {
        std::cout << "关节插补完成" << std::endl;
    }
}


/* 笛卡尔空间直线插补 */
void Robot::moveL(std::array<float, NUM_JOINTS> _pose, float _speed, float _start_speed, float _end_speed)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;

    // std::ofstream csvFile("robot_joints_data.csv", std::ios::trunc);
    // if (csvFile.is_open()) {
    //     csvFile.close();
    // }

    // CSV写入计数器
    int csv_write_counter = 0;
    const int CSV_WRITE_INTERVAL = 10; // 每隔10个点写入一次

    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);

    // 计算起始和目标位置
    Eigen::Vector3d start_p(current_pose.X, current_pose.Y, current_pose.Z);
    Eigen::Vector3d end_p(_pose[0], _pose[1], _pose[2]);
    
    // 计算起始和目标姿态四元数
    Eigen::Quaterniond start_q = Eigen::AngleAxisd(current_pose.C, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(current_pose.B, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(current_pose.A, Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond end_q = Eigen::AngleAxisd(_pose[5], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(_pose[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(_pose[3], Eigen::Vector3d::UnitX());

    // 计算位置距离
    double position_distance = (end_p - start_p).norm();
    
    // 计算姿态距离（四元数角度差）
    double orientation_angle = start_q.angularDistance(end_q);  // 弧度
    
    // 将姿态角度转换为等效的线性距离（转换系数可调整）
    // 假设1弧度的姿态变化等效于100mm的位置变化
    const double ORIENTATION_TO_POSITION_RATIO = 100.0;  // mm/rad
    double orientation_distance = orientation_angle * ORIENTATION_TO_POSITION_RATIO;
    
    // 取位置距离和姿态距离的最大值作为插补距离
    double distance = std::max(position_distance, orientation_distance);
    
    if (distance < 1e-6) {
        std::cout << "目标位置与当前位置过于接近，运动结束" << std::endl;
        return;
    }

    // 设置速度规划器参数
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 100.0;  // 笛卡尔空间最大速度 (mm/s)
    params.maxAcceleration = 200.0;  // 最大加速度 (mm/s²)
    params.maxJerk = 800.0;  // 最大加加速度 (mm/s³)
    params.targetVelocity = _speed;  // 用户设定速度
    params.startVelocity = _start_speed;  // 起始速度
    params.endVelocity = _end_speed;    // 结束速度
    params.totalDistance = distance;  // 总距离
    
    // 初始化速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "笛卡尔空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    std::cout << "开始笛卡尔直线运动，距离: " << distance << " mm" << std::endl;

    // 运动控制循环
    bool motionCompleted = false;
    int cycleCount = 0;
    
    while (!motionCompleted) {
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }

        // 计算插补比例
        double interp_ratio = state.position / distance;
        interp_ratio = std::max(0.0, std::min(1.0, interp_ratio));
        
        // 位置插值（线性插值）
        Eigen::Vector3d current_p = start_p + interp_ratio * (end_p - start_p);
        
        // 姿态插值（球面线性插值）
        Eigen::Quaterniond current_q = start_q.slerp(interp_ratio, end_q);
        
        // 将四元数转换回欧拉角
        Eigen::Matrix3d R = current_q.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX顺序
        
        // 构造当前目标位姿
        Kine6d target_pose;
        target_pose.X = current_p.x();
        target_pose.Y = current_p.y();
        target_pose.Z = current_p.z();
        // target_pose.A = euler[2]; // Roll (X)
        // target_pose.B = euler[1]; // Pitch (Y)
        // target_pose.C = euler[0]; // Yaw (Z)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_pose.R[i * 3 + j] = R(i, j);
            }
        }
        target_pose.fgR = 1;
        
        // 通过逆向运动学求解关节角度
        Kine6dSol q_sol;
        classic6dofInvKine(&target_pose, m_curJoints.data(), &q_sol);
        
        // 选择最优解 - 改进版本
        bool valid[8];
        int best = -1;
        float best_score = 1e6;
        static int last_best_solution = -1; // 记录上一次选择的解

        for (int k = 0; k < 8; ++k) {
            valid[k] = true;
            // 检查关节限制
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j]) {
                    valid[k] = false;
                    break;
                }
            }
            
            if (valid[k]) {
                // 计算综合评分
                float position_dist = 0;
                float velocity_penalty = 0;
                float continuity_bonus = 0;
                
                // 位置距离权重
                for (int j = 0; j < NUM_JOINTS; ++j) {
                    position_dist += std::pow(m_curJoints[j] - q_sol.sol[k][j], 2);
                }
                
                // 速度连续性惩罚（如果有上一次的解）
                if (last_best_solution >= 0 && cycleCount > 0) {
                    for (int j = 0; j < NUM_JOINTS; ++j) {
                        float vel_diff = std::abs(q_sol.sol[k][j] - q_sol.sol[last_best_solution][j]);
                        velocity_penalty += vel_diff * 10.0; // 速度跳跃惩罚权重
                    }
                }
                
                // 解一致性奖励
                if (k == last_best_solution) {
                    continuity_bonus = -50.0; // 优先选择相同分支的解
                }
                
                float total_score = position_dist + velocity_penalty + continuity_bonus;
                
                if (total_score < best_score) {
                    best = k;
                    best_score = total_score;
                }
            }
        }

        // 更新记录
        if (best >= 0) {
            last_best_solution = best;
            //检查关节角度变化
            bool angleChangeOk = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (fabs(q_sol.sol[best][j] - m_curJoints[j]) > 0.1) {
                    angleChangeOk = false;
                    break;
                }
            }
            
            if (!angleChangeOk) {
                std::cerr << "关节角度变化过大,退出插补！" << std::endl;
                return;
            }

            std::array<float, NUM_JOINTS> target_joints;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target_joints[j] = q_sol.sol[best][j];
            
            // 更新当前关节角度
            m_curJoints = target_joints;
            
            //输出到CSV文件
            // csv_write_counter++;
            // if (csv_write_counter >= CSV_WRITE_INTERVAL) {
            //     std::ofstream csvFile("robot_joints_data.csv", std::ios::app);
            //     if (csvFile.is_open()) {
            //         csvFile << target_joints[0] << "," << target_joints[1] << "," << target_joints[2] 
            //                << "," << target_joints[3] << "," << target_joints[4] << "," << target_joints[5] << "\n";
            //         csvFile.close();
            //     }
            //     csv_write_counter = 0; // 重置计数器
            // }
            
            // 发送关节角度到底层控制器
            moveJoints(target_joints);
        } else {
            std::cout << "逆运动学求解失败，运动停止" << std::endl;
            break;
        }
        
        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        
        cycleCount++;
        if (cycleCount % 100 == 0) {
            std::cout << "周期: " << cycleCount
                      << ", 位置: [" << current_p.x() << ", " << current_p.y() << ", " << current_p.z() << "]"
                      << ", 速度: " << state.velocity
                      << ", 加速度: " << state.acceleration
                      << ", 插补比例: " << interp_ratio * 100.0 << "%" << std::endl;
        }
        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "直线插补停止" << std::endl;
    } else {
        std::cout << "直线插补完成" << std::endl;
    }
}


/*笛卡尔空间圆弧插补*/
void Robot::moveC(std::array<float, NUM_JOINTS> mid_pose, std::array<float, NUM_JOINTS> end_pose, float speed, float start_speed, float end_speed)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;

    // CSV写入计数器
    int csv_write_counter = 0;
    const int CSV_WRITE_INTERVAL = 10; // 每隔10个点写入一次

    // 获取当前位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);
    
    // 转换为Eigen向量
    Eigen::Vector3d start_pos(current_pose.X, current_pose.Y, current_pose.Z);
    Eigen::Vector3d mid_pos(mid_pose[0], mid_pose[1], mid_pose[2]);
    Eigen::Vector3d end_pos(end_pose[0], end_pose[1], end_pose[2]);
    
    // 检查三点是否共线
    Eigen::Vector3d v1 = mid_pos - start_pos;
    Eigen::Vector3d v2 = end_pos - mid_pos;
    Eigen::Vector3d cross = v1.cross(v2);
    
    if (cross.norm() < 1e-6) {
        std::cout << "三点共线，切换到直线插补" << std::endl;
        moveL(end_pose, speed, start_speed, end_speed);
        return;
    }
    
    // 计算圆弧参数
    Eigen::Vector3d center;
    double radius;
    Eigen::Vector3d normal;
    double angle_total;
    
    // 计算圆心
    Eigen::Vector3d mid1 = (start_pos + mid_pos) / 2.0;
    Eigen::Vector3d mid2 = (mid_pos + end_pos) / 2.0;
    
    Eigen::Vector3d dir1 = (mid_pos - start_pos).normalized();
    Eigen::Vector3d dir2 = (end_pos - mid_pos).normalized();
    
    normal = dir1.cross(dir2).normalized();
    
    Eigen::Vector3d perp1 = normal.cross(dir1);
    Eigen::Vector3d perp2 = normal.cross(dir2);
    
    // 求解圆心
    Eigen::Matrix3d A;
    A.col(0) = perp1;
    A.col(1) = -perp2;
    A.col(2) = normal;
    
    Eigen::Vector3d b = mid2 - mid1;
    Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);
    
    center = mid1 + solution(0) * perp1;
    radius = (start_pos - center).norm();
    
    // 计算总角度
    Eigen::Vector3d vec_start = (start_pos - center).normalized();
    Eigen::Vector3d vec_end = (end_pos - center).normalized();
    angle_total = std::acos(std::clamp(vec_start.dot(vec_end), -1.0, 1.0));
    
    // 计算圆弧长度
    double arc_length = radius * angle_total;

    if (arc_length < 1e-6) {
        std::cout << "目标位置与当前位置过于接近，运动结束" << std::endl;
        return;
    }
    
    // 初始化速度规划器
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;  // 1ms
    params.maxVelocity = 100.0;  // mm/s
    params.maxAcceleration = 200.0;  // mm/s²
    params.maxJerk = 600.0;  // mm/s³
    params.targetVelocity = speed;
    params.startVelocity = start_speed;
    params.endVelocity = end_speed;
    params.totalDistance = arc_length;
    
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "速度规划器初始化失败" << std::endl;
        return;
    }
    
    // 姿态插值准备
    Eigen::Matrix3d R_start, R_end;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_start(i, j) = current_pose.R[i * 3 + j];
        }
    }
    
    // 计算目标姿态
    Kine6d target_pose;
    target_pose.X = end_pose[0];
    target_pose.Y = end_pose[1];
    target_pose.Z = end_pose[2];
    target_pose.A = end_pose[3];
    target_pose.B = end_pose[4];
    target_pose.C = end_pose[5];
    target_pose.fgR = 0;
    
    Eigen::Vector3d euler_end(target_pose.C, target_pose.B, target_pose.A);
    R_end = Eigen::AngleAxisd(euler_end[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_end[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_end[2], Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond quat_start(R_start);
    Eigen::Quaterniond quat_end(R_end);
    
    // 计算圆弧插值的基向量
    Eigen::Vector3d n = vec_start;
    Eigen::Vector3d m = normal.cross(n).normalized();
    
    int cycleCount = 0;
    bool motionCompleted = false;
    
    // 插补循环
    while (!motionCompleted) {
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插值比例
        double ratio = state.position / arc_length;
        ratio = std::clamp(ratio, 0.0, 1.0);
        
        // 圆弧位置插值
        double theta = ratio * angle_total;
        Eigen::Vector3d pos = center + std::cos(theta) * n * radius + std::sin(theta) * m * radius;
        
        // 姿态插值
        Eigen::Quaterniond quat_interp = quat_start.slerp(ratio, quat_end);
        Eigen::Matrix3d R = quat_interp.toRotationMatrix();
        
        // 构造目标位姿
        Kine6d pose;
        pose.X = pos.x();
        pose.Y = pos.y();
        pose.Z = pos.z();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                pose.R[i * 3 + j] = R(i, j);
            }
        }
        pose.fgR = 1;
        
        // 逆运动学求解
        Kine6dSol q_sol;
        classic6dofInvKine(&pose, m_curJoints.data(), &q_sol);
        
        // 选择最优解
        bool valid[8];
        int best = -1;
        float best_dist = 1e6;
        for (int k = 0; k < 8; ++k) {
            valid[k] = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j]) {
                    valid[k] = false;
                    break;
                }
            }
            if (valid[k]) {
                float dist = 0;
                for (int j = 0; j < NUM_JOINTS; ++j)
                    dist += std::pow(m_curJoints[j] - q_sol.sol[k][j], 2);
                if (dist < best_dist) {
                    best = k;
                    best_dist = dist;
                }
            }
        }
        
        // 检查关节角度变化
        if (best >= 0) {
            bool angleChangeOk = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (fabs(q_sol.sol[best][j] - m_curJoints[j]) > 0.1) {
                    angleChangeOk = false;
                    break;
                }
            }
            
            if (!angleChangeOk) {
                std::cerr << "关节角度变化过大,退出插补！" << std::endl;
                return;
            }
            
            std::array<float, NUM_JOINTS> target_joints;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target_joints[j] = q_sol.sol[best][j];
            m_curJoints = target_joints;

            // 输出到CSV文件
            // csv_write_counter++;
            // if (csv_write_counter >= CSV_WRITE_INTERVAL) {
            //     std::ofstream csvFile("robot_joints_data.csv", std::ios::app);
            //     if (csvFile.is_open()) {
            //         csvFile << target_joints[0] << "," << target_joints[1] << "," << target_joints[2] 
            //                << "," << target_joints[3] << "," << target_joints[4] << "," << target_joints[5] << "\n";
            //         csvFile.close();
            //     }
            //     csv_write_counter = 0; // 重置计数器
            // }

            //发送关节角度到底层控制器
            moveJoints(target_joints);
        }

        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        
        cycleCount++;
        // 每100个周期打印调试信息
        if (cycleCount % 100 == 0) {
            std::cout << "周期: " << cycleCount 
                      << ", 位置: (" << pose.X << ", " << pose.Y << ", " << pose.Z << ")"
                      << ", 速度: " << state.velocity << " mm/s"
                      << ", 加速度: " << state.acceleration << " mm/s²"
                      << ", 插补比例: " << ratio << std::endl;
        }        
    }

    if (GlobalParams::isStop) {
        std::cout << "圆弧插补停止" << std::endl;
    } else {
        std::cout << "圆弧插补完成" << std::endl;
    }  
}



void Robot::moveCF(std::array<float, NUM_JOINTS> pose1, std::array<float, NUM_JOINTS> pose2, float speed)
{
    
}

/* 关节空间手动模式（连续和寸动）*/
void Robot::jogJ(int _mode, int _index, int _direction, int speed, int angle)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
        
    // 点动速度设置
    const float jog_speed = speed / 4.0f;  // 点动速度 (度/秒)
    // 微动角度设置
    const float micro_move_angle = (M_PI / 180.0f) * angle;  // 微动角度 (弧度)

    // 获取关节限位
    float joint_limit_max = m_angleLimitMax[_index];  // 弧度
    float joint_limit_min = m_angleLimitMin[_index];  // 弧度
    
    // 获取当前关节位置
    float current_pos = m_curJoints[_index];
    //目标位置
    float target_pos = 0.0f;
    
    //判断连续还是微动
    if (_mode == 0)
    {
        // 根据方向确定目标位置
        if (_direction == 1) {
            target_pos = joint_limit_max;  // 正向运动到上限位
            
            // 检查是否已经在上限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_max_deg = joint_limit_max * 180.0f / M_PI;
            
            if (current_pos_deg >= limit_max_deg - 0.1f) {
                std::cout << "jogJ - 关节" << _index << " 已在上限位附近,无需点动" << std::endl;
                return;
            }
        } else {
            target_pos = joint_limit_min;  // 负向运动到下限位
            
            // 检查是否已经在下限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_min_deg = joint_limit_min * 180.0f / M_PI;
            
            if (current_pos_deg <= limit_min_deg + 0.1f) {
                std::cout << "jogJ - 关节" << _index << " 已在下限位附近,无需点动" << std::endl;
                return;
            }
        }
    }

    if (_mode == 1)
    {
        // 根据方向确定目标位置
        if (_direction == 1) {
            target_pos = current_pos + micro_move_angle;  // 正向微动
            
            // 检查是否已经在上限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_max_deg = joint_limit_max * 180.0f / M_PI;
            
            if (current_pos_deg >= limit_max_deg - 0.1f) {
                std::cout << "jogJ - 关节" << _index << " 已在上限位附近,无需点动" << std::endl;
                return;
            }
        } else {
            target_pos = current_pos - micro_move_angle;  // 负向微动
            
            // 检查是否已经在下限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_min_deg = joint_limit_min * 180.0f / M_PI;
            
            if (current_pos_deg <= limit_min_deg + 0.1f) {
                std::cout << "jogJ - 关节" << _index << " 已在下限位附近,无需点动" << std::endl;
                return;
            }
        }
    }
    
    
    // 构造目标关节位置数组 - 只移动指定关节,其他关节保持当前位置
    std::array<float, NUM_JOINTS> target_joints = m_curJoints;
    target_joints[_index] = target_pos;
    
    // 调用moveJ函数执行点动到限位
    moveJ(target_joints, jog_speed, 0, 0);
    
    std::cout << "jogJ - 关节" << _index << " 点动完成,当前位置: " 
              << m_curJoints[_index] * 180.0f / M_PI << "度" << std::endl;
}

/* 笛卡尔空间手动模式（寸动）*/
void Robot::jogL(int mode, int axis, int _direction, int speed, int line_distance, int angle_distance)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);
    
    // 点动参数设置
    const float linear_jog_distance = line_distance;   // 线性轴点动距离 (mm)
    const float angular_jog_distance = angle_distance;  // 角度轴点动距离 (度)
    const float jog_speed = speed;             // 点动速度
    
    // 计算目标位姿
    Kine6d target_pose = current_pose;
    bool _flag = false;
    Kine6dSol _q;
    Kine6d _pose;
    if (mode == 0)          
    {
        switch (axis)
        {
            case 1:
            //直接给X轴工作空间最大限位,在这里循环判断,递增或递减目标位置的X值,直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.X = m_workspaceLimitMaxX;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.X -= 10.0f; // 如果逆解验证失败,递减X轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 X 位置无任何有效逆解，继续向负方向搜索，直到越界为止
                            target_pose.X -= 10.0f;
                            if (target_pose.X < current_pose.X) {
                                std::cerr << "No valid solution found within X workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.X = m_workspaceLimitMinX;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.X += 10.0f; // 如果逆解验证失败,递增X轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 X 位置无任何有效逆解，继续向正方向搜索，直到越界为止
                            target_pose.X += 10.0f;
                            if (target_pose.X > current_pose.X) {
                                std::cerr << "No valid solution found within X workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                std::cout << "jogL - X轴移动 " << std::endl;
                break;
            case 2:
            //直接给Y轴工作空间最大限位,在这里循环判断,递增或递减目标位置的Y值,直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.Y = m_workspaceLimitMaxY;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Y -= 10.0f; // 如果逆解验证失败,递减Y轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 Y 位置无任何有效逆解，继续向负方向搜索，直到越界为止
                            target_pose.Y -= 10.0f;
                            if (target_pose.Y < current_pose.Y) {
                                std::cerr << "No valid solution found within Y workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.Y = m_workspaceLimitMinY;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Y += 10.0f; // 如果逆解验证失败,递增Y轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 Y 位置无任何有效逆解，继续向正方向搜索，直到越界为止
                            target_pose.Y += 10.0f;
                            if (target_pose.Y > current_pose.Y) {
                                std::cerr << "No valid solution found within Y workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                std::cout << "jogL - Y轴移动 " << std::endl;               
                break;
            case 3:
            //直接给Z轴工作空间最大限位,在这里循环判断,递增或递减目标位置的Z值,直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.Z = m_workspaceLimitMaxZ;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Z -= 10.0f; // 如果逆解验证失败,递减Z轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 Z 位置无任何有效逆解，继续向负方向搜索，直到越界为止
                            target_pose.Z -= 10.0f;
                            if (target_pose.Z < current_pose.Z) {
                                std::cerr << "No valid solution found within Z workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.Z = m_workspaceLimitMinZ;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Z += 10.0f; // 如果逆解验证失败,递增Z轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            // 若当前 Z 位置无任何有效逆解，继续向正方向搜索，直到越界为止
                            target_pose.Z += 10.0f;
                            if (target_pose.Z > current_pose.Z) {
                                std::cerr << "No valid solution found within Z workspace!" << std::endl;
                                return;
                            }
                        }
                    }
                }
                std::cout << "jogL - Z轴移动 " << std::endl;
                break;
            case 4:
                target_pose.A = (_direction == 1) ? M_PI : -M_PI;//(175.0f * M_PI / 180.0f) : -(175.0f * M_PI / 180.0f);
                std::cout << "jogL - A轴旋转 " << std::endl; 
                break;
            case 5:
                //Pitch俯仰角在接近正负90度和正负180度时姿态奇异
                target_pose.B = (_direction == 1) ? (90.0f * M_PI / 180.0f) : -(90.0f * M_PI / 180.0f);
                std::cout << "jogL - B轴旋转 " << std::endl; 
                break;
            case 6:
                target_pose.C = (_direction == 1) ? M_PI : -M_PI;//(175.0f * M_PI / 180.0f) : -(175.0f * M_PI / 180.0f);
                std::cout << "jogL - C轴旋转 " << std::endl; 
                break;
            default:
                break;
        }
    }
    if (mode == 1)
    {
    // 根据轴和方向设置目标位姿
        float increment = 0.0f;
        switch (axis) {
            case 1:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.X += increment;
                std::cout << "jogL - X轴移动 " << increment << "mm" << std::endl;
                break;
                
            case 2:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.Y += increment;
                std::cout << "jogL - Y轴移动 " << increment << "mm" << std::endl;
                break;

            case 3:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.Z += increment;
                std::cout << "jogL - Z轴移动 " << increment << "mm" << std::endl;
                break;
                
            case 4:
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.A += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - A轴旋转 " << increment << "度" << std::endl;
                break;
                
            case 5:
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.B += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - B轴旋转 " << increment << "度" << std::endl;
                break;
                
            case 6:  // Yaw轴（绕Z轴旋转）
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.C += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - C轴旋转 " << increment << "度" << std::endl;
                break;
        }
    } 
    // 构造目标位姿数组
    std::array<float, 6> target_pose_array = {
        target_pose.X, target_pose.Y, target_pose.Z,
        target_pose.A, target_pose.B, target_pose.C
    };
    
    // std::cout << "target_pose.fR = " << target_pose.fgR << std::endl;
    std::cout << "jogL - 目标位姿: X=" << target_pose.X 
              << " Y=" << target_pose.Y << " Z=" << target_pose.Z
              << " A=" << target_pose.A * 180.0f / M_PI
              << " B=" << target_pose.B * 180.0f / M_PI  
              << " C=" << target_pose.C * 180.0f / M_PI << std::endl;

    // 调用moveL函数执行笛卡尔空间运动
    moveL(target_pose_array, jog_speed, 0, 0);
    
    std::cout << "jogL - 笛卡尔点动完成" << std::endl;
}

void Robot::twoMoveL_BSplineTransition(std::array<float, NUM_JOINTS>& first_pose,std::array<float, NUM_JOINTS>& second_pose,const std::array<float, 2>& _speed, const std::array<float, 2>& _start_speed, const std::array<float, 2>& _end_speed)
{

    struct timespec start, end;
    // 记录开始时间（CLOCK_MONOTONIC：单调时钟，类似steady_clock）
    clock_gettime(CLOCK_MONOTONIC, &start);

    

    BSplineSmoothing *smoother = BSplineSmoothing::getInstance(3, 60, 50, 1);

    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);


    Point3D start_point;

    start_point.x = current_pose.X;
    start_point.y = current_pose.Y;
    start_point.z = current_pose.Z;


    LineSegment seg1(start_point, Point3D(first_pose[0],first_pose[1],first_pose[2]));
    LineSegment seg2(Point3D(first_pose[0],first_pose[1],first_pose[2]), Point3D(second_pose[0],second_pose[1],second_pose[2]));
    smoother->generateSmoothCurve(seg1, seg2, 1000);
    Point3D BSpline_start_point = smoother->getSmoothedCurve().front();
    Point3D BSpline_end_point = smoother->getSmoothedCurve().back();


    first_pose[0] = BSpline_start_point.x;
    first_pose[1] = BSpline_start_point.y;
    first_pose[2] = BSpline_start_point.z;




    moveL_Avoid_points(first_pose,20,0,20);


    moveBSpline(20,20,20);


    // moveL_Avoid_points(second_pose,20,20,0);



    JointDataManager::getInstance()->save_all_change_CSV("all_jointdata.csv");

    // 记录结束时间
    clock_gettime(CLOCK_MONOTONIC, &end);
    // 计算时间差（纳秒级）
    long long duration_ns = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
    double duration_ms = duration_ns / 1000000.0;  // 转换为毫秒
    std::cout << "B样条平滑计算" << duration_ms << " 毫秒\n";

}

void Robot::moveBSpline(float _speed, float _start_speed, float _end_speed)
{

    BSplineSmoothing* bsplineSmoothing = BSplineSmoothing::getInstance();

    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;


    // CSV写入计数器
    int csv_write_counter = 0;
    const int CSV_WRITE_INTERVAL = 10; // 每隔10个点写入一次

    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);

    // 计算起始和目标位置
    Eigen::Vector3d start_p(current_pose.X, current_pose.Y, current_pose.Z);
//    Eigen::Vector3d end_p(_pose[0], _pose[1], _pose[2]);
    
    // 计算起始和目标姿态四元数
    // B样条插补时先默认姿态不变
    Eigen::Quaterniond start_q = Eigen::AngleAxisd(current_pose.C, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(current_pose.B, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(current_pose.A, Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond end_q = Eigen::AngleAxisd(current_pose.C, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(current_pose.B, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(current_pose.A, Eigen::Vector3d::UnitX());

    // 计算直线距离
    // double distance = (end_p - start_p).norm();
    double distance = bsplineSmoothing->getCurveLength();
    
    
    if (distance < 1e-6) {
        std::cout << "目标位置与当前位置过于接近，运动结束" << std::endl;
        return;
    }

    // 设置速度规划器参数
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 100.0;  // 笛卡尔空间最大速度 (mm/s)
    params.maxAcceleration = 200.0;  // 最大加速度 (mm/s²)
    params.maxJerk = 600.0;  // 最大加加速度 (mm/s³)
    params.targetVelocity = _speed;  // 用户设定速度
    params.startVelocity = _start_speed;  // 起始速度
    params.endVelocity = _end_speed;    // 结束速度
    params.totalDistance = distance;  // 总距离
    
    // 初始化速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "笛卡尔空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    std::cout << "开始笛卡尔直线运动，距离: " << distance << " mm" << std::endl;

    // 运动控制循环
    bool motionCompleted = false;
    int cycleCount = 0;
    

    JointDataManager* a = JointDataManager::getInstance();

    while (!motionCompleted) {
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插补比例
        double interp_ratio = state.position / distance;
        interp_ratio = std::max(0.0, std::min(1.0, interp_ratio));
        
        // 位置插值（线性插值）
        // Eigen::Vector3d current_p = start_p + interp_ratio * (end_p - start_p);
        Point3D current_p = bsplineSmoothing->get_BSpline_point(interp_ratio);
        
        // 姿态插值（球面线性插值）
        Eigen::Quaterniond current_q = start_q.slerp(interp_ratio, end_q);
        
        // 将四元数转换回欧拉角
        Eigen::Matrix3d R = current_q.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX顺序
        
        // 构造当前目标位姿
        Kine6d target_pose;
        target_pose.X = current_p.x;
        target_pose.Y = current_p.y;
        target_pose.Z = current_p.z;
        // target_pose.A = euler[2]; // Roll (X)
        // target_pose.B = euler[1]; // Pitch (Y)
        // target_pose.C = euler[0]; // Yaw (Z)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_pose.R[i * 3 + j] = R(i, j);
            }
        }
        target_pose.fgR = 1;
        
        // 通过逆向运动学求解关节角度
        Kine6dSol q_sol;
        classic6dofInvKine(&target_pose, m_curJoints.data(), &q_sol);
        
        // 选择最优解
        bool valid[8];
        int best = -1;
        float best_dist = 1e6;
        for (int k = 0; k < 8; ++k) {
            valid[k] = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j]) {
                    valid[k] = false;
                    break;
                }
            }
            if (valid[k]) {
                float dist = 0;
                for (int j = 0; j < NUM_JOINTS; ++j)
                    dist += std::pow(m_curJoints[j] - q_sol.sol[k][j], 2);
                if (dist < best_dist) {
                    best = k;
                    best_dist = dist;
                }
            }
        }
        
        if (best >= 0) {
            std::array<float, NUM_JOINTS> target_joints;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target_joints[j] = q_sol.sol[best][j];
            
            // 更新当前关节角度
            m_curJoints = target_joints;
            a->addData(m_curJoints,q_sol.sol_flag[best][0],q_sol.sol_flag[best][1],q_sol.sol_flag[best][2]);

            // 发送关节角度到底层控制器
            // moveJoints(target_joints);
        } else {
            // std::cout << "逆运动学求解失败，运动停止" << std::endl;
            break;
        }
        
        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        
        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "直线插补停止" << std::endl;
    } else {
        std::cout << "直线插补完成" << std::endl;
    }



    a->all_changed_jointDataList.insert(a->all_changed_jointDataList.end(),a->m_jointDataList.begin(),a->m_jointDataList.end());
    a->clear();
}

void Robot::moveL_Avoid_points(std::array<float, NUM_JOINTS> tar_pose, float _speed, float _start_speed, float _end_speed)
{
    
    JointDataManager* a = JointDataManager::getInstance();

    struct timespec start, end;
    // 记录开始时间（CLOCK_MONOTONIC：单调时钟，类似steady_clock）
    clock_gettime(CLOCK_MONOTONIC, &start);


    // 用于储存奇异区域
    long long index = 0L;
    long long _start_index =0L;
    long long _end_index = 0L;
    bool singular_flag = false;


    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;


    std::array<float, NUM_JOINTS> tmp_curJoints = m_curJoints;

    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(tmp_curJoints.data(), &current_pose);

    // 计算起始和目标位置
    Eigen::Vector3d start_p(current_pose.X, current_pose.Y, current_pose.Z);
    Eigen::Vector3d end_p(tar_pose[0], tar_pose[1], tar_pose[2]);
    
    // 计算起始和目标姿态四元数
    Eigen::Quaterniond start_q = Eigen::AngleAxisd(current_pose.C, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(current_pose.B, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(current_pose.A, Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond end_q = Eigen::AngleAxisd(tar_pose[5], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(tar_pose[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(tar_pose[3], Eigen::Vector3d::UnitX());

    // 计算直线距离
    double distance = (end_p - start_p).norm();
    
    if (distance < 1e-6) {
        std::cout << "目标位置与当前位置过于接近，运动结束" << std::endl;
        return;
    }

    // 设置速度规划器参数
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 100.0;  // 笛卡尔空间最大速度 (mm/s)
    params.maxAcceleration = 200.0;  // 最大加速度 (mm/s²)
    params.maxJerk = 600.0;  // 最大加加速度 (mm/s³)
    params.targetVelocity = _speed;  // 用户设定速度
    params.startVelocity = _start_speed;  // 起始速度
    params.endVelocity = _end_speed;    // 结束速度
    params.totalDistance = distance;  // 总距离
    
    // 初始化速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "笛卡尔空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    std::cout << "开始笛卡尔直线运动（奇异点规避），距离: " << distance << " mm" << std::endl;

    // 运动控制循环
    bool motionCompleted = false;
    int cycleCount = 0;

    // 奇异区域分隔
    JointDataManager::getInstance()->addData({0,0,0,0,0,0}, 0, 0, 0);
    
    while (!motionCompleted) {

        index++;
        
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插补比例
        double interp_ratio = state.position / distance;
        interp_ratio = std::max(0.0, std::min(1.0, interp_ratio));
        
        // 位置插值（线性插值）
        Eigen::Vector3d current_p = start_p + interp_ratio * (end_p - start_p);
        
        // 姿态插值（球面线性插值）
        Eigen::Quaterniond current_q = start_q.slerp(interp_ratio, end_q);
        
        // 将四元数转换回欧拉角
        Eigen::Matrix3d R = current_q.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX顺序
        
        // 构造当前目标位姿
        Kine6d target_pose;
        target_pose.X = current_p.x();
        target_pose.Y = current_p.y();
        target_pose.Z = current_p.z();
        // target_pose.A = euler[2]; // Roll (X)
        // target_pose.B = euler[1]; // Pitch (Y)
        // target_pose.C = euler[0]; // Yaw (Z)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_pose.R[i * 3 + j] = R(i, j);
            }
        }
        target_pose.fgR = 1;
        
        // 通过逆向运动学求解关节角度
        Kine6dSol q_sol;
        classic6dofInvKine(&target_pose, tmp_curJoints.data(), &q_sol);
        
        // 计算是否是奇异区域
        bool isSingular = (q_sol.sol_flag[0][0] == -1) || (q_sol.sol_flag[0][2] == -1) || (q_sol.sol_flag[0][1] == -1);

        if(isSingular)
        {
            // 刚进入奇异区域
            if(_start_index == 0)
            {
                _start_index = index;
            }
            else
            {
                _end_index = index;
            }
        }
        else
        {
            if(_start_index != _end_index)
            {
                _start_index = 0;
                _end_index = 0;
                singular_flag = true;
            }
        }


        // 选择最优解
        int is_over_limit[8] = {0,0,0,0,0,0,0,0};
        bool valid[8];
        int best = -1;
        float best_dist = 1e6;
        for (int k = 0; k < 8; ++k) {
            valid[k] = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j]) {
                    valid[k] = false;
                    is_over_limit[k] = 1;
                    break;
                }
            }
            if (valid[k]) {
                float dist = 0;
                for (int j = 0; j < NUM_JOINTS; ++j)
                    dist += std::pow(tmp_curJoints[j] - q_sol.sol[k][j], 2);
                if (dist < best_dist) {
                    best = k;
                    best_dist = dist;
                }
            }
        }
        
        if (best >= 0) {
            std::array<float, NUM_JOINTS> target_joints;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target_joints[j] = q_sol.sol[best][j];
            
            // 更新当前关节角度
            tmp_curJoints = target_joints;

            if(isSingular)
            {
                a->addData(target_joints, q_sol.sol_flag[best][0], q_sol.sol_flag[best][1], q_sol.sol_flag[best][2]);
            }

            if(singular_flag)
            {
                a->addData(target_joints, q_sol.sol_flag[best][0], q_sol.sol_flag[best][1], q_sol.sol_flag[best][2]);
                a->addData({0,0,0,0,0,0}, 0, 0, 0);
                singular_flag = false;
            }

        } else {
            // std::cout << "逆运动学求解失败，运动停止" << std::endl;
        }
        
        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        

        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "直线插补停止" << std::endl;
    } else {
        std::cout << "直线插补完成" << std::endl;
    }


    // 记录结束时间
    clock_gettime(CLOCK_MONOTONIC, &end);
    // 计算时间差（纳秒级）
    long long duration_ns = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
    double duration_ms = duration_ns / 1000000.0;  // 转换为毫秒
    std::cout << "笛卡尔直线运动预计算" << duration_ms << " 毫秒\n";

//    a->save_all_CSV("all_joint.csv");
//    a->save_CSV("signular.csv");


    if(a->collect_singular_area())
    {
        a->m_jointDataList.clear();
        moveL_handle(tar_pose,_speed,_start_speed,_end_speed);
    }
    else
    {
        a->m_jointDataList.clear();
        avoid_moveL(tar_pose,_speed,_start_speed,_end_speed);
    }
    a->all_changed_jointDataList.insert(a->all_changed_jointDataList.end(),a->m_jointDataList.begin(),a->m_jointDataList.end());
    a->clear();

}

void Robot::moveL_handle(std::array<float, 6> tar_tmp, float _speed, float _start_speed, float _end_speed)
{
    int double_index = 0;
    std::array<float, NUM_JOINTS> cur_joint_pos;     // 后续用于走分段moveL
    Kine6d tmp_pose;                  // 目标位姿
    if(JointDataManager::getInstance()->singular_pointsList.empty())
    {
        moveL(tar_tmp,_speed,_start_speed,_end_speed);
        return;
    }
    for(const auto &data : JointDataManager::getInstance()->singular_pointsList)
    {
        if(double_index % 2 == 0)
        {
            cur_joint_pos = data.targetJoints;
            classic6dofForKine(cur_joint_pos.data(), &tmp_pose);                             // 目标位姿 正解获取
            std::array<float, NUM_JOINTS> tmp = {tmp_pose.X,tmp_pose.Y,tmp_pose.Z,tmp_pose.A,tmp_pose.B,tmp_pose.C};;
            avoid_moveL(tmp,_speed,_start_speed,_speed);
        }
        else
        {
            cur_joint_pos = data.targetJoints;
            avoid_moveJ(cur_joint_pos,_speed,_speed,_speed);        // 奇异区域的过渡速度保持一致
            JointDataManager::getInstance()->combine(3000);
        }
        double_index++;
    }
    classic6dofForKine(cur_joint_pos.data(), &tmp_pose);                             // 目标位姿 正解获取

    if(fabs(tmp_pose.X -300) >= 0.1 || fabs(tmp_pose.Y -0) >= 0.1 || fabs(tmp_pose.Z -600) >= 0.1)
    {
        avoid_moveL(tar_tmp,_speed,_speed,_end_speed);   // 起始速度为_speed和之前前面的衔接
    }
}


void Robot::avoid_moveJ(const std::array<float, NUM_JOINTS>& _joint_pos, float _speed, float _start_speed, float _end_speed)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
    // 检查目标关节位置是否在限制范围内
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (_joint_pos[i] > m_angleLimitMax[i] || _joint_pos[i] < m_angleLimitMin[i]) {
            std::cerr << "关节 " << i << " 目标位置超出限制范围!" << std::endl;
            return;
        }
    }
    
    // 计算每个关节的运动距离，找出最大距离
    std::array<double, NUM_JOINTS> jointDistances;
    double maxDistance = 0.0;
    int maxDistanceJoint = 0;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        jointDistances[i] = std::abs(_joint_pos[i] - m_curJoints[i]);
        if (jointDistances[i] > maxDistance) {
            maxDistance = jointDistances[i];
            maxDistanceJoint = i;
        }
    }
    
    // 如果距离太小，直接返回
    if (maxDistance < 1e-6) {
        std::cout << "目标位置与当前位置相同，无需移动" << std::endl;
        return;
    }
    
    // 设置主规划器参数（基于最长距离关节）
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 20 * M_PI / 180.0;  // 直接使用关节最大速度作为系统限制
    params.maxAcceleration = 20 * M_PI / 180.0;  // 不使用比例系数
    params.maxJerk = 60 * M_PI / 180.0;
    params.targetVelocity = _speed * M_PI / 180.0;  // 直接使用用户输入的设定速度
    params.startVelocity = _start_speed * M_PI / 180.0;  // 
    params.endVelocity = _end_speed * M_PI / 180.0;    // 
    params.totalDistance = maxDistance;  // 使用最大距离作为规划距离
    
    // 初始化主速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "关节空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    // 记录起始关节位置
    std::array<float, NUM_JOINTS> startJoints = m_curJoints;
    
    // 执行插补循环
    bool motionCompleted = false;
    int cycleCount = 0;

    while (!motionCompleted && !GlobalParams::isStop) {
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插补比例系数
        double lambda = state.position / maxDistance;
        
        // 限制插补比例在[0,1]范围内
        lambda = std::max(0.0, std::min(1.0, lambda));

        
        // 根据插补比例计算所有关节的当前位置
        for (int i = 0; i < NUM_JOINTS; i++) {
            // 使用插补比例系数同步所有关节
            m_curJoints[i] = startJoints[i] + lambda * (_joint_pos[i] - startJoints[i]);
            
            // 计算关节速度（用于监控）
            if (jointDistances[i] > 1e-6) {
                double jointDirection = (_joint_pos[i] > startJoints[i]) ? 1.0 : -1.0;
                m_curVelocity[i] = jointDirection * state.velocity * (jointDistances[i] / maxDistance);
            } else {
                m_curVelocity[i] = 0.0;
            }
            
            // 限制关节位置在安全范围内
            m_curJoints[i] = std::max(m_angleLimitMin[i], 
                                     std::min(m_angleLimitMax[i], m_curJoints[i]));
        }
        
        // 检查运动是否完成
        motionCompleted = m_velocityPlanner.isCompleted();
        
        // 发送关节位置到硬件
        // moveJoints(m_curJoints);
        JointDataManager::getInstance()->tmpaddData(m_curJoints, 1, 1, 1);
        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "关节插补停止" << std::endl;
    } else {
        std::cout << "关节插补完成" << std::endl;
    }
}


void Robot::avoid_moveL(std::array<float, NUM_JOINTS> _pose, float _speed, float _start_speed, float _end_speed)
{
    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;

    // CSV写入计数器
    int csv_write_counter = 0;
    const int CSV_WRITE_INTERVAL = 10; // 每隔10个点写入一次

    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);

    // 计算起始和目标位置
    Eigen::Vector3d start_p(current_pose.X, current_pose.Y, current_pose.Z);
    Eigen::Vector3d end_p(_pose[0], _pose[1], _pose[2]);
    
    // 计算起始和目标姿态四元数
    Eigen::Quaterniond start_q = Eigen::AngleAxisd(current_pose.C, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(current_pose.B, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(current_pose.A, Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond end_q = Eigen::AngleAxisd(_pose[5], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(_pose[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(_pose[3], Eigen::Vector3d::UnitX());

    // 计算直线距离
    double distance = (end_p - start_p).norm();
    
    if (distance < 1e-6) {
        std::cout << "目标位置与当前位置过于接近，运动结束" << std::endl;
        return;
    }

    // 设置速度规划器参数
    VelocityPlanner::PlanningParams params;
    params.cycleTime = 0.001;    // 1ms插补周期
    params.maxVelocity = 100.0;  // 笛卡尔空间最大速度 (mm/s)
    params.maxAcceleration = 200.0;  // 最大加速度 (mm/s²)
    params.maxJerk = 600.0;  // 最大加加速度 (mm/s³)
    params.targetVelocity = _speed;  // 用户设定速度
    params.startVelocity = _start_speed;  // 起始速度
    params.endVelocity = _end_speed;    // 结束速度
    params.totalDistance = distance;  // 总距离
    
    // 初始化速度规划器
    if (!m_velocityPlanner.initialize(params)) {
        std::cerr << "笛卡尔空间速度规划器初始化失败!" << std::endl;
        return;
    }
    
    std::cout << "开始笛卡尔直线运动，距离: " << distance << " mm" << std::endl;

    // 运动控制循环
    bool motionCompleted = false;
    int cycleCount = 0;
    
    while (!motionCompleted) {
        // 检查暂停/恢复（非阻塞）
        if (GlobalParams::isPause) {
            m_velocityPlanner.pause();
        } else if (GlobalParams::isResume && m_velocityPlanner.isPaused()) {
            m_velocityPlanner.resume();
        }
        
        // 处理紧急停止
        if (GlobalParams::isStop) {
            m_velocityPlanner.emergencyStop();
            break;
        }
        
        // 获取当前运动状态
        VelocityPlanner::MotionState state = m_velocityPlanner.getNextState();
        
        // 这里仍可处理高优先级命令（不阻塞主循环）
        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
        
        // 计算插补比例
        double interp_ratio = state.position / distance;
        interp_ratio = std::max(0.0, std::min(1.0, interp_ratio));
        
        // 位置插值（线性插值）
        Eigen::Vector3d current_p = start_p + interp_ratio * (end_p - start_p);
        
        // 姿态插值（球面线性插值）
        Eigen::Quaterniond current_q = start_q.slerp(interp_ratio, end_q);
        
        // 将四元数转换回欧拉角
        Eigen::Matrix3d R = current_q.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX顺序
        
        // 构造当前目标位姿
        Kine6d target_pose;
        target_pose.X = current_p.x();
        target_pose.Y = current_p.y();
        target_pose.Z = current_p.z();
        // target_pose.A = euler[2]; // Roll (X)
        // target_pose.B = euler[1]; // Pitch (Y)
        // target_pose.C = euler[0]; // Yaw (Z)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_pose.R[i * 3 + j] = R(i, j);
            }
        }
        target_pose.fgR = 1;
        
        // 通过逆向运动学求解关节角度
        Kine6dSol q_sol;
        classic6dofInvKine(&target_pose, m_curJoints.data(), &q_sol);
        
        // 选择最优解
        bool valid[8];
        int best = -1;
        float best_dist = 1e6;
        for (int k = 0; k < 8; ++k) {
            valid[k] = true;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j]) {
                    valid[k] = false;
                    break;
                }
            }
            if (valid[k]) {
                float dist = 0;
                for (int j = 0; j < NUM_JOINTS; ++j)
                    dist += std::pow(m_curJoints[j] - q_sol.sol[k][j], 2);
                if (dist < best_dist) {
                    best = k;
                    best_dist = dist;
                }
            }
        }
        
        if (best >= 0) {
            std::array<float, NUM_JOINTS> target_joints;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target_joints[j] = q_sol.sol[best][j];
            
            // 更新当前关节角度
            m_curJoints = target_joints;

            JointDataManager::getInstance()->addData(m_curJoints, q_sol.sol_flag[best][0], q_sol.sol_flag[best][1], q_sol.sol_flag[best][2]);

            
            // 发送关节角度到底层控制器
            // moveJoints(target_joints);
        } else {
            // std::cout << "逆运动学求解失败，运动停止" << std::endl;

        }
        
         // 检查运动是否完成
         motionCompleted = m_velocityPlanner.isCompleted();
        
        
        // 等待下一个插补周期
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if (GlobalParams::isStop) {
        std::cout << "直线插补停止" << std::endl;
    } else {
        std::cout << "直线插补完成" << std::endl;
    }

    // JointDataManager::getInstance()->save_CSV("moveL_data.csv");
}

void Robot::moveJoints(const std::array<float, NUM_JOINTS>& _joints)
{
    // 关节空间插补指令
    // 直接发送到电机
    LowLevelCommand montor_cmd = {
        .mode = CSP,
    };
    
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        montor_cmd.joint_pos[i] = (_joints[i] - REST_JOINT[i]) / M_PI / 2.0f * m_GearRatio[i] * pow(2, m_Encoderbit[i]);   //关节角度转换到电机脉冲
    }

    // 入队列
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
    }
}

bool Robot::isPoseInWorkspace(const std::array<float, 6>& pose)
{
    
}

void Robot::calibrationTCP()
{
    m_tcpCalibrator.startCalibration();
}

bool Robot::addTCPCalibrationPoint()
{
    // 获取当前机器人末端法兰的位姿（通过正运动学）
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);

    // 将当前法兰位姿添加到标定点列表
    std::array<float, 6> pose = {current_pose.X, current_pose.Y, current_pose.Z,
                                current_pose.A, current_pose.B, current_pose.C};
    return m_tcpCalibrator.addCalibrationPoint(pose);
}

bool Robot::calculateTCP()
{
    return m_tcpCalibrator.calculateTCP();
}

void Robot::clearTCPCalibrationData()
{
    m_tcpCalibrator.clearCalibrationData();
}

bool Robot::isTCPCalibrationReady()
{
    return m_tcpCalibrator.isCalibrationReady();
}

std::array<float, 3> Robot::getTCPOffset() const
{
    return m_tcpCalibrator.getTCPOffset();
}

std::array<float, 3> Robot::getTCPRotation() const
{
    return m_tcpCalibrator.getTCPRotation();
}

std::array<float, 6> Robot::getTCPPoseInBase() const
{
    return m_tcpCalibrator.getTCPPoseInBase(m_curJoints);
}

bool Robot::isTCPCalibrated() const
{
    return m_tcpCalibrator.isCalibrated();
}

bool Robot::addManualTCPCalibrationPoint(float x, float y, float z, float a, float b, float c)
{
    return m_tcpCalibrator.addManualCalibrationPoint(x, y, z, a, b, c);
}

void Robot::testTCPCalibration()
{
    m_tcpCalibrator.runTestProgram();
}


void Robot::setSpeed(float _speed)
{ 
    //设置全局速度,关节空间和笛卡尔空间速度统一
    //笛卡尔空间最大速度70,关节空间最大速度20
    //设置转换系数
    //这个设定速度函数可设定1-100之间
    //设定100对应笛卡尔空间70,对应关节空间20
    
}

void Robot::homing()
{

    std::cout << "开始回零操作...\n";

    // 重置状态标志,开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    // updateJointStates();
    // 执行回零运动
    moveJ(REST_JOINT, DEFAULT_JOINT_SPEED, 0, 0);
    
    std::cout << "回零操作完成\n"<< std::endl;
    // updateJointStates();

}

void Robot::pause()
{
    std::cout << "暂停运动...\n";
    GlobalParams::isPause = true;
    GlobalParams::isResume = false;
}

void Robot::resume()
{
    std::cout << "恢复运动...\n";
    GlobalParams::isPause = false;
    GlobalParams::isResume = true;
}

void Robot::stop()
{
    std::cout << "停止运动...\n";
    GlobalParams::isStop = true;
    // 重置所有相关状态标志,确保能从任何状态退出
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
}

void Robot::updatePose()
{
    RobotState state;
    shm().state_buffer.read(state);

    Kine6d cur_pose;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        m_curJoints[i] = state.joint_state[i].position;
    }
    classic6dofForKine(m_curJoints.data(), &cur_pose);
        std::cout << "cur_pose: "
          << cur_pose.X << " "
          << cur_pose.Y << " "
          << cur_pose.Z << " "
          << cur_pose.A << " "
          << cur_pose.B << " "
          << cur_pose.C << std::endl;

}

void Robot::updateJointStates()
{
    RobotState state;
    shm().state_buffer.read(state);
 
    // 更新当前关节位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
        m_curJoints[i] = state.joint_state[i].position;
        m_curVelocity[i] = state.joint_state[i].velocity;
        m_curTorque[i] = state.joint_state[i].torque;
    }

    std::cout << "joints: ";
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        std::cout << " " << m_curJoints[i];
    }
    std::cout << std::endl;
}


void Robot::controlLoop()
{
    HighLevelCommand cmd;

    if (shm().high_prio_cmd_queue.pop(cmd))
    {
        handleHighPriorityCommand(cmd);
        return;
    }


    if (shm().cmd_queue.pop(cmd))
    {
        handleNormalCommand(cmd);
    }
}

void Robot::handleHighPriorityCommand(const HighLevelCommand &_cmd)
{
    switch (_cmd.command_type)
    {
    case HighLevelCommandType::Stop:
    {
        std::cout << "Stop!\n";
        stop();
        // 清空普通命令队列
        HighLevelCommand dummy;
        while (shm().cmd_queue.pop(dummy)) {
            // 循环弹出直到队列为空
        }
        // 停止后清零当前自动运动索引
        shm().cur_cmd_index.store(0);
        break;
    }
    case HighLevelCommandType::Pause:
        if (GlobalParams::isMoving && !GlobalParams::isStop)
        {
            std::cout << "Pause!\n";
            pause();  
        }
        break;
    case HighLevelCommandType::Resume:
    {
        if (GlobalParams::isPause && !GlobalParams::isStop)
        {
            std::cout << "Resume!\n";
            resume();
        }
        break;
    }

//        case HighLevelCommandType::SetParm:
//        {
//            handleParameterOrder(cmd);
//            break;
//        }
    default:
        break;
    }
}


void Robot::handleNormalCommand(const HighLevelCommand &cmd)
{
    // 检查机器人状态,如果处于暂停状态,不执行运动指令
    if (GlobalParams::isPause) {
        std::cout << "机器人处于暂停状态,忽略运动指令\n";
        return;
    }

    switch (cmd.command_type)
    {    
        case HighLevelCommandType::Homing:
        {
            std::cout << "Homing!\n";
            // updateJointStates();
            // updatePose();
            homing();
            // updateJointStates();
            // updatePose();
            break;
        }
        case HighLevelCommandType::MoveJ:
        {
            shm().cur_cmd_index.store(cmd.command_index);

            std::array<float, NUM_JOINTS> pos;
            std::copy(std::begin(cmd.movej_params.target_joint_pos),
                    std::end(cmd.movej_params.target_joint_pos),
                    pos.begin());
            float speed = cmd.movej_params.velocity;
            float startspeed = cmd.movej_params.start_speed;
            float endspeed = cmd.movej_params.end_speed;

            updateJointStates();
            updatePose();
            moveJ(pos, speed, startspeed, endspeed);
            updateJointStates();
            updatePose();

            // 指令完成后清零
            shm().cur_cmd_index.store(0);

            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::MoveL:
        {
            shm().cur_cmd_index.store(cmd.command_index);

            std::array<float, 6> pose;
            std::copy(std::begin(cmd.movel_params.target_pose),
                    std::end(cmd.movel_params.target_pose),
                    pose.begin());
            float speed = cmd.movel_params.velocity;
            float startspeed = cmd.movel_params.startspeed;
            float endspeed = cmd.movel_params.endspeed;

            updateJointStates();
            updatePose();
            moveL(pose, speed, startspeed, endspeed);
            updateJointStates();
            updatePose();

            shm().cur_cmd_index.store(0);

            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::MoveC:
        {
            shm().cur_cmd_index.store(cmd.command_index);

            std::array<float, 6> pose_mid;
            std::array<float, 6> pose_end;
            std::copy(std::begin(cmd.movec_params.via_pose),
                      std::end(cmd.movec_params.via_pose),
                      pose_mid.begin());
            std::copy(std::begin(cmd.movec_params.target_pose),
                      std::end(cmd.movec_params.target_pose),
                      pose_end.begin());
            float speed = cmd.movec_params.velocity;
            float startspeed = cmd.movec_params.startspeed;
            float endspeed = cmd.movec_params.endspeed;
            updateJointStates();
            updatePose();
            moveC(pose_mid, pose_end, speed, startspeed, endspeed);
            updateJointStates();
            updatePose();

            shm().cur_cmd_index.store(0);

            break;
        }
        case HighLevelCommandType::MoveCF:
        {
            break;
        }
        case HighLevelCommandType::MoveP:
        {
            break;
        }
        case HighLevelCommandType::SetParm:
        {
            handleParameterOrder(cmd);
            break;
        }
        
        case HighLevelCommandType::JogJ:
        {
            int mode = cmd.jogj_params.mode;
            int joint_index = cmd.jogj_params.joint_index;
            int direction = cmd.jogj_params.direction;
            int speed = cmd.jogj_params.speed;
            int angle = cmd.jogj_params.angle;

            std::cout << "JogJ - " << (mode == 0 ? "点动" : "寸动") << " 关节" << joint_index << " 方向: " << direction << std::endl;

            // updateJointStates();
            // updatePose();
            jogJ(mode, joint_index, direction, speed, angle);
            // updateJointStates();
            // updatePose();
            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::JogL:
        {
            int mode = cmd.jogl_params.mode;
            int axis = cmd.jogl_params.axis;
            int direction = cmd.jogl_params.direction;
            int speed = cmd.jogl_params.speed;
            int line_distance = cmd.jogl_params.line_distance;
            int angle_distance = cmd.jogl_params.angle_distance;


            std::cout << "JogL - " << (mode == 0 ? "点动" : "寸动") << " 轴" << axis << " 方向: " << direction << std::endl;
            // updateJointStates();
            // updatePose();
            jogL(mode, axis, direction, speed, line_distance, angle_distance);
            // updateJointStates();
            // updatePose();
            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::TCPCalibration:
        {
            testTCPCalibration();
            break;
        }
        case HighLevelCommandType::MoveLLBSpline:
        {
            shm().cur_cmd_index.store(cmd.command_index);

            std::array<float, 6> first_pose;
            std::array<float, 6> second_pose;
            std::copy(std::begin(cmd.movell_BSpline_params.first_pose),
                      std::end(cmd.movell_BSpline_params.first_pose),
                      first_pose.begin());
            std::copy(std::begin(cmd.movell_BSpline_params.second_pose),
                      std::end(cmd.movell_BSpline_params.second_pose),
                      second_pose.begin());

            std::array<float, 2> _speed;
            std::array<float, 2> _start_speed;
            std::array<float, 2> _end_speed;
            std::copy(std::begin(cmd.movell_BSpline_params.velocity),
                      std::end(cmd.movell_BSpline_params.velocity),
                      _speed.begin());
            std::copy(std::begin(cmd.movell_BSpline_params.startspeed),
                      std::end(cmd.movell_BSpline_params.startspeed),
                      _start_speed.begin());
            std::copy(std::begin(cmd.movell_BSpline_params.endspeed),
                      std::end(cmd.movell_BSpline_params.endspeed),
                      _end_speed.begin());

            twoMoveL_BSplineTransition(first_pose,second_pose,_speed,_start_speed,_end_speed);

            shm().cur_cmd_index.store(0);

            break;
        }
        case HighLevelCommandType::Print:
        {
            GlobalParams::print.store(cmd.print_params.flag);

            std::cout << "cmd.print_params.flag = " << cmd.print_params.flag << std::endl;

        }
        default:
            break;
    }
}

void Robot::handleParameterOrder(const HighLevelCommand &_cmd)
{
    const auto& packet = _cmd.setparms;

    switch (packet.mainType) {
    case 0x01:
        switch (packet.subType) {
        case 0x01:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_angleLimitMax[i] = packet.values[i];
            }
            break;
        case 0x02:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_angleLimitMin[i] = packet.values[i];
            }
            break;
        case 0x03:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointPosMax[i] = packet.values[i];
            }
            break;
        case 0x04:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointPosMin[i] = packet.values[i];
            }
            break;
        case 0x05:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointSpeedMax[i] = packet.values[i];
            }
            break;
        case 0x06:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointSpeedMax[i] = packet.values[i];
            }
            break;
        case 0x07:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointTorqueMax[i] = packet.values[i];
            }
            break;
        case 0x08:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointTorqueMax[i] = packet.values[i];
            }
            break;
        }
    case 0x10:
        switch (packet.subType) {
        // 最大轴速度
        case 0x01:
            for (int i = 0; i < packet.valueCount; ++i)
            {
//                m_AxisAccMax[i] = packet.values[i];
            }
            break;
        case 0x02:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_AxisAccMax[i] = packet.values[i];
            }
            break;
        case 0x03:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_AxisAccAccMax[i] = packet.values[i];
            }
            break;
        case 0x04:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointCompliance[i] = packet.values[i];
            }
            break;
        case 0x05:
            if (packet.valueCount == 5)
            {
                m_AccRatio = packet.values[0];
                m_AccRampUpTime = packet.values[1];
                m_DecRampUpTime = packet.values[2];
                m_SpeedSmoothingFactor = packet.values[3];
                m_JogAccRampUpTime = packet.values[4];
            }
            break;
        case 0x07:
            m_LookAheadFactor = packet.values[0];
            break;

            // ...
        }
        break;
    }
}

