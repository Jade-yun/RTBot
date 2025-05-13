#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>
#include <iostream>

extern SharedMemoryManager<SharedMemoryData> shm;

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);
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

bool Robot::isMoving()
{
    // 需要判断电机状态

//    for (int i = 0; i < NUM_JOINTS; i++)
//    {
//        return;
//    }
    return false;
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

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    constexpr int NUM_STEPS = 100; // 插补步数，按路径长或速度自适应计算
    std::array<float, NUM_JOINTS> start_pos;

    // 获取当前位置
//    getCurrentJointPosition();
    start_pos = m_curJoints;

    // 计算每一步的增量
    std::array<float, NUM_JOINTS> delta;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        delta[i] = (_joint_pos[i] - start_pos[i]) / NUM_STEPS;
    }

    LowLevelCommand montor_cmd = {
        .mode = CSP,
    };
    for (int step = 1; step <= NUM_STEPS; ++step)
    {
        // 生成当前插补点
        for (int j = 0; j < NUM_JOINTS; ++j)
        {
            montor_cmd.joint_pos[j] = start_pos[j] + delta[j] * step;
        }

        while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
        {
//            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 发送最后一点确保精确到达
    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        montor_cmd.joint_pos[j] = _joint_pos[j];
    }
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        std::this_thread::yield();
    }
}


void Robot::moveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    //
    // float temp[NUM_JOINTS] = joint_pos;
    LowLevelCommand montor_cmd = {
        .mode = CSP
    };

    std::copy(_joint_pos.begin(), _joint_pos.end(), montor_cmd.joint_pos);

    // 入队列
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        // std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}



void Robot::moveL(std::array<float, NUM_JOINTS> _pose)
{
    // 运动学逆解
//    solveIK
    //
    std::array<float, NUM_JOINTS> pos = {

    };
    moveJ(pos);
}

void Robot::setSpeed(float _speed)
{
    if (_speed < 0) _speed = 0;
//    else if (_speed > 200) _speed = 200;

    m_jointSpeed = _speed * m_jointSpeedRatio;
}

void Robot::updatePose()
{
    //
    //    solveFK(m_curJoints, m_currentPose);
}

void Robot::updateJointStates()
{
    // 从ethercat读取电机位置信息之后, 调用更新
//    m_curJoints =
    RobotState state;
    shm().state_buffer.read(state);

    // 更新当前关节位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
        m_curJoints[i] = state.joint_state[i].position;
    }

}

void Robot::controlLoop()
{
    HighLevelCommand cmd;
    if (!shm().cmd_queue.pop(cmd))
        return;

    switch (cmd.command_type)
    {
    case HighLevelCommandType::Homing:
        /* code */
        break;
    case HighLevelCommandType::Stop:
    {
        std::cout << "stop!\n";
        exit(0);
        break;
    }
    case HighLevelCommandType::MoveJ:
    {
        std::array<float, NUM_JOINTS> pos;
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            pos[i] = cmd.movej_params.target_joint_pos[i];
        }
        planMoveJ(pos);
        std::cout << "joint: ";
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            std::cout << pos[i] << " ";
        }
        std::cout << "\n";
        break;
    }
    case HighLevelCommandType::MoveL:
    {
        std::array<float, NUM_JOINTS> pose;
        std::copy(std::begin(cmd.movel_params.target_pose),
                  std::end(cmd.movel_params.target_pose),
                  pose.begin());
        moveL(pose);

        // std::cout << "pose: ";
        // for (int i = 0; i < NUM_JOINTS; i++)
        // {
        //     std::cout << pose[i] << " ";
        // }
        // std::cout << "\n";
        break;
    }
    case HighLevelCommandType::MoveP:
    {
        int traj_index = cmd.movep_params.traj_index;
        const auto &traj = shm().trajectory_pool[traj_index];

        for (uint32_t i = 0; i < traj.point_count; i++)
        {
            std::array<float, NUM_JOINTS> pos;
            std::memcpy(pos.data(), traj.points[i].joint_pos, sizeof(float) * NUM_JOINTS);
            moveJ(pos);
        }

    }
    default:
        break;
    }
}
