#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>
#include <iostream>

extern SharedMemoryManager<SharedMemoryData> shm;

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Creator);
}

void Robot::setEnable(bool _enabled)
{
    // motorJ[ALL]->SetEnable(_enable);
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

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    // //
    LowLevelCommand montor_cmd;

    // // 入队列
    // while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    // {
    //     // std::this_thread::yield();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(2));
    // }
    while (1)
    {
        // 插补运算
        float points[6];
        // montor_cmd.joint_pos

        // 发送插补后的点
        
        while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
        {
            std::this_thread::yield();
            // std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // 是否插补完成
        // if ()
        // break;

    }

    // 插补计算

    // moveJ()
}

void Robot::moveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    //
    // float temp[NUM_JOINTS] = joint_pos;
    LowLevelCommand montor_cmd = {
        .mode = CSP};

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

    //
    std::array<float, NUM_JOINTS> pos = {

    };
    moveJ(pos);
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
    default:
        break;
    }
}
