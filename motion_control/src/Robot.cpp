#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);
}

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &target_positions)
{
    // 
    LowLevelCommand montor_cmd;

    // 入队列
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        // std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // // 出队列
    // if (GlobalParams::joint_commands.try_dequeue(montor_cmd)) 
    // {
    //     // 
    // }

}

void Robot::controlLoop()
{
    
    HighLevelCommand cmd;
    if (!shm().cmd_queue.pop(cmd)) return;

    switch (cmd.command_type)
    {
    case HighLevelCommandType::Homing:
        /* code */
        break;
    case HighLevelCommandType::MoveJ:
        std::array<float, NUM_JOINTS> pos;
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            pos[i] =  cmd.movej_params.target_joint_pos[i];
        }
        planMoveJ(pos);
    
    default:
        break;
    }
}
