#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>

void Robot::init()
{
    shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Creator);
}

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &target_positions)
{
    // 
    GlobalParams::LowLevelCommand montor_cmd;

    while (GlobalParams::requst_cmd.push(montor_cmd))
    {
        std::this_thread::yield();
        continue;
    }

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
