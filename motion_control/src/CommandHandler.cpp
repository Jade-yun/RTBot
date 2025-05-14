#include "CommandHandler.h"

#include <iostream>
#include <thread>

#include "Robot.h"


CommandHandler::CommandHandler(Robot *_context)
    : context(_context)
{

}

void CommandHandler::parseCommand(HighLevelCommand &_cmd)
{
    switch (_cmd.command_type)
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
            pos[i] = _cmd.movej_params.target_joint_pos[i];
        }
        // 初始时
        // context->planMoveJ(context->m_targetJoints);

        while (context->isMoving() && context->isEnabled())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        //
        break;
    }
    case HighLevelCommandType::MoveL:
    {
        std::array<float, NUM_JOINTS> pose;
        std::copy(std::begin(_cmd.movel_params.target_pose),
                  std::end(_cmd.movel_params.target_pose),
                  pose.begin());
        context->moveL(pose);
        break;
    }
    default:
        break;
    }

}
