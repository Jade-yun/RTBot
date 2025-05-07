#include <iostream>
#include <thread>
#include <chrono>

#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"

// === HMI 程序 ===
SharedMemoryManager<SharedMemoryData> shm(SharedMemoryManager<SharedMemoryData>::Attacher);

int main() {

    HighLevelCommand cmd;
    RobotState state;
    int index = 0;

    while (true) 
    {
        std::cout << "=== Robot Command Line Interface ===\n";
        std::cout << "Use >x,x,x,x,x,x,speed for MoveJ\n";
        std::cout << "Use @x,x,x,x,x,x,speed for MoveL\n";
        std::cout << "Use q to stop\n\n";

        std::string cmd_str;
        int argNum = 0;

        // std::cout << ">";
        std::cin >> cmd_str;

        std::memset(&cmd, 0, sizeof(cmd));
        if (cmd_str[0] == 'q') {
            index++;
            cmd.command_type = HighLevelCommandType::Stop;
            cmd.command_index = index;
            // cmd.target_joint_pos[0] = 0;
        }
        else if (cmd_str[0] == '>')
        {
            float joints[6];
            float speed;

            argNum = sscanf(cmd_str.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                            joints + 3, joints + 4, joints + 5, &speed);
            
            if (argNum < 6) continue;

            index++;
            cmd.command_type = HighLevelCommandType::MoveJ;
            cmd.command_index = index;

            ::memcpy(cmd.movej_params.target_joint_pos, joints, sizeof(float) * NUM_JOINTS); 
        } 
        else if (cmd_str[0] == '@')
        {
            float pose[6];
            float speed;

            argNum = sscanf(cmd_str.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                            pose + 3, pose + 4, pose + 5, &speed);

            if (argNum < 6) continue;

            index++;
            cmd.command_type = HighLevelCommandType::MoveL; 
            cmd.command_index = index;

            ::memcpy(cmd.movel_params.target_pose, pose, sizeof(float) * NUM_JOINTS);          
        }
        else {
            continue;
        }

        if (shm().cmd_queue.push(cmd))
        {
            std::cout << "Sent new command...\n";
        }

        if (shm().state_buffer.read(state))
        {
            // 
            std::cout << "Recv new State...\n";
        }

    }

    return 0;
}
