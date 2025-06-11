#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

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
        if (shm().state_buffer.read(state))
        {
            // 
            std::cout << "Recv new State...\n";
        }

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

            //printf("joint1:%f, joint2:%f, joint3:%f, joint4:%f, joint5:%f, joint6:%f, speed:%f\n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], speed);
            
            if (argNum < 6) continue;

            // 转换为弧度
            for (int i = 0; i < 6; i++)
            {
                joints[i] *= M_PI / 180;
            }

            //指令类型
            index++;
            cmd.command_type = HighLevelCommandType::MoveJ;
            cmd.command_index = index;

            ::memcpy(cmd.movej_params.target_joint_pos, joints, sizeof(float) * NUM_JOINTS); 
            cmd.movej_params.velocity = speed;
        } 
        else if (cmd_str[0] == '@')
        {
            float pose[6];
            float speed;

            argNum = sscanf(cmd_str.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                            pose + 3, pose + 4, pose + 5, &speed);

            if (argNum < 6) continue;


            for (int i = 4; i < 6; i++)
            {
                pose[i] *= M_PI / 180;
            }

            index++;
            cmd.command_type = HighLevelCommandType::MoveL; 
            cmd.command_index = index;

            ::memcpy(cmd.movel_params.target_pose, pose, sizeof(float) * NUM_JOINTS);
            cmd.movel_params.velocity = speed;
        }
        else {
            continue;
        }

        if (shm().cmd_queue.push(cmd))
        {
            std::cout << "Sent new command...\n";
            RobotState state;
            shm().state_buffer.read(state);

            std::cout << "joints pos: ";
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                std::cout << " " << state.joint_state[i].position;;
            }
            std::cout << std::endl;
        }

    }

    return 0;
}
