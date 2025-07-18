#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"

// === HMI 程序 ===
// 
// 点动功能使用示例：
// 1. 输入 j0+ 启动关节0正向点动
// 2. 输入 s 停止点动
// 3. 输入 j1- 启动关节1负向点动
// 4. 输入 s 停止点动
//
SharedMemoryManager<SharedMemoryData> shm(SharedMemoryManager<SharedMemoryData>::Attacher);

int main() {

    HighLevelCommand cmd;
    RobotState state;
    int index = 0;

    while (true) 
    {
        if (shm().state_buffer.read(state))
        {
            std::cout << "\n";
            std::cout << "Recv new State...\n";
            std::cout << "\n";
        }

        std::cout << "=== Robot Command Line Interface ===\n";
        std::cout << "----------------AUTO----------------\n";
        std::cout << "Use >x,x,x,x,x,x,speed for MoveJ (speed <= 25)\n";
        std::cout << "Use @x,x,x,x,x,x,speed for MoveL (speed <= 75)\n";
        std::cout << "Use #(x,x,x,x,x,x),(x,x,x,x,x,x),speed for MoveC\n";        
        std::cout << "---------------MANUAL---------------\n";
        std::cout << "Use j<mode><joint_index><direction> for JogJ (mode:0连续,1微动; joint_index:0-5; direction:1正向,0负向)\n";
        std::cout << "Use l<mode><axis><direction> for JogL (mode:0连续,1微动; axis:1-6; direction:1正向,0负向)\n";
        std::cout << "---------------COMMON----------------\n";
        std::cout << "Use p to Pause\n";
        std::cout << "Use r to Resume\n";
        std::cout << "Use q to stop\n";
        std::cout << "Use h to Homing\n";
        std::cout << "====================================\n\n";

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
        else if(cmd_str[0] == 'h')
        {
            index++;
            cmd.command_type = HighLevelCommandType::Homing;
            cmd.command_index = index;
        }
        else if (cmd_str[0] == '#')
        {
            float pose_mid[6];
            float pose_end[6];
            float speed;
            argNum = sscanf(cmd_str.c_str(), "#(%f,%f,%f,%f,%f,%f),(%f,%f,%f,%f,%f,%f),%f ",
                            pose_mid, pose_mid + 1, pose_mid + 2,pose_mid + 3, pose_mid + 4, pose_mid + 5,
                            pose_end, pose_end + 1, pose_end + 2,pose_end + 3, pose_end + 4, pose_end + 5,
                            &speed);

            if (argNum != 13)
            {
                std::cout << "moveC input error!\n";
                continue;
            }

            index++;
            cmd.command_type = HighLevelCommandType::MoveC;
            cmd.command_index = index;

            ::memcpy(cmd.movec_params.via_pose, pose_mid, sizeof(float) * NUM_JOINTS);
            ::memcpy(cmd.movec_params.target_pose, pose_end, sizeof(float) * NUM_JOINTS);

            cmd.movec_params.velocity = speed;

        }
        else if (cmd_str[0] == '>')
        {
            float joints[6];
            float speed;

            argNum = sscanf(cmd_str.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                            joints + 3, joints + 4, joints + 5, &speed);
            
            if (argNum < 6) continue;

            if (speed > 25)
            {
                std::cout << "输入速度超出限制,已将速度改为25!\n";
                speed = 25;
            }
            
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

            if (speed > 75)
            {
                std::cout << "输入速度超出限制,已将速度改为75!\n";
                speed = 75;
            }

            for (int i = 3; i < 6; i++)
            {
                pose[i] *= M_PI / 180;
            }

            index++;
            cmd.command_type = HighLevelCommandType::MoveL; 
            cmd.command_index = index;

            ::memcpy(cmd.movel_params.target_pose, pose, sizeof(float) * NUM_JOINTS);
            cmd.movel_params.velocity = speed;
        }
        else if (cmd_str[0] == 'p')
        {
            index++;
            cmd.command_type = HighLevelCommandType::Pause;
            cmd.command_index = index;
        }
        else if (cmd_str[0] == 'r')
        {
            index++;
            cmd.command_type = HighLevelCommandType::Resume;
            cmd.command_index = index;
        }
        else if (cmd_str[0] == 'j' && cmd_str.length() >= 4)
        {
            // 解析点动命令，格式：j<mode><joint_index><direction>
            int mode = cmd_str[1] - '0';         // 模式 0 连续点动, 1 微动
            int joint_index = cmd_str[2] - '0';  // 关节索引
            int direction = cmd_str[3] - '0';         // 方向
            if ((mode == 0 || mode == 1) && joint_index >= 0 && joint_index < NUM_JOINTS &&
                (direction == 1 || direction == 0)) // 方向 '1' 正向, '0' 负向
            {
                index++;
                cmd.command_type = HighLevelCommandType::JogJ;
                cmd.command_index = index;
                cmd.jogj_params.mode = mode;
                cmd.jogj_params.joint_index = joint_index;
                cmd.jogj_params.direction = direction;
                
                std::cout << "发送关节点动命令: 模式 " << mode << ", 关节" << joint_index << " 方向: " << direction << std::endl;
            }
            else
            {
                std::cout << "无效的关节点动命令格式！请使用 j<mode><joint_index><1/0> 格式\n";
                continue;
            }
        }
        else if (cmd_str[0] == 'l' && cmd_str.length() >= 4)
        {
            // 解析笛卡尔点动命令，格式：l<mode><axis><direction>
            int mode = cmd_str[1] - '0';      // 模式 0 连续点动, 1 微动
            int axis = cmd_str[2] - '0';      // 轴
            int direction = cmd_str[3] - '0'; // 方向

            if ((mode == 0 || mode == 1) && (axis == 1 || axis == 2 || axis == 3 || axis == 4 || axis == 5 || axis == 6) &&
                (direction == 1 || direction == 0))
            {
                index++;
                cmd.command_type = HighLevelCommandType::JogL;
                cmd.command_index = index;
                cmd.jogl_params.mode = mode;
                cmd.jogl_params.axis = axis;
                cmd.jogl_params.direction = direction;

                std::cout << "发送笛卡尔点动命令: 模式 " << mode << ", 轴" << axis << " 方向: " << direction << std::endl;
            }
            else
            {
                std::cout << "无效的笛卡尔点动命令格式！请使用 l<0/1><1/2/3/4/5/6><1/0> 格式\n";
                continue;
            }
        }
        else 
        {
            continue;
        }

        // 区分高优先级命令和普通命令
        bool isHighPriorityCmd = (cmd.command_type == HighLevelCommandType::Stop || 
                                  cmd.command_type == HighLevelCommandType::Homing ||
                                  cmd.command_type == HighLevelCommandType::Pause ||
                                  cmd.command_type == HighLevelCommandType::Resume);
        
        bool cmdSent = false;   
        if (isHighPriorityCmd)
        {
            cmdSent = shm().high_prio_cmd_queue.push(cmd);
            if (cmdSent)
            {
                std::cout << "发送高优先级命令...\n";
            }
        }
        else
        {
            cmdSent = shm().cmd_queue.push(cmd);
            if (cmdSent)
            {
                std::cout << "发送普通命令...\n";
            }
        }

        if (cmdSent)
        {
            RobotState state;
            shm().state_buffer.read(state);

            std::cout << "joints pos: ";
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                std::cout << " " << state.joint_state[i].position;
            }
            std::cout << std::endl;
        }

    }

    return 0;
}
