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

    // 启动后台线程：持续打印“当前自动运动索引”，每200ms刷新一次（单行刷新）
    // std::thread([](){
    //     while (true) {
    //         auto cur_idx = shm().cur_cmd_index.load();
    //         // std::cout << "\r[当前自动运动索引] " << cur_idx << "    " << std::flush;
    //         std::cout << "[当前自动运动索引] " << cur_idx << std::endl;
    //         std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //     }
    // }).detach();

    while (true) 
    {
        if (shm().state_buffer.read(state))
        {
            std::cout << "\n";
            std::cout << "Recv new State...\n";
            std::cout << "\n";
        }

        // // 显示当前自动运动指令索引（仅MoveJ/MoveL/MoveC/BSpline时由RT侧填入）
        // {
        //     auto cur_idx = shm().cur_cmd_index.load();
        //     std::cout << "[当前自动运动索引] " << cur_idx << std::endl;
        // }

        std::cout << "=== Robot Command Line Interface ===\n";
        std::cout << "----------------AUTO----------------\n";
        std::cout << "Use >x,x,x,x,x,x,speed for MoveJ\n";
        std::cout << "Use @x,x,x,x,x,x,speed for MoveL\n";
        std::cout << "Use #(x,x,x,x,x,x),(x,x,x,x,x,x),speed for MoveC\n";
        std::cout << "Use %(x,x,x,x,x,x,x,x,x)%(x,x,x,x,x,x,x,x,x),speed for MoveLL_BSpline\n";           
        std::cout << "---------------MANUAL---------------\n";
        std::cout << "Use j<mode><joint_index><direction> for JogJ (mode:0连续,1微动; joint_index:0-5; direction:1正向,0负向)\n";
        std::cout << "Use l<mode><axis><direction> for JogL (mode:0连续,1微动; axis:1-6; direction:1正向,0负向)\n";
        std::cout << "---------------COMMON----------------\n";
        std::cout << "Use p to Pause\n";
        std::cout << "Use r to Resume\n";
        std::cout << "Use q to stop\n";
        std::cout << "Use h to Homing\n";
        std::cout << "Use tcp to TCP Calibration\n";
        std::cout << "====================================\n\n";

        std::string cmd_str;
        int argNum = 0;

        // std::cout << ">";
        std::cin >> cmd_str;

        std::memset(&cmd, 0, sizeof(cmd));
        if (cmd_str[0] == 'q') {
            cmd.command_type = HighLevelCommandType::Stop;
            // cmd.target_joint_pos[0] = 0;
            index = 0;
            cmd.command_index = index;
        }
        else if(cmd_str[0] == 'h')
        {
            cmd.command_type = HighLevelCommandType::Homing;
        }
        else if (cmd_str[0] == '#')
        {
            float pose_mid[6];
            float pose_end[6];
            float speed;
            float startspeed;
            float endspeed;
            argNum = sscanf(cmd_str.c_str(), "#(%f,%f,%f,%f,%f,%f),(%f,%f,%f,%f,%f,%f),%f,%f,%f",
                            pose_mid, pose_mid + 1, pose_mid + 2,pose_mid + 3, pose_mid + 4, pose_mid + 5,
                            pose_end, pose_end + 1, pose_end + 2,pose_end + 3, pose_end + 4, pose_end + 5,
                            &speed, &startspeed, &endspeed);

            for (int i = 3; i < 6; i++)
            {
                pose_mid[i] *= M_PI / 180;
            }

            for (int i = 3; i < 6; i++)
            {
                pose_end[i] *= M_PI / 180;
            }

            if (argNum != 15)
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
            cmd.movec_params.startspeed = startspeed;
            cmd.movec_params.endspeed = endspeed;

        }
        else if (cmd_str[0] == '>')
        {
            float joints[6];
            float speed;
            float startspeed;
            float endspeed;

            argNum = sscanf(cmd_str.c_str(), ">%f,%f,%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                            joints + 3, joints + 4, joints + 5, &speed, &startspeed, &endspeed);
            
            if (argNum < 8) continue;

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
            cmd.movej_params.start_speed = startspeed;
            cmd.movej_params.end_speed = endspeed;
        } 
        else if (cmd_str[0] == '@')
        {
            float pose[6];
            float speed;
            float startspeed;
            float endspeed;
            argNum = sscanf(cmd_str.c_str(), "@%f,%f,%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                            pose + 3, pose + 4, pose + 5, &speed, &startspeed, &endspeed);

            if (argNum < 8) continue;


            for (int i = 3; i < 6; i++)
            {
                pose[i] *= M_PI / 180;
            }

            index++;
            cmd.command_type = HighLevelCommandType::MoveL; 
            cmd.command_index = index;

            ::memcpy(cmd.movel_params.target_pose, pose, sizeof(float) * NUM_JOINTS);
            cmd.movel_params.velocity = speed;
            cmd.movel_params.startspeed = startspeed;
            cmd.movel_params.endspeed = endspeed;
        }
        else if (cmd_str[0] == 'p')
        {
            cmd.command_type = HighLevelCommandType::Pause;
        }
        else if (cmd_str[0] == 'r')
        {
            cmd.command_type = HighLevelCommandType::Resume;
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
                cmd.command_type = HighLevelCommandType::JogJ;
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
                cmd.command_type = HighLevelCommandType::JogL;
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
        else if (cmd_str == "tcp")
        {
            cmd.command_type = HighLevelCommandType::TCPCalibration;
        }
        else if (cmd_str[0] == '%')
        {

            // 声明变量
            float first_pose[6], second_pose[6];
            float speed[2], startspeed[2], endspeed[2];
            int argNum1, argNum2;

            // 检查字符串是否以%开头
            if (cmd_str.empty() || cmd_str[0] != '%') {
                std::cerr << "格式错误：字符串应以%开头" << std::endl;
                continue;
            }

            // 去掉开头的%
            std::string data_str = cmd_str.substr(1);

            // 查找分隔符%的位置
            size_t delimiter_pos = data_str.find('%');
            if (delimiter_pos == std::string::npos) {
                std::cerr << "未找到分隔符%" << std::endl;
                continue;
            }

            // 分割字符串
            std::string first_part = data_str.substr(0, delimiter_pos);
            std::string second_part = data_str.substr(delimiter_pos + 1);
            // std::cerr << "-1 " << std::endl;

            // 解析第一组数据（不需要@前缀）
            argNum1 = sscanf(first_part.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                            &first_pose[0], &first_pose[1], &first_pose[2], 
                            &first_pose[3], &first_pose[4], &first_pose[5], 
                            &speed[0], &startspeed[0], &endspeed[0]);

            // 解析第二组数据（不需要@前缀）
            argNum2 = sscanf(second_part.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                            &second_pose[0], &second_pose[1], &second_pose[2], 
                            &second_pose[3], &second_pose[4], &second_pose[5], 
                            &speed[1], &startspeed[1], &endspeed[1]);

            if (argNum1 != 9 || argNum2 != 9)
            {
                std::cerr << "格式错误" << std::endl;
                continue;
            }


            std::cerr << "0 " << std::endl;
            for (int i = 3; i < 6; i++)
            {
                first_pose[i] *= M_PI / 180;
                second_pose[i] *= M_PI / 180;
            }
            std::cerr << "1 " << std::endl;
            index++;
            cmd.command_type = HighLevelCommandType::MoveLLBSpline; 
            cmd.command_index = index;

            ::memcpy(cmd.movell_BSpline_params.first_pose, first_pose, sizeof(float) * NUM_JOINTS);
            std::cerr << "2 " << std::endl;
            ::memcpy(cmd.movell_BSpline_params.second_pose, second_pose, sizeof(float) * NUM_JOINTS);
            std::cerr << "3 " << std::endl;

            ::memcpy(cmd.movell_BSpline_params.velocity, speed, sizeof(float) * 2);
            std::cerr << "4 " << std::endl;
            ::memcpy(cmd.movell_BSpline_params.startspeed, startspeed, sizeof(float) * 2);
            std::cerr << "5 " << std::endl;
            ::memcpy(cmd.movell_BSpline_params.endspeed, endspeed, sizeof(float) * 2);
            std::cerr << "6 " << std::endl; 

            std::cerr << "B样条测试 " << std::endl;

        }
        else 
        {
            continue;
        }

        // 区分高优先级命令和普通命令
        bool isHighPriorityCmd = (cmd.command_type == HighLevelCommandType::Stop || 
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
