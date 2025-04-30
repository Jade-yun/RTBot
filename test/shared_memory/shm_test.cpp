// test_shm
#include "Utilities/SharedMemoryManager.h"
#include "ControlParameters/SharedData.h"
#include <iostream>
#include <thread>
#include <chrono>


// === HMI 程序 ===

#if 1
int main() {
    SharedMemoryManager<SharedData> shm(SharedMemoryManager<SharedData>::ShmMode::Creator);

    SharedData* data = shm.get();
    SharedData* data = &shm();

    RtCommand cmd;
    RtState state;
    while (true) 
    {
        std::string str;

        std::cout << ">";
        std::cin >> str;

        if (str == "stop") {
            cmd.mode = ControlMode::STOP;
            cmd.target_joint_pos[0] = 0;
        }
        else if (str == "start") {
            cmd.mode = ControlMode::JOG;

            cmd.mode = ControlMode::STOP;
            cmd.target_joint_pos[0] = 30;
        }
        cmd.command_id++;

        if (data->cmd_queue.push(cmd))
        {
            std::cout << "Sent new command... \n";
        }

        data->state_buffer.read(state);

        std::cout << "Heartbeat: " << state.heartbeat_counter << std::endl;
    }

    return 0;
}

// === 控制器程序 ===
#else
int main() {
    SharedMemoryManager<SharedData> shm(SharedMemoryManager<SharedData>::ShmMode::Creator);

    SharedData* data = &shm();

    RtCommand cmd;
    RtState state;
    while (true) 
    {
        // 存在新指令
        if (data->cmd_queue.pop(cmd)){
            if (cmd.mode == ControlMode::JOG) {
                std::cout << "Change Mode: JOG ." << std::endl;
            }
            else if (cmd.mode == ControlMode::STOP) {
                std::cout << "Change Mode: STOP.\n";
            }
        }

        state.heartbeat_counter++;
        data->state_buffer.write(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
#endif
