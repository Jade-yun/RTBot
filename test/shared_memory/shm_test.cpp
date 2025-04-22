// test_shm
#include "SharedMemoryManager.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    auto& shm = SharedMemoryManager::getInstance();
    if (!shm.init(ShmMode::Creator)) {
        std::cerr << "Motion init failed.\n";
        return -1;
    }

    SharedData* data = shm.getData();

    RtCommand cmd;
    RtState state;
    while (true) 
    {
        data->cmd_queue.pop(cmd);
        if (cmd.mode == ControlMode::IDLE) {
            if (cmd.mode == 1) {  
            std::cout << "it is in idle state." << std::endl;
        }
        else if (cmd.mode == ControlMode::STOP) {
            std::cout << "it is stop.\n";
        }

        state.heartbeat_counter += 1;
        data->state_buffer.write(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
