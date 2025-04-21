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

    while (true) 
    {
        std::cout << "mode: " << (int)data->command.mode << std::endl;
        
        if (data->command.mode == ControlMode::STOP) {
            std::cout << "Stop " << std::endl;
        }
        else {
            data->state.heartbeat_counter += 1;
        }


        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
