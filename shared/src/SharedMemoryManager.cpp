#include "SharedMemoryManager.h"
#include <iostream>

SharedMemoryManager& SharedMemoryManager::getInstance() {
    static SharedMemoryManager instance;
    return instance;
}

bool SharedMemoryManager::init(ShmMode mode) {
    if (shm_fd_ != -1) {
        std::cerr << "[SHM] Already initialized." << std::endl;
        return false;
    }

    if (mode == ShmMode::Creator) {
        // 创建共享内存对象
        shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) {
            std::cerr << "[SHM] Failed to create shared memory object." << std::endl;
            return false;
        }

        // 设置共享内存的大小
        if (ftruncate(shm_fd_, SHM_SIZE) == -1) {
            std::cerr << "[SHM] Failed to set shared memory size." << std::endl;
            close(shm_fd_);
            return false;
        }
    }
    else if (mode == ShmMode::Attacher) {
        // 附加到已存在的共享内存对象
        shm_fd_ = shm_open(SHM_NAME, O_RDWR, 0666);
        if (shm_fd_ == -1) {
            std::cerr << "[SHM] Failed to open existing shared memory object." << std::endl;
            return false;
        }
    } 
    else {
        std::cerr << "[SHM] Invalid mode." << std::endl;
        return false;
    }

    // 映射共享内存到进程地址空间
    data_ = static_cast<SharedData*>(mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (data_ == MAP_FAILED) {
        std::cerr << "[SHM] Failed to map shared memory." << std::endl;
        return false;
    }

    // 如果是创建者，初始化数据
    if (mode == ShmMode::Creator) {
        data_->command.new_cmd = false;
        data_->state.heartbeat_counter = 0;
        // TO DO 
        // initialize data here 


    }

    return true;
}

void SharedMemoryManager::destroy() {
    if (data_) {
        munmap(data_, SHM_SIZE);
        data_ = nullptr;
    }
    if (shm_fd_ != -1) {
        close(shm_fd_); 
        shm_fd_ = -1;
    }
}

SharedMemoryManager::~SharedMemoryManager() {
    destroy();
}
