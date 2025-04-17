#include "../include/shared_data.h"
#include <iostream>
#include <cstring>

// 单例模式实现
SharedMemoryManager& SharedMemoryManager::getInstance() {
    static SharedMemoryManager instance;
    return instance;
}

bool SharedMemoryManager::init() {
    // 创建或打开共享内存对象
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory object." << std::endl;
        return false;
    }

    // 设置共享内存大小
    if (ftruncate(shm_fd_, SHM_SIZE) == -1) {
        std::cerr << "Failed to set shared memory size." << std::endl;
        return false;
    }

    // 映射共享内存到进程地址空间
    data_ = static_cast<SharedData*>(mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (data_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory." << std::endl;
        return false;
    }

    // 初始化互斥锁
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&data_->mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    // 初始化共享数据
    data_->command.run_flag = 0;
    data_->status.position = 0.0;
    data_->status.velocity = 0.0;

    return true;
}

void SharedMemoryManager::destroy() {
    if (data_) {
        munmap(data_, SHM_SIZE);  // 取消映射
        data_ = nullptr;
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);           // 关闭共享内存描述符
        shm_unlink(SHM_NAME);     // 删除共享内存对象
    }
}

SharedMemoryManager::~SharedMemoryManager() {
    destroy();
}