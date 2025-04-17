// shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#define SHM_NAME "/motion_shm"
#define SHM_SIZE sizeof(SharedData)

struct MotionCommand {
    int run_flag;      // 1=start, 0=stop
};

struct MotionStatus {
    double position;
    double velocity;
};

struct SharedData {
    pthread_mutex_t mutex;
    MotionCommand command;
    MotionStatus status;
};

// 共享内存管理类
class SharedMemoryManager {
    public:
        static SharedMemoryManager& getInstance();  // 单例模式
    
        bool init();                                // 初始化共享内存
        void destroy();                             // 销毁共享内存
    
        SharedData* getData() const { return data_; }
    
    private:
        SharedMemoryManager() = default;
        ~SharedMemoryManager();
    
        int shm_fd_;
        SharedData* data_ = nullptr;
    };

#endif
