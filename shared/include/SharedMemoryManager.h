#ifndef SHARED_MEMORY_MANAGER_H
#define SHARED_MEMORY_MANAGER_H

#include "shared_data.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>

#define SHM_NAME "/motion_shm"
#define SHM_SIZE sizeof(SharedData)

enum class ShmMode {
    Creator,  // 主进程，创建共享内存
    Attacher   // 从进程，附加已有共享内存
};

class SharedMemoryManager 
{
public:
    static SharedMemoryManager& getInstance();

    bool init(ShmMode mode); // 初始化共享内存
    void destroy();          // 关闭共享内存

    SharedData* getData() const;

private:
    SharedMemoryManager() = default;
    ~SharedMemoryManager();

    int shm_fd_ = -1;
    SharedData* data_ = nullptr;
};
#endif