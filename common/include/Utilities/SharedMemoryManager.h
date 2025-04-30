#ifndef PROJECT_SHAREDMEMORYMANAGER_H
#define PROJECT_SHAREDMEMORYMANAGER_H

#include "Utilities/SharedMemory.h"

template <typename T>
class SharedMemoryManager
{
public:
    enum ShmMode
    {
        Creator, // 主进程，创建共享内存
        Attacher // 从进程，附加已有共享内存
    };

    static constexpr const char *SharedMemoryName = "/motion_shm"; // <-- 全局唯一名称

    SharedMemoryManager(ShmMode mode, bool allowOverwrite = false)
        : _mode(mode)
    {
        if (mode == ShmMode::Creator)
        {
            _sharedMemory.createNew(SharedMemoryName, allowOverwrite);
        }
        else
        {
            _sharedMemory.attach(SharedMemoryName);
        }
    }

    ~SharedMemoryManager()
    {
        try
        {
            if (_mode == ShmMode::Creator)
            {
                _sharedMemory.closeNew();
            }
            else
            {
                _sharedMemory.detach();
            }
        }
        catch (const std::exception &e)
        {
            printf("[SharedMemoryManager] Exception in destructor: %s\n", e.what());
        }
    }

    T *get() { return _sharedMemory.get(); }
    T &operator()() { return _sharedMemory(); }

private:
    ShmMode _mode;
    SharedMemoryObject<T> _sharedMemory;
};

#endif // PROJECT_SHAREDMEMORYMANAGER_H
