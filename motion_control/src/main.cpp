#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>

#include "ethercat/EtherCATInterface.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"
#include "Robot.h"

#include <unistd.h>
#include <sched.h>

EtherCATInterface ethercatIf;
Robot robot;

void delay_nanoseconds(long nanoseconds) {
    struct timespec req, rem;

    req.tv_sec = nanoseconds / 1000000000;
    req.tv_nsec = nanoseconds % 1000000000;

    if (nanosleep(&req, &rem) == -1) {
            perror("nanosleep");
    }
}

void eStopHandler()
{
    // 1. 设置控制字为Quick Stop（Bit 2 = 1）
    // ecrt_master_send(master);

    // 2. 关闭使能（通过硬件回路）
    // set_gpio(DRIVER_ENABLE_PIN, LOW);
}

void monitorEStop()
{
    // // 1. 读取硬件急停GPIO状态
    // bool e_stop_triggered = read_gpio(ESTOP_PIN);

    // // 2. 检查软件安全标志（如关节超限）
    // if (safety_monitor.is_unsafe())
    //     e_stop_triggered = true;

    // // 3. 触发急停
    // if (e_stop_triggered)
    // {
    //     emergency_stop_handler();
    //     break; // 终止线程
    // }
}

// 信号处理函数
void signal_handler(int signum) {
    if (signum == SIGINT) {
        printf("接收到 Ctrl+C 信号，正在退出程序...\n");

        ethercatIf.signal_handler();
    }
}


SharedMemoryManager<SharedMemoryData> shm =
    SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Creator, true);

int main()
{


    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(3, &mask);  // 添加核心2

    if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
        std::cerr << "Failed to set CPU affinity:" << strerror(errno);
    }

    PeriodicTaskManager taskManager;

    int ret = ethercatIf.init();
    if (!ret)
    {
        printf("Failed to init EtherCAT master.\n");
        exit(-2);
    }

    // auto sig_handler = [&](int signum) {
    //     if (signum == SIGINT) {
    //         printf("接收到 Ctrl+C 信号，正在退出程序...\n");
    
    //         ethercatIf.signal_handler();
    //     }
    // };

    signal(SIGINT, signal_handler);

   PeriodicMemberFunction<EtherCATInterface> ecatTask(
       &taskManager, .001, "ecat", &EtherCATInterface::runTask, &ethercatIf);

   ecatTask.start();
   ecatTask.setThreadPriority(99);

//    Robot robot;
    robot.init();

    PeriodicMemberFunction<Robot> robotTask(
        &taskManager, 0.05, "robot", &Robot::controlLoop, &robot);

    robotTask.start();
    robotTask.setThreadPriority(80);

    for (;;)
    {

        // taskManager.printStatus();
        // taskManager.printStatusOfSlowTasks();

        std::this_thread::sleep_for(std::chrono::milliseconds(5 * 1000));
    }
}
