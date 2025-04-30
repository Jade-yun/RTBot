#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>

#include "ethercat/EtherCATInterface.h"
#include "Utilities/PeriodicTask.h"

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

int main()
{


    PeriodicTaskManager taskManager;

    EtherCATInterface ethercatIf;

    ethercatIf.init();
    PeriodicMemberFunction<EtherCATInterface> ecatTask(
        &taskManager, .001, "ecat", &EtherCATInterface::runTask, &ethercatIf);

    ecatTask.start();

    for (;;)
    {
        taskManager.printStatus();
        taskManager.printStatusOfSlowTasks();

        std::this_thread::sleep_for(std::chrono::milliseconds(1 * 1000));
    }

    // signal(SIGINT, signal_handler);
}
