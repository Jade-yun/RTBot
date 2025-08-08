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
    CPU_SET(2, &mask);  // 添加核心2
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


    robot.init();

    PeriodicMemberFunction<Robot> robotTask(
        &taskManager, 0.05, "robot", &Robot::controlLoop, &robot);

    robotTask.start();
    robotTask.setThreadPriority(80);

    // // 等待机器人和EtherCAT稳定
    // std::this_thread::sleep_for(std::chrono::seconds(3));

    // // 定义moveJ参数
    // std::array<float, NUM_JOINTS> joint_pos1 = {60.0f * M_PI / 180.0f, 0, 0, 0, 0, 0};
    // float speed1 = 30.0f;
    // float start_speed1 = 0.0f;
    // float end_speed1 = 10.0f;

    // std::cout << "Executing first MoveJ command..." << std::endl;
    // robot.moveJ(joint_pos1, speed1, start_speed1, end_speed1);
    // std::cout << "First MoveJ command finished." << std::endl;


    // std::array<float, NUM_JOINTS> joint_pos2 = {120.0f * M_PI / 180.0f, 0, 0, 0, 0, 0};
    // float speed2 = 20.0f;
    // float start_speed2 = 10.0f;
    // float end_speed2 = 0.0f;

    // std::cout << "Executing second MoveJ command..." << std::endl;
    // robot.moveJ(joint_pos2, speed2, start_speed2, end_speed2);
    // std::cout << "Second MoveJ command finished." << std::endl;


    // 定义moveL 参数
    // std::array<float, NUM_JOINTS> pose1 = {200.0f, 100.0f, 600.0f, 0, 0, 0};
    // float speed1 = 120.0f;
    // float start_speed1 = 0.0f;
    // float end_speed1 = 50.0f;

    // std::cout << "Executing first MoveL command..." << std::endl;
    // robot.moveL(pose1, speed1, start_speed1, end_speed1);
    // std::cout << "First MoveL command finished." << std::endl;


    // std::array<float, NUM_JOINTS> pose2 {300.0f, 300.0f, 500.0f, 0, 0, 0};
    // float speed2 = 80.0f;
    // float start_speed2 = 50.0f;
    // float end_speed2 = 0.0f;

    // std::cout << "Executing second MoveL command..." << std::endl;
    // robot.moveL(pose2, speed2, start_speed2, end_speed2);
    // std::cout << "Second MoveL command finished." << std::endl;

    //定义moveC参数
    // std::array<float, NUM_JOINTS> mid_pose = {100.0f, 0.0f, 600.0f, 0, 0, 0};
    // std::array<float, NUM_JOINTS> end_pose = {200.0f, -100.0f, 600.0f, 0, 0, 0};

    // float speed3 = 50.0f;
    // float start_speed3 = 50.0f;
    // float end_speed3 = 50.0f;

    // std::cout << "Executing first MoveC command..." << std::endl;
    // robot.moveC(mid_pose, end_pose, speed3, start_speed3, end_speed3);
    // std::cout << "First MoveC command finished." << std::endl;


    // std::array<float, NUM_JOINTS> mid_pose2 = {300.0f, 0.0f, 600.0f, 0, 0, 0};
    // std::array<float, NUM_JOINTS> end_pose2 = {200.0f, 100.0f, 600.0f, 0, 0, 0};

    // float speed4 = 50.0f;
    // float start_speed4 = 50.0f;
    // float end_speed4 = 0.0f;

    // std::cout << "Executing second MoveC command..." << std::endl;
    // robot.moveC(mid_pose2, end_pose2, speed4, start_speed4, end_speed4);
    // std::cout << "Second MoveC command finished." << std::endl;





    for (;;)
    {

        // taskManager.printStatus();
        // taskManager.printStatusOfSlowTasks();

        std::this_thread::sleep_for(std::chrono::milliseconds(5 * 1000));
    }
}
