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
    // std::this_thread::sleep_for(std::chrono::seconds(5));

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
    // std::array<float, NUM_JOINTS> pose1 = {280.0f, 150.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed1 = 30.0f;
    // float start_speed1 = 0.0f;
    // float end_speed1 = 10.0f;

    // std::cout << "Executing 1 MoveL command..." << std::endl;
    // robot.moveL(pose1, speed1, start_speed1, end_speed1);
    // std::cout << "1 MoveL command finished." << std::endl;
    
    // std::array<float, NUM_JOINTS> pose2 {380.0f, 50.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed2 = 30.0f;
    // float start_speed2 = 30.0f;
    // float end_speed2 = 30.0f;

    // std::cout << "Executing 2 MoveL command..." << std::endl;
    // robot.moveL(pose2, speed2, start_speed2, end_speed2);
    // std::cout << "2 MoveL command finished." << std::endl;
    // //
    // std::array<float, NUM_JOINTS> pose3 {380.0f, -50.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed3 = 30.0f;
    // float start_speed3 = 30.0f;
    // float end_speed3 = 30.0f;

    // std::cout << "Executing 3 MoveL command..." << std::endl;
    // robot.moveL(pose3, speed3, start_speed3, end_speed3);
    // std::cout << "3 MoveL command finished." << std::endl;
    // //
    // std::array<float, NUM_JOINTS> pose4 {280.0f, -50.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed4 = 30.0f;
    // float start_speed4 = 30.0f;
    // float end_speed4 = 30.0f;

    // std::cout << "Executing 4 MoveL command..." << std::endl;
    // robot.moveL(pose4, speed4, start_speed4, end_speed4);
    // std::cout << "4 MoveL command finished." << std::endl;
    // //
    // std::array<float, NUM_JOINTS> pose5 {280.0f, 0.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed5 = 30.0f;
    // float start_speed5 = 30.0f;
    // float end_speed5 = 0.0f;

    // std::cout << "Executing 5 MoveL command..." << std::endl;
    // robot.moveL(pose5, speed5, start_speed5, end_speed5);
    // std::cout << "5 MoveL command finished." << std::endl;
    // //
    // std::array<float, NUM_JOINTS> pose6 {230.0f, 0.0f, 540.0f, M_PI, 0, -M_PI};
    // float speed6 = 50.0f;
    // float start_speed6 = 50.0f;
    // float end_speed6 = 0.0f;

    // std::cout << "Executing 6 MoveL command..." << std::endl;
    // robot.moveL(pose6, speed6, start_speed6, end_speed6);
    // std::cout << "6 MoveL command finished." << std::endl;

    // //定义moveC参数
    // std::array<float, NUM_JOINTS> mid_pose = {430.0f, 0.0f, 540.0f, M_PI, 0, -M_PI};
    // std::array<float, NUM_JOINTS> end_pose = {280.0f, -150.0f, 540.0f, M_PI, 0, -M_PI};

    // float speed3 = 30.0f;
    // float start_speed3 = 10.0f;
    // float end_speed3 = 0.0f;

    // std::cout << "Executing first MoveC command..." << std::endl;
    // robot.moveC(mid_pose, end_pose, speed3, start_speed3, end_speed3);
    // std::cout << "First MoveC command finished." << std::endl;


    // std::array<float, NUM_JOINTS> mid_pose2 = {330.0f, -50.0f, 540.0f, M_PI, 0, -M_PI};
    // std::array<float, NUM_JOINTS> end_pose2 = {280.0f, 0.0f, 540.0f, M_PI, 0, -M_PI};

    // float speed4 = 50.0f;
    // float start_speed4 = 50.0f;
    // float end_speed4 = 0.0f;

    // std::cout << "Executing second MoveC command..." << std::endl;
    // robot.moveC(mid_pose2, end_pose2, speed4, start_speed4, end_speed4);
    // std::cout << "Second MoveC command finished." << std::endl;
    
    // robot.jogL(0, 3, 0);




    for (;;)
    {

        // taskManager.printStatus();
        // taskManager.printStatusOfSlowTasks();

        std::this_thread::sleep_for(std::chrono::milliseconds(5 * 1000));
    }
}
