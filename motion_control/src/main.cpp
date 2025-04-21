#include "ethercat_interface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "SharedMemoryManager.h"

enum class ControlAxisMode {
    CSP, // Cyclic Synchronous Position
    CST  // Cyclic Synchronous Torque
};

ControlAxisMode axis_mode[AXIS_NUM] = {
    ControlAxisMode::CSP, // 轴0使用位置模式
    ControlAxisMode::CST  // 轴1使用转矩模式
};

void wait_until(struct timespec& next_time) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    next_time.tv_nsec += CYCLE_NS;
    if (next_time.tv_nsec >= 1'000'000'000) {
        next_time.tv_nsec -= 1'000'000'000;
        next_time.tv_sec += 1;
    }
}

// 状态机控制：状态字跳转判断 + 控制字设置
void enableDrive(EtherCATInterface& ec, int axis) {
    uint16_t status = ec.readStatusWord(axis);
    uint16_t control = 0;

    if (status & 0x08) { // fault
        control = 0x0080; // Fault Reset
    } else if ((status & 0x004F) == 0x0040) {
        control = 0x0006; // Shutdown
    } else if ((status & 0x006F) == 0x0021) {
        control = 0x0007; // Switch On
    } else if ((status & 0x027F) == 0x0233 || (status & 0x027F) == 0x0273) {
        control = 0x000F; // Enable Operation
    }

    ec.writeControlWord(axis, control);
}

int main() {

    EtherCATInterface ec_iface;
    if (!ec_iface.init("eth0", AXIS_NUM)) {
        std::cerr << "Init failed.\n";
        return -1;
    }

    // 设置模式：通过 SDO 设置操作模式，0x6060:0
    // for (int i = 0; i < AXIS_NUM; ++i) {
    //     int8_t mode_val = (axis_mode[i] == ControlMode::CSP) ? 8 : 10;
    //     // ec_iface.setOperationMode(i, mode_val); // 8: CSP, 10: CST
    // }

    std::cout << "Start motion loop...\n";
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    int32_t position[AXIS_NUM] = {0};
    int16_t torque[AXIS_NUM] = {0};

    while (true) {
        wait_until(next);

        ec_iface.update();  // 获取输入PDO数据

        for (int i = 0; i < AXIS_NUM; ++i) {
            enableDrive(ec_iface, i);

            if (axis_mode[i] == ControlAxisMode::CSP) {
                position[i] += 10; // 每周期增加10
                ec_iface.writeTargetPosition(i, position[i]);
            } else if (axis_mode[i] == ControlAxisMode::CST) {
                torque[i] = 100; // 示意恒定转矩值
                ec_iface.writeTargetTorque(i, torque[i]);
            }
        }

        ec_iface.update();  // 写出PDO数据
    }

    return 0;
}
