#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>

#include "ethercat/EtherCATInterface.h"
#include "Utilities/PeriodicTask.h"

/*
//Application parameters
#define AnaInSlavePos  0, 0
#define AnaInSlavePos2 0, 1
#define ZeroErr 0x000116c7, 0x005e0402
#define ZeroErr2 0x000116c7, 0x006b0402

#define  NUM_SLAVE 2
#define  FREQUENCY 125
#define  CLOCK_TO_USE CLOCK_REALTIME

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};
static uint8_t *domain_pd = NULL;

static ec_slave_config_t *slave_config2[2];
static ec_slave_config_state_t sc_state[2] = {};
uint16_t SlavePos1[2] = {0, 0};
uint16_t SlaveNum1[2] = {0, 1};

uint32_t SlaveVID1[2] = {0x000116C7, 0x000116C7};
uint32_t SlavePID1[2] = {0x005e0402, 0x006b0402};

const struct timespec cycletime = {0, PERIOD_NS};

struct timespec wakeupTime, time3;

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}


static struct{
    unsigned int Status_Word[2];
    unsigned int Control_word[2];
    unsigned int Target_Position[2];
    unsigned int Position_Actual_Value[2];
    unsigned int Control_Mode_Display[2];
    unsigned int Control_Mode[2];
    unsigned int Target_velocity[2];
    unsigned int Max_motor_speed[2];
    unsigned int Velocity_actual_value[2];
    unsigned int Profile_acceleration[2];
    unsigned int Profile_dec[2];
    unsigned int Profile_Max_Velocity[2];
    unsigned int Max_Torque[2];
    unsigned int Min_Torque[2];
    unsigned int Profile_Velocity[2];
    unsigned int Max_acceleration[2];
    unsigned int Max_dec[2];
}offset;*/

// //PDO偏移
// ec_pdo_entry_info_t slave_0_pdo_entries[] = {
//     //Rx
//     {0x6040, 0x00, 16}, /* Control Word */
//     {0x6060, 0x00, 8}, /* Modes of operation  */
//     {0x6071, 0x00, 16}, /* Target_torque */
//     {0x607A, 0x00, 32}, /* Target Position  */
//     {0x6080, 0x00, 32}, /* Max Speed  */
//     {0x60B8, 0x00, 16}, /* Probe_function */
//     {0x60FF, 0x00, 32}, /* Target Velocity */
//     {0x6081, 0x00, 32}, /* Profile_Velocity */

//     //Tx
//     {0x603F, 0x00, 16}, /* Error code */
//     {0x6041, 0x00, 16}, /* Status Word */
//     {0x6061, 0x00, 8},  /* Modes of operation Display */
//     {0x6064, 0x00, 32}, /* Position actual value */
//     {0x606C, 0x00, 32}, /* Velocity actual value */
//     {0x6077, 0x00, 16}, /* T actual value */
//     {0x60B9, 0x00, 16}, /*  */
//     {0x60BA, 0x00, 32}, /*  */
//     {0x60BB, 0x00, 32}, /*  */
//     {0x60FD, 0x00, 32}, /* Digital input */
// };

// ec_pdo_info_t slave_0_pdos[] = {
//     {0x1600, 8, slave_0_pdo_entries + 0}, /* 2nd RxPDO-Mapping */
//     {0x1a00, 10, slave_0_pdo_entries + 8}, /* 2nd TxPDO-Mapping */
// };

// ec_sync_info_t slave_0_syncs[] = {
//     {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
//     {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
//     {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
//     {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
//     {0xff}
// };

// const static ec_pdo_entry_reg_t domain_regs[] = {
//     //  {Alias, Position, Vendor ID, Product Code, PDO Index, -Subindex, Offset}
//     //AnaInSlavePos
//     {AnaInSlavePos, ZeroErr, 0x6041, 0x00, &offset.Status_Word[0]},
//     {AnaInSlavePos, ZeroErr, 0x6040, 0x00, &offset.Control_word[0]},
//     {AnaInSlavePos, ZeroErr, 0x6064, 0x00, &offset.Position_Actual_Value[0]},
//     {AnaInSlavePos, ZeroErr, 0x6060, 0x00, &offset.Control_Mode[0]},
//     {AnaInSlavePos, ZeroErr, 0x60FF, 0x00, &offset.Target_velocity[0]},
//     {AnaInSlavePos, ZeroErr, 0x606c, 0x00, &offset.Velocity_actual_value[0]},
//     {AnaInSlavePos, ZeroErr, 0x6061, 0x00, &offset.Control_Mode_Display[0]},
//     {AnaInSlavePos, ZeroErr, 0x6080, 0x00, &offset.Max_motor_speed[0]},
//     {AnaInSlavePos, ZeroErr, 0x6081, 0x00, &offset.Profile_Velocity[0]},
//     {AnaInSlavePos, ZeroErr, 0x607A, 0x00, &offset.Target_Position[0]},

//     //AnaInSlavePos1
//     {AnaInSlavePos2, ZeroErr2, 0x6041, 0x00, &offset.Status_Word[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6040, 0x00, &offset.Control_word[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6064, 0x00, &offset.Position_Actual_Value[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6060, 0x00, &offset.Control_Mode[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x60FF, 0x00, &offset.Target_velocity[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x606c, 0x00, &offset.Velocity_actual_value[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6061, 0x00, &offset.Control_Mode_Display[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6080, 0x00, &offset.Max_motor_speed[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x6081, 0x00, &offset.Profile_Velocity[1]},
//     {AnaInSlavePos2, ZeroErr2, 0x607A, 0x00, &offset.Target_Position[1]},
//     {}
// };

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

/*
// 信号处理函数
void signal_handler(int signum) {
    if (signum == SIGINT) {
        printf("接收到 Ctrl+C 信号，正在退出程序...\n");

        //deal ethercat signal
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        check_domain_state2();
        //Check for master state
        check_master_state2();
        //Check for slave configuration state(s)
        for(int i = 0; i < NUM_SLAVE; i++)
        {
            check_slave_config_states2(slave_config2[i], i);
        }

        for(int i = 0; i < NUM_SLAVE; i++) {
            //stop motor
            EC_WRITE_S32(domain_pd + offset.Target_velocity[i], 0);
            //disable motor
            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0);
            //change not mode
            EC_WRITE_S8(domain_pd + offset.Control_Mode[i], 0);
        }

        //stop deal
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        //exit handle
        munlockall();
        exit(0);
    }
}*/


int main()
{
    PeriodicTaskManager taskManager;

    EtherCATInterface ethercatIf;

    int ret = ethercatIf.init();
    if (!ret)
    {
        printf("Failed to init EtherCAT master.\n");
        exit(-2);
    }

    PeriodicMemberFunction<EtherCATInterface> ecatTask(
        &taskManager, .001, "ecat", &EtherCATInterface::runTask, &ethercatIf);

    ecatTask.start();

    for (;;)
    {
        taskManager.printStatus();

        std::this_thread::sleep_for(std::chrono::milliseconds(1 * 1000));
    }

    //signal(SIGINT, signal_handler);
}
