// ethercat/EtherCATInterface.cpp

#include <iostream>
#include <unistd.h>        // sleep
#include <string.h>        // memset
#include <etherlab/ecrt.h> // IGH EtherCAT 主站API
#include <signal.h>
#include <math.h>
#include "ethercat/EtherCATInterface.h"

#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"
#include "Parameters/GlobalParameters.h"
#include <chrono>


#ifdef USE_DYNAMICS_MODEL
#include "dynamics/dynamical_model.h"
DynamicalModel model;
#endif

#define SYNC_MASTER_TO_REF

#define CYCLE_TIME_NS (1000000) // 1ms
#define NSEC_PER_SEC (1000000000L)

#define VENDOR_ID (0x000116C7)
#define PRODUCT_ID (0x006b0402)
#define CLOCK_TO_USE CLOCK_REALTIME

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

constexpr int NUM_SLAVES = 2;

uint32_t Joint_Zero_Offset[6] = {500000, 150000}; //记录偏移

uint32_t SlaveVID[] = {0x000116C7, 0x000116C7, 0x000116C7, 0x000116C7, 0x000116C7, 0x000116C7};
uint32_t SlavePID[] = {0x005e0402, 0x006b0402, 0x005e0402, 0x005e0402, 0x005e0402, 0x005e0402};
// #define PRODUCT_ID 0x005e0402, 0x006b0402, 0x006b0402;


/*Offsets for PDO entries*/
static struct{
    unsigned int Status_Word[NUM_SLAVES];
    unsigned int Control_word[NUM_SLAVES];
    unsigned int Target_Position[NUM_SLAVES];
    unsigned int Position_Actual_Value[NUM_SLAVES];
    unsigned int Control_Mode_Display[NUM_SLAVES];
    unsigned int Control_Mode[NUM_SLAVES];
    unsigned int Target_velocity[NUM_SLAVES];
    unsigned int Velocity_actual_value[NUM_SLAVES];

    unsigned int targetTorque[NUM_SLAVES];
    unsigned int actualTorque[NUM_SLAVES];

    unsigned int Max_motor_speed[NUM_SLAVES];
    unsigned int Profile_acceleration[NUM_SLAVES];
    unsigned int Profile_dec[NUM_SLAVES];
    unsigned int Profile_Max_Velocity[NUM_SLAVES];
    unsigned int Max_Torque[NUM_SLAVES];
    unsigned int Min_Torque[NUM_SLAVES];
    unsigned int Profile_Velocity[NUM_SLAVES];
    unsigned int Max_acceleration[NUM_SLAVES];
    unsigned int Max_dec[NUM_SLAVES];
}offset;


// 列出所有需要通信的对象字典条目
ec_pdo_entry_info_t slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Control word
//    {0x6060, 0x00,  8}, // Control mode
    {0x607A, 0x00, 32}, // Target position
    {0x60FF, 0x00, 32}, // Target Velocity
    {0x6071, 0x00, 16}, // Target torque

    //
    {0x6041, 0x00, 16}, // Status word
//    {0x6061, 0x00,  8}, // Control_Mode_Display
    {0x6064, 0x00, 32}, // Actual position
    {0x606C, 0x00, 32}, // Actual velocity
    {0x6077, 0x00, 16}, // Actual torque
};

// 按方向分组，组织成PDO包
// 0x1600：输出PDO
// 0x1A00：输入PDO
ec_pdo_info_t slave_pdos[] = {
    {0x1600, 4, slave_pdo_entries + 0},
    {0x1a00, 4, slave_pdo_entries + 4},
};


// 把PDO挂到正确的同步管理器上
ec_sync_info_t slave_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
        {0xff}
    };
    

    // SharedMemoryManager<SharedMemoryData>  shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);
extern SharedMemoryManager<SharedMemoryData> shm;

EtherCATInterface::EtherCATInterface()
    : running_(false), master_(nullptr), domain_(nullptr), domain_pd_(nullptr)
{

}

EtherCATInterface::~EtherCATInterface()
{

}

// 信号处理函数
void EtherCATInterface::signal_handler() {
        //deal ethercat signal
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        check_domain_state();
        //Check for master state
        check_master_state();
        //Check for slave configuration state(s)
        for(int i = 0; i < NUM_SLAVES; i++)
        {
            check_slave_config_states(slave_config[i], i);
        }

        for(int i = 0; i < NUM_SLAVES; i++) {
            //stop motor
            EC_WRITE_S32(domain_pd_ + offset.Target_velocity[i], 0);
            //disable motor
            EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0);
            //change not mode
//            EC_WRITE_S8(domain_pd_ + offset.Control_Mode[i], 0);
        }

        //stop deal
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);

        //exit handle
        munlockall();
        exit(0);
}
/*****************************************************************************/
/* Note: Anything relying on definition of SYNC_MASTER_TO_REF is essentially copy-pasted from /rtdm_rtai_dc/main.c */

#ifdef SYNC_MASTER_TO_REF

/* First used in system_time_ns() */
static int64_t  system_time_base = 0LL;
/* First used in sync_distributed_clocks() */
static uint64_t dc_time_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
/* First used in update_master_clock() */
static int32_t  dc_diff_ns = 0;
static unsigned int cycle_ns = CYCLE_TIME_NS;
static uint8_t  dc_started = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#define DC_FILTER_CNT          1024
/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

static uint64_t dc_start_time_ns = 0LL;

#endif
/*****************************************************************************/

#ifdef SYNC_MASTER_TO_REF

/** Get the time in ns for the current cpu, adjusted by system_time_base.
 *
 * \attention Rather than calling rt_get_time_ns() directly, all application
 * time calls should use this method instead.
 *
 * \ret The time in ns.
 */
uint64_t system_time_ns(void)
{
    struct timespec time;
    int64_t time_ns;
    clock_gettime(CLOCK_MONOTONIC, &time);
    time_ns = TIMESPEC2NS(time);

    if (system_time_base > time_ns)
    {
        printf("%s() error: system_time_base greater than"
               " system time (system_time_base: %ld, time: %lu\n",
            __func__, system_time_base, time_ns);
        return time_ns;
    }
    else
    {
        return time_ns - system_time_base;
    }
}
/** Synchronise the distributed clocks
 */
void EtherCATInterface::sync_distributed_clocks(void)
{

    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;

    dc_time_ns = system_time_ns();

    // set master time in nano-seconds
    ecrt_master_application_time(master_, dc_time_ns);

    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master_, &ref_time);
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master_);
}
/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void EtherCATInterface::update_master_clock(void)
{

    // calc drift (via un-normalised time diff)
    int32_t delta = dc_diff_ns - prev_dc_diff_ns;
    //printf("%d\n", (int) delta);
    prev_dc_diff_ns = dc_diff_ns;

    // normalise the time diff
    dc_diff_ns = ((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);

    // only update if primary master
    if (dc_started)
    {

        // add to totals
        dc_diff_total_ns += dc_diff_ns;
        dc_delta_total_ns += delta;
        dc_filter_idx++;

        if (dc_filter_idx >= DC_FILTER_CNT)
        {
            // add rounded delta average
            dc_adjust_ns += ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

            // and add adjustment for general diff (to pull in drift)
            dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

            // limit crazy numbers (0.1% of std cycle time)
//            if (dc_adjust_ns < -1000)
//            {
//                dc_adjust_ns = -1000;
//            }
//            if (dc_adjust_ns > 1000)
//            {
//                dc_adjust_ns =  1000;
//            }
            if (dc_adjust_ns < - (CYCLE_TIME_NS / 1000))
            {
                dc_adjust_ns = -(CYCLE_TIME_NS / 1000);
            }
            if (dc_adjust_ns > (CYCLE_TIME_NS / 1000))
            {
                dc_adjust_ns =  (CYCLE_TIME_NS / 1000);
            }

            // reset
            dc_diff_total_ns = 0LL;
            dc_delta_total_ns = 0LL;
            dc_filter_idx = 0;
        }

        // add cycles adjustment to time base (including a spot adjustment)
        system_time_base += dc_adjust_ns + sign(dc_diff_ns);
    }
    else
    {
        dc_started = (dc_diff_ns != 0);

        if (dc_started)
        {
            // output first diff
            printf("First master diff: %d.\n", dc_diff_ns);

            // record the time of this initial cycle
            dc_start_time_ns = dc_time_ns;
        }
    }
}

#endif

void ODwrite(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint8_t objectValue)
{
    /* Blocks until a reponse is received */
    uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, &objectValue, sizeof(objectValue), NULL);
    /* retVal != 0: Failure */
    if (retVal)
        printf("OD write unsuccessful\n");
}

void initDrive(ec_master_t* master, uint16_t slavePos, uint8_t mode)
{
    /* Mode of operation*/
    ODwrite(master, slavePos, 0x6060, 0x00, mode);  // 0x09 for CSV mode
    /* Reset alarm */
    ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

bool EtherCATInterface::init()
{  
    master_ = ecrt_request_master(0);
    if (!master_)
    {
        std::cerr << "Failed to request master.\n";
        return false;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_)
    {
        std::cerr << "Failed to create domain.\n";
        return false;
    }

    for (int i = 0; i < NUM_SLAVES; i++)
    {
        // CSP
        initDrive(master_, i, 0x08);
    }

    // 初始化 slaves
    for (int i = 0; i < NUM_SLAVES; i++)
    {
         slave_config[i] = ecrt_master_slave_config(master_, 0, i, SlaveVID[i], SlavePID[i]);
//        slave_config[i] = ecrt_master_slave_config(master_, SlavePos[i], SlaveNum[i], SlaveVID[i], SlavePID[i]);

        if (!slave_config[i])
        {
            std::cerr << "Failed to get slave config for slave " << i << ".\n";
            return false;
        }

        if (ecrt_slave_config_pdos(slave_config[i], EC_END, slave_syncs))
        {
            std::cerr << "Failed to get slave" << i << "configuration.\n";
            return false;
        }
    }

    printf("Configuring PDOs...\n");

    std::vector<ec_pdo_entry_reg_t> domain_regs;
//    domain_regs.reserve(NUM_SLAVES * 8 + 1);

    for (int i = 0; i < NUM_SLAVES; i++) {
        uint16_t alias = 0;
        uint16_t position = static_cast<uint16_t>(i);

        // {Alias, Position, Vendor ID, Product Code, PDO Index, PDO entry subindex, Offset, bit_position}
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6040, 0x00, &offset.Control_word[i], nullptr});
//        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6060, 0x00, &offset.Control_Mode[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x607A, 0x00, &offset.Target_Position[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x60FF, 0x00, &offset.Target_velocity[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6071, 0x00, &offset.targetTorque[i], nullptr});

        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6041, 0x00, &offset.Status_Word[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6064, 0x00, &offset.Position_Actual_Value[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x606c, 0x00, &offset.Velocity_actual_value[i], nullptr});
        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6077, 0x00, &offset.actualTorque[i], nullptr});
//        domain_regs.push_back({alias, position, SlaveVID[i], SlavePID[i], 0x6061, 0x00, &offset.Control_Mode_Display[i], nullptr});
    }

    domain_regs.push_back({});




    if (ecrt_domain_reg_pdo_entry_list(domain_, domain_regs.data()))
    {
        fprintf(stderr, "pdo入口注册失败\n");
        return -1;
    }

    for(int i = 0; i < NUM_SLAVES; i++)
    {
        ecrt_slave_config_dc(slave_config[i], 0x0300, CYCLE_TIME_NS, CYCLE_TIME_NS/2, 0, 0);
    }

#ifdef SYNC_MASTER_TO_REF
    /* Initialize master application time. */
    dc_start_time_ns = system_time_ns();
    dc_time_ns = dc_start_time_ns;
    ecrt_master_application_time(master_, dc_start_time_ns);

    if (ecrt_master_select_reference_clock(master_, slave_config[0]))
    {
        printf("Selecting slave 0 as reference clock failed!\n");
        return -1;
    }
#endif

    if (ecrt_master_activate(master_))
    {
        std::cerr << "Failed to activate master.\n";
        return false;
    }

    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_)
    {
        std::cerr << "Failed to get domain data pointer.\n";
        return false;
    }

    return true;
}

void EtherCATInterface::runTask()
{
    static uint16_t cycle_counter = 0;
#ifdef USE_DYNAMICS_MODEL
        // if (cycle_counter %100 == 0)
        // {
    //        auto start = std::chrono::high_resolution_clock::now();

    //        // 期望轨迹
    //        Vector6d qd;
    ////        q.setZero();
    ////        q << 30, 45, 60;
    //        qd << 30, 45, 60, 30, 60, 45;
    //        qd *= M_PI/180;

    //        Vector6d dqd = qd*2;
    //        Vector6d ddqd = qd*3;

    //        // 实际轨迹
    //        Vector6d q = qd;
    //        Vector6d dq = dqd;
    //        Vector6d ddq = ddqd;


    //        Vector6d tau = model.rnea(q, dq, ddq);

    //        ddq.setZero();
    //        dq = dqd;
    //        ddq.setZero();
    //        Vector6d zero6d = Vector6d::Zero();
    //        Vector6d tau_coriolis = model.rnea(q, dq, zero6d);

    //        Vector6d G;
    //        dq.setZero();
    //        ddq.setZero();
    //        G = model.rnea(q, dq, ddq);

    //        Eigen::MatrixXd M;
    //        M.resize(6,6);
    //        M.setZero();

    //        Vector6d tau_i;
    //        tau_i.setZero();
    //        dq.setZero();
    //        for(short int i=0; i<6; i++){
    //            ddq.setZero();
    //            ddq[i] = 1;
    //            tau_i = model.rnea( q, dq, ddq);
    //            for(short int j=0; j<6; j++){
    //                M(j,i) = tau_i[j] - G[j];
    //            }
    //            tau_i.setZero();
    //        }
    //        std::cout << "--------------------tau vector------------------" << std::endl;
    //        std::cout << tau << std::endl;

    //        std::cout << "--------------------M matrix--------------------" << std::endl;
    //        std::cout << M << std::endl;

    //        std::cout << "--------------------C(q,dq)q+G(q)--------------------" << std::endl;
    //        std::cout << tau_coriolis << std::endl;

    //        std::cout << "--------------------G(q)------------------------" << std::endl;
    //        std::cout << G << std::endl;


    //        auto end = std::chrono::high_resolution_clock::now();
    //        std::chrono::duration<double, std::milli> duration = end - start;
    //        std::cout << "Takes Time: " << duration.count() << " ms" << std::endl;
        // }
#endif

    // 更新和发送过程数据
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);
    motor_start_flag = 1;

    cycle_counter++;
    if(!(cycle_counter % 500))
    {
        cycle_counter = 0;
        check_domain_state();
        check_master_state();
        for(int i = 0; i < NUM_SLAVES; i++)
        {
            check_slave_config_states(slave_config[i], i);
        }
    }

    

    // read status word
    uint16_t state_value[NUM_SLAVES];
    for (int i = 0; i < NUM_SLAVES; i++)
    {
        state_value[i] = EC_READ_U16(domain_pd_ + offset.Status_Word[i]); // 读取电机状态字
    }

    //printf("statue: %d\n", state_value[1]);

    cia402_state_t servo_state[NUM_SLAVES];

    bool all_switched_on = true;
    bool all_operation_enable = true;



    // 电机当前位置的脉冲值
    std::array<int32_t, NUM_SLAVES> actual_pos_pulse;
    std::array<int32_t, NUM_SLAVES> actual_vel;
    std::array<int32_t, NUM_SLAVES> actual_torque;
    for (int16_t i = 0; i < NUM_SLAVES; i++)
    {
        servo_state[i] = get_axis_state(state_value[i]);
        all_switched_on = all_switched_on && (servo_state[i] == switched_on);
        all_operation_enable = all_operation_enable && (servo_state[i] == operation_enable);
        
        actual_pos_pulse[i] = EC_READ_S32(domain_pd_ + offset.Position_Actual_Value[i]);

        actual_vel[i] = EC_READ_S32(domain_pd_ + offset.Velocity_actual_value[i]);

        actual_torque[i] = EC_READ_S32(domain_pd_ + offset.actualTorque[i]);
    }
    static std::array<signed int, NUM_SLAVES> target_pos_pulse;
    static bool target_pos_initialized = false;
    static bool last_all_operation_enable = false;
    
    // 只在第一次或者电机刚启用时同步位置，避免突转和脉冲缓慢变小
    if (!target_pos_initialized || (!last_all_operation_enable && all_operation_enable)) {
        target_pos_pulse = actual_pos_pulse;
        target_pos_initialized = true;
    }
    last_all_operation_enable = all_operation_enable;

    static uint32_t print_cnt = 0;
    if (print_cnt == 100)
    {
        printf("Joint Current: %d %d\n", actual_pos_pulse[0], actual_pos_pulse[1]);   
        printf("target pulse: %d %d\n",target_pos_pulse[0], target_pos_pulse[1]);
        // printf("Joint Velocity: %d %d\n", actual_vel[0], actual_vel[1]); 
        printf("GlobalParams::isMoving: %d\n", GlobalParams::isMoving);
  
        // printf("Joint Torque: %d %d\n", actual_torque[0], actual_torque[1]); 

        print_cnt = 0;
    }   
    print_cnt++;

    // static std::array<signed int, NUM_SLAVES> target_pos_pulse;
    // target_pos_pulse = actual_pos_pulse;

    if (all_operation_enable)
    {

        // 获取电机指令
        LowLevelCommand montor_cmd;
        if (GlobalParams::joint_commands.try_dequeue(montor_cmd))
        {
            for(int i = 0; i < NUM_SLAVES; i++) 
            {

                target_pos_pulse[i] = montor_cmd.joint_pos[i];
            }
            // printf("Joint Current: %d %d\n", actual_pos_pulse[0], actual_pos_pulse[1]);   
            // printf("Joint Velocity: %d %d\n", actual_vel[0], actual_vel[1]);   
        }
        
        for (int i = 0; i < NUM_SLAVES; i++)
        {
//            static int value[NUM_SLAVES] = {0};
//            auto cur_value = (EC_READ_S32(domain_pd_ + offset.Position_Actual_Value[i]));
//            EC_WRITE_U32(domain_pd_ + offset.Target_Position[i], cur_value + value[i]);
//            value[i] += 0xff * print_cnt;

            // auto value = (EC_READ_S32(domain_pd_ + offset.Position_Actual_Value[i]));
            EC_WRITE_U32(domain_pd_ + offset.Target_Position[i], target_pos_pulse[i]);
        }
    }
    
    // enable motor
    if (motor_start_flag == 1)
    {
        for (int16_t i = 0; i < NUM_SLAVES; i++)
        {
            switch (servo_state[i])
            {
            case (no_ready_to_switch_on):
                EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x80);
                break;
            case (switch_on_disable):
                EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x06);
                break;
            case (ready_to_switch_on):
                EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x07);
                break;
                
                /*
                case (switched_on):
                    EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x0f);
                break;*/

//            case (operation_enable):
//            {
////                auto value = (EC_READ_S32(domain_pd_ + offset.Position_Actual_Value[i]));
////                EC_WRITE_U32(domain_pd_ + offset.Target_Position[i], value + target_pos[i]);
//                break;
//            }
            case (quick_stop_active):
            case (fault_reaction_active):
                break;
            case (fault):
                EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x80);
                break;
            default:
                break;
            }

            if (all_switched_on)
            {//enable
                
                EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x0f);
            }
        }
    }
    else if (motor_start_flag == 0)
    {
        // disable motor
        for (int16_t i = 0; i < NUM_SLAVES; i++)
        {
            EC_WRITE_U16(domain_pd_ + offset.Control_word[i], 0x00);
        }
    }

//    // sync every cycle
//    clock_gettime(CLOCK_TO_USE, &time);
//    ecrt_master_sync_reference_clock_to(master_, TIMESPEC2NS(time));
//    ecrt_master_sync_slave_clocks(master_);

//    sendProcessData();

#ifdef SYNC_MASTER_TO_REF
    ecrt_domain_queue(domain_);
    // sync distributed clock just before master_send to set
    // most accurate master clock time
    sync_distributed_clocks();

    /* Sends all datagrams in the queue.
   This method takes all datagrams that have been queued for transmission,
   puts them into frames, and passes them to the Ethernet device for sending.
*/
    ecrt_master_send(master_);
    // update the master clock
    // Note: called after ecrt_master_send() to reduce time
    // jitter in the sync_distributed_clocks() call
    update_master_clock();
#endif


    // uint32_t jointsStateFlag = 0;

    // for(int i = 0; i < NUM_SLAVES; i++) 
    // {
    //     // 当实际脉冲和目标脉冲在正负2个脉冲值误差范围内时，认为已到达目标位置

    //     if (abs(pos[i] - actual_pos_pulse[i]) <= 2)
    //         jointsStateFlag |= (1 << i);
    //     else
    //         jointsStateFlag &= ~(1 << i);
    // }

    // 传递电机状态 以供其他模块使用
    RobotState cur_state;
    bool moveFlag;
    for (int i = 0; i < NUM_SLAVES; i++)
    {

        // 脉冲 -> 角度(弧度)
        double current_pos; 
        // current_pos = actual_pos_pulse[i] * robot.m_gearRatio[i] * (M_PI / 360.0f);
        current_pos = actual_pos_pulse[i] / 500000.0 * (M_PI / 13.1072);
        cur_state.joint_state[i].position = current_pos;
        cur_state.joint_state[i].velocity = actual_vel[i] / 500000.0 * (M_PI / 13.1072);
        // cur_state.joint_state[i].torque = actual_torque[i]; 

        if (abs(target_pos_pulse[i] - actual_pos_pulse[i]) <= 2)
        {
            cur_state.joint_state[i].motor_state = 0;
        }
        else 
        {
            cur_state.joint_state[i].motor_state = 1;
        }
        moveFlag = moveFlag || cur_state.joint_state[i].motor_state;
        GlobalParams::isMoving = moveFlag;
        
    }

    shm().state_buffer.write(cur_state);

    //    robot->updateJointStates();

    //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
}




void EtherCATInterface::check_domain_state()
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain_ , &ds);
    if(ds.working_counter != domain_state.working_counter)
    {
        printf("发生了不完整的数据帧传输，当前工作计数器为%u\n",ds.working_counter);
    }
    if(ds.wc_state != domain_state.wc_state)
    {
        printf("工作计数器状态改变为%u\n",ds.wc_state);
    }
    domain_state = ds;
}

void EtherCATInterface::check_master_state()
{
    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

void EtherCATInterface::check_slave_config_states(ec_slave_config_t *sc, int i)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc, &s);
    if (s.al_state != sc_state[i].al_state)
    {
        printf("slave: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_state[i].online)
    {
        printf("slave: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_state[i].operational)
    {
        printf("slave: %soperational.\n", s.operational ? "" : "Not ");
    }
    sc_state[i] = s;
}

cia402_state_t EtherCATInterface::get_axis_state(uint16_t status_word)
{
    if ((status_word & 0x4F) == 0x40)
        return switch_on_disable; // cannot enable
    if ((status_word & 0x6F) == 0x21)
        return ready_to_switch_on; // can enable
    if ((status_word & 0x6F) == 0x23)
        return switched_on; // can enable
    if ((status_word & 0x6F) == 0x27)
        return operation_enable; // can send position
    if ((status_word & 0x6F) == 0x07)
        return quick_stop_active;
    if ((status_word & 0x4F) == 0xF)
        return fault_reaction_active;
    if ((status_word & 0x4F) == 0x08)
        return fault;
    else
        return no_ready_to_switch_on;
}
