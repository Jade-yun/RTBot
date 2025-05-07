// ethercat/EtherCATInterface.cpp

#include <iostream>
#include <unistd.h>        // sleep
#include <string.h>        // memset
#include <etherlab/ecrt.h> // IGH EtherCAT 主站API
#include "ethercat/EtherCATInterface.h"

#include "Utilities/SharedMemoryManager.h"
#include "Parameters/SharedDataType.h"

#define CYCLE_TIME_NS (1000000) // 1ms
#define NSEC_PER_SEC (1000000000L)

#define VENDOR_ID (0x000116C7)
#define PRODUCT_ID (0x006b0402)
#define CLOCK_TO_USE CLOCK_REALTIME

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

constexpr int NUM_SLAVES = 2;
// #define PRODUCT_ID 0x005e0402, 0x006b0402, 0x006b0402;

// 每个 PDO 对应的信息：index, subindex, PDOOffsets 中的字段偏移
struct PdoEntryInfo
{
    uint16_t index;
    uint8_t subindex;
    size_t offset_in_struct;
};

// 统一表（你以后加成员，只要加一行就好）
static constexpr PdoEntryInfo pdo_entry_info_table[] = {
    {0x6040, 0x00, offsetof(PDOOffsets, control_word)},
    {0x6041, 0x00, offsetof(PDOOffsets, status_word)},
    {0x607A, 0x00, offsetof(PDOOffsets, target_position)},
    {0x6064, 0x00, offsetof(PDOOffsets, position_actual_value)},
    {0x6060, 0x00, offsetof(PDOOffsets, control_mode)},
    {0x6061, 0x00, offsetof(PDOOffsets, control_mode_display)},
    {0x60FF, 0x00, offsetof(PDOOffsets, target_velocity)},
    {0x606C, 0x00, offsetof(PDOOffsets, velocity_actual_value)},
    {0x6071, 0x00, offsetof(PDOOffsets, target_torque)},
    {0x6077, 0x00, offsetof(PDOOffsets, torque_actual_value)},
};

// 列出所有需要通信的对象字典条目
static ec_pdo_entry_info_t slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Control word
    {0x607A, 0x00, 32}, // Target position
    {0x6071, 0x00, 16}, // Target torque

    //
    {0x6041, 0x00, 16}, // Status word
    {0x6064, 0x00, 32}, // Actual position
    {0x606C, 0x00, 32}, // Actual velocity
    {0x6077, 0x00, 16}, // Actual torque
};

// 按方向分组，组织成PDO包
// 0x1600：输出PDO
// 0x1A00：输入PDO
static ec_pdo_info_t slave_pdos[] = {
    {0x1600, 3, &slave_pdo_entries[0]},
    {0x1A00, 4, &slave_pdo_entries[3]},
};

// 把PDO挂到正确的同步管理器上
// Sync Manager 0：发出去
// Sync Manager 1：收回来（从从站发给主站）
static constexpr ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, &slave_pdos[0], EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, &slave_pdos[1]},
    {0xff}};


// static SharedMemoryManager<SharedMemoryData> shm(SharedMemoryManager<SharedMemoryData>::Creator);

EtherCATInterface::EtherCATInterface()
    : running_(false), master_(nullptr), domain_(nullptr), domain_pd_(nullptr)
{
    pdo_offsets_.resize(NUM_SLAVES);
}

EtherCATInterface::~EtherCATInterface()
{

}

#if USE_PERIODICTASK_
void EtherCATInterface::init()
#else
bool EtherCATInterface::init()
{
#endif    
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

    // 初始化 slaves
    for (int i = 0; i < NUM_SLAVES; ++i)
    {
        ec_slave_config_t *sc = ecrt_master_slave_config(master_, 0, i, VENDOR_ID, PRODUCT_ID);

        if (!sc)
        {
            std::cerr << "Failed to get slave config for slave " << i << ".\n";
            return false;
        }
        slave_configs_.push_back(sc);

        ecrt_slave_config_sdo8(sc, 0x6060, 0x00, 8); // Mode of operation: 8 (CSP), 10 (CST)

        if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs))
        {
            std::cerr << "Failed to get slave" << i << "configuration.\n";
            return false;
        }

        // Enable DC Synchronization
        // ecrt_slave_config_dc(sc, 0x0300, CYCLE_TIME_NS, 4 * CYCLE_TIME_NS + 400 * 1000, 0, 0);
        ecrt_slave_config_dc(sc, 0x0300, CYCLE_TIME_NS, CYCLE_TIME_NS / 2, 0, 0);
    }

    // 配置同步管理器等
    // TO DO 

    std::vector<ec_pdo_entry_reg_t> regs;

    // for (uint16_t i = 0; i < NUM_SLAVES; ++i)
    // {

    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x6040, 0x00, &pdo_offsets_[i].control_word, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x6041, 0x00, &pdo_offsets_[i].status_word, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x607A, 0x00, &pdo_offsets_[i].target_position, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x6064, 0x00, &pdo_offsets_[i].position_actual_value, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x6071, 0x00, &pdo_offsets_[i].control_mode, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x6077, 0x00, &pdo_offsets_[i].control_mode_display, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x606C, 0x00, &pdo_offsets_[i].target_velocity, nullptr});
    //     // regs.push_back({0, i, VENDOR_ID, PRODUCT_ID, 0x606C, 0x00, &pdo_offsets_[i].velocity_actual_value, nullptr});

    // }
    for (uint16_t i = 0; i < NUM_SLAVES; ++i)
    {
        for (const auto &entry : pdo_entry_info_table)
        {
            regs.push_back(ec_pdo_entry_reg_t{
                0, i, VENDOR_ID, PRODUCT_ID,
                entry.index,
                entry.subindex,
                reinterpret_cast<unsigned int *>(
                    reinterpret_cast<uint8_t *>(&pdo_offsets_[i]) + entry.offset_in_struct),
                nullptr});
        }
    }
    // 最后一项必须是全 0 结构体，表示结束
    regs.push_back(ec_pdo_entry_reg_t{});

    if (ecrt_domain_reg_pdo_entry_list(domain_, &regs[0]))
    {
        fprintf(stderr, "pdo入口注册失败\n");
        return -1;
    }

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
#if 0
void EtherCATInterface::cyclicTask()
{

    struct timespec wakeup_time, time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while (running_)
    {
        wakeup_time.tv_nsec += CYCLE_TIME_NS;
        while (wakeup_time.tv_nsec >= 1000000000)
        {
            wakeup_time.tv_nsec -= 1000000000;
            wakeup_time.tv_sec += 1;
        }

        // 更新和发送过程数据
        updateProcessData();

        // cycle_counter++;
        // if(!(cycle_counter % 500))
        // {
        //     cycle_counter = 0;
        //     check_master_state();
        //     for(int i = 0; i < NUM_SLAVES; i++)
        //     {
        //         check_slave_config_states(slave_config[i], i);
        //     }
        // }

        // 设置电机控制模式
        for (int i = 0; i < NUM_SLAVES; i++)
        {
            EC_WRITE_S8(domain_pd_ + pdo_offsets_[i].control_mode, 8);
        }

        // read status word
        uint16_t state_value[NUM_SLAVES];
        for (int i = 0; i < NUM_SLAVES; i++)
        {
            state_value[i] = EC_READ_U16(domain_pd_ + pdo_offsets_[i].status_word); // 读取电机状态字
        }

        cia402_state_t servo_state[NUM_SLAVES];

        bool all_enable = false;
        for (int16_t i = 0; i < NUM_SLAVES; i++)
        {
            servo_state[i] = get_axis_state(state_value[i]);
            all_enable = all_enable && servo_state[i];
        }

        // enable motor
        if (motor_start_flag == 1)
        {
            for (int16_t i = 0; i < NUM_SLAVES; i++)
            {
                switch (servo_state[i])
                {
                case (no_ready_to_switch_on):
                    EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x80);
                    break;
                case (switch_on_disable):
                    EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x06);
                    break;
                case (ready_to_switch_on):
                    EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x07);
                    break;
                    /*
                    case (switched_on):
                            EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].Control_word[i], 0x0f);
                            break; */

                case (operation_enable):
                    auto value = (EC_READ_S32(domain_pd_ + pdo_offsets_[i].position_actual_value) + 0x3fff);
                    EC_WRITE_U32(domain_pd_ + pdo_offsets_[i].target_position, value);
                    break;
                case (quick_stop_active):
                case (fault_reaction_active):
                    break;
                case (fault):
                    EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x80);
                    break;
                default:
                    break;
                }

                if (all_enable)
                {
                    
                    EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x0f);
                }
            }
        }
        else if (motor_start_flag == 0)
        {
            // disable motor
            for (int16_t i = 0; i < NUM_SLAVES; i++)
            {
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x00);
            }
        }

        // read current position
        static uint16_t s = 0;
        if (!(s % 100))
        {
            for (int i = 0; i < NUM_SLAVES; i++)
            printf("pos%d: %x", i, EC_READ_U32(domain_pd_ + pdo_offsets_[i].position_actual_value));
        }

        sendProcessData();

        // sync every cycle
        clock_gettime(CLOCK_TO_USE, &time);
        ecrt_master_sync_reference_clock_to(master_, TIMESPEC2NS(time));
        ecrt_master_sync_slave_clocks(master_);

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
    }
}
#endif

void EtherCATInterface::runTask()
{

    // 更新和发送过程数据
    updateProcessData();

    // cycle_counter++;
    // if(!(cycle_counter % 500))
    // {
    //     cycle_counter = 0;
    //     check_master_state();
    //     for(int i = 0; i < NUM_SLAVES; i++)
    //     {
    //         check_slave_config_states(slave_config[i], i);
    //     }
    // }

    // 设置电机控制模式
    for (int i = 0; i < NUM_SLAVES; i++)
    {
        MotorMode mode = CSP;
        EC_WRITE_S8(domain_pd_ + pdo_offsets_[i].control_mode, mode);
    }

    // read status word
    uint16_t state_value[NUM_SLAVES];
    for (int i = 0; i < NUM_SLAVES; i++)
    {
        state_value[i] = EC_READ_U16(domain_pd_ + pdo_offsets_[i].status_word); // 读取电机状态字
    }

    cia402_state_t servo_state[NUM_SLAVES];

    bool all_switched_on = true;
    for (int16_t i = 0; i < NUM_SLAVES; i++)
    {
        servo_state[i] = get_axis_state(state_value[i]);
        all_switched_on = all_switched_on && (servo_state[i] == switched_on);
    }

    // enable motor
    if (motor_start_flag == 1)
    {
        for (int16_t i = 0; i < NUM_SLAVES; i++)
        {
            switch (servo_state[i])
            {
            case (no_ready_to_switch_on):
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x80);
                break;
            case (switch_on_disable):
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x06);
                break;
            case (ready_to_switch_on):
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x07);
                break;
                /*
                case (switched_on):
                        EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].Control_word[i], 0x0f);
                        break; */

            case (operation_enable):
            {
                auto value = (EC_READ_S32(domain_pd_ + pdo_offsets_[i].position_actual_value) + 0x3fff);
                EC_WRITE_U32(domain_pd_ + pdo_offsets_[i].target_position, value);
                break;
            }
            case (quick_stop_active):
            case (fault_reaction_active):
                break;
            case (fault):
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x80);
                break;
            default:
                break;
            }

            if (all_switched_on)
            {
                
                EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x0f);
            }
        }
    }
    else if (motor_start_flag == 0)
    {
        // disable motor
        for (int16_t i = 0; i < NUM_SLAVES; i++)
        {
            EC_WRITE_U16(domain_pd_ + pdo_offsets_[i].control_word, 0x00);
        }
    }

    // read current position
    static uint16_t s = 0;
    if (!(s % 100))
    {
        for (int i = 0; i < NUM_SLAVES; i++)
        printf("pos%d: %x", i, EC_READ_U32(domain_pd_ + pdo_offsets_[i].position_actual_value));
    }

    sendProcessData();

    // sync every cycle
    clock_gettime(CLOCK_TO_USE, &time);
    ecrt_master_sync_reference_clock_to(master_, TIMESPEC2NS(time));
    ecrt_master_sync_slave_clocks(master_);

    // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
 
}


void EtherCATInterface::updateProcessData()
{
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);

    // 可在此处添加从站状态检查等
}

void EtherCATInterface::sendProcessData()
{
    ecrt_domain_queue(domain_);
    ecrt_master_send(master_);
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

// --- 控制指令发送 ---
void EtherCATInterface::sendCSPCommand(int slave_id, int32_t position_target)
{
    if (slave_id < 0 || slave_id >= NUM_SLAVES)
        return;
    int offset = pdo_offsets_[slave_id].target_position;
    EC_WRITE_S32(domain_pd_ + offset, position_target);
}

void EtherCATInterface::sendCSTCommand(int slave_id, int16_t torque_target)
{
    if (slave_id < 0 || slave_id >= NUM_SLAVES)
        return;
    int offset = pdo_offsets_[slave_id].target_torque;
    EC_WRITE_S16(domain_pd_ + offset, torque_target);
}

void EtherCATInterface::sendCSVCommand(int slave_id, int32_t velocity_target)
{
    if (slave_id < 0 || slave_id >= NUM_SLAVES)
        return;
    int offset = pdo_offsets_[slave_id].target_velocity;
    EC_WRITE_S32(domain_pd_ + offset, velocity_target);
}

// --- 状态读取 ---
void EtherCATInterface::readStatus(int slave_id, StatusData &data)
{
    if (slave_id < 0 || slave_id >= NUM_SLAVES)
        return;

    data.position_actual = EC_READ_S32(domain_pd_ + pdo_offsets_[slave_id].position_actual_value);
    data.velocity_actual = EC_READ_S32(domain_pd_ + pdo_offsets_[slave_id].velocity_actual_value);
    data.torque_actual = EC_READ_S16(domain_pd_ + pdo_offsets_[slave_id].torque_actual_value);
    data.status_word = EC_READ_U16(domain_pd_ + pdo_offsets_[slave_id].status_word);
}

#if 0
// --- 状态检查 ---
bool EtherCATInterface::checkMasterState()
{
    ec_master_state_t master_state;
    ecrt_master_state(master_, &master_state);
    return (master_state.slaves_responding == NUM_SLAVES);
}

bool EtherCATInterface::checkDomainState()
{
    static ec_domain_state_t domain_state_last;

    ec_domain_state_t domain_state;
    ecrt_domain_state(domain_, &domain_state);
    return (domain_state.working_counter != domain_state_last.working_counter);
    domain_state_last = domain_state;
}

bool EtherCATInterface::checkSlaveState(int slave_id)
{
    if (slave_id < 0 || slave_id >= NUM_SLAVES)
        return false;

    ec_slave_config_state_t sc_state;
    ecrt_slave_config_state(slave_configs_[slave_id], &sc_state);
    return (sc_state.al_state == 0x08); // OPERATIONAL
}
#endif