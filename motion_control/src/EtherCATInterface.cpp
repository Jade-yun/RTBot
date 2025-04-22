#include "EtherCATInterface.h"

//定义主站和域
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};
static uint8_t *domain_pd = NULL;

static ec_slave_config_t *slave_config[NUM_SLAVE];
static ec_slave_config_state_t sc_state[NUM_SLAVE] = {};

volatile int motor_speed = 0; // 电机速度
volatile int motor_position = 0; // 电机位置
volatile int motor_start_flag = 0; // 电机启动标志

/*************************************************************************** */
const struct timespec cycletime = {0, PERIOD_NS};

uint16_t SlavePos[NUM_SLAVE] = {0, 0, 0};
uint16_t SlaveNum[NUM_SLAVE] = {0, 1, 2};

uint32_t SlaveVID[NUM_SLAVE] = {0x000116C7, 0x000116C7, 0x000116C7};
uint32_t SlavePID[NUM_SLAVE] = {0x005e0402, 0x006b0402, 0x006b0402};

//PDO偏移
ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    //Rx
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* Modes of operation  */
    {0x607A, 0x00, 32}, /* Target Position  */
    {0x60FF, 0x00, 32}, /* Target Velocity */

    //Tx
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6061, 0x00, 8},  /* Modes of operation Display */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606C, 0x00, 32}, /* Velocity actual value */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 4, slave_0_pdo_entries + 0}, /* 2nd RxPDO-Mapping */
    {0x1a00, 4, slave_0_pdo_entries + 4}, /* 2nd TxPDO-Mapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

//定义PDO信息
pdo_offset_t hcfa_x5_pdo_offsets[] = {
    {0x6040, offset.Control_word},
    {0x6060, offset.Control_Mode},
    {0x607a, offset.Target_Position},
    {0x6041, offset.Status_Word},
    {0x6064, offset.Position_Actual_Value},
    {0x606C, offset.Velocity_actual_value},
    {0x6061, offset.Control_Mode_Display},
    {0x60FF, offset.Target_velocity},
  };
ec_pdo_entry_reg_t domain_regs[NUM_SLAVE*NUM_PDOS+NUM_SLAVE];

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

/************************************************************************** */
//发送消息的频率设置
// 计时的时候需要使用到
struct period_info
{
    struct timespec next_period;
    long period_ns;
};

static void inc_period(struct period_info *pinfo)
{
    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000)
    {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

static void periodic_task_init(struct period_info *pinfo)
{
    /* for simplicity, hardcoding a 100 ms period */
    pinfo->period_ns = PERIOD_NS;

    /* find timestamp to first run */
    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

static void wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    &pinfo->next_period, NULL);
}

/************************************************************************* */
//监测技术器状态
void check_domain_state(void) {
    ec_domain_state_t ds;
    ecrt_domain_state(domain , &ds);
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

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
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

void check_slave_config_states(ec_slave_config_t *sc, int i)
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

/******************************************************************** */
void delay_nanoseconds(long nanoseconds) {
    struct timespec req, rem;

    req.tv_sec = nanoseconds / 1000000000;
    req.tv_nsec = nanoseconds % 1000000000;

    if (nanosleep(&req, &rem) == -1) {
            perror("nanosleep");
    }
}

/********************************************************************* */
//init pdo entries
void init_pdo_entries(void)
{
    unsigned int i, j, k;
    for (i = 0; i < NUM_SLAVE; i++) {
      //更改PDO偏移量
        for (j = 0; j < NUM_PDOS; j++) {
          int index = i * NUM_PDOS + j; // 计算一维数组的索引 
          // 填充PDO条目信息
          domain_regs[index].alias = MASTER_ID;
          domain_regs[index].position = i; // 从站号

          domain_regs[index].vendor_id = SlaveVID[i]; // 供应商ID
          domain_regs[index].product_code = SlavePID[i]; // 产品代码
          
          domain_regs[index].index= hcfa_x5_pdo_offsets[j].PDOIndex;          
          domain_regs[index].subindex = 0x00; // 子索引默认为零
          domain_regs[index].offset = hcfa_x5_pdo_offsets[j].Offset + i; // 偏移量
        }
    }
}

/*************************************************************************** */
//judge axis status
cia402_state_t get_axis_state(uint16_t status_word) {
    if ((status_word & 0x4F) == 0x40)
        return switch_on_disable;  //cannot enable
    if ((status_word & 0x6F) == 0x21)
        return ready_to_switch_on; //can enable
    if ((status_word & 0x6F) == 0x23)
        return switched_on;        //can enable 
    if ((status_word & 0x6F) == 0x27)
        return operation_enable;   //can send position
    if ((status_word & 0x6F) == 0x07)
        return quick_stop_active;  
    if ((status_word & 0x4F) == 0xF)
        return fault_reaction_active;
    if ((status_word & 0x4F) == 0x08)
        return fault;
    else
    return no_ready_to_switch_on;
}

/**************************************************************************** */
//cycle task start
void *simple_cyclic_task (void *data){
    printf("循环任务开始\n");
    struct period_info pinfo;
    periodic_task_init(&pinfo);

    struct timespec wakeupTime, time;
    uint16_t state_value[NUM_SLAVE];
    int32_t Current_Pos[NUM_SLAVE];

    uint8_t Current_Mode[NUM_SLAVE];
    
    int cycle_counter = 0; 
    motor_start_flag = 1;

    //get point

    while(1) {
        //clock_gettime(CLOCK_TO_USE, &T1);
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        check_domain_state();

        cycle_counter++;
        if(!(cycle_counter % 500))
        {
            cycle_counter = 0;
            check_master_state();
            for(int i = 0; i < NUM_SLAVE; i++)
            {
                check_slave_config_states(slave_config[i], i);
            }
        }

        uint32_t ref_time = 0;

        //read status word
        uint16_t state_value[NUM_SLAVE];
        for(int i = 0; i < NUM_SLAVE; i++) {
            state_value[i] = EC_READ_U16(domain_pd + offset.Status_Word[i]); //读取电机状态字
        }

        //read status
        cia402_state_t servo_state[NUM_SLAVE];
        for(int16_t i = 0; i < NUM_SLAVE; i++)
        {
            servo_state[i] = none;
        }
        for(int16_t i = 0; i < NUM_SLAVE; i++)
        {
            servo_state[i] = get_axis_state(state_value[i]);
        }
    
        //CSP MODE
        for(int i = 0; i < NUM_SLAVE; i++)
        {
            Current_Mode[i] = EC_READ_S8(domain_pd + offset.Control_Mode_Display[i]);
            if(Current_Mode[i] != 8) {
                EC_WRITE_S8(domain_pd + offset.Control_Mode[i], 8);  //Change mode to pp
            }
        }
        
        //enable motor
        if(motor_start_flag == 1) {
            for(int16_t i = 0; i < NUM_SLAVE; i++) {
                switch(servo_state[i]) {
                    case (no_ready_to_switch_on):
                            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x80);
                            break;
                    case (switch_on_disable):
                            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x06);
                            break;
                    case (ready_to_switch_on):
                            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x07);
                            break;
                    /*
                    case (switched_on):
                            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x0f);
                            break; */

                    case (operation_enable):
                            EC_WRITE_U32(domain_pd + offset.Target_Position[i], (EC_READ_S32(domain_pd + offset.Position_Actual_Value[i]) + 0x3fff));
                            break;
                    case (quick_stop_active):
                    case (fault_reaction_active):
                            break;
                    case (fault):
                            EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x80);
                            break;
                    default:
                            break;
                }
                if(servo_state[0] == switched_on && servo_state[1] == switched_on && servo_state[2] == switched_on) {
                    EC_WRITE_U16(domain_pd + offset.Control_word[0], 0x0f);
                    EC_WRITE_U16(domain_pd + offset.Control_word[1], 0x0f);
                    EC_WRITE_U16(domain_pd + offset.Control_word[2], 0x0f);
                }
            }
        } else if(motor_start_flag == 2) {
            
        } else if(motor_start_flag == 0) {
            //disable motor
            for(int16_t i = 0; i < NUM_SLAVE; i++) {
                EC_WRITE_U16(domain_pd + offset.Control_word[i], 0x00);
            }
        }

        //read current position
        static uint16_t s = 0;
        if(!(s%100))
            printf("S1: %x, S2: %x, S3: %x\n", EC_READ_U32(domain_pd + offset.Position_Actual_Value[0]), EC_READ_U32(domain_pd + offset.Position_Actual_Value[1]), EC_READ_U32(domain_pd + offset.Position_Actual_Value[2]));
        
        //sync every cycle
        clock_gettime(CLOCK_TO_USE, &time);
        ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        ecrt_master_sync_slave_clocks(master);
        
        //send process data
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        //delay_nanoseconds(1000);

        wakeupTime = timespec_add(wakeupTime, cycletime);
        wait_rest_of_period(&pinfo);
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
    }
    return NULL;
}

int init_ethercat() {

    int i = 0;

    //请求建立一个master
    master = ecrt_request_master(0);
    if(!master) {
        return -1;
    }
    
    //建立一个域
    domain = ecrt_master_create_domain(master);
    if(!domain) {
        return -1;
    }

    //用宏定义配置master slave
    //对齐PDO信息
    printf("配置PDO信息\n");
    for(i = 0; i < NUM_SLAVE; i++) {
        if(!(slave_config[i] = ecrt_master_slave_config(master, SlavePos[i], SlaveNum[i], SlaveVID[i], SlavePID[i])))
        {
            fprintf(stderr, "Failed to get slave %d configuration.\n", i);
            return -1;
        }
        if(ecrt_slave_config_pdos(slave_config[i], EC_END, slave_0_syncs))
        {
            fprintf(stderr, "Failed to get slave %d configuration.\n", i);
            return -1;
        }
    }

    //set PDOS
    init_pdo_entries();

    //在domain中注册PDO条目
    if(ecrt_domain_reg_pdo_entry_list(domain,domain_regs)){
        fprintf(stderr, "pdo入口注册失败\n");
        return -1;
    }

    //sync sc 0.5ms
    for(i = 0; i < NUM_SLAVE; i++) {
        ecrt_slave_config_dc(slave_config[i], 0x0300, 8000000, 0, 0, 0);
    }

    //激活master
    if(ecrt_master_activate(master)){
        return -1;
    }
    
    //把域的数据地址给domain_pd
    printf("激活master成功\n");
    if(!(domain_pd = ecrt_domain_data(domain))){
        return -1;
    }
    return 0;
}

/******************************************************************** */
// 信号处理函数
void signal_handler(int signum) {
    if (signum == SIGINT) {
        printf("接收到 Ctrl+C 信号，正在退出程序...\n");

        //deal ethercat signal
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        check_domain_state();
        //Check for master state
        check_master_state();
        //Check for slave configuration state(s)
        for(int i = 0; i < NUM_SLAVE; i++)
        {
            check_slave_config_states(slave_config[i], i);
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
}