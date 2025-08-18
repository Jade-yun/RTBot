// ethercat/EtherCATInterface.h

#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#include <cstdint>
#include <vector>
#include <string>
#include <thread>
#include <math.h>

#include "ethercat/config_hcfa.h"


#define MINROBOTPOSITION 1000000  //轴运动最小坐标
extern uint32_t Joint_Zero_Offset[6];

class EtherCATInterface
{
public:
    EtherCATInterface();
    ~EtherCATInterface();

    bool init();     // 初始化总线
    void runTask();

    void signal_handler();
public:
    uint8_t motor_start_flag = 0;

private:

    void sync_distributed_clocks();
    void update_master_clock();

    //EtherCAT check
    void check_domain_state();
    void check_master_state();
    void check_slave_config_states(ec_slave_config_t *sc, int i);

    cia402_state_t get_axis_state(uint16_t status_word);

    bool running_;

    struct timespec time;

    // EtherCAT master, domain, slaves
    ec_master_t* master_;
    ec_domain_t* domain_;
    uint8_t* domain_pd_;
    ec_domain_state_t domain_state = {};
    ec_master_state_t master_state = {};

    ec_slave_config_t *slave_config[6];
    ec_slave_config_state_t sc_state[6] = {};

private:
    // 需要设置的参数
    // 电机减速比
//    float radio = 95.87;
    // std::array<float, 6> m_GearRatio = {76, 76, 76, 60, 50, 50};
    // std::array<float, 6> m_Encoderbit = {17, 17, 17, 17, 17,17};
    // const std::array<float, 6> REST_JOINT = {M_PI_2, 0, 0, 0, 0, 0};
    // const std::array<float, 6> REST_JOINT = {0, 0, M_PI_2, 0, M_PI_2, 0};



};

#endif // ETHERCAT_INTERFACE_H