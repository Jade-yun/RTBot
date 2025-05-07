// ethercat/EtherCATInterface.h

#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#include <cstdint>
#include <vector>
#include <string>
#include <thread>

#include "ethercat/config_hcfa.h" 

struct StatusData {
    int32_t position_actual;
    int32_t velocity_actual;
    int16_t torque_actual;
    uint16_t status_word;
};

// ---- PDO 映射偏移量 ----
// 每个电机需要知道自己控制和状态字、目标位置等变量在PDO中的偏移
struct PDOOffsets {
  unsigned int control_word = 0xFFFFFFFF;
  unsigned int status_word = 0xFFFFFFFF;
  unsigned int target_position = 0xFFFFFFFF;
  unsigned int position_actual_value = 0xFFFFFFFF;
  unsigned int control_mode = 0xFFFFFFFF;
  unsigned int control_mode_display = 0xFFFFFFFF;
  unsigned int target_velocity = 0xFFFFFFFF;
  unsigned int velocity_actual_value = 0xFFFFFFFF;
  unsigned int target_torque = 0xFFFFFFFF;
  unsigned int torque_actual_value = 0xFFFFFFFF;

  // unsigned int Status_Word[NUM_SLAVE];
  // unsigned int Control_word[NUM_SLAVE];
  // unsigned int Target_Position[NUM_SLAVE];
  // unsigned int Position_Actual_Value[NUM_SLAVE];
  // unsigned int Control_Mode_Display[NUM_SLAVE];
  // unsigned int Control_Mode[NUM_SLAVE];
  // unsigned int Target_velocity[NUM_SLAVE];
  // unsigned int Velocity_actual_value[NUM_SLAVE];
};

class EtherCATInterface
{
public:
    EtherCATInterface();
    ~EtherCATInterface();

    bool init();     // 初始化总线
    void runTask();

    // --- 控制接口 ---
    void sendCSPCommand(int slave_id, int32_t position_target);
    void sendCSTCommand(int slave_id, int16_t torque_target);
    void sendCSVCommand(int slave_id, int32_t velocity_target);

    // --- 状态读取接口 ---
    void readStatus(int slave_id, StatusData& data);

    // --- 状态检查 ---
    // bool checkMasterState();
    // bool checkDomainState();
    // bool checkSlaveState(int slave_id);
public:
    uint8_t motor_start_flag = 0;

private:
    void updateProcessData();
    void sendProcessData();

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

    ec_slave_config_t *slave_config[2];
    ec_slave_config_state_t sc_state[2] = {};;

    std::vector<ec_slave_config_t*> slave_configs_;

    // PDO 偏移量
    std::vector<PDOOffsets> pdo_offsets_;
};

#endif // ETHERCAT_INTERFACE_H