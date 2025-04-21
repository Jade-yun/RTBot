#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#include <vector>
#include <etherlab/ecrt.h>

#define VENDOR_ID 0x0000066F  // 示例：Beckhoff
#define PRODUCT_ID 0x515050a1 // 示例：EL7031

// #define PANASONIC        0,0                        /*EtherCAT address on the bus*/

// #define VID_PID          0x0000066F,0x515050a1   /*Vendor ID, product code*/  /*供应商ID与产品码*/

#define AXIS_NUM 2
#define CYCLE_NS 1000000

struct PDOOffsets {
    unsigned int control_word;
    unsigned int target_position;
    unsigned int target_torque;
    unsigned int status_word;
    unsigned int actual_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
};

class EtherCATInterface {
public:
    ~EtherCATInterface();

    bool init(const char* ifname, int axis_num);
    void update();

    void writeTargetPosition(int axis, int32_t value);
    void writeTargetTorque(int axis, int16_t value);
    void writeControlWord(int axis, uint16_t value);

    uint16_t readStatusWord(int axis) const;
    int32_t readActualPosition(int axis) const;
    int32_t readActualVelocity(int axis) const;
    int16_t readActualTorque(int axis) const;

    // bool isDCSyncActive() const;
    // int64_t getDCSyncDelay() const;

private:
    ec_master_t* master = nullptr;
    ec_domain_t* domain_ = nullptr;
    uint8_t* domain_pd_ = nullptr;

    std::vector<ec_slave_config_t*> configs_;
    std::vector<PDOOffsets> offsets_;

    unsigned int control_word_[AXIS_NUM];
    unsigned int target_position_[AXIS_NUM];
    unsigned int target_torque_[AXIS_NUM];
    unsigned int status_word_[AXIS_NUM];
    unsigned int actual_position_[AXIS_NUM];
    unsigned int actual_velocity_[AXIS_NUM];
    unsigned int actual_torque_[AXIS_NUM];
};

#endif // ETHERCAT_INTERFACE_H
