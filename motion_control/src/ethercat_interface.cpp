#include "ethercat_interface.h"
#include <iostream>
#include <string.h>

EtherCATInterface::~EtherCATInterface() {
    if (master) {
        ecrt_release_master(master);
    }
}

bool EtherCATInterface::init(const char* ifname, int axis_num) {
    master = ecrt_request_master(0);
    if (!master) return false;

    domain_ = ecrt_master_create_domain(master);
    if (!domain_) return false;

    for (int i = 0; i < axis_num; ++i) {
        ec_slave_config_t* sc = ecrt_master_slave_config(master, 0, i, VENDOR_ID, PRODUCT_ID);
        if (!sc) return false;

        configs_.push_back(sc);

        ecrt_slave_config_sdo8(sc, 0x6060, 0x00, 8); // Mode of operation: 8 (CSP), 10 (CST)

        ec_pdo_entry_info_t slave_pdo_entries[] = {
            {0x6040, 0x00, 16}, // Control word
            {0x607A, 0x00, 32}, // Target position
            {0x6071, 0x00, 16}, // Target torque
            {0x6041, 0x00, 16}, // Status word
            {0x6064, 0x00, 32}, // Actual position
            {0x606C, 0x00, 32}, // Actual velocity
            {0x6077, 0x00, 16}, // Actual torque
        };

        ec_pdo_info_t slave_pdos[] = {
            {0x1600, 3, &slave_pdo_entries[0]}, // TxPDO: Control word, Target position, Target torque
            {0x1A00, 4, &slave_pdo_entries[3]}, // RxPDO: Status word, Actual position, Actual velocity, Actual torque
        };

        ec_sync_info_t slave_syncs[] = {
            {0, EC_DIR_OUTPUT, 1, &slave_pdos[0], EC_WD_ENABLE},
            {1, EC_DIR_INPUT, 1, &slave_pdos[1]},
            {0xff}
        };

        ecrt_slave_config_pdos(sc, EC_END, slave_syncs);

        // Enable DC Synchronization
        if (ecrt_slave_config_dc(sc, 0x0300, CYCLE_NS, 4 * CYCLE_NS + 400 * 1000, 0, 0)) {
            std::cerr << "Failed to configure DC sync for axis " << i << std::endl;
            return false;
        }
    }

        // -------------------------------
    // 正确的 PDO Entry 注册方式
    // -------------------------------

    std::vector<ec_pdo_entry_reg_t> regs;

    for (int i = 0; i < axis_num; ++i) {
        PDOOffsets ofs;
    
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x6040, 0x00, &ofs.control_word, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x607A, 0x00, &ofs.target_position, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x6071, 0x00, &ofs.target_torque, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x6041, 0x00, &ofs.status_word, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x6064, 0x00, &ofs.actual_position, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x606C, 0x00, &ofs.actual_velocity, nullptr});
        regs.push_back(ec_pdo_entry_reg_t{0, static_cast<uint16_t>(i), VENDOR_ID, PRODUCT_ID, 0x6077, 0x00, &ofs.actual_torque, nullptr});
    
        offsets_.push_back(ofs);
    }
    
    // 最后一项必须是全 0 结构体，表示结束
    regs.push_back(ec_pdo_entry_reg_t{});

    if (ecrt_master_activate(master)) return false;
    domain_pd_ = ecrt_domain_data(domain_);

    return true;
}

void EtherCATInterface::update() {
    ecrt_master_receive(master);
    ecrt_domain_process(domain_);

    ecrt_domain_queue(domain_);
    ecrt_master_send(master);
}

void EtherCATInterface::writeTargetPosition(int axis, int32_t value) {
    EC_WRITE_S32(domain_pd_ + offsets_[axis].target_position, value);
}

void EtherCATInterface::writeTargetTorque(int axis, int16_t value) {
    EC_WRITE_S16(domain_pd_ + offsets_[axis].target_torque, value);
}

void EtherCATInterface::writeControlWord(int axis, uint16_t value) {
    EC_WRITE_U16(domain_pd_ + offsets_[axis].control_word, value);
}

uint16_t EtherCATInterface::readStatusWord(int axis) const {
    return EC_READ_U16(domain_pd_ + offsets_[axis].status_word);
}

int32_t EtherCATInterface::readActualPosition(int axis) const {
    return EC_READ_S32(domain_pd_ + offsets_[axis].actual_position);
}

int32_t EtherCATInterface::readActualVelocity(int axis) const {
    return EC_READ_S32(domain_pd_ + offsets_[axis].actual_velocity);
}

int16_t EtherCATInterface::readActualTorque(int axis) const {
    return EC_READ_S16(domain_pd_ + offsets_[axis].actual_torque);
}


// bool EtherCATInterface::isDCSyncActive() const {
//     if (!master) return false;

//     ec_master_state_t* state;
//     ecrt_master_state(master, state);
//     return state->al_states != 0;
// }


// int64_t EtherCATInterface::getDCSyncDelay() const {
//     if (!master) return -1;

//     ec_master_clock_t master_clock;
//     if (ecrt_master_get_clock(master, &master_clock)) {
//         std::cerr << "Failed to get master clock" << std::endl;
//         return -1;
//     }

//     int64_t system_time = 0;
//     if (ecrt_master_get_time(master, &system_time)) {
//         std::cerr << "Failed to get DC system time" << std::endl;
//         return -1;
//     }

//     int64_t app_time = ecrt_master_application_time(master);
//     return system_time - app_time;
// }
