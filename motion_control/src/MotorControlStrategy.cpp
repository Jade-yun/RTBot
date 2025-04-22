#include "MotorControlStrategy.hpp"
#include <iostream>

namespace {

class CSPStrategy : public MotorControlStrategy {
public:
    MotorControlMode mode() const override {
        return MotorControlMode::CSP;
    }

    void setTarget(int axis, uint8_t *domain_pd, const PDOOffsets& offset) override {
        int32_t pos = EC_READ_S32(domain_pd + offset.actual_position);
        EC_WRITE_S32(domain_pd + offset.target_position, pos + 0x3FFF);
    }

    void handleState(int axis, uint8_t *domain_pd, const PDOOffsets& offset, cia402_state_t state) override {
        switch (state) {
            case no_ready_to_switch_on:  EC_WRITE_U16(domain_pd + offset.control_word, 0x80); break;
            case switch_on_disable:      EC_WRITE_U16(domain_pd + offset.control_word, 0x06); break;
            case ready_to_switch_on:     EC_WRITE_U16(domain_pd + offset.control_word, 0x07); break;
            case switched_on:            EC_WRITE_U16(domain_pd + offset.control_word, 0x0F); break;
            case operation_enable:       setTarget(axis, domain_pd, offset); break;
            case fault:                  EC_WRITE_U16(domain_pd + offset.control_word, 0x80); break;
            default: break;
        }
    }
};

class CSTStrategy : public MotorControlStrategy {
public:
    MotorControlMode mode() const override {
        return MotorControlMode::CST;
    }

    void setTarget(int axis, uint8_t *domain_pd, const PDOOffsets& offset) override {
        int16_t torque = 100;
        EC_WRITE_S16(domain_pd + offset.target_torque, torque);
    }

    void handleState(int axis, uint8_t *domain_pd, const PDOOffsets& offset, cia402_state_t state) override {
        switch (state) {
            case no_ready_to_switch_on:  EC_WRITE_U16(domain_pd + offset.control_word, 0x80); break;
            case switch_on_disable:      EC_WRITE_U16(domain_pd + offset.control_word, 0x06); break;
            case ready_to_switch_on:     EC_WRITE_U16(domain_pd + offset.control_word, 0x07); break;
            case switched_on:            EC_WRITE_U16(domain_pd + offset.control_word, 0x0F); break;
            case operation_enable:       setTarget(axis, domain_pd, offset); break;
            case fault:                  EC_WRITE_U16(domain_pd + offset.control_word, 0x80); break;
            default: break;
        }
    }
};

CSPStrategy csp_strategy;
CSTStrategy cst_strategy;
MotorControlStrategy* g_current_strategy = nullptr;

} // namespace

MotorControlStrategy* getCurrentStrategy() {
    return g_current_strategy;
}

void setCurrentStrategy(MotorControlStrategy* strategy) {
    g_current_strategy = strategy;
}

MotorControlStrategy* createCSPStrategy() {
    return &csp_strategy;
}

MotorControlStrategy* createCSTStrategy() {
    return &cst_strategy;
}