#ifndef MOTOR_CONTROL_STRATEGY_H
#define MOTOR_CONTROL_STRATEGY_H

#include <cstdint>
#include "cia402_state.h"
#include "Offsets.h"
#include <etherlab/ecrt.h>

enum class MotorControlMode : uint8_t {
    CSP = 8,
    CST = 10,
};

class MotorControlStrategy {
public:
    virtual ~MotorControlStrategy() = default;
    virtual MotorControlMode mode() const = 0;
    virtual void setTarget(int axis, uint8_t *domain_pd, const PDOOffsets& offset) = 0;
    virtual void handleState(int axis, uint8_t *domain_pd, const PDOOffsets& offset, cia402_state_t state) = 0;
};

MotorControlStrategy* getCurrentStrategy();
void setCurrentStrategy(MotorControlStrategy* strategy);

MotorControlStrategy* createCSPStrategy();
MotorControlStrategy* createCSTStrategy();

#endif