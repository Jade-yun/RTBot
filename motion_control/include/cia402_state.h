#pragma once

#include <cstdint>

enum cia402_state_t {
    no_ready_to_switch_on = 0,
    switch_on_disable     = 1,
    ready_to_switch_on    = 2,
    switched_on           = 3,
    operation_enable      = 4,
    quick_stop_active     = 5,
    fault_reaction_active = 6,
    fault                 = 7,
    none                  = 8
};

inline cia402_state_t get_axis_state(uint16_t status_word) {
    if ((status_word & 0x004F) == 0x0000) return no_ready_to_switch_on;
    if ((status_word & 0x004F) == 0x0040) return switch_on_disable;
    if ((status_word & 0x006F) == 0x0021) return ready_to_switch_on;
    if ((status_word & 0x006F) == 0x0023) return switched_on;
    if ((status_word & 0x006F) == 0x0027) return operation_enable;
    if ((status_word & 0x006F) == 0x0007) return quick_stop_active;
    if ((status_word & 0x004F) == 0x000F) return fault_reaction_active;
    if ((status_word & 0x004F) == 0x0008) return fault;
    return none;
}