#ifndef MONTOR_OFFSET_H
#define MONTOR_OFFSET_H
#include <cstdint>

struct PDOOffsets {
    unsigned int control_word;
    unsigned int target_position;
    unsigned int target_torque;
    unsigned int status_word;
    unsigned int actual_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
};

#endif