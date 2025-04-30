#include "Parameters/GlobalParameters.h"


std::array<JointState, NUM_JOINTS> GlobalParams::joint_state = {};

moodycamel::ReaderWriterQueue<TrajectoryPoint, MAX_TRAJECTORY_POINTS> GlobalParams::traj_buffer;

moodycamel::ReaderWriterQueue<LowLevelCommand, 128> GlobalParams::joint_commands;