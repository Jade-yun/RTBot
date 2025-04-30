#include "Parameters/GlobalParameters.h"


std::array<std::atomic<GlobalParams::LowLevelState>, NUM_JOINTS> GlobalParams::joint_state = {};

boost::lockfree::spsc_queue<TrajectoryPoint,
    boost::lockfree::capacity<MAX_TRAJECTORY_POINTS>> GlobalParams::traj_buffer;
