#include "Parameters/GlobalParameters.h"


//std::array<JointState, NUM_JOINTS> GlobalParams::joint_state = {};

moodycamel::ReaderWriterQueue<TrajectoryPoint, MAX_TRAJECTORY_POINTS> GlobalParams::traj_buffer;

moodycamel::ReaderWriterQueue<LowLevelCommand, 16> GlobalParams::joint_commands;

bool GlobalParams::isPause = false;
bool GlobalParams::isResume = false;
bool GlobalParams::isStop = false;
bool GlobalParams::isMoving = false;
std::atomic<int> GlobalParams::print = {0};
