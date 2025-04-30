#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <array>
#include "Parameters/SharedDataType.h"
#include "Utilities/SharedMemoryManager.h"

class Robot
{

    void init();

    void planMoveJ(const std::array<float, NUM_JOINTS>& target_positions);

    void handleHMICommand(const HighLevelCommand& cmd);

    // void sendCommadToEcat(const RobotCommand& cmd);
    // void recvMontorState(const JointState& joint_state);

    void controlLoop();

private:
    SharedMemoryManager<SharedMemoryData> shm;

};


#endif
