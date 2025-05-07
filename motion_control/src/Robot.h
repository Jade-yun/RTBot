#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <array>
#include "Parameters/SharedDataType.h"
#include "Utilities/SharedMemoryManager.h"

class Robot
{
public:
    void init();

    void setEnable(bool _enabled);
    bool isEnabled() const;
    void emergecyStop();

    // 
    void planMoveJ(const std::array<float, NUM_JOINTS>& _joint_pos);
    void moveJ(const std::array<float, NUM_JOINTS>& _joint_pos);
    void moveL(std::array<float, NUM_JOINTS> _pose);

    // void handleHMICommand(const HighLevelCommand& _cmd);

    // void sendCommadToEcat(const RobotCommand& cmd);
    // void recvMontorState(const JointState& joint_state);

    void controlLoop();

private:
    bool enabled = false;

};


#endif
