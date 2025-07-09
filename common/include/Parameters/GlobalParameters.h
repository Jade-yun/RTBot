#ifndef _GLOBAL_PARAMETERS_H
#define _GLOBAL_PARAMETERS_H

#include <array>
#include <readerwriterqueue.h>
#include "SharedDataType.h"

class GlobalParams
{
public:


    // 缓存插补轨迹
    static moodycamel::ReaderWriterQueue<TrajectoryPoint, MAX_TRAJECTORY_POINTS> traj_buffer;

    // 控制器发送， ethercat 读取发给电机驱动器
    static moodycamel::ReaderWriterQueue<LowLevelCommand, 16> joint_commands;
//    // 关节参数， ethercat 写入
//    // 需要加锁进行读取写入
//    static std::array<JointState, NUM_JOINTS> joint_state;

    static bool isPause;
    static bool isStop;
    static bool isMoving;
    static bool isResume;

};

#endif
