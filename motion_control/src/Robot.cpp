#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>
#include <iostream>
#include <math.h>

extern SharedMemoryManager<SharedMemoryData> shm;

Robot::Robot()
{

}

Robot::~Robot()
{
}

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        m_angleLimitMax[i] = 180;
        m_angleLimitMin[i] = -180;
    }
}

void Robot::setEnable(bool _enabled)
{
    // motorJ[ALL]->SetEnable(_enable);
    // 控制器使能
//    ctrl->setEnable(_enabled);
    enabled = _enabled;
}

bool Robot::isEnabled() const
{
    return enabled;
}

bool Robot::isMoving()
{
    // 需要判断电机状态

//    for (int i = 0; i < NUM_JOINTS; i++)
//    {
//        return;
//    }
    return false;
}

void Robot::emergecyStop()
{
    // TO DO
    // context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
    //     context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    // context->MoveJoints(context->targetJoints);
    enabled = false;
    // clearFifo();
}

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    //插补步数，按路径长或速度自适应计算
    double NUM_STEPS = 0;
    double T_num = 0; //插值过程时间参数
    std::array<double, 4> Joint_Tar_pos; //求的插值目标位置
    //多项式系数
    std::array<double, 4> Inter_a0;
    std::array<double, 4> Inter_a1;
    std::array<double, 4> Inter_a2;
    std::array<double, 4> Inter_a3;
    //时间参数
    double Joint_T_temp = 0; double Joint_T_max = 0;
    //各个轴减速比
    std::array<double, 4> RC_ratio;
    //编码器位数
    std::array<unsigned int, 4> Enc_motorbit;

    //插值开始时的脉冲
    std::array<signed int, 4> Start_PulseCounter;
    //当前读取回的脉冲
    std::array<signed int, 4> Pulse_TotalCounter;

    //当前插值点位置脉冲
    std::array<signed int, 4> Axise_CurInterPulse;
    //上一个插值点位置脉冲
    std::array<signed int, 4> Axise_LastInterPulse;
    //脉冲增量
    std::array<signed int, 4> Axise_AddInterPulse;
    std::array<signed int, 4> Axise_TotalInterPulse;

    // 获取当前位置
    //getCurrentJointPosition();
    //start_pos = m_curJoints;

    //默认赋值-减速比-实际需要从参数设置处添加修改
    for(int i = 0; i < 4; i++) {
        RC_ratio[i] = 95.87; //各轴统一减速比
    }
    //默认赋值-编码器位数-实际需要从参数设置处修改
    for(int i = 0; i < 4; i++) {
        Enc_motorbit[i] = 0x1FFFF; //17位编码器
    }

    std::array<float, NUM_JOINTS> start_pos = m_curJoints;
    //默认赋值-实际脉冲数值
//    for(int i = 0; i < 4; i++) {
//        Pulse_TotalCounter[i] = 0;
//        Start_PulseCounter[i] = Pulse_TotalCounter[i];
//        Axise_LastInterPulse[i] = Pulse_TotalCounter[i];
//        start_pos[i] = Pulse_TotalCounter[i] * PI / 500000.0;
//    }
    for(int i = 0; i < 4; i++) {
        Pulse_TotalCounter[i] = start_pos[i] / M_PI * 500000.0;
        Start_PulseCounter[i] = Pulse_TotalCounter[i];
        Axise_LastInterPulse[i] = Pulse_TotalCounter[i];
//        start_pos[i] = Pulse_TotalCounter[i] * PI / 500000.0;
    }

    //求最大关节位移的运行时间
    for(int i = 0; i < 4; i++) {
        Joint_T_temp = fabs(_joint_pos[i] - start_pos[i]) / m_jointSpeed;
        if(Joint_T_temp > Joint_T_max) {
            Joint_T_max = Joint_T_temp;
        }
    }


    NUM_STEPS = Joint_T_max / 0.01;  //EtherCAT插补周期为1ms
    //采用三次多项式插值方式求解系数
    for(int i = 0; i < 4; i++) {
        Inter_a0[i] = start_pos[i];
        Inter_a1[i] = 0;
        Inter_a2[i] = 3.0 / (NUM_STEPS*NUM_STEPS) * (_joint_pos[i] - start_pos[i]);
        Inter_a3[i] = -2.0 / (NUM_STEPS*NUM_STEPS*NUM_STEPS) * (_joint_pos[i] - start_pos[i]);
    }

    printf("a2: %f \n", NUM_STEPS);

    printf("a2: %.15f %.15f %.15f %.15f \n", Inter_a3[0], Inter_a3[1], Inter_a3[2], Inter_a3[3]);

    //计算插值点
    T_num = 1; //从第一个插值点开始

    while(T_num <= NUM_STEPS) {
        //得到每个插值点的关节角度
        for(int i = 0; i < 4; i++) {
            Joint_Tar_pos[i] = Inter_a0[i] + Inter_a1[i]*T_num + Inter_a2[i]*(T_num*T_num) + Inter_a3[i]*(T_num*T_num*T_num);
        }
       // printf("Tar_joint1: %f, Tar_joint2: %f, Tar_joint3: %f, Tar_joint4: %f, Tar_joint5: %f, Tar_joint6: %f\n", Joint_Tar_pos[0], Joint_Tar_pos[1], Joint_Tar_pos[2], Joint_Tar_pos[3], Joint_Tar_pos[4], Joint_Tar_pos[5]);
        
        //判断新位置是否在关节限位之内

        //关节角度转换到电机脉冲
        for(int i = 0; i < 4; i++) {
            Axise_CurInterPulse[i] = Joint_Tar_pos[i] * 500000.0 / PI;

            //判断运行方向
            if(Axise_CurInterPulse[i] >= Axise_LastInterPulse[i]) {
                //正向
                Axise_AddInterPulse[i] = Axise_CurInterPulse[i] - Axise_LastInterPulse[i];
                Axise_TotalInterPulse[i] = Axise_TotalInterPulse[i] + Axise_AddInterPulse[i];
            }
            else {
                //负向
                Axise_AddInterPulse[i] = Axise_CurInterPulse[i] - Axise_LastInterPulse[i];
                Axise_TotalInterPulse[i] = Axise_TotalInterPulse[i] + Axise_AddInterPulse[i];
            }
            Axise_LastInterPulse[i] = Axise_CurInterPulse[i];
        }
    
        //脉冲增量发送
        LowLevelCommand montor_cmd = {
            .mode = CSP,
        };

        for(int i = 0; i < 4; i++) {
            montor_cmd.joint_pos[i] = Axise_AddInterPulse[i];
        }
        //printf("Joint1_pos: %d\n", montor_cmd.joint_pos[0]);

        //发送位置数据
        while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
        {
//            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        //插值增量
        if(NUM_STEPS == T_num) {
            //计算完最后一个后退出
            break;
        }
        T_num++;
        if(T_num > NUM_STEPS) {
            //超出之前没达到时强制达到，以计算准确的目标点
            T_num = NUM_STEPS;
        }
    }

    //插值结束处理

}


void Robot::moveJ(const std::array<float, NUM_JOINTS> &_joint_pos)
{
    //
    // float temp[NUM_JOINTS] = joint_pos;
    LowLevelCommand montor_cmd = {
        .mode = CSP
    };

    std::copy(_joint_pos.begin(), _joint_pos.end(), montor_cmd.joint_pos);

    // 入队列
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        // std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}



void Robot::moveL(std::array<float, NUM_JOINTS> _pose)
{
#if NUM_JOINTS == 6
    Kine6d pose;
    pose.X = _pose[0];
    pose.Y = _pose[1];
    pose.Z = _pose[2];
    pose.A = _pose[3];
    pose.B = _pose[4];
    pose.C = _pose[5];
    pose.fgR = 0;

    Kine6dSol q_sol;

//    float q_last[6];

//    classic6dofInvKine(&pose, q_last, &q_sol);
    classic6dofInvKine(&pose, m_curJoints.data(), &q_sol);

        bool valid[8];
        int validCnt = 0;

        for (int i = 0; i < 8; i++)
        {
            valid[i] = true;

            for (int j = 1; j <= 6; j++)
            {
                if (q_sol.sol[i][j-1] > m_angleLimitMax[j-1] ||
                    q_sol.sol[i][j-1] < m_angleLimitMin[j-1])
                {
                    valid[i] = false;
                    continue;
                }
            }
            if (valid[i]) validCnt++;
        }

        if (validCnt > 0) {
            // 选择距离当前关节位置最近的解
            float minDist = std::numeric_limits<float>::max();
            int bestIndex = -1;

            for (int i = 0; i < 8; i++) {
                if (!valid[i]) continue;

                float dist = 0.0f;
                for (int j = 0; j < NUM_JOINTS; j++)
                {
                    float d = m_curJoints[j] - q_sol.sol[i][j];
                    dist += d * d;
                }

                if (dist < minDist) {
                    minDist = dist;
                    bestIndex = i;
                }
            }

            if (bestIndex >= 0) {
                std::array<float, NUM_JOINTS> targetJoints;
                for (int j = 0; j < NUM_JOINTS; j++)
                {
                    targetJoints[j] = q_sol.sol[bestIndex][j];
                }

                // 加入插值、运动规划等

                for (int j = 0; j < NUM_JOINTS; j++)
                {
                    targetJoints[j] = q_sol.sol[bestIndex][j];
                }
                fprintf(stderr, "MoveL joint pos: %f %f %f %f %f %f\n",
                       targetJoints[0], targetJoints[1], targetJoints[2],
                       targetJoints[3], targetJoints[4], targetJoints[5]);

                planMoveJ(targetJoints);
            }
        }

#elif NUM_JOINTS == 4

#endif

}

void Robot::setSpeed(float _speed)
{
    if (_speed < 10) _speed = 10;
    else if (_speed > 200) _speed = 200;

    m_jointSpeed = _speed * m_SpeedRatio;
}

void Robot::updatePose()
{
    //
    //    solveFK(m_curJoints, m_currentPose);
}

void Robot::updateJointStates()
{
//    m_curJoints =
    RobotState state;
    shm().state_buffer.read(state);

    // 更新当前关节位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
        m_curJoints[i] = state.joint_state[i].position;
        m_curVelocity[i] = state.joint_state[i].velocity;
        m_curTorque[i] = state.joint_state[i].torque;
    }

    std::cout << "joints pos: ";
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        std::cout << " " << m_curJoints[i];
    }
    std::cout << std::endl;

}

void Robot::controlLoop()
{
    HighLevelCommand cmd;
    if (!shm().cmd_queue.pop(cmd))
        return;

    switch (cmd.command_type)
    {
    case HighLevelCommandType::Homing:
        /* code */
        break;
    case HighLevelCommandType::Stop:
    {
        std::cout << "stop!\n";
//        exit(0);
        break;
    }
    case HighLevelCommandType::MoveJ:
    {
        std::array<float, NUM_JOINTS> pos;
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            pos[i] = cmd.movej_params.target_joint_pos[i];
        }
        float vel = cmd.movej_params.velocity;

//        printf("joint1:%f, joint2:%f, joint3:%f, joint4:%f, joint5:%f, joint6:%f, speed:%f\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], vel);
        //cmd.movej_params.velocity;
        //cmd.movej_params.acceleration;
        setSpeed(vel);

        updateJointStates();
        planMoveJ(pos);

//        for (int i = 0; i < 5; i++)
//        {
//            updateJointStates();
//            std::this_thread::sleep_for(std::chrono::milliseconds(20));
//        }

        break;
    }
    case HighLevelCommandType::MoveL:
    {
        std::array<float, NUM_JOINTS> pose;
        std::copy(std::begin(cmd.movel_params.target_pose),
                  std::end(cmd.movel_params.target_pose),
                  pose.begin());
        float vel = cmd.movel_params.velocity;
        setSpeed(vel);
        updateJointStates();
        moveL(pose);
        break;
    }
    case HighLevelCommandType::MoveP:
    {
        int traj_index = cmd.movep_params.traj_index;
        const auto &traj = shm().trajectory_pool[traj_index];

        for (uint32_t i = 0; i < traj.point_count; i++)
        {
            std::array<float, NUM_JOINTS> pos;
            std::memcpy(pos.data(), traj.points[i].joint_pos, sizeof(float) * NUM_JOINTS);
            moveJ(pos);
        }

    }
    case HighLevelCommandType::SetParm:
    {
        handleParameterOrder(cmd);
        break;
    }
    default:
        break;
    }
}

void Robot::handleParameterOrder(HighLevelCommand &_cmd)
{
    const auto& packet = _cmd.setparms;

    switch (packet.mainType) {
    case 0x01:
        switch (packet.subType) {
        case 0x01:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_angleLimitMax[i] = packet.values[i];
            }
            break;
        case 0x02:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_angleLimitMin[i] = packet.values[i];
            }
            break;
        case 0x03:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointPosMax[i] = packet.values[i];
            }
            break;
        case 0x04:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointPosMin[i] = packet.values[i];
            }
            break;
        case 0x05:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointSpeedMax[i] = packet.values[i];
            }
            break;
        case 0x06:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointSpeedMax[i] = packet.values[i];
            }
            break;
        case 0x07:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointTorqueMax[i] = packet.values[i];
            }
            break;
        case 0x08:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_ReducedJointTorqueMax[i] = packet.values[i];
            }
            break;
        }
    case 0x10:
        switch (packet.subType) {
        // 最大轴速度
        case 0x01:
            for (int i = 0; i < packet.valueCount; ++i)
            {
//                m_AxisAccMax[i] = packet.values[i];
            }
            break;
        case 0x02:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_AxisAccMax[i] = packet.values[i];
            }
            break;
        case 0x03:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_AxisAccAccMax[i] = packet.values[i];
            }
            break;
        case 0x04:
            for (int i = 0; i < packet.valueCount; ++i)
            {
                m_JointCompliance[i] = packet.values[i];
            }
            break;
        case 0x05:
            if (packet.valueCount == 5)
            {
                m_AccRatio = packet.values[0];
                m_AccRampUpTime = packet.values[1];
                m_DecRampUpTime = packet.values[2];
                m_SpeedSmoothingFactor = packet.values[3];
                m_JogAccRampUpTime = packet.values[4];
            }
            break;
        case 0x07:
            m_LookAheadFactor = packet.values[0];
            break;

            // ...
        }
        break;
    }
}
