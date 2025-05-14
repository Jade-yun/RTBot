#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include <thread>
#include <iostream>
#include <math.h>

extern SharedMemoryManager<SharedMemoryData> shm;

void Robot::init()
{
    // shm = SharedMemoryManager<SharedMemoryData>(SharedMemoryManager<SharedMemoryData>::Attacher, true);
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

void Robot::planMoveJ(const std::array<float, NUM_JOINTS> &_joint_pos, float _arg_vel)
{
    //插补步数，按路径长或速度自适应计算
    double NUM_STEPS = 0;
    double T_num = 0; //插值过程时间参数
    std::array<float, 4> start_pos;
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

    //默认赋值-实际脉冲数值
    for(int i = 0; i < 4; i++) {
        Pulse_TotalCounter[i] = 0;
        Start_PulseCounter[i] = Pulse_TotalCounter[i];
        Axise_LastInterPulse[i] = Pulse_TotalCounter[i];
        start_pos[i] = Pulse_TotalCounter[i] * PI / 500000.0;
    }

    //求最大关节位移的运行时间
    for(int i = 0; i < 4; i++) {
        Joint_T_temp = fabs(_joint_pos[i] - start_pos[i]) / _arg_vel;
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
    // 运动学逆解
//    solveIK
    //
    std::array<float, NUM_JOINTS> pos = {

    };
    moveJ(pos);
}

void Robot::setSpeed(float _speed)
{
    if (_speed < 0) _speed = 0;
//    else if (_speed > 200) _speed = 200;

    m_jointSpeed = _speed * m_jointSpeedRatio;
}

void Robot::updatePose()
{
    //
    //    solveFK(m_curJoints, m_currentPose);
}

void Robot::updateJointStates()
{
    // 从ethercat读取电机位置信息之后, 调用更新
//    m_curJoints =
    RobotState state;
    shm().state_buffer.read(state);

    // 更新当前关节位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
        m_curJoints[i] = state.joint_state[i].position;
    }

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
        exit(0);
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

        printf("joint1:%f, joint2:%f, joint3:%f, joint4:%f, joint5:%f, joint6:%f, speed:%f\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], vel);
        //cmd.movej_params.velocity;
        //cmd.movej_params.acceleration;
        planMoveJ(pos,vel);
        std::cout << "joint: ";
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            std::cout << pos[i] << " ";
        }
        std::cout << "\n";
        break;
    }
    case HighLevelCommandType::MoveL:
    {
        std::array<float, NUM_JOINTS> pose;
        std::copy(std::begin(cmd.movel_params.target_pose),
                  std::end(cmd.movel_params.target_pose),
                  pose.begin());
        moveL(pose);

        // std::cout << "pose: ";
        // for (int i = 0; i < NUM_JOINTS; i++)
        // {
        //     std::cout << pose[i] << " ";
        // }
        // std::cout << "\n";
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
    default:
        break;
    }
}
