#include "Robot.h"
#include "Parameters/GlobalParameters.h"
#include "ethercat/EtherCATInterface.h"
#include <thread>
#include <iostream>
#include <math.h>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

//#define VISUALIZE_INTERP_RES

#ifdef VISUALIZE_INTERP_RES
#include <fstream>
#endif
#include <algorithm>

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
        m_angleLimitMax[i] = M_PI;   // π 弧度 (180度)
        m_angleLimitMin[i] = -M_PI;  // -π 弧度 (-180度)
    }
    
    std::cout << "机器人初始化完成，关节限位设置为 ±180度 (±π弧度)" << std::endl;
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


void Robot::emergecyStop()
{
    // TO DO
    // context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
    //     context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    // context->MoveJoints(context->targetJoints);
    enabled = false;
    // clearFifo();
}

/* 关节空间点到点插补 */
void Robot::moveJ(const std::array<float, NUM_JOINTS> &_joint_pos, float _speed)
{
    // 重置状态标志，开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
    char Speed_planning_step = 0; //0:未规划 1：预测减速阶段 2：加速处理阶段 3：匀速预测减速阶段 4：三段减速阶段
    double current_cycle_tim = 0.001; //当前插补时间
    double temp_accumu_tim = 0;   //预测减速段的累积时间
    std::array<float, NUM_JOINTS> Joint_Tar_pos; //求的插值目标位置
    //各个轴减速比
    std::array<float, NUM_JOINTS> RC_ratio;
    //编码器位数
    std::array<unsigned int, NUM_JOINTS> Enc_motorbit;
    //插值开始时的脉冲
    std::array<signed int, NUM_JOINTS> Start_PulseCounter;
    //当前插值点位置脉冲
    std::array<signed int, NUM_JOINTS> Axise_CurInterPulse;
    //经过原点偏移后的脉冲
    std::array<signed int, NUM_JOINTS> offsetPosition;
    //插补比例系数
    double interpol_propor = 0.0;
    //速度规划变量
    double current_joint_v = 0.0;
    double current_joint_a = 0.0;
    double current_joint_j = 0.0;
    //预测增加
    double tempnext_joint_v = 0.0;
    double tempnext_joint_a = 0.0;
    double tempnext_joint_j = 0.0;
    //距离变量
    double current_joint_l = 0.0;       //当前周期需要走的距离
    double sum_joint_l = 0.0;           //需要移动的总距离
    double current_joint_suml = 0.0;    //当前已经走过的距离
    //预测增加的距离变量
    double tempnext_joint_l = 0.0;      //预增加已走的距离
    double tempnext_accdec_l = 0.0;     //减加速段走的距离
    double tempnext_accdec_v = 0.0;     //减加速段末速度
    double tempnext_dec_l = 0.0;        //减速段距离
    double acc_dec_num = 0;             //减加速过程的插补点数-计算得到
    double dec_count_num = 0;           //减速过程当前插补点数
    uint32_t dec_count_start = 0;       //阶段性开始插补参数
    uint32_t last_dec_count_num = 0;    //减加速过程上一个插补点数
    uint32_t acc_dec_currentn = 0;      //减加速阶段插补次数
    double acc_dec_start_a = 0.0;       //减加速阶段初始参数
    double acc_dec_start_v = 0.0;
    double acc_dec_start_l = 0.0;
    //三段减速段相关参数
    uint8_t Deceleration_section = 0.0; //减速段的实际段数
    double decelera_t5 = 0.0;           //第一减速段时间
    double decelera_t6 = 0.0;           //第二减速段时间
    double decelera_t7 = 0.0;           //第三减速段时间
    double decelera_amin = 0.0;         //由减速初末速度限制的减速最小减速度
    double dec_start_a = 0.0;           //减速阶段初始参数
    double dec_start_v = 0.0;
    double dec_start_l = 0.0;
    //插补余量控制
    double residual_amount = 0.0;       //各阶段切换时的插补余量
    //运动参数限制
    double limit_joint_amax = 100;       //关节空间运动时的最大加速度-最大力矩输出另外限制
    double limit_joint_jmax = 400;       //关节空间运动时的最大加加速度
    double limit_joint_vmax = 20;      //关节空间指令设定的最大速度
    //结束标志
    bool interpol_finish = 0.0;         //插补结束


    //默认赋值-减速比-实际需要从参数设置处添加修改
    for(int i = 0; i < NUM_JOINTS; i++) {
        RC_ratio[i] = 95.87; //各轴统一减速比
    }
    //默认赋值-编码器位数-实际需要从参数设置处修改
    for(int i = 0; i < NUM_JOINTS; i++) {
        Enc_motorbit[i] = 0x1FFFF; //17位编码器
    }

    std::array<float, NUM_JOINTS> start_pos = m_curJoints;
    std::array<float, NUM_JOINTS> end_pos = _joint_pos; 


    // //当前脉冲转换到弧度
    // for(int i = 0; i < NUM_JOINTS; i++) 
    // {
    //     Start_PulseCounter[i] = m_curtotalpulse[i] - MINROBOTPOSITION;

    //     //脉冲偏移调整
    //     offsetPosition[i] = Start_PulseCounter[i] - Joint_Zero_Offset[i];
    //     start_pos[i] = offsetPosition[i] * M_PI / 500000.0;
    // }

    //求最大关节位移弧度
    for(int i = 0; i < NUM_JOINTS; i++)  
    {
        if(sum_joint_l < fabs(_joint_pos[i] - start_pos[i])) 
        {
            sum_joint_l = fabs(_joint_pos[i] - start_pos[i]);
        }
    }

    sum_joint_l = sum_joint_l*180.0 / M_PI; //统一单位-以角度进行插补
    if (sum_joint_l < 0.001) 
    {
        std::cout << "目标关节位置与当前关节位置相同，跳过插补！\n";
        return; // 如果目标位置和当前关节位置相同，直接返回
    }
    
    // double joint_max_speed = (3000 / 60.0) / m_GearRatio; 

    limit_joint_vmax = _speed * m_SpeedRatio; //实际再多乘个比例系数
    Speed_planning_step = 1;   //开始插补

    while(interpol_finish != true) 
    {
        // 检查停止和暂停状态
        if (GlobalParams::isStop) {
            GlobalParams::isStop = false; // 重置停止标志
            // 平滑减速到停止
            // if (current_joint_v > 0.01) 
            while(current_joint_v > 0.01)
            {  // 如果还有速度
                current_joint_j = -limit_joint_jmax;  // 使用最大减速度
                current_joint_a = current_joint_a + current_joint_j * current_cycle_tim;
                current_joint_v = current_joint_v + current_joint_a * current_cycle_tim;
                if (current_joint_v < 0) current_joint_v = 0;  // 防止负速度
                
                // 继续插补到当前减速位置
                current_joint_l = current_joint_v * current_cycle_tim;
                current_joint_suml += current_joint_l;
                
                interpol_propor = current_joint_suml / sum_joint_l;
                for(int i = 0; i < NUM_JOINTS; i++) 
                {
                    Joint_Tar_pos[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * interpol_propor;
                }
                m_curJoints = Joint_Tar_pos;
                moveJoints(m_curJoints);
                
                continue;
            }
            // GlobalParams::isStop = false;
            break;
        }
        
        // 暂停处理
        if (GlobalParams::isPause) {
            std::cout << "运动暂停中...\n";
            
            // 平滑减速到停止
            if (current_joint_v > 0.01) {  // 如果还有速度
                current_joint_j = -(limit_joint_jmax * 0.5);  // 使用最大减速度
                current_joint_a = current_joint_a + current_joint_j * current_cycle_tim;
                current_joint_v = current_joint_v + current_joint_a * current_cycle_tim;
                if (current_joint_v < 0) current_joint_v = 0;  // 防止负速度
                
                // 继续插补到当前减速位置
                current_joint_l = current_joint_v * current_cycle_tim;
                current_joint_suml += current_joint_l;
                
                interpol_propor = current_joint_suml / sum_joint_l;
                for(int i = 0; i < NUM_JOINTS; i++) 
                {
                    Joint_Tar_pos[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * interpol_propor;
                }
                m_curJoints = Joint_Tar_pos;
                moveJoints(m_curJoints);

                continue;
            }
            
            // 速度已经停止，等待继续指令
            std::cout << "关节运动已暂停，等待继续指令...\n";
            while (GlobalParams::isPause) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                
                HighLevelCommand cmd;
                if (shm().high_prio_cmd_queue.pop(cmd))
                {
                    handleHighPriorityCommand(cmd);
                }
                
                // 检测停止命令，如果收到停止命令则退出暂停状态
                if (GlobalParams::isStop) {
                    std::cout << "收到停止命令，退出暂停状态\n";
                    break;
                }
            }
            
            // 如果收到停止命令，退出函数
            if (GlobalParams::isStop) {
                std::cout << "收到停止命令，退出关节运动\n";
                GlobalParams::isStop = false;
                break;
            }
            
            // 如果是继续指令，重新计算剩余距离
            if (GlobalParams::isResume) {
                GlobalParams::isResume = false;
                std::cout << "恢复关节运动，重新规划轨迹\n";
                
                // 重新计算剩余距离
                double remaining_distance = 0;
                for(int i = 0; i < NUM_JOINTS; i++)  
                {
                    if(remaining_distance < fabs(end_pos[i] - m_curJoints[i])) 
                    {
                        remaining_distance = fabs(end_pos[i] - m_curJoints[i]);
                    }
                }
                
                // 更新起始位置和总距离
                start_pos = m_curJoints;
                sum_joint_l = remaining_distance * 180.0 / M_PI;
                current_joint_suml = 0;
                
                // 重置速度规划参数
                current_joint_v = 0;
                current_joint_a = 0;
                current_joint_j = 0;
                Speed_planning_step = 1;
                
                std::cout << "剩余距离: " << remaining_distance << " 弧度\n";
            }
        }

        //加减速处理
        if(Speed_planning_step == 1) {
        //预测减速阶段
            //Step1-以J进行加速
            tempnext_joint_j = limit_joint_jmax;
            tempnext_joint_a = tempnext_joint_j*current_cycle_tim + current_joint_a; //以J对a进行增加
            tempnext_joint_v = current_joint_v + current_joint_a*current_cycle_tim + tempnext_joint_j*current_cycle_tim*current_cycle_tim / 2;
            tempnext_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 + \
                            tempnext_joint_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            //先判断速度是否超限位
            if(tempnext_joint_v > limit_joint_vmax) {
                tempnext_joint_v = limit_joint_vmax;
                tempnext_joint_j = 2*(tempnext_joint_v - current_joint_v - current_joint_a*current_cycle_tim) / current_cycle_tim*current_cycle_tim;
                tempnext_joint_a = current_joint_a + tempnext_joint_j*current_cycle_tim;
                tempnext_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 \
                                + tempnext_joint_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }
            //再判断加速度是否超限位
            if(tempnext_joint_a > limit_joint_amax) {
                //预测阶段超过最大加速度
                tempnext_joint_a = limit_joint_amax;
                tempnext_joint_j = (tempnext_joint_a - current_joint_a) / current_cycle_tim;
                tempnext_joint_v = current_joint_v + current_joint_a * current_cycle_tim + tempnext_joint_j*current_cycle_tim*current_cycle_tim / 2;
                tempnext_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 \
                                + tempnext_joint_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }

            //Step2-以-J进行减速-得到减加速的距离-同时获得减加速后的速度
            tempnext_accdec_v = tempnext_joint_v + tempnext_joint_a*tempnext_joint_a / (2*limit_joint_jmax);
            tempnext_accdec_l = tempnext_joint_v * tempnext_joint_a / limit_joint_jmax + tempnext_joint_a*tempnext_joint_a*tempnext_joint_a / (3*limit_joint_jmax*limit_joint_jmax);

            //Step3-以三阶段减速方式-计算减速距离
            if(tempnext_accdec_v >= limit_joint_vmax) {
                //减加速后达到的速度超过设定速度-即刻进加速处理阶段
                Speed_planning_step = 2; 
                acc_dec_currentn = 1;         //进入时初始化插补次数
                acc_dec_num = abs(current_joint_a / (limit_joint_jmax*current_cycle_tim)); //加速阶段插补点数
                //减加速阶段初始参数
                acc_dec_start_a = current_joint_a;
                acc_dec_start_v = current_joint_v;
                acc_dec_start_l = current_joint_l;
            }else {
                //计算三段各自的时间-最小a 最大a-距离、速度-距离后面判断
                if(tempnext_accdec_v > limit_joint_amax*limit_joint_amax / limit_joint_jmax) {
                    //存在三段减速段
                    tempnext_dec_l = tempnext_accdec_v * ( limit_joint_amax/(2*limit_joint_jmax) + tempnext_accdec_v/(2*limit_joint_amax) );
                    //得到三段减速数据
                    decelera_t5 = limit_joint_amax / (limit_joint_jmax*current_cycle_tim);
                    decelera_t6 = (tempnext_accdec_v / (limit_joint_amax*current_cycle_tim)) - decelera_t5;
                    decelera_t7 = decelera_t5;
                    decelera_amin = -limit_joint_amax;
                    Deceleration_section = 3;
                }else {
                    //只有两段减速段
                    tempnext_dec_l = (tempnext_accdec_v) * sqrt(tempnext_accdec_v / limit_joint_jmax);
                    //得到两段减速数据
                    decelera_t5 = sqrt(tempnext_accdec_v / (limit_joint_jmax))/current_cycle_tim;
                    decelera_t6 = 0;
                    decelera_t7 = decelera_t5;
                    decelera_amin = limit_joint_jmax * decelera_t5;
                    Deceleration_section = 2;
                }
                //判断三阶段减速距离 tempnext_joint_v~0
                if( (sum_joint_l-current_joint_suml) < tempnext_dec_l+tempnext_accdec_l+tempnext_joint_l) {
                    //距离超过所给距离-进入加速处理阶段
                    Speed_planning_step = 2;
                    acc_dec_currentn = 1;         //进入时初始化插补次数
                    acc_dec_num = abs(current_joint_a / (limit_joint_jmax*current_cycle_tim)); //加速阶段插补点数
                    acc_dec_start_a = current_joint_a;
                    acc_dec_start_v = current_joint_v;
                    acc_dec_start_l = current_joint_l;
                }else {
                    //距离还够用-继续预测减速阶段
                    Speed_planning_step = 1;
                    current_joint_a = tempnext_joint_a; //用于下次规划判断
                    current_joint_v = tempnext_joint_v;
                    current_joint_j = tempnext_joint_j;
                    current_joint_l = tempnext_joint_l; //用于长轴增量插补
                    current_joint_suml += current_joint_l;
                    //剩余距离-可写可不写了
                }
            }
        }
        if(Speed_planning_step == 2) {
            //减加速处理阶段
            current_joint_j = -limit_joint_jmax;
            if(acc_dec_currentn <= acc_dec_num) {

                current_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 \
                                    + current_joint_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_joint_a = acc_dec_start_a + current_joint_j*acc_dec_currentn*current_cycle_tim;

                current_joint_v = acc_dec_start_v + acc_dec_start_a*acc_dec_currentn*current_cycle_tim + current_joint_j \
                                * acc_dec_currentn*acc_dec_currentn*current_cycle_tim*current_cycle_tim / 2;


                current_joint_suml += current_joint_l;
                acc_dec_currentn++; //减加速阶段插补次数自增

            }

            if(acc_dec_currentn > acc_dec_num)
            {
                //到达插补点数-进入匀速预测减速阶段
                Speed_planning_step = 3;
            }
        }
        if(Speed_planning_step == 3) {
            //匀速预测减速阶段
                //Step1-匀速计算一个周期
                tempnext_joint_a = 0;
                tempnext_joint_j = 0;
                tempnext_joint_v = current_joint_v;
                tempnext_joint_l = tempnext_joint_v * current_cycle_tim; //10ms周期走一个


                if(Deceleration_section == 3) {
                    //计算三段减速距离
                    tempnext_dec_l = (current_joint_v) * (limit_joint_amax / (2*limit_joint_jmax) + (current_joint_v / 2*limit_joint_amax));
                    decelera_t5 = limit_joint_amax / (limit_joint_jmax*current_cycle_tim);
                    decelera_t6 = (tempnext_joint_v / (limit_joint_amax*current_cycle_tim) ) - decelera_t5;
                    decelera_t7 = decelera_t5;
                    decelera_amin = -limit_joint_amax;
                }else if(Deceleration_section == 2) {
                    //只有两段减速距离
                    tempnext_dec_l = (current_joint_v)*sqrt(current_joint_v / limit_joint_jmax);
                    decelera_t5 = sqrt(tempnext_joint_v / (limit_joint_jmax))/current_cycle_tim;
                    if(decelera_t5 - (int)decelera_t5 >= 0.5) {
                        decelera_t5 = decelera_t5+1;
                    }
                    decelera_t6 = 0;
                    decelera_t7 = decelera_t5;
                    decelera_amin = limit_joint_jmax * decelera_t5;
                }
                
                //判断是否超过剩余距离
                if(tempnext_dec_l+tempnext_joint_l > (sum_joint_l - current_joint_suml)) {
                    //超过剩余距离，不可匀速
                    Speed_planning_step = 4;
                    //进减速阶段初始状态
                    current_joint_l = (sum_joint_l - current_joint_suml) - tempnext_dec_l;
                    current_joint_suml += current_joint_l;

                    dec_count_num = 1;
                    
                    dec_start_a = current_joint_a;
                    dec_start_v = current_joint_v;
                    dec_start_l = current_joint_l;
                }else {
                    current_joint_j = tempnext_joint_j;
                    current_joint_a = tempnext_joint_a;
                    current_joint_v = tempnext_joint_v;
                    current_joint_l = tempnext_joint_l;
                    current_joint_suml += current_joint_l;
                    //重新赋值减速量-以上面算的为准
                }
        }
        if(Speed_planning_step == 4) {
            //三段减速处理阶段
            last_dec_count_num = dec_count_num;

            if(dec_count_num <= decelera_t5) {
                current_joint_j = -limit_joint_jmax;

                    current_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 \
                                        + current_joint_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_joint_a = dec_start_a + current_joint_j*dec_count_num*current_cycle_tim;

                current_joint_v = dec_start_v + dec_start_a*dec_count_num*current_cycle_tim + current_joint_j * dec_count_num*dec_count_num*current_cycle_tim*current_cycle_tim / 2;


                current_joint_suml += current_joint_l;

            }else if(dec_count_num > decelera_t5 && dec_count_num <= decelera_t6+decelera_t5) {
                current_joint_j = 0;
                current_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2;
                current_joint_a = dec_start_a;
                current_joint_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim;

                current_joint_suml += current_joint_l;

            }else if(dec_count_num > decelera_t6+decelera_t5 && dec_count_num <= decelera_t6+decelera_t5+decelera_t7) {
                
                    current_joint_j = limit_joint_jmax;

                    current_joint_l = current_joint_v*current_cycle_tim + current_joint_a*current_cycle_tim*current_cycle_tim / 2 \
                                        + current_joint_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                    current_joint_a = dec_start_a + current_joint_j*(dec_count_num-dec_count_start)*current_cycle_tim;

                    current_joint_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim + current_joint_j * (dec_count_num-dec_count_start)*(dec_count_num-dec_count_start)*current_cycle_tim*current_cycle_tim / 2;


                current_joint_suml += current_joint_l;

            }

            dec_count_num++;//不考虑插补余量-预计会有到达误差

            if(dec_count_num >= decelera_t5+decelera_t6+decelera_t7) {
                //插补计算结束
                interpol_finish = 1;
            }

            //阶段切换时初始位置保存
            if(last_dec_count_num <= decelera_t5 && dec_count_num > decelera_t5) {

                    dec_start_a = current_joint_a;
                    dec_start_v = current_joint_v;
                    dec_count_start = decelera_t5;

            }else if(last_dec_count_num <= decelera_t5+decelera_t6 && dec_count_num > decelera_t5+decelera_t6) {

                    dec_start_a = current_joint_a;
                    dec_start_v = current_joint_v;
                    dec_count_start = decelera_t5+decelera_t6 ;

            }
        }
        
        interpol_propor = current_joint_suml / sum_joint_l;
        //计算各轴目标位置
        for(int i = 0; i < NUM_JOINTS; i++) 
        {
            //interpol_propor[i] = current_joint_suml / sum_joint_l;
            Joint_Tar_pos[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * interpol_propor;
            //判断位置是否在限制范围内

            // montor_cmd.joint_pos[i] = Joint_Tar_pos[i] * 500000.0 / M_PI * 13.1072;   //关节角度转换到电机脉冲
            //Axise_CurInterPulse[i] = Joint_Tar_pos[i] * 500000.0 / M_PI ;   //关节角度转换到电机脉冲

            //绝对位置转换为对应编码器位数
            //montor_cmd.joint_pos[i] = Axise_CurInterPulse[i] * 13.1072;
        }
        //测试
        // printf("duanshu: %d\n", Speed_planning_step);
        // printf("cur_l: %f\n", current_joint_suml);
        // printf("target_joint: %f %f %f %f %f %f \n", Joint_Tar_pos[0], Joint_Tar_pos[1], Joint_Tar_pos[2], Joint_Tar_pos[3], Joint_Tar_pos[4], Joint_Tar_pos[5]);
        //更新当前关节位置
        m_curJoints = Joint_Tar_pos;

        moveJoints(m_curJoints);
    }
    printf("当前角度：%f %f %f %f %f %f \n", m_curJoints[0], m_curJoints[1], m_curJoints[2], m_curJoints[3], m_curJoints[4], m_curJoints[5] );
    Kine6d cur_pose;
    classic6dofForKine(m_curJoints.data(), &cur_pose);
    printf("当前位姿：%f %f %f %f %f %f \n", cur_pose.X, cur_pose.Y, cur_pose.Z, cur_pose.A, cur_pose.B, cur_pose.C);
}

/* 笛卡尔空间直线插补 */
void Robot::moveL(std::array<float, NUM_JOINTS> _pose, float _speed)
{
    // 重置状态标志，开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
   /*位置和姿态插补相关变量*/
    Kine6d start_pose;                  //开始位姿
    Kine6d end_pose;                    //结束位姿
    double lamda = 0.0;                 //插补比例系数
    float interp_X, interp_Y, interp_Z; //位置插值
    Eigen::Quaternionf q_interp;
    Eigen::Matrix3f rotm;
    Eigen::Vector3f euler_angles;       //ZYX顺序欧拉角
    float interp_C, interp_B, interp_A; //姿态插值
    Kine6d interp_pose;
    Kine6dSol q_sol;                    //逆解得到的关节角度    

    /*默认赋值和脉冲相关变量*/
    std::array<double, NUM_JOINTS> RC_ratio;                 //各个轴减速比
    std::array<unsigned int, NUM_JOINTS> Enc_motorbit;       //编码器位数
    std::array<signed int, NUM_JOINTS> Start_PulseCounter;   //插值开始时的脉冲
    std::array<signed int, NUM_JOINTS> Axise_CurInterPulse;  //当前插值点位置脉冲
    std::array<signed int, NUM_JOINTS> offsetPosition;       //经过原点偏移后的脉冲

    /*速度规划相关变量*/
    char Speed_planning_step = 0;       //0:未规划 1：预测减速阶段 2：加速处理阶段 3：匀速预测减速阶段 4：三段减速阶段
    double current_cycle_tim = 0.001;   //当前插补时间
    double temp_accumu_tim = 0;         //预测减速段的累积时间
    double current_cartesian_v = 0.0;   //当前笛卡尔空间速度
    double current_cartesian_a = 0.0;   //当前笛卡尔空间加速度
    double current_cartesian_j = 0.0;   //当前笛卡尔空间加加速度
    double tempnext_cartesian_v = 0.0;  //预测下一步笛卡尔空间速度
    double tempnext_cartesian_a = 0.0;  //预测下一步笛卡尔空间加速度
    double tempnext_cartesian_j = 0.0;  //预测下一步笛卡尔空间加加速度
    double current_cartesian_l = 0.0;   //当前周期需要走的距离
    double sum_cartesian_l = 0.0;       //需要移动的总距离
    double current_cartesian_suml = 0.0;//当前已经走过的距离
    double tempnext_cartesian_l = 0.0;  //预增加已走的距离
    double tempnext_accdec_l = 0.0;     //减加速段走的距离
    double tempnext_accdec_v = 0.0;     //减加速段末速度
    double tempnext_dec_l = 0.0;        //减速段距离
    double acc_dec_num = 0;             //减加速过程的插补点数-计算得到
    double dec_count_num = 0;           //减速过程当前插补点数
    uint32_t dec_count_start = 0;       //阶段性开始插补参数
    uint32_t last_dec_count_num = 0;    //减加速过程上一个插补点数
    uint32_t acc_dec_currentn = 0;      //减加速阶段插补次数
    double acc_dec_start_a = 0.0;       //减加速阶段初始加速度
    double acc_dec_start_v = 0.0;       //减加速阶段初始速度
    double acc_dec_start_l = 0.0;       //减加速阶段初始位移
    uint8_t Deceleration_section = 0.0; //减速段的实际段数
    double decelera_t5 = 0.0;           //第一减速段时间
    double decelera_t6 = 0.0;           //第二减速段时间
    double decelera_t7 = 0.0;           //第三减速段时间
    double decelera_amin = 0.0;         //由减速初末速度限制的减速最小减速度
    double dec_start_a = 0.0;           //减速阶段初始加速度
    double dec_start_v = 0.0;           //减速阶段初始速度
    double dec_start_l = 0.0;           //减速阶段初始位移
    double residual_amount = 0.0;       //各阶段切换时的插补余量
    double limit_cartesian_amax = 250;//关节空间运动时的最大加速度-最大力矩输出另外限制（m/s^2）
    double limit_cartesian_jmax = 800;//关节空间运动时的最大加加速度(m/s^3)
    double limit_cartesian_vmax = 0.0;  //关节空间指令设定的最大速度(m/s)
    bool interpol_finish = 0.0;         //插补结束标志


    classic6dofForKine(m_curJoints.data(), &start_pose);//初始位姿 正解获取
    end_pose = {_pose[0], _pose[1], _pose[2],_pose[3], _pose[4], _pose[5], {0}, 0};//目标位姿 自行设定

    //直接先判断逆解得到的角度 正解得到的位姿是否与目标位姿相同
    Kine6dSol q_sol_f;
    classic6dofInvKine(&end_pose, m_curJoints.data(), &q_sol_f);
    bool valid[8];
    int validCnt = 0;
    for (int i = 0; i < 8; i++) {
        valid[i] = true;
        for (int j = 0; j < NUM_JOINTS; j++) {
            if (q_sol_f.sol[i][j] > m_angleLimitMax[j] || q_sol_f.sol[i][j] < m_angleLimitMin[j]) {
                valid[i] = false;
                break;
            }
        }
        if (valid[i]) validCnt++;
    }
    if (validCnt > 0) 
    {
        // 选择距离当前关节位置最近的解
        float minDist = std::numeric_limits<float>::max();
        int bestIndex = -1;
        for (int i = 0; i < 8; i++) 
        {
            if (!valid[i]) continue;

            float dist = 0.0f;
            for (int j = 0; j < NUM_JOINTS; j++)
            {
                float d = m_curJoints[j] - q_sol_f.sol[i][j];
                dist += d * d;
            }

            if (dist < minDist) {
                minDist = dist;
                bestIndex = i;
            }
        }
        if (bestIndex >= 0) 
        {
            std::array<float, NUM_JOINTS> joint_f;
            for (int i = 0; i < NUM_JOINTS; i++) 
            {
                joint_f[i] = q_sol_f.sol[bestIndex][i];//单位弧度
            }
            classic6dofForKine(joint_f.data(), &end_pose); //正解判断结束位姿是否等于输入的目标位姿
            if (fabs(end_pose.X - _pose[0]) > 1e-3 || fabs(end_pose.Y - _pose[1]) > 1e-3 || fabs(end_pose.Z - _pose[2]) > 1e-3) {
                std::cerr << "目标点超出工作空间!" << std::endl;
                return; // 如果逆解验证失败，直接返回 
            }
        } 
    }
    else
    {
        std::cerr << "No valid solution found!" << std::endl;
        return;
    }


    double cartesian_dist = std::sqrt(
        std::pow(end_pose.X - start_pose.X, 2) +
        std::pow(end_pose.Y - start_pose.Y, 2) +
        std::pow(end_pose.Z - start_pose.Z, 2)
    );//笛卡尔空间直线插补总距离

    /*欧拉角转四元数*/
    Eigen::AngleAxisf rotA_start(start_pose.A, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotB_start(start_pose.B, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotC_start(start_pose.C, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quat_start = rotC_start * rotB_start * rotA_start;

    Eigen::AngleAxisf rotA_end(end_pose.A, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotB_end(end_pose.B, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotC_end(end_pose.C, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf quat_end = rotC_end * rotB_end * rotA_end;

    // 计算四元数夹角（姿态变化距离，单位：弧度）
    double angle_dist = quat_start.angularDistance(quat_end); // Eigen自带方法
    double rot_dist = angle_dist * 150; // 150是单位转换系数，假设每弧度对应150mm

    double interpDist = std::max(cartesian_dist, rot_dist); // 取位置和姿态变化的最大值作为总距离
    sum_cartesian_l = interpDist; // 总插补距离

    limit_cartesian_vmax = _speed * m_SpeedRatio; //实际再多乘个比例系数
    Speed_planning_step = 1;   //开始插补

    /*----------带预测减速的S型速度规划----------*/
    while(interpol_finish != true) 
    {
        // 检查停止和暂停状态
        if (GlobalParams::isStop) {
            // 平滑减速到停止
            if (current_cartesian_v > 0.01) {  // 如果还有速度
                current_cartesian_j = -limit_cartesian_jmax;  // 使用最大减速度
                current_cartesian_a = current_cartesian_a + current_cartesian_j * current_cycle_tim;
                // 减速过程中加速度可以为负，不要强制设置为0
                current_cartesian_v = current_cartesian_v + current_cartesian_a * current_cycle_tim;
                if (current_cartesian_v < 0) current_cartesian_v = 0;  // 防止负速度
                
                // 继续插补到当前减速位置
                current_cartesian_l = current_cartesian_v * current_cycle_tim;
                current_cartesian_suml += current_cartesian_l;
                
                // 位置和姿态插补
                lamda = current_cartesian_suml / sum_cartesian_l;
                float interp_X = start_pose.X + lamda * (end_pose.X - start_pose.X);
                float interp_Y = start_pose.Y + lamda * (end_pose.Y - start_pose.Y);
                float interp_Z = start_pose.Z + lamda * (end_pose.Z - start_pose.Z);
                
                Eigen::Quaternionf q_interp = quat_start.slerp(lamda, quat_end);
                Eigen::Matrix3f rotm = q_interp.toRotationMatrix();
                Eigen::Vector3f euler_angles = rotm.eulerAngles(2, 1, 0);
                
                Kine6d interp_pose = {interp_X, interp_Y, interp_Z, euler_angles[2], euler_angles[1], euler_angles[0], {0}, 0};
                Kine6dSol q_sol;
                classic6dofInvKine(&interp_pose, m_curJoints.data(), &q_sol);
                
                // 选择最优解并发送
                bool valid[8];
                int validCnt = 0;
                for (int i = 0; i < 8; i++) {
                    valid[i] = true;
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        if (q_sol.sol[i][j] > m_angleLimitMax[j] || q_sol.sol[i][j] < m_angleLimitMin[j]) {
                            valid[i] = false;
                            break;
                        }
                    }
                    if (valid[i]) validCnt++;
                }
                
                if (validCnt > 0) {
                    float minDist = std::numeric_limits<float>::max();
                    int bestIndex = -1;
                    for (int i = 0; i < 8; i++) {
                        if (!valid[i]) continue;
                        float dist = 0.0f;
                        for (int j = 0; j < NUM_JOINTS; j++) {
                            float d = m_curJoints[j] - q_sol.sol[i][j];
                            dist += d * d;
                        }
                        if (dist < minDist) {
                            minDist = dist;
                            bestIndex = i;
                        }
                    }
                    
                    if (bestIndex >= 0) {
                        for (int j = 0; j < NUM_JOINTS; j++) {
                            m_curJoints[j] = q_sol.sol[bestIndex][j];
                        }
                        moveJoints(m_curJoints);
                    }
                }
                continue;
            }
            GlobalParams::isStop = false;
            break;
        }
        
        // 暂停处理
        if (GlobalParams::isPause) {
            std::cout << "直线运动暂停中...\n";
            
            // 平滑减速到停止
            if (current_cartesian_v > 0.01) {  // 如果还有速度
                current_cartesian_j = -(limit_cartesian_jmax * 0.5);  // 使用0.5倍减减速度
                current_cartesian_a = current_cartesian_a + current_cartesian_j * current_cycle_tim;
                // 减速过程中加速度可以为负，不要强制设置为0
                current_cartesian_v = current_cartesian_v + current_cartesian_a * current_cycle_tim;
                if (current_cartesian_v < 0) current_cartesian_v = 0;  // 防止负速度
                
                // 继续插补到当前减速位置
                current_cartesian_l = current_cartesian_v * current_cycle_tim;
                current_cartesian_suml += current_cartesian_l;
                
                // 位置和姿态插补
                lamda = current_cartesian_suml / sum_cartesian_l;
                float interp_X = start_pose.X + lamda * (end_pose.X - start_pose.X);
                float interp_Y = start_pose.Y + lamda * (end_pose.Y - start_pose.Y);
                float interp_Z = start_pose.Z + lamda * (end_pose.Z - start_pose.Z);
                
                Eigen::Quaternionf q_interp = quat_start.slerp(lamda, quat_end);
                Eigen::Matrix3f rotm = q_interp.toRotationMatrix();
                Eigen::Vector3f euler_angles = rotm.eulerAngles(2, 1, 0);
                
                Kine6d interp_pose = {interp_X, interp_Y, interp_Z, euler_angles[2], euler_angles[1], euler_angles[0], {0}, 0};
                Kine6dSol q_sol;
                classic6dofInvKine(&interp_pose, m_curJoints.data(), &q_sol);
                
                // 选择最优解并发送
                bool valid[8];
                int validCnt = 0;
                for (int i = 0; i < 8; i++) {
                    valid[i] = true;
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        if (q_sol.sol[i][j] > m_angleLimitMax[j] || q_sol.sol[i][j] < m_angleLimitMin[j]) {
                            valid[i] = false;
                            break;
                        }
                    }
                    if (valid[i]) validCnt++;
                }
                
                if (validCnt > 0) {
                    float minDist = std::numeric_limits<float>::max();
                    int bestIndex = -1;
                    for (int i = 0; i < 8; i++) {
                        if (!valid[i]) continue;
                        float dist = 0.0f;
                        for (int j = 0; j < NUM_JOINTS; j++) {
                            float d = m_curJoints[j] - q_sol.sol[i][j];
                            dist += d * d;
                        }
                        if (dist < minDist) {
                            minDist = dist;
                            bestIndex = i;
                        }
                    }
                    
                    if (bestIndex >= 0) {
                        for (int j = 0; j < NUM_JOINTS; j++) {
                            m_curJoints[j] = q_sol.sol[bestIndex][j];
                        }
                        moveJoints(m_curJoints);
                    }
                }

                continue;
            }
            
            // 速度已经停止，等待继续指令
            std::cout << "直线运动已暂停，等待继续指令...\n";
            while (GlobalParams::isPause) {
                HighLevelCommand cmd;
                if (shm().high_prio_cmd_queue.pop(cmd))
                {
                    handleHighPriorityCommand(cmd);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 检测停止命令，如果收到停止命令则退出暂停状态
                if (GlobalParams::isStop) {
                    std::cout << "收到停止命令，退出暂停状态\n";
                    break;
                }
            }
            
            // 如果收到停止命令，退出函数
            if (GlobalParams::isStop) {
                std::cout << "收到停止命令，退出直线运动\n";
                GlobalParams::isStop = false;
                break;
            }
            
            // 如果是继续指令，重新计算剩余距离
            if (GlobalParams::isResume) {
                GlobalParams::isResume = false;
                std::cout << "恢复直线运动，重新规划轨迹\n";
                
                // 重新计算起始位姿
                classic6dofForKine(m_curJoints.data(), &start_pose);
                
                // 重新计算剩余距离
                double remaining_dist = std::sqrt(
                    std::pow(end_pose.X - start_pose.X, 2) +
                    std::pow(end_pose.Y - start_pose.Y, 2) +
                    std::pow(end_pose.Z - start_pose.Z, 2)
                );
                
                // 更新参数
                sum_cartesian_l = remaining_dist;
                current_cartesian_suml = 0;
                
                // 重新计算四元数
                Eigen::AngleAxisf rotA_start_new(start_pose.A, Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf rotB_start_new(start_pose.B, Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf rotC_start_new(start_pose.C, Eigen::Vector3f::UnitZ());
                quat_start = rotC_start_new * rotB_start_new * rotA_start_new;
                
                // 重置速度规划参数
                current_cartesian_v = 0;
                current_cartesian_a = 0;
                current_cartesian_j = 0;
                Speed_planning_step = 1;
                
                std::cout << "剩余距离: " << remaining_dist << " 米\n";
            }
        }
        
        //加减速处理
        if(Speed_planning_step == 1) {
        //预测减速阶段
            //Step1-以J进行加速
            tempnext_cartesian_j = limit_cartesian_jmax;
            tempnext_cartesian_a = tempnext_cartesian_j*current_cycle_tim + current_cartesian_a; //以J对a进行增加
            tempnext_cartesian_v = current_cartesian_v + current_cartesian_a*current_cycle_tim + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim / 2;
            tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 + \
                                    tempnext_cartesian_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            //先判断速度是否超限位
            if(tempnext_cartesian_v > limit_cartesian_vmax) {
                tempnext_cartesian_v = limit_cartesian_vmax;
                tempnext_cartesian_j = 2*(tempnext_cartesian_v - current_cartesian_v - current_cartesian_a*current_cycle_tim) / current_cycle_tim*current_cycle_tim;
                tempnext_cartesian_a = current_cartesian_a + tempnext_cartesian_j*current_cycle_tim;
                tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                                    + tempnext_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }
            //再判断加速度是否超限位
            if(tempnext_cartesian_a > limit_cartesian_amax) {
                //预测阶段超过最大加速度
                tempnext_cartesian_a = limit_cartesian_amax;
                tempnext_cartesian_j = (tempnext_cartesian_a - current_cartesian_a) / current_cycle_tim;
                tempnext_cartesian_v = current_cartesian_v + current_cartesian_a * current_cycle_tim + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim / 2;
                tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                                + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }

            //Step2-以-J进行减速-得到减加速的距离-同时获得减加速后的速度
            tempnext_accdec_v = tempnext_cartesian_v + tempnext_cartesian_a*tempnext_cartesian_a / (2*limit_cartesian_jmax);
            tempnext_accdec_l = tempnext_cartesian_v * tempnext_cartesian_a / limit_cartesian_jmax + tempnext_cartesian_a*tempnext_cartesian_a*tempnext_cartesian_a / (3*limit_cartesian_jmax*limit_cartesian_jmax);

            //Step3-以三阶段减速方式-计算减速距离
            if(tempnext_accdec_v >= limit_cartesian_vmax) {
                //减加速后达到的速度超过设定速度-即刻进加速处理阶段
                Speed_planning_step = 2; 
                acc_dec_currentn = 1;         //进入时初始化插补次数
                acc_dec_num = abs(current_cartesian_a / (limit_cartesian_jmax*current_cycle_tim)); //加速阶段插补点数
                //减加速阶段初始参数
                acc_dec_start_a = current_cartesian_a;
                acc_dec_start_v = current_cartesian_v;
                acc_dec_start_l = current_cartesian_l;
            }else {
                //计算三段各自的时间-最小a 最大a-距离、速度-距离后面判断
                if(tempnext_accdec_v > limit_cartesian_amax*limit_cartesian_amax / limit_cartesian_jmax) {
                    //存在三段减速段
                    tempnext_dec_l = tempnext_accdec_v * ( limit_cartesian_amax/(2*limit_cartesian_jmax) + tempnext_accdec_v/(2*limit_cartesian_amax) );
                    //得到三段减速数据
                    decelera_t5 = limit_cartesian_amax / (limit_cartesian_jmax*current_cycle_tim);
                    decelera_t6 = (tempnext_accdec_v / (limit_cartesian_amax*current_cycle_tim)) - decelera_t5;
                    decelera_t7 = decelera_t5;
                    decelera_amin = -limit_cartesian_amax;
                    Deceleration_section = 3;
                }else {
                    //只有两段减速段
                    tempnext_dec_l = (tempnext_accdec_v) * sqrt(tempnext_accdec_v / limit_cartesian_jmax);
                    //得到两段减速数据
                    decelera_t5 = sqrt(tempnext_accdec_v / (limit_cartesian_jmax))/current_cycle_tim;
                    decelera_t6 = 0;
                    decelera_t7 = decelera_t5;
                    decelera_amin = limit_cartesian_jmax * decelera_t5;
                    Deceleration_section = 2;
                }
                //判断三阶段减速距离 tempnext_cartesian_v~0
                if( (sum_cartesian_l-current_cartesian_suml) < tempnext_dec_l+tempnext_accdec_l+tempnext_cartesian_l) {
                    //距离超过所给距离-进入加速处理阶段
                    Speed_planning_step = 2;
                    acc_dec_currentn = 1;         //进入时初始化插补次数
                    acc_dec_num = abs(current_cartesian_a / (limit_cartesian_jmax*current_cycle_tim)); //加速阶段插补点数
                    acc_dec_start_a = current_cartesian_a;
                    acc_dec_start_v = current_cartesian_v;
                    acc_dec_start_l = current_cartesian_l;
                }else {
                    //距离还够用-继续预测减速阶段
                    Speed_planning_step = 1;
                    current_cartesian_a = tempnext_cartesian_a; //用于下次规划判断
                    current_cartesian_v = tempnext_cartesian_v;
                    current_cartesian_j = tempnext_cartesian_j;
                    current_cartesian_l = tempnext_cartesian_l; //用于长轴增量插补
                    current_cartesian_suml += current_cartesian_l;
                    //剩余距离-可写可不写了
                }
            }
        }
        if(Speed_planning_step == 2) {
            //减加速处理阶段
            current_cartesian_j = -limit_cartesian_jmax;
            if(acc_dec_currentn <= acc_dec_num) {

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                                    + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = acc_dec_start_a + current_cartesian_j*acc_dec_currentn*current_cycle_tim;

                current_cartesian_v = acc_dec_start_v + acc_dec_start_a*acc_dec_currentn*current_cycle_tim + current_cartesian_j \
                                * acc_dec_currentn*acc_dec_currentn*current_cycle_tim*current_cycle_tim / 2;


                current_cartesian_suml += current_cartesian_l;
                acc_dec_currentn++; //减加速阶段插补次数自增

            }

            if(acc_dec_currentn > acc_dec_num)
            {
                //到达插补点数-进入匀速预测减速阶段
                Speed_planning_step = 3;
            }
        }
        if(Speed_planning_step == 3) {
            //匀速预测减速阶段
                //Step1-匀速计算一个周期
                tempnext_cartesian_a = 0;
                tempnext_cartesian_j = 0;
                tempnext_cartesian_v = current_cartesian_v;
                tempnext_cartesian_l = tempnext_cartesian_v * current_cycle_tim;


                if(Deceleration_section == 3) {
                    //计算三段减速距离
                    tempnext_dec_l = (current_cartesian_v) * (limit_cartesian_amax / (2*limit_cartesian_jmax) + (current_cartesian_v / 2*limit_cartesian_amax));
                    decelera_t5 = limit_cartesian_amax / (limit_cartesian_jmax*current_cycle_tim);
                    decelera_t6 = (tempnext_cartesian_v / (limit_cartesian_amax*current_cycle_tim) ) - decelera_t5;
                    decelera_t7 = decelera_t5;
                    decelera_amin = -limit_cartesian_amax;
                }else if(Deceleration_section == 2) {
                    //只有两段减速距离
                    tempnext_dec_l = (current_cartesian_v)*sqrt(current_cartesian_v / limit_cartesian_jmax);
                    decelera_t5 = sqrt(tempnext_cartesian_v / (limit_cartesian_jmax))/current_cycle_tim;
                    if(decelera_t5 - (int)decelera_t5 >= 0.5) {
                        decelera_t5 = decelera_t5+1;
                    }
                    decelera_t6 = 0;
                    decelera_t7 = decelera_t5;
                    decelera_amin = limit_cartesian_jmax * decelera_t5;
                }
                
                //判断是否超过剩余距离
                if(tempnext_dec_l+tempnext_cartesian_l > (sum_cartesian_l - current_cartesian_suml)) {
                    //超过剩余距离，不可匀速
                    Speed_planning_step = 4;
                    //进减速阶段初始状态
                    current_cartesian_l = (sum_cartesian_l - current_cartesian_suml) - tempnext_dec_l;
                    current_cartesian_suml += current_cartesian_l;

                    dec_count_num = 1;
                    
                    dec_start_a = current_cartesian_a;
                    dec_start_v = current_cartesian_v;
                    dec_start_l = current_cartesian_l;
                }else {
                    current_cartesian_j = tempnext_cartesian_j;
                    current_cartesian_a = tempnext_cartesian_a;
                    current_cartesian_v = tempnext_cartesian_v;
                    current_cartesian_l = tempnext_cartesian_l;
                    current_cartesian_suml += current_cartesian_l;
                    //重新赋值减速量-以上面算的为准
                }
        }
        if(Speed_planning_step == 4) {
            //三段减速处理阶段
            last_dec_count_num = dec_count_num;

            if(dec_count_num <= decelera_t5) {
                current_cartesian_j = -limit_cartesian_jmax;

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                                    + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = dec_start_a + current_cartesian_j*dec_count_num*current_cycle_tim;

                current_cartesian_v = dec_start_v + dec_start_a*dec_count_num*current_cycle_tim + current_cartesian_j * dec_count_num \
                                    * dec_count_num * current_cycle_tim * current_cycle_tim / 2;

                current_cartesian_suml += current_cartesian_l;

            }else if(dec_count_num > decelera_t5 && dec_count_num <= decelera_t6+decelera_t5) {
                current_cartesian_j = 0;
                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2;
                current_cartesian_a = dec_start_a;
                current_cartesian_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim;

                current_cartesian_suml += current_cartesian_l;

            }else if(dec_count_num > decelera_t6+decelera_t5 && dec_count_num <= decelera_t6+decelera_t5+decelera_t7) {
                
                current_cartesian_j = limit_cartesian_jmax;

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                                    + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = dec_start_a + current_cartesian_j*(dec_count_num-dec_count_start)*current_cycle_tim;

                current_cartesian_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim + current_cartesian_j \
                                    * (dec_count_num-dec_count_start)*(dec_count_num-dec_count_start)*current_cycle_tim*current_cycle_tim / 2;

                current_cartesian_suml += current_cartesian_l;

            }

            dec_count_num++;//不考虑插补余量-预计会有到达误差

            if(dec_count_num >= decelera_t5+decelera_t6+decelera_t7) {
                //插补计算结束  
                interpol_finish = 1;
            }

            //阶段切换时初始位置保存
            if(last_dec_count_num <= decelera_t5 && dec_count_num > decelera_t5) {

                dec_start_a = current_cartesian_a;
                dec_start_v = current_cartesian_v;
                dec_count_start = decelera_t5;

            }else if(last_dec_count_num <= decelera_t5+decelera_t6 && dec_count_num > decelera_t5+decelera_t6) {

                dec_start_a = current_cartesian_a;
                dec_start_v = current_cartesian_v;
                dec_count_start = decelera_t5+decelera_t6 ;
            }
        }

        lamda = current_cartesian_suml / sum_cartesian_l;

        lamda = std::min(1.0, lamda); // 确保lamda不超过1
        lamda = std::max(0.0, lamda); // 确保lamda不小于0

        /*位置插补*/
        interp_X = start_pose.X + lamda * (end_pose.X - start_pose.X);
        interp_Y = start_pose.Y + lamda * (end_pose.Y - start_pose.Y);
        interp_Z = start_pose.Z + lamda * (end_pose.Z - start_pose.Z);
        /*姿态插补*/
        q_interp = quat_start.slerp(lamda, quat_end);
        rotm = q_interp.toRotationMatrix();
        euler_angles = rotm.eulerAngles(2, 1, 0); // ZYX顺序欧拉角
        interp_C = euler_angles[0]; 
        interp_B = euler_angles[1]; 
        interp_A = euler_angles[2]; 

        interp_pose = {interp_X, interp_Y, interp_Z, interp_A, interp_B, interp_C, {0}, 0};
        
        /*插补得到的位姿逆解*/
        classic6dofInvKine(&interp_pose, m_curJoints.data(), &q_sol); 
        
        /* 选择逆解最优解 */
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

        if (validCnt > 0) 
        {
            // 选择距离当前关节位置最近的解
            float minDist = std::numeric_limits<float>::max();
            int bestIndex = -1;

            for (int i = 0; i < 8; i++) 
            {
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
            //防止所有解的角度都变化过大
            if (fabs(q_sol.sol[bestIndex][0] - m_curJoints[0]) > 0.1 ||
                fabs(q_sol.sol[bestIndex][1] - m_curJoints[1]) > 0.1 ||
                fabs(q_sol.sol[bestIndex][2] - m_curJoints[2]) > 0.1 ||
                fabs(q_sol.sol[bestIndex][3] - m_curJoints[3]) > 0.1 ||
                fabs(q_sol.sol[bestIndex][4] - m_curJoints[4]) > 0.1 ||
                fabs(q_sol.sol[bestIndex][5] - m_curJoints[5]) > 0.1)
            {
                std::cerr << "关节角度变化过大，退出插补！" << std::endl;
                return; // 如果角度变化过大，直接返回
            }
            
            if (bestIndex >= 0) 
            {
                std::array<float, NUM_JOINTS> targetJoints;
                for (int i = 0; i < NUM_JOINTS; i++) 
                {
                    targetJoints[i] = q_sol.sol[bestIndex][i];//单位弧度
                }

                // printf("moveL, current joints: ");
                // for (int i = 0; i < NUM_JOINTS; i++)
                // {
                //     std::cout << targetJoints[i] << " ";
                // }
                // std::cout << "\n";

                //更新状态以进行下一次迭代
                m_curJoints = targetJoints;

                moveJoints(targetJoints);
            } 
            else
            {
                std::cerr << "No valid solution found!" << std::endl;
            }
        }
        else
        {
            std::cerr << "No valid solution found!" << std::endl;
            return;
        }
    }
    printf("当前角度：%f %f %f %f %f %f \n", m_curJoints[0], m_curJoints[1], m_curJoints[2], m_curJoints[3], m_curJoints[4], m_curJoints[5]);
    Kine6d cur_pose;
    classic6dofForKine(m_curJoints.data(), &cur_pose);
    printf("当前位姿：%f %f %f %f %f %f \n", cur_pose.X, cur_pose.Y, cur_pose.Z, cur_pose.A, cur_pose.B, cur_pose.C);
}

/*笛卡尔空间圆弧插补*/
void Robot::moveC(std::array<float, NUM_JOINTS> mid_pose, std::array<float, NUM_JOINTS> end_pose, float speed)
{
    // 圆弧插补
//    // 状态复位
//    GlobalParams::isStop = false;
//    GlobalParams::isPause = false;
//    GlobalParams::isResume = false;

    // 起点、圆弧过渡点、终点
    Kine6d start_pose;
    classic6dofForKine(m_curJoints.data(), &start_pose);

    Eigen::Vector3d P0(start_pose.X, start_pose.Y, start_pose.Z);
    Eigen::Vector3d P1(mid_pose[0], mid_pose[1], mid_pose[2]);
    Eigen::Vector3d P2(end_pose[0], end_pose[1], end_pose[2]);

    // 姿态转换为四元数
    auto eulerToQuat = [](double A, double B, double C) {
        return Eigen::AngleAxisd(C, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(B, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(A, Eigen::Vector3d::UnitX());
    };

    Eigen::Quaterniond quat_start = eulerToQuat(start_pose.A, start_pose.B, start_pose.C);
    Eigen::Quaterniond quat_mid   = eulerToQuat(mid_pose[3], mid_pose[4], mid_pose[5]);
    Eigen::Quaterniond quat_end   = eulerToQuat(end_pose[3], end_pose[4], end_pose[5]);

    // 插值圆弧路径参数
    Eigen::Vector3d v1 = P1 - P0;
    Eigen::Vector3d v2 = P2 - P1;
    Eigen::Vector3d normal = v1.cross(v2).normalized();

    Eigen::Vector3d mid1 = (P0 + P1) / 2.0f;
    Eigen::Vector3d mid2 = (P1 + P2) / 2.0f;
    Eigen::Vector3d dir1 = normal.cross(P1 - P0).normalized();
    Eigen::Vector3d dir2 = normal.cross(P2 - P1).normalized();

    // 解最小二乘：mid1 + t1*dir1 = mid2 + t2*dir2
    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = dir1;
    A.col(1) = -dir2;
    Eigen::Vector3d b = mid2 - mid1;

    Eigen::Vector2d x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    Eigen::Vector3d center = mid1 + x(0) * dir1;

    double radius = (P0 - center).norm();
    Eigen::Vector3d n = (P0 - center).normalized();
    Eigen::Vector3d m = normal.cross(n);

    double angle_total = std::atan2((P2 - center).dot(m), (P2 - center).dot(n));
    double theta_total = std::abs(angle_total); // 总角度


    /*速度规划相关变量*/
    int Speed_planning_step = 0;       //0:未规划 1：预测减速阶段 2：加速处理阶段 3：匀速预测减速阶段 4：三段减速阶段
    double current_cycle_tim = 0.001;   //当前插补时间
//    double temp_accumu_tim = 0;         //预测减速段的累积时间
    double current_cartesian_v = 0.0;   //当前笛卡尔空间速度
    double current_cartesian_a = 0.0;   //当前笛卡尔空间加速度
    double current_cartesian_j = 0.0;   //当前笛卡尔空间加加速度
    double tempnext_cartesian_v = 0.0;  //预测下一步笛卡尔空间速度
    double tempnext_cartesian_a = 0.0;  //预测下一步笛卡尔空间加速度
    double tempnext_cartesian_j = 0.0;  //预测下一步笛卡尔空间加加速度
    double tempnext_cartesian_l = 0.0;  //预增加已走的距离
    double tempnext_accdec_l = 0.0;     //减加速段走的距离
    double tempnext_accdec_v = 0.0;     //减加速段末速度
    double tempnext_dec_l = 0.0;        //减速段距离
    double acc_dec_num = 0;             //减加速过程的插补点数-计算得到
    double dec_count_num = 0;           //减速过程当前插补点数
    uint32_t dec_count_start = 0;       //阶段性开始插补参数
    uint32_t last_dec_count_num = 0;    //减加速过程上一个插补点数
    uint32_t acc_dec_currentn = 0;      //减加速阶段插补次数
    double acc_dec_start_a = 0.0;       //减加速阶段初始加速度
    double acc_dec_start_v = 0.0;       //减加速阶段初始速度
    double acc_dec_start_l = 0.0;       //减加速阶段初始位移
    uint8_t Deceleration_section = 0.0; //减速段的实际段数
    double decelera_t5 = 0.0;           //第一减速段时间
    double decelera_t6 = 0.0;           //第二减速段时间
    double decelera_t7 = 0.0;           //第三减速段时间
    double decelera_amin = 0.0;         //由减速初末速度限制的减速最小减速度
    double dec_start_a = 0.0;           //减速阶段初始加速度
    double dec_start_v = 0.0;           //减速阶段初始速度
    double dec_start_l = 0.0;           //减速阶段初始位移
//    double residual_amount = 0.0;       //各阶段切换时的插补余量
    double limit_cartesian_amax = 250;//关节空间运动时的最大加速度-最大力矩输出另外限制（m/s^2）
    double limit_cartesian_jmax = 800;//关节空间运动时的最大加加速度(m/s^3)
    //关节空间指令设定的最大速度(m/s)
    double limit_cartesian_vmax = speed * m_SpeedRatio; //实际再多乘个比例系数;

    bool interpol_finish = false;         //插补结束标志


    double current_cartesian_l = 0.0;   //当前周期需要走的距离
    double current_cartesian_suml = 0.0;//当前已经走过的距离

    // 需要移动的总距离
    // 笛卡尔空间圆弧长
    double sum_cartesian_l = radius * theta_total;

    double ratio = 0.0;

    Speed_planning_step = 1;   //开始插补

#ifdef VISUALIZE_INTERP_RES

    std::ofstream fout("arc_trajectory.csv");
    fout << "x,y,z,qx,qy,qz,qw\n";
#endif

    /*----------带预测减速的S型速度规划----------*/
    while(interpol_finish != true)
    {
        //加减速处理
        if(Speed_planning_step == 1) {
            //预测减速阶段
            //Step1-以J进行加速
            tempnext_cartesian_j = limit_cartesian_jmax;
            tempnext_cartesian_a = tempnext_cartesian_j*current_cycle_tim + current_cartesian_a; //以J对a进行增加
            tempnext_cartesian_v = current_cartesian_v + current_cartesian_a*current_cycle_tim + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim / 2;
            tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 + \
                    tempnext_cartesian_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            //先判断速度是否超限位
            if(tempnext_cartesian_v > limit_cartesian_vmax) {
                tempnext_cartesian_v = limit_cartesian_vmax;
                tempnext_cartesian_j = 2*(tempnext_cartesian_v - current_cartesian_v - current_cartesian_a*current_cycle_tim) / current_cycle_tim*current_cycle_tim;
                tempnext_cartesian_a = current_cartesian_a + tempnext_cartesian_j*current_cycle_tim;
                tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                        + tempnext_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }
            //再判断加速度是否超限位
            if(tempnext_cartesian_a > limit_cartesian_amax) {
                //预测阶段超过最大加速度
                tempnext_cartesian_a = limit_cartesian_amax;
                tempnext_cartesian_j = (tempnext_cartesian_a - current_cartesian_a) / current_cycle_tim;
                tempnext_cartesian_v = current_cartesian_v + current_cartesian_a * current_cycle_tim + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim / 2;
                tempnext_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                        + tempnext_cartesian_j*current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;
            }

            //Step2-以-J进行减速-得到减加速的距离-同时获得减加速后的速度
            tempnext_accdec_v = tempnext_cartesian_v + tempnext_cartesian_a*tempnext_cartesian_a / (2*limit_cartesian_jmax);
            tempnext_accdec_l = tempnext_cartesian_v * tempnext_cartesian_a / limit_cartesian_jmax + tempnext_cartesian_a*tempnext_cartesian_a*tempnext_cartesian_a / (3*limit_cartesian_jmax*limit_cartesian_jmax);

            //Step3-以三阶段减速方式-计算减速距离
            if(tempnext_accdec_v >= limit_cartesian_vmax) {
                //减加速后达到的速度超过设定速度-即刻进加速处理阶段
                Speed_planning_step = 2;
                acc_dec_currentn = 1;         //进入时初始化插补次数
                acc_dec_num = abs(current_cartesian_a / (limit_cartesian_jmax*current_cycle_tim)); //加速阶段插补点数
                //减加速阶段初始参数
                acc_dec_start_a = current_cartesian_a;
                acc_dec_start_v = current_cartesian_v;
                acc_dec_start_l = current_cartesian_l;
            }else {
                //计算三段各自的时间-最小a 最大a-距离、速度-距离后面判断
                if(tempnext_accdec_v > limit_cartesian_amax*limit_cartesian_amax / limit_cartesian_jmax) {
                    //存在三段减速段
                    tempnext_dec_l = tempnext_accdec_v * ( limit_cartesian_amax/(2*limit_cartesian_jmax) + tempnext_accdec_v/(2*limit_cartesian_amax) );
                    //得到三段减速数据
                    decelera_t5 = limit_cartesian_amax / (limit_cartesian_jmax*current_cycle_tim);
                    decelera_t6 = (tempnext_accdec_v / (limit_cartesian_amax*current_cycle_tim)) - decelera_t5;
                    decelera_t7 = decelera_t5;
                    decelera_amin = -limit_cartesian_amax;
                    Deceleration_section = 3;
                }else {
                    //只有两段减速段
                    tempnext_dec_l = (tempnext_accdec_v) * sqrt(tempnext_accdec_v / limit_cartesian_jmax);
                    //得到两段减速数据
                    decelera_t5 = sqrt(tempnext_accdec_v / (limit_cartesian_jmax))/current_cycle_tim;
                    decelera_t6 = 0;
                    decelera_t7 = decelera_t5;
                    decelera_amin = limit_cartesian_jmax * decelera_t5;
                    Deceleration_section = 2;
                }
                //判断三阶段减速距离 tempnext_cartesian_v~0
                if( (sum_cartesian_l-current_cartesian_suml) < tempnext_dec_l+tempnext_accdec_l+tempnext_cartesian_l) {
                    //距离超过所给距离-进入加速处理阶段
                    Speed_planning_step = 2;
                    acc_dec_currentn = 1;         //进入时初始化插补次数
                    acc_dec_num = abs(current_cartesian_a / (limit_cartesian_jmax*current_cycle_tim)); //加速阶段插补点数
                    acc_dec_start_a = current_cartesian_a;
                    acc_dec_start_v = current_cartesian_v;
                    acc_dec_start_l = current_cartesian_l;
                }else {
                    //距离还够用-继续预测减速阶段
                    Speed_planning_step = 1;
                    current_cartesian_a = tempnext_cartesian_a; //用于下次规划判断
                    current_cartesian_v = tempnext_cartesian_v;
                    current_cartesian_j = tempnext_cartesian_j;
                    current_cartesian_l = tempnext_cartesian_l; //用于长轴增量插补
                    current_cartesian_suml += current_cartesian_l;
                    //剩余距离-可写可不写了
                }
            }
        }
        if(Speed_planning_step == 2) {
            //减加速处理阶段
            current_cartesian_j = -limit_cartesian_jmax;
            if(acc_dec_currentn <= acc_dec_num) {

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                        + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = acc_dec_start_a + current_cartesian_j*acc_dec_currentn*current_cycle_tim;

                current_cartesian_v = acc_dec_start_v + acc_dec_start_a*acc_dec_currentn*current_cycle_tim + current_cartesian_j \
                        * acc_dec_currentn*acc_dec_currentn*current_cycle_tim*current_cycle_tim / 2;


                current_cartesian_suml += current_cartesian_l;
                acc_dec_currentn++; //减加速阶段插补次数自增

            }

            if(acc_dec_currentn > acc_dec_num)
            {
                //到达插补点数-进入匀速预测减速阶段
                Speed_planning_step = 3;
            }
        }
        if(Speed_planning_step == 3) {
            //匀速预测减速阶段
            //Step1-匀速计算一个周期
            tempnext_cartesian_a = 0;
            tempnext_cartesian_j = 0;
            tempnext_cartesian_v = current_cartesian_v;
            tempnext_cartesian_l = tempnext_cartesian_v * current_cycle_tim;


            if(Deceleration_section == 3) {
                //计算三段减速距离
                tempnext_dec_l = (current_cartesian_v) * (limit_cartesian_amax / (2*limit_cartesian_jmax) + (current_cartesian_v / 2*limit_cartesian_amax));
                decelera_t5 = limit_cartesian_amax / (limit_cartesian_jmax*current_cycle_tim);
                decelera_t6 = (tempnext_cartesian_v / (limit_cartesian_amax*current_cycle_tim) ) - decelera_t5;
                decelera_t7 = decelera_t5;
                decelera_amin = -limit_cartesian_amax;
            }else if(Deceleration_section == 2) {
                //只有两段减速距离
                tempnext_dec_l = (current_cartesian_v)*sqrt(current_cartesian_v / limit_cartesian_jmax);
                decelera_t5 = sqrt(tempnext_cartesian_v / (limit_cartesian_jmax))/current_cycle_tim;
                if(decelera_t5 - (int)decelera_t5 >= 0.5) {
                    decelera_t5 = decelera_t5+1;
                }
                decelera_t6 = 0;
                decelera_t7 = decelera_t5;
                decelera_amin = limit_cartesian_jmax * decelera_t5;
            }

            //判断是否超过剩余距离
            if(tempnext_dec_l+tempnext_cartesian_l > (sum_cartesian_l - current_cartesian_suml)) {
                //超过剩余距离，不可匀速
                Speed_planning_step = 4;
                //进减速阶段初始状态
                current_cartesian_l = (sum_cartesian_l - current_cartesian_suml) - tempnext_dec_l;
                current_cartesian_suml += current_cartesian_l;

                dec_count_num = 1;

                dec_start_a = current_cartesian_a;
                dec_start_v = current_cartesian_v;
                dec_start_l = current_cartesian_l;
            }else {
                current_cartesian_j = tempnext_cartesian_j;
                current_cartesian_a = tempnext_cartesian_a;
                current_cartesian_v = tempnext_cartesian_v;
                current_cartesian_l = tempnext_cartesian_l;
                current_cartesian_suml += current_cartesian_l;
                //重新赋值减速量-以上面算的为准
            }
        }
        if(Speed_planning_step == 4) {
            //三段减速处理阶段
            last_dec_count_num = dec_count_num;

            if(dec_count_num <= decelera_t5) {
                current_cartesian_j = -limit_cartesian_jmax;

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                        + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = dec_start_a + current_cartesian_j*dec_count_num*current_cycle_tim;

                current_cartesian_v = dec_start_v + dec_start_a*dec_count_num*current_cycle_tim + current_cartesian_j * dec_count_num \
                        * dec_count_num * current_cycle_tim * current_cycle_tim / 2;

                current_cartesian_suml += current_cartesian_l;

            }else if(dec_count_num > decelera_t5 && dec_count_num <= decelera_t6+decelera_t5) {
                current_cartesian_j = 0;
                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2;
                current_cartesian_a = dec_start_a;
                current_cartesian_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim;

                current_cartesian_suml += current_cartesian_l;

            }else if(dec_count_num > decelera_t6+decelera_t5 && dec_count_num <= decelera_t6+decelera_t5+decelera_t7) {

                current_cartesian_j = limit_cartesian_jmax;

                current_cartesian_l = current_cartesian_v*current_cycle_tim + current_cartesian_a*current_cycle_tim*current_cycle_tim / 2 \
                        + current_cartesian_j * current_cycle_tim*current_cycle_tim*current_cycle_tim / 6;

                current_cartesian_a = dec_start_a + current_cartesian_j*(dec_count_num-dec_count_start)*current_cycle_tim;

                current_cartesian_v = dec_start_v + dec_start_a*(dec_count_num-dec_count_start)*current_cycle_tim + current_cartesian_j \
                        * (dec_count_num-dec_count_start)*(dec_count_num-dec_count_start)*current_cycle_tim*current_cycle_tim / 2;

                current_cartesian_suml += current_cartesian_l;

            }

            dec_count_num++;//不考虑插补余量-预计会有到达误差

            if(dec_count_num >= decelera_t5+decelera_t6+decelera_t7) {
                //插补计算结束
                interpol_finish = 1;
            }

            //阶段切换时初始位置保存
            if(last_dec_count_num <= decelera_t5 && dec_count_num > decelera_t5) {

                dec_start_a = current_cartesian_a;
                dec_start_v = current_cartesian_v;
                dec_count_start = decelera_t5;

            }else if(last_dec_count_num <= decelera_t5+decelera_t6 && dec_count_num > decelera_t5+decelera_t6) {

                dec_start_a = current_cartesian_a;
                dec_start_v = current_cartesian_v;
                dec_count_start = decelera_t5+decelera_t6 ;
            }
        }

        ratio = current_cartesian_suml / sum_cartesian_l;


        double theta = ratio * angle_total;

        Eigen::Vector3d pos = center + std::cos(theta) * n * radius + std::sin(theta) * m * radius;

        Eigen::Quaterniond quat_interp;
        quat_interp = quat_start.slerp(ratio, quat_end);

        Eigen::Matrix3d R = quat_interp.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX
#ifdef VISUALIZE_INTERP_RES
        fout << pos.x() << "," << pos.y() << "," << pos.z() << ","
             << quat_interp.x() << "," << quat_interp.y() << "," << quat_interp.z() << "," << quat_interp.w() << "\n";
#endif
        Kine6d pose;
        pose.X = pos.x();
        pose.Y = pos.y();
        pose.Z = pos.z();
        pose.A = euler[2];
        pose.B = euler[1];
        pose.C = euler[0];
        pose.fgR = 0;

        Kine6dSol q_sol;
        classic6dofInvKine(&pose, m_curJoints.data(), &q_sol);

        // 选最优解
        bool valid[8];
        int best = -1;
        float best_dist = 1e6;
        for (int k = 0; k < 8; ++k)
        {
            valid[k] = true;
            for (int j = 0; j < NUM_JOINTS; ++j)
            {
                if (q_sol.sol[k][j] < m_angleLimitMin[j] || q_sol.sol[k][j] > m_angleLimitMax[j])
                {
                    valid[k] = false;
                    break;
                }
            }
            if (valid[k])
            {
                float dist = 0;
                for (int j = 0; j < NUM_JOINTS; ++j)
                    dist += std::pow(m_curJoints[j] - q_sol.sol[k][j], 2);
                if (dist < best_dist)
                {
                    best = k;
                    best_dist = dist;
                }
            }
        }

        if (best >= 0)
        {
            std::array<float, NUM_JOINTS> target;
            for (int j = 0; j < NUM_JOINTS; ++j)
                target[j] = q_sol.sol[best][j];
            m_curJoints = target;

            printf("moveC, target Joints: %f %f %f %f %f %f %f\n",
                   target[0], target[1], target[2], target[3],target[4], target[5]);
            moveJoints(target);
        }

//        std::this_thread::sleep_for(std::chrono::milliseconds(10));

//        if (GlobalParams::isStop) break;
//        while (GlobalParams::isPause) {
//            std::this_thread::sleep_for(std::chrono::milliseconds(10));
//            if (GlobalParams::isStop) return;
//        }
    }

#ifdef VISUALIZE_INTERP_RES

    fout.close();
#endif
    
}

void Robot::moveCF(std::array<float, NUM_JOINTS> pose1, std::array<float, NUM_JOINTS> pose2, float speed)
{
    
}

/* 关节空间手动模式（连续和寸动）*/
void Robot::jogJ(int _mode, int _joint_index, int _direction)
{
    // 重置状态标志，开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
        
    // 更新当前关节状态
    // updateJointStates();
    
    // 点动速度设置
    const float jog_speed = 10.0f;  // 点动速度 (度/秒)
    // 微动角度设置
    const float micro_move_angle = (M_PI / 180.0f);  // 微动角度 (弧度)

    // 获取关节限位
    float joint_limit_max = m_angleLimitMax[_joint_index];  // 弧度
    float joint_limit_min = m_angleLimitMin[_joint_index];  // 弧度
    
    // 获取当前关节位置
    float current_pos = m_curJoints[_joint_index];
    //目标位置
    float target_pos = 0.0f;
    
    //判断连续还是微动
    if (_mode == 0)
    {
        // 根据方向确定目标位置
        if (_direction == 1) {
            target_pos = joint_limit_max;  // 正向运动到上限位
            
            // 检查是否已经在上限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_max_deg = joint_limit_max * 180.0f / M_PI;
            
            if (current_pos_deg >= limit_max_deg - 0.1f) {
                std::cout << "jogJ - 关节" << _joint_index << " 已在上限位附近，无需点动" << std::endl;
                return;
            }
        } else {
            target_pos = joint_limit_min;  // 负向运动到下限位
            
            // 检查是否已经在下限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_min_deg = joint_limit_min * 180.0f / M_PI;
            
            if (current_pos_deg <= limit_min_deg + 0.1f) {
                std::cout << "jogJ - 关节" << _joint_index << " 已在下限位附近，无需点动" << std::endl;
                return;
            }
        }
    }

    if (_mode == 1)
    {
        // 根据方向确定目标位置
        if (_direction == 1) {
            target_pos = current_pos + micro_move_angle;  // 正向微动
            
            // 检查是否已经在上限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_max_deg = joint_limit_max * 180.0f / M_PI;
            
            if (current_pos_deg >= limit_max_deg - 0.1f) {
                std::cout << "jogJ - 关节" << _joint_index << " 已在上限位附近，无需点动" << std::endl;
                return;
            }
        } else {
            target_pos = current_pos - micro_move_angle;  // 负向微动
            
            // 检查是否已经在下限位附近
            float current_pos_deg = current_pos * 180.0f / M_PI;
            float limit_min_deg = joint_limit_min * 180.0f / M_PI;
            
            if (current_pos_deg <= limit_min_deg + 0.1f) {
                std::cout << "jogJ - 关节" << _joint_index << " 已在下限位附近，无需点动" << std::endl;
                return;
            }
        }
    }
    
    
    // 构造目标关节位置数组 - 只移动指定关节，其他关节保持当前位置
    std::array<float, NUM_JOINTS> target_joints = m_curJoints;
    target_joints[_joint_index] = target_pos;
    
    // 调用moveJ函数执行点动到限位
    moveJ(target_joints, jog_speed);
    
    // 更新当前关节状态
    // updateJointStates();
    
    std::cout << "jogJ - 关节" << _joint_index << " 点动完成，当前位置: " 
              << m_curJoints[_joint_index] * 180.0f / M_PI << "度" << std::endl;
}

/* 笛卡尔空间手动模式（寸动）*/
void Robot::jogL(int mode, int axis, int _direction)
{
    // 重置状态标志，开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;

    // 更新当前关节状态
    // updateJointStates();
    
    // 获取当前末端位姿
    Kine6d current_pose;
    classic6dofForKine(m_curJoints.data(), &current_pose);
    
    // 点动参数设置
    const float linear_jog_distance = 30.0f;   // 线性轴点动距离 (mm)
    const float angular_jog_distance = 10.0f;  // 角度轴点动距离 (度)
    const float jog_speed = 30.0f;             // 点动速度
    
    // 计算目标位姿
    Kine6d target_pose = current_pose;
    float temp_dist = 0.0f;
    bool _flag = false;
    Kine6dSol _q;
    Kine6d _pose;
    if (mode == 0)          
    {
        switch (axis)
        {
            case 1:
            //直接给X轴工作空间最大限位，在这里循环判断，递增或递减目标位置的X值，直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.X = 850.0f;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.X -= 10.0f; // 如果逆解验证失败，递减X轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.X = -850.0f;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 && fabs(target_pose.Y - _pose.Y) > 1e-3 && fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.X += 10.0f; // 如果逆解验证失败，递增X轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                std::cout << "jogL - X轴移动 " << std::endl;
                break;
            case 2:
            //直接给Y轴工作空间最大限位，在这里循环判断，递增或递减目标位置的Y值，直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.Y = 850.0f;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Y -= 10.0f; // 如果逆解验证失败，递减Y轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.Y = -850.0f;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 && fabs(target_pose.Y - _pose.Y) > 1e-3 && fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Y += 10.0f; // 如果逆解验证失败，递增Y轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                std::cout << "jogL - Y轴移动 " << std::endl;               
                break;
            case 3:
            //直接给Z轴工作空间最大限位，在这里循环判断，递增或递减目标位置的Z值，直到不超出目标位置（误差10mm之内）
                if (_direction == 1)
                {
                    target_pose.Z = 900.0f;
                    while(!_flag)
                    {
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 || fabs(target_pose.Y - _pose.Y) > 1e-3 || fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Z -= 10.0f; // 如果逆解验证失败，递减Z轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                    
                if (_direction == 0)
                {
                    target_pose.Z = -700.0f;
                    while (!_flag)
                    {   
                        classic6dofInvKine(&target_pose, m_curJoints.data(), &_q);
                        bool valid[8];
                        int validCnt = 0;
                        for (int i = 0; i < 8; i++) {
                            valid[i] = true;
                            for (int j = 0; j < NUM_JOINTS; j++) {
                                if (_q.sol[i][j] > m_angleLimitMax[j] || _q.sol[i][j] < m_angleLimitMin[j]) {
                                    valid[i] = false;
                                    break;
                                }
                            }
                            if (valid[i]) validCnt++;
                        }
                        if (validCnt > 0) 
                        {
                            // 选择距离当前关节位置最近的解
                            float minDist = std::numeric_limits<float>::max();
                            int bestIndex = -1;
                            for (int i = 0; i < 8; i++) 
                            {
                                if (!valid[i]) continue;

                                float dist = 0.0f;
                                for (int j = 0; j < NUM_JOINTS; j++)
                                {
                                    float d = m_curJoints[j] - _q.sol[i][j];
                                    dist += d * d;
                                }

                                if (dist < minDist) {
                                    minDist = dist;
                                    bestIndex = i;
                                }
                            }
                            if (bestIndex >= 0) 
                            {
                                std::array<float, NUM_JOINTS> joint_f;
                                for (int i = 0; i < NUM_JOINTS; i++) 
                                {
                                    joint_f[i] = _q.sol[bestIndex][i];//单位弧度
                                }
                                classic6dofForKine(joint_f.data(), &_pose); //正解判断结束位姿是否等于输入的目标位姿
                                if (fabs(target_pose.X - _pose.X) > 1e-3 && fabs(target_pose.Y - _pose.Y) > 1e-3 && fabs(target_pose.Z - _pose.Z) > 1e-3) {
                                    target_pose.Z += 10.0f; // 如果逆解验证失败，递增Z轴位置
                                }
                                else
                                {
                                    _flag = true;
                                }
                            } 
                        }
                        else
                        {
                            std::cerr << "No valid solution found!" << std::endl;
                            return;
                        }
                    }
                }
                std::cout << "jogL - Z轴移动 " << std::endl;
                break;
            case 4:
                target_pose.A = (_direction == 1) ? (175.0f * M_PI / 180.0f) : -(175.0f * M_PI / 180.0f);
                std::cout << "jogL - A轴旋转 " << std::endl; 
                break;
            case 5:
                //Pitch俯仰角在接近正负90度和正负180度时姿态奇异，所以这里先限定到正负85度
                target_pose.B = (_direction == 1) ? (85.0f * M_PI / 180.0f) : -(85.0f * M_PI / 180.0f);
                std::cout << "jogL - B轴旋转 " << std::endl; 
                break;
            case 6:
                target_pose.C = (_direction == 1) ? (175.0f * M_PI / 180.0f) : -(175.0f * M_PI / 180.0f);
                std::cout << "jogL - C轴旋转 " << std::endl; 
                break;
            default:
                break;
        }
    }
    if (mode == 1)
    {
    // 根据轴和方向设置目标位姿
        float increment = 0.0f;
        switch (axis) {
            case 1:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.X += increment;
                std::cout << "jogL - X轴移动 " << increment << "mm" << std::endl;
                break;
                
            case 2:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.Y += increment;
                std::cout << "jogL - Y轴移动 " << increment << "mm" << std::endl;
                break;

            case 3:
                increment = (_direction == 1) ? linear_jog_distance : -linear_jog_distance;
                target_pose.Z += increment;
                std::cout << "jogL - Z轴移动 " << increment << "mm" << std::endl;
                break;
                
            case 4:
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.A += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - A轴旋转 " << increment << "度" << std::endl;
                break;
                
            case 5:
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.B += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - B轴旋转 " << increment << "度" << std::endl;
                break;
                
            case 6:  // Yaw轴（绕Z轴旋转）
                increment = (_direction == 1) ? angular_jog_distance : -angular_jog_distance;
                target_pose.C += increment * M_PI / 180.0f;  // 转换为弧度
                std::cout << "jogL - C轴旋转 " << increment << "度" << std::endl;
                break;
        }
    } 
    // 构造目标位姿数组
    std::array<float, 6> target_pose_array = {
        target_pose.X, target_pose.Y, target_pose.Z,
        target_pose.A, target_pose.B, target_pose.C
    };
    
    // std::cout << "target_pose.fR = " << target_pose.fgR << std::endl;
    std::cout << "jogL - 目标位姿: X=" << target_pose.X 
              << " Y=" << target_pose.Y << " Z=" << target_pose.Z
              << " A=" << target_pose.A * 180.0f / M_PI
              << " B=" << target_pose.B * 180.0f / M_PI  
              << " C=" << target_pose.C * 180.0f / M_PI << std::endl;

#if 0    
    if (axis == 1 || axis == 2 || axis == 3 || axis == 4 || axis == 5 || axis == 6)
    {
        // 调用moveL函数执行笛卡尔空间运动
        moveL(target_pose_array, jog_speed);
    }
    else if (axis == 7)
    {
        // 1. 逆解目标位姿各关节角度
        Kine6dSol q_sol;
        float q_last[6] = {0};
        target_pose.fgR = 0;  // 确保fgR为0，避免影响逆解
        classic6dofInvKine(&target_pose, q_last, &q_sol);

        // 2. 选择最优逆解（距离当前关节位置最近且在限位范围内）
        bool valid[8];
        int validCnt = 0;
        for (int i = 0; i < 8; i++) {
            valid[i] = true;
            for (int j = 0; j < NUM_JOINTS; j++) {
                if (q_sol.sol[i][j] > m_angleLimitMax[j] || q_sol.sol[i][j] < m_angleLimitMin[j]) {
                    valid[i] = false;
                    break;
                }
            }
            if (valid[i]) validCnt++;
        }

        int bestIndex = -1;
        if (validCnt > 0) {
            float minDist = std::numeric_limits<float>::max();
            for (int i = 0; i < 8; i++) {
                if (!valid[i]) continue;
                float dist = 0.0f;
                for (int j = 0; j < NUM_JOINTS; j++) {
                    float d = m_curJoints[j] - q_sol.sol[i][j];
                    dist += d * d;
                }
                if (dist < minDist) {
                    minDist = dist;
                    bestIndex = i;
                }
            }
        }

        // 3. 调用moveJ函数执行关节空间运动
        if (bestIndex >= 0) {
            std::array<float, NUM_JOINTS> targetJoints;
            for (int i = 0; i < NUM_JOINTS; i++) {
                targetJoints[i] = q_sol.sol[bestIndex][i];
            }
            moveJ(targetJoints, 10);
        }
        else {
            std::cerr << "jogL - 未找到有效逆解，无法执行姿态点动！" << std::endl;
        }
    }
#endif    
    // 调用moveL函数执行笛卡尔空间运动
    moveL(target_pose_array, jog_speed);
    
    // 更新当前关节状态
    // updateJointStates();
    
    std::cout << "jogL - 笛卡尔点动完成" << std::endl;
}

void Robot::moveJoints(const std::array<float, NUM_JOINTS>& _joints)
{
    // 关节空间插补指令
    // 直接发送到电机
    LowLevelCommand montor_cmd = {
        .mode = CSP,
    };
    
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        montor_cmd.joint_pos[i] = _joints[i] * (500000.0 / M_PI * 13.1072);   //关节角度转换到电机脉冲
    }

    // 入队列
    while (!GlobalParams::joint_commands.try_enqueue(montor_cmd))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        HighLevelCommand cmd;
        if (shm().high_prio_cmd_queue.pop(cmd))
        {
            handleHighPriorityCommand(cmd);
        }
    }
}


void Robot::homing()
{

    std::cout << "开始回零操作...\n";

    // 重置状态标志，开始新的运动
    GlobalParams::isStop = false;
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
    
    // 更新当前关节状态
    // updateJointStates();
    
    // 执行回零运动
    moveJ(REST_JOINT, DEFAULT_JOINT_SPEED);
    
    std::cout << "回零操作完成\n"<< std::endl;

}

void Robot::pause()
{
    std::cout << "暂停运动...\n";
    GlobalParams::isPause = true;
    GlobalParams::isResume = false;
}

void Robot::resume()
{
    std::cout << "恢复运动...\n";
    GlobalParams::isPause = false;
    GlobalParams::isResume = true;
}

void Robot::stop()
{
    std::cout << "停止运动...\n";
    GlobalParams::isStop = true;
    // 重置所有相关状态标志，确保能从任何状态退出
    GlobalParams::isPause = false;
    GlobalParams::isResume = false;
}

void Robot::updatePose()
{
    RobotState state;
    shm().state_buffer.read(state);

    Kine6d cur_pose;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        m_curJoints[i] = state.joint_state[i].position;
    }
    classic6dofForKine(m_curJoints.data(), &cur_pose);
    std::cout << "cur_pose: ";
    for (int i = 0; i < 6; i++)
    {
        std::cout << "cur_pose: "
          << cur_pose.X << " "
          << cur_pose.Y << " "
          << cur_pose.Z << " "
          << cur_pose.A << " "
          << cur_pose.B << " "
          << cur_pose.C << std::endl;
    }
}

void Robot::updateJointStates()
{
    RobotState state;
    shm().state_buffer.read(state);
 
    // 更新当前关节位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
        m_curJoints[i] = state.joint_state[i].position;
        m_curVelocity[i] = state.joint_state[i].velocity;
        m_curTorque[i] = state.joint_state[i].torque;

        // moveFlag = moveFlag || state.joint_state[i].motor_state;
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

    if (shm().high_prio_cmd_queue.pop(cmd))
    {
        handleHighPriorityCommand(cmd);

        return;
    }


    if (shm().cmd_queue.pop(cmd))
    {

        handleNormalCommand(cmd);
    }
}

void Robot::handleHighPriorityCommand(const HighLevelCommand &_cmd)
{
    switch (_cmd.command_type)
    {
    case HighLevelCommandType::Homing:
        if (!GlobalParams::isMoving && !GlobalParams::isStop)
        {
            std::cout << "Homing!\n";
            homing();
        }
        break;
    case HighLevelCommandType::Stop:
    {
        std::cout << "Stop!\n";
        stop();
        // 清空普通命令队列
        HighLevelCommand dummy;
        while (shm().cmd_queue.pop(dummy)) {
            // 循环弹出直到队列为空
        }
        break;
    }
    case HighLevelCommandType::Pause:
        if (GlobalParams::isMoving && !GlobalParams::isStop)
        {
            std::cout << "Pause!\n";
            pause();  
        }
        break;
    case HighLevelCommandType::Resume:
    {
        if (!GlobalParams::isMoving && GlobalParams::isPause && !GlobalParams::isStop)
        {
            std::cout << "Resume!\n";
            resume();
        }
        break;
    }

//        case HighLevelCommandType::SetParm:
//        {
//            handleParameterOrder(cmd);
//            break;
//        }
    default:
        break;
    }
}


void Robot::handleNormalCommand(const HighLevelCommand &cmd)
{
    // 检查机器人状态，如果处于暂停状态，不执行运动指令
    if (GlobalParams::isPause) {
        std::cout << "机器人处于暂停状态，忽略运动指令\n";
        return;
    }

    switch (cmd.command_type)
    {      
        case HighLevelCommandType::MoveJ:
        {
            std::array<float, NUM_JOINTS> pos;
            std::copy(std::begin(cmd.movej_params.target_joint_pos),
                    std::end(cmd.movej_params.target_joint_pos),
                    pos.begin());
            float speed = cmd.movej_params.velocity;

//            updateJointStates();
            moveJ(pos, speed);
            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::MoveL:
        {
            std::array<float, 6> pose;
            std::copy(std::begin(cmd.movel_params.target_pose),
                    std::end(cmd.movel_params.target_pose),
                    pose.begin());
            float speed = cmd.movel_params.velocity;

            // updateJointStates();
            moveL(pose, speed);
            // updateJointStates();
            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::MoveC:
        {
            std::array<float, 6> pose_mid;
            std::array<float, 6> pose_end;
            std::copy(std::begin(cmd.movec_params.via_pose),
                      std::end(cmd.movec_params.via_pose),
                      pose_mid.begin());
            std::copy(std::begin(cmd.movec_params.target_pose),
                      std::end(cmd.movec_params.target_pose),
                      pose_end.begin());
            float speed = cmd.movec_params.velocity;

            moveC(pose_mid, pose_end, speed);
            break;
        }
        case HighLevelCommandType::MoveCF:
        {
            std::array<float, 6> pose1;
            std::array<float, 6> pose2;
            std::copy(std::begin(cmd.movecf_params.pose1),
                    std::end(cmd.movecf_params.pose1),
                    pose1.begin());
            std::copy(std::begin(cmd.movecf_params.pose2),
                    std::end(cmd.movecf_params.pose2),
                    pose2.begin());
            float speed = cmd.movecf_params.velocity;

            moveCF(pose1, pose2, speed);
            break;
        }
        case HighLevelCommandType::MoveP:
        {
            break;
        }
        case HighLevelCommandType::SetParm:
        {
            handleParameterOrder(cmd);
            break;
        }
        
        case HighLevelCommandType::JogJ:
        {
            int mode = cmd.jogj_params.mode;
            int joint_index = cmd.jogj_params.joint_index;
            int direction = cmd.jogj_params.direction;

            std::cout << "JogJ - " << (mode == 0 ? "连续" : "微动") << " 关节" << joint_index << " 方向: " << direction << std::endl;

            // updateJointStates();
            jogJ(mode, joint_index, direction);
            // updateJointStates();
            std::cout << std::endl;
            break;
        }
        case HighLevelCommandType::JogL:
        {
            int mode = cmd.jogl_params.mode;
            int axis = cmd.jogl_params.axis;
            int direction = cmd.jogl_params.direction;

            std::cout << "JogL - " << (mode == 0 ? "连续" : "微动") << " 轴" << axis << " 方向: " << direction << std::endl;
            // updateJointStates();
            jogL(mode, axis, direction);
            // updateJointStates();
            std::cout << std::endl;
            break;
        }
        
        default:
            break;
    }
}

void Robot::handleParameterOrder(const HighLevelCommand &_cmd)
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



