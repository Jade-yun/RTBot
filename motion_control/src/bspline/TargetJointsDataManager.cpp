#include "bspline/TargetJointsDataManager.h"
#include <iostream>
#include <stdexcept>
#include <iterator>
#include <fstream>
#include <list>
#include <array>
#include <string>
#include <cmath>
#include <limits>


// static JointDataManager *m_instance;
JointDataManager *JointDataManager::m_instance = nullptr;

// 内部结构体构造函数实现
JointDataManager::JointData::JointData(
    const std::array<float, 6> &joints,
    int flagVal0,
    int flagVal1,
    int flagVal2) : targetJoints(joints), flag0(flagVal0), flag1(flagVal1), flag2(flagVal2)
{
}

// 单例模式：获取唯一实例
JointDataManager *JointDataManager::getInstance()
{
    if (!m_instance)
        m_instance = new JointDataManager();
    return m_instance;
}

void JointDataManager::set_speed(float x)
{
    speed = x;
}

// 辅助函数：判断是否为分隔符（全0数据）
bool JointDataManager::is_delimiter(const JointData& data) const
{
    // 检查6个float是否全为0
    for (float val : data.targetJoints)
    {
        if (val != 0.0f) {
            return false;
        }
    }
    // 检查3个int是否全为0
    return (data.flag0 == 0 && data.flag1 == 0 && data.flag2 == 0);
}

std::pair<JointDataManager::JointData, JointDataManager::JointData> JointDataManager::get_singular_area()
{
    // 定义一个"空标志"（全0数据），用于表示没有更多区域
    std::array<float, 6> zero_joints{}; // 自动初始化为全0
    JointData empty_flag(zero_joints, 0, 0, 0);


    // 1. 如果列表为空，直接返回空标志（这是唯一的提前返回情况）
    if (m_jointDataList.empty()) {
        return {empty_flag, empty_flag};
    }

    // 2. 如果当前迭代器在末尾，重置到开头（可选：根据是否需要循环读取决定）
    // 若不需要循环，可删除此句，保持从末尾开始（会自然返回空）
    if (m_currentIt == m_jointDataList.end()) {
        m_currentIt = m_jointDataList.begin();
    }

    // 3. 寻找起始分隔符（允许从当前位置继续，即使之前在末尾）
    std::list<JointData>::iterator start_delimiter = m_jointDataList.end();
    while (m_currentIt != m_jointDataList.end())
    {
        if (is_delimiter(*m_currentIt))
        {
            start_delimiter = m_currentIt;
            ++m_currentIt; // 移动到分隔符后（可能是区域起点）
            break;
        }
        ++m_currentIt;
    }

    // 若未找到起始分隔符，说明已遍历完所有数据
    if (start_delimiter == m_jointDataList.end()) {
        return {empty_flag, empty_flag};
    }

    // 4. 寻找结束分隔符
    std::list<JointData>::iterator end_delimiter = m_jointDataList.end();
    std::list<JointData>::iterator area_end = m_jointDataList.end();
    while (m_currentIt != m_jointDataList.end())
    {
        if (is_delimiter(*m_currentIt)) {
            end_delimiter = m_currentIt;
            if (m_currentIt != m_jointDataList.begin())
            {
                area_end = std::prev(m_currentIt);
            }
            ++m_currentIt; // 移动到结束分隔符后，为下次调用准备
            break;
        }
        ++m_currentIt;
    }

    // 校验区域有效性
    if (end_delimiter == m_jointDataList.end() || area_end == m_jointDataList.end())
    {
        return {empty_flag, empty_flag};
    }

    // 5. 区域起点是起始分隔符的下一个元素
    std::list<JointData>::iterator area_start = std::next(start_delimiter);

    // 确保起点在区域内（防止两个分隔符相邻）
    if (area_start == end_delimiter) {
        return {empty_flag, empty_flag}; // 空区域（分隔符直接相邻）
    }

    return {*area_start, *area_end};
}





bool JointDataManager::collect_singular_area()
{
    float tmp_angleLimitMax[6];
    float tmp_angleLimitMin[6];

    tmp_angleLimitMax[0] = M_PI * 175.0f / 180.0f;
    tmp_angleLimitMin[0] = -M_PI * 175.0f / 180.0f;
    tmp_angleLimitMax[1] = M_PI_2 * 120.0f / 90.0f;
    tmp_angleLimitMin[1] = -M_PI_2 * 120.0f / 90.0f;
    tmp_angleLimitMax[2] = M_PI * 175.0f / 180.0f;
    tmp_angleLimitMin[2] = M_PI * 5.0f / 180.0f;;
    tmp_angleLimitMax[3] = M_PI * 175.0f / 180.0f;
    tmp_angleLimitMin[3] = -M_PI * 175.0f / 180.0f;
    tmp_angleLimitMax[4] = M_PI * 130.0f / 180.0f;
    tmp_angleLimitMin[4] = -M_PI * 130.0f / 180.0f;
    tmp_angleLimitMax[5] = M_PI;
    tmp_angleLimitMin[5] = -M_PI;


    std::pair<JointDataManager::JointData, JointDataManager::JointData> area = JointDataManager::getInstance()->get_singular_area();
    JointDataManager::JointData& start = area.first;  // 区域起始元素
    JointDataManager::JointData& end = area.second;   // 区域结束元素
    std::cout<< "***********起始于***********"<< std::endl;
    JointDataManager::getInstance()->print(start);
    std::cout<< "***********结束于***********"<< std::endl;
    JointDataManager::getInstance()->print(end);

    singular_pointsList.emplace_back(start.targetJoints,start.flag0,start.flag1,start.flag2);
    singular_pointsList.emplace_back(end.targetJoints,end.flag0,end.flag1,end.flag2);

    if(JointDataManager::getInstance()->is_delimiter(start) && JointDataManager::getInstance()->is_delimiter(end))
    {
        return false;
    }



    Kine6d singalur_end_pose;                  // 奇异区域结束位姿
    Kine6dSol q_sol; // 逆解得到的关节角度
    std::array<float, 6> targetJoints{};

    if(start.targetJoints[0] >= 0)
    {
        start.targetJoints[0] -= 3.14f;
    }
    else
    {
        start.targetJoints[0] += 3.14f;
    }

    std::array<float, NUM_JOINTS> tmp_curJoints = {start.targetJoints[0],0,0,0,0,0};
    classic6dofForKine(end.targetJoints.data(), &singalur_end_pose);

    classic6dofInvKine(&singalur_end_pose, tmp_curJoints.data(), &q_sol);  // // 奇异区域结束位姿的8组解

    /* 选择逆解最优解 */
    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;
        for (int j = 1; j <= 6; j++)
        {
            if (q_sol.sol[i][j - 1] > tmp_angleLimitMax[j - 1] ||
                q_sol.sol[i][j - 1] < tmp_angleLimitMin[j - 1])
            {
                valid[i] = false;
                continue;
            }
        }
        if (valid[i])
            validCnt++;
    }

    if (validCnt > 0)
    {

        // 选择距离当前关节位置最近的解
        float minDist = std::numeric_limits<float>::max();
        int bestIndex = -1;

        for (int i = 0; i < 8; i++)
        {
            if (!valid[i])
                continue;
            if(q_sol.sol[i][0] == end.targetJoints[0])
                continue;

            float dist = 0.0f;
            for (int j = 0; j < NUM_JOINTS; j++)
            {
                float d = start.targetJoints[j] - q_sol.sol[i][j];   // 筛选出和奇异区域开始差距最小的一组解
                dist += d * d;
            }
            if (dist < minDist)
            {
                minDist = dist;
                bestIndex = i;
            }
        }

        if (bestIndex >= 0)
        {

            for (int i = 0; i < 6; i++)
            {
                targetJoints[i] = q_sol.sol[bestIndex][i]; // 单位弧度
            }
            // moveJoints(targetJoints);
        }
    }


    JointData tar_point(targetJoints,1,1,1);
    singular_pointsList.pop_back();
    singular_pointsList.emplace_back(tar_point.targetJoints,1,1,1);

    return true;

}

void JointDataManager::combine(int m_seconds)
{
    // 临时列表为空时直接返回
    if (tmp_jointDataList.empty())
    {
        return;
    }

    size_t tmpSize = tmp_jointDataList.size();

    if (m_seconds <= 0)
    {
        return;
    }


    if (tmpSize <= static_cast<size_t>(m_seconds))
    {
        m_jointDataList.insert(
            m_jointDataList.end(),
            tmp_jointDataList.begin(),
            tmp_jointDataList.end()
        );
    }
    // 情况2：临时列表元素数量超过m_seconds，需要均匀抽样
    else {
        // 将list转为vector便于随机访问（list不支持索引访问）
        std::vector<JointData> tmpVec(tmp_jointDataList.begin(), tmp_jointDataList.end());
        size_t sampleCount = static_cast<size_t>(m_seconds);

        for (size_t i = 0; i < sampleCount; ++i)
        {
            size_t index;
            if (sampleCount == 1)
            {
                // 仅需抽样1个时，取中间位置元素
                index = (tmpSize - 1) / 2;
            }
            else
            {
                // 均匀抽样算法：通过线性映射计算索引，确保覆盖首尾且分布均匀
                // 公式：index = i * (总数量-1) / (抽样数量-1)
                index = static_cast<size_t>(
                    (i * static_cast<double>(tmpSize - 1)) / (sampleCount - 1)
                );
            }
            // 插入抽样元素到目标列表尾部
            m_jointDataList.push_back(tmpVec[index]);
        }
    }

    // 清空临时列表，避免下次调用时重复处理
    tmp_jointDataList.clear();
}

std::pair<std::vector<int>, int> JointDataManager::filtering_solution
(float interp_X, float interp_Y, float interp_Z, Kine6dSol q_sol,std::array<float, NUM_JOINTS> m_angleLimitMax,std::array<float, NUM_JOINTS> m_angleLimitMin,std::array<float, NUM_JOINTS> m_curJoints)
{
    /* 选择逆解最优解 */
    bool valid[8];
    int bestIndex = -1;
    std::vector<int> is_over_limit = {0, 0, 0, 0, 0, 0, 0, 0}; // 初始化为0
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;
        is_over_limit[i] = 0;

        for (int j = 1; j <= 6; j++)
        {
            if (q_sol.sol[i][j - 1] > m_angleLimitMax[j - 1] ||
                q_sol.sol[i][j - 1] < m_angleLimitMin[j - 1])
            {
                valid[i] = false;
                is_over_limit[i] = 1;
                continue;
            }
        }
        if (valid[i])
            validCnt++;
    }


    if(validCnt <= 0)
    {
        return {is_over_limit, bestIndex};
    }


    float q;
    if(interp_X == interp_Y && interp_Y == 0)
    {
        return {is_over_limit, bestIndex};
    }
    else
    {
        float q = atan2(interp_Y,interp_X);
    }


    float minDist = std::numeric_limits<float>::max();

    for (int i = 0; i < 8; i++)
    {
        if (!valid[i])
            continue;
        // 排除反向的情况
        if(q * q_sol.sol[i][0] < 0)
        {
            continue;
        }

        float dist = 0.0f;
        for (int j = 0; j < NUM_JOINTS; j++)
        {
            float d = m_curJoints[j] - q_sol.sol[i][j];
            dist += d * d;
        }
        if (dist < minDist)
        {
            minDist = dist;
            bestIndex = i;
        }
    }
    return {is_over_limit, bestIndex};

}

void JointDataManager::save_all_change_CSV(const std::string &filename) const
{
    // 尝试打开文件，使用截断模式(覆盖现有文件)
    std::ofstream outFile(filename, std::ios::trunc);

    // 检查文件是否成功打开
    if (!outFile.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入CSV表头
    outFile << "joint0,joint1,joint2,joint3,joint4,joint5,flag0,flag1,flag2,index\n";

    // 检查容器是否为空
    if (all_changed_jointDataList.empty())
    {
        std::cout << "警告：没有数据可保存，容器为空。" << std::endl;
        outFile.close();
        return;
    }

    // 遍历所有关节数据并写入文件
    size_t count = 0;
    for (const auto &data : all_changed_jointDataList)
    {
        count = count % 1;
        // 抽样
        if (count == 0)
        {
            // 写入6个关节角度
            for (size_t i = 0; i < data.targetJoints.size(); ++i)
            {
                outFile << data.targetJoints[i];
                if (i < data.targetJoints.size() - 1)
                {
                    outFile << ",";
                }
            }
            // 写入三个标志位
            outFile << "," << data.flag0
                    << "," << data.flag1
                    << "," << data.flag2<< "\n";
        }
        count++;
    }

    // 关闭文件
    outFile.close();
    // 输出保存信息
    std::cout << "成功保存 " << count << " 条数据到 " << filename << std::endl;
}

void JointDataManager::save_all_CSV(const std::string &filename) const
{
    // 尝试打开文件，使用截断模式(覆盖现有文件)
    std::ofstream outFile(filename, std::ios::trunc);

    // 检查文件是否成功打开
    if (!outFile.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入CSV表头
    outFile << "joint0,joint1,joint2,joint3,joint4,joint5,flag0,flag1,flag2,index\n";

    // 检查容器是否为空
    if (all_jointDataList.empty())
    {
        std::cout << "警告：没有数据可保存，容器为空。" << std::endl;
        outFile.close();
        return;
    }

    // 遍历所有关节数据并写入文件
    size_t count = 0;
    for (const auto &data : all_jointDataList)
    {
        count = count % 1;
        // 抽样
        if (count == 0)
        {
            // 写入6个关节角度
            for (size_t i = 0; i < data.targetJoints.size(); ++i)
            {
                outFile << data.targetJoints[i];
                if (i < data.targetJoints.size() - 1)
                {
                    outFile << ",";
                }
            }
            // 写入三个标志位
            outFile << "," << data.flag0
                    << "," << data.flag1
                    << "," << data.flag2<< "\n";
        }
        count++;
    }

    // 关闭文件
    outFile.close();
    // 输出保存信息
    std::cout << "成功保存 " << count << " 条数据到 " << filename << std::endl;
}

// save_CSV()函数实现
void JointDataManager::save_CSV(const std::string &filename) const
{
    // 尝试打开文件，使用截断模式(覆盖现有文件)
    std::ofstream outFile(filename, std::ios::trunc);

    // 检查文件是否成功打开
    if (!outFile.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入CSV表头
    outFile << "joint0,joint1,joint2,joint3,joint4,joint5,flag0,flag1,flag2,index\n";

    // 检查容器是否为空
    if (m_jointDataList.empty())
    {
        std::cout << "警告：没有数据可保存，容器为空。" << std::endl;
        outFile.close();
        return;
    }

    // 遍历所有关节数据并写入文件
    size_t count = 0;
    for (const auto &data : m_jointDataList)
    {
        count = count % 1;
        // 抽样
        if (count == 0)
        {
            // 写入6个关节角度
            for (size_t i = 0; i < data.targetJoints.size(); ++i)
            {
                outFile << data.targetJoints[i];
                if (i < data.targetJoints.size() - 1)
                {
                    outFile << ",";
                }
            }
            // 写入三个标志位
            outFile << "," << data.flag0
                    << "," << data.flag1
                    << "," << data.flag2<< "\n";
        }
        count++;
    }

    // 关闭文件
    outFile.close();
    // 输出保存信息
    std::cout << "成功保存 " << count << " 条数据到 " << filename << std::endl;
}
// save_CSV()函数实现
void JointDataManager::singular_changed_save_CSV(const std::string &filename) const
{
    // 尝试打开文件，使用截断模式(覆盖现有文件)
    std::ofstream outFile(filename, std::ios::trunc);

    // 检查文件是否成功打开
    if (!outFile.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入CSV表头
    outFile << "joint0,joint1,joint2,joint3,joint4,joint5,flag0,flag1,flag2\n";

    // 检查容器是否为空
    if (tmp_jointDataList.empty())
    {
        std::cout << "警告：没有数据可保存，容器为空。" << std::endl;
        outFile.close();
        return;
    }

    // 遍历所有关节数据并写入文件
    size_t count = 0;
    for (const auto &data : tmp_jointDataList)
    {

        // 抽样
        if (count % 100 == 0)
        {
            // 写入6个关节角度
            for (size_t i = 0; i < data.targetJoints.size(); ++i)
            {
                outFile << data.targetJoints[i];
                if (i < data.targetJoints.size() - 1)
                {
                    outFile << ",";
                }
            }

            // 写入三个标志位
            outFile << "," << data.flag0
                    << "," << data.flag1
                    << "," << data.flag2 << "\n";
        }
//        print()
        count++;
    }

    // 关闭文件
    outFile.close();
    // 输出保存信息
    std::cout << "成功保存 " << count << " 条数据到 " << filename << std::endl;
}
void JointDataManager::singular_points_save_CSV(const std::string &filename) const
{
    // 打开文件（输出模式，覆盖已有文件）
    std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
    if (!ofs.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filename); // 处理文件打开失败
    }

    try
    {
        // 写入CSV表头（与JointData成员一一对应）
        ofs << "targetJoint1,targetJoint2,targetJoint3,"
            << "targetJoint4,targetJoint5,targetJoint6,"
            << "flag0,flag1,flag2\n"; // 换行

        int count = 0;
        // 遍历所有含-1标志位的迭代器
        for (const auto &it : m_negativeOneIterators)
        {
            // 抽样
            if (count % 10 == 0)
            {
                // 解引用迭代器，获取当前JointData
                const JointData &data = *it;

                // 写入6个目标关节角度（用逗号分隔）
                for (size_t i = 0; i < 6; ++i)
                {
                    // 最后一个关节角度后不加逗号，其他加逗号
                    ofs << data.targetJoints[i] << (i < 5 ? "," : "");
                }
                ofs << ","; // 关节角度与标志位之间的分隔

                // 写入3个标志位（用逗号分隔）
                ofs << data.flag0 << ","
                    << data.flag1 << ","
                    << data.flag2; // 最后一个标志位后不加逗号

                ofs << "\n"; // 一行结束
            }

            count++;
        }

        ofs.close(); // 关闭文件
        std::cout << "奇异点数据已保存到: " << filename << "（共" << m_negativeOneIterators.size() << "条）" << std::endl;
    }
    catch (...)
    {
        ofs.close(); // 异常时确保文件关闭
        throw;       // 重新抛出异常，让调用者处理
    }
}

// 添加数据实现
void JointDataManager::tmpaddData(
    const std::array<float, 6> &joints,
    int flag0,
    int flag1,
    int flag2)
{
    tmp_jointDataList.emplace_back(joints, flag0, flag1, flag2);
}
// 清空数据实现
void JointDataManager::tmpclear()
{
    tmp_jointDataList.clear();
}
// 检测奇异点

std::tuple<JointDataManager::JointData, JointDataManager::JointData, int, int> JointDataManager::find_singular_points() const
{
    // 处理空容器情况（无任何含-1标志位的元素）
    if (m_negativeOneIterators.empty())
    {
        // 返回默认空数据（可根据需求改为抛出异常或其他处理）
        return {
            JointData({0, 0, 0, 0, 0, 0}, -1, -1, -1),
            JointData({0, 0, 0, 0, 0, 0}, -1, -1, -1),
            -1,
            -1};
    }

    // 第一个连续区间的开头：默认是第一个元素
    auto start_it = m_negativeOneIterators[0];
    auto end_it = start_it; // 结尾先初始化为开头

    // 遍历 m_negativeOneIterators，查找连续区间
    for (size_t i = 1; i < m_negativeOneIterators.size(); ++i)
    {
        // 当前元素的迭代器
        auto current_it = m_negativeOneIterators[i];
        // 上一个元素的迭代器
        auto prev_it = m_negativeOneIterators[i - 1];

        // 计算两个元素在 m_jointDataList 中的全局索引差
        // 若差为1，说明连续；否则说明区间中断
        size_t prev_index = std::distance(m_jointDataList.begin(), prev_it);
        size_t curr_index = std::distance(m_jointDataList.begin(), current_it);

        if (curr_index - prev_index == 1)
        {
            // 连续，更新区间结尾为当前元素
            end_it = current_it;
        }
        else
        {
            // 不连续，第一个区间已找到，直接退出循环
            break;
        }
    }

    // 计算start_index和end_index
    int start_index = std::distance(m_jointDataList.begin(), start_it);
    int end_index = std::distance(m_jointDataList.begin(), end_it);

    // 返回第一个连续区间的开头和结尾数据
    return {*start_it, *end_it, start_index, end_index};
}

std::tuple<JointDataManager::JointData, JointDataManager::JointData, int, int> JointDataManager::find_Mutation(float distance_max,int start_index)
{
    // 检查列表是否有足够的数据进行比较
    if (m_jointDataList.size() < 2)
    {
        throw std::runtime_error("Not enough data in m_jointDataList to check for mutations");
    }

    // 检查start_index合法性
    if (start_index < 0)
    {
        throw std::invalid_argument("start_index cannot be negative");
    }
    // 最大有效start_index为size-2（因为需要next_it存在）
    if (static_cast<size_t>(start_index) >= m_jointDataList.size() - 1)
    {
        throw std::out_of_range("start_index is too large (no enough subsequent elements)");
    }

    // 将迭代器定位到start_index位置
    auto it = m_jointDataList.begin();
    std::advance(it, start_index); // 移动迭代器到start_index处
    auto next_it = std::next(it);  // 下一个元素的迭代器

    int index = start_index; // 从start_index开始计数

    // 从start_index位置开始遍历后续相邻的数据对
    while (next_it != m_jointDataList.end())
    {

        bool has_exceed = false;

        // 检查每一对对应的关节值差异
        for (size_t i = 0; i < 5; ++i)
        {
            float diff = std::fabs(it->targetJoints[i] - next_it->targetJoints[i]);
            if (diff > distance_max)
            {
                has_exceed = true;
                break;
            }
        }

        if (has_exceed)
        {
            return std::make_tuple(*it, *next_it, index, index + 1);
        }

        // 移动到下一对
        ++it;
        ++next_it;
        ++index;
    }

    // 没有发现超过阈值的情况
    std::array<float, 6> tmp = {0,0,0,0,0,0};
    return std::make_tuple(JointData(tmp,1,1,1), JointData(tmp,1,1,1), -1, -1);
}

// 辅助函数：计算点到原点的距离
float distance_from_origin(float x, float y) {
    return std::sqrt(x*x + y*y);
}


std::tuple<int, int> JointDataManager::find_arm_point(float distance)
{
    if (m_jointDataList.size() < 2) {
        return std::make_tuple(-1, -1); // 点数不足，无法形成直线
    }

    int first_intersect = -1;
    int second_intersect = -1;
    bool found_first = false;

    Kine6d_ prev_pose, cur_pose;
    float prev_dist, cur_dist;

    // 获取第一个点的距离
    auto it_prev = m_jointDataList.begin();
    classic6dofForKine(it_prev->targetJoints.data(), &prev_pose);
    prev_dist = distance_from_origin(prev_pose.X, prev_pose.Y);

    int index = 1; // 从第二个点开始索引（1基）
    for (auto it = std::next(m_jointDataList.begin()); it != m_jointDataList.end(); ++it) {
        // 计算当前点的距离
        classic6dofForKine(it->targetJoints.data(), &cur_pose);
        cur_dist = distance_from_origin(cur_pose.X, cur_pose.Y);

        // 判断当前线段是否跨过圆（距离从小于radius到大于等于，或反之）
        bool prev_inside = (prev_dist < distance);
        bool cur_inside = (cur_dist < distance);

        // 跨过圆，说明存在交点
        if (prev_inside != cur_inside) {
            if (!found_first) {
                first_intersect = index - 1; // 记录交点所在线段的起始索引
                found_first = true;
            } else {
                second_intersect = index - 1;
                break; // 找到两个交点即可退出
            }
        }

        // 处理点正好在圆上的情况
        else if (std::fabs(cur_dist - distance) < 1e-6) { // 考虑浮点误差
            if (!found_first) {
                first_intersect = index;
                found_first = true;
            } else {
                second_intersect = index;
                break;
            }
        }

        // 更新上一个点的数据
        prev_pose = cur_pose;
        prev_dist = cur_dist;
        index++;
    }

    return std::make_tuple(first_intersect, second_intersect);
}
void JointDataManager::delete_arm_points(int first_index, int second_index)
{
    // 检查容器是否为空
    if (m_jointDataList.empty()) {
        return;
    }

    // 确保first_index <= second_index
    if (first_index > second_index) {
        std::swap(first_index, second_index);
    }

    // 转换为0基索引（假设原索引是1基的）
    size_t start = static_cast<size_t>(first_index - 1);
    size_t end = static_cast<size_t>(second_index - 1);

    // 检查索引有效性
    if (start >= m_jointDataList.size() || end >= m_jointDataList.size() || start > end) {
        return; // 索引超出范围或无效，不执行删除
    }

    // 获取起始迭代器（对于std::list需使用std::next）
    auto start_it = m_jointDataList.begin();
    std::advance(start_it, start); // 使用advance移动到起始位置

    // 获取结束迭代器
    auto end_it = m_jointDataList.begin();
    std::advance(end_it, end + 1); // 结束位置是end+1（因为erase是左闭右开区间）

    // 删除范围内的元素
    m_jointDataList.erase(start_it, end_it);
}

// void JointDataManager::moveL_SingularityAvoidance(Robot *robot)
// {
//     std::tuple<JointDataManager::JointData, JointDataManager::JointData, int, int> tmp = find_singular_points();
// //     while (std::get<2>(tmp) != -1 && std::get<3>(tmp) != -1)
// //     {
//     std::cout << "奇异区域开始于" << std::get<2>(tmp) << std::endl;
//     JointDataManager::getInstance()->print(std::get<0>(tmp));
//     std::cout << "奇异区域结束于" << std::get<3>(tmp) << std::endl;
//     JointDataManager::getInstance()->print(std::get<1>(tmp));
// //     }

// //     robot->moveJ(std::get<1>(tmp).targetJoints, std::get<0>(tmp).targetJoints, 5);

//     auto start_point = std::get<2>(tmp);
//     auto end_point = std::get<3>(tmp);
//     replace_singular_Data(m_jointDataList, tmp_jointDataList, start_point, end_point, 10);
// }
// 添加数据实现
void JointDataManager::addData(
    const std::array<float, 6> &joints,
    int flag0,
    int flag1,
    int flag2)
{
    m_jointDataList.emplace_back(joints, flag0, flag1, flag2);

//    if (flag0 != 1 || flag1 != 1 || flag2 != 1)
//    {
//        auto it = m_jointDataList.end();
//        --it;
//        m_negativeOneIterators.push_back(it);
    //    }
}

void JointDataManager::all_addData(
        const std::array<float, 6> &joints,
        int flag0,
        int flag1,
        int flag2)
{
    all_jointDataList.emplace_back(joints, flag0, flag1, flag2);
}

// 获取指定位置关节数据实现
const std::array<float, 6> &JointDataManager::getJointsAt(size_t index) const
{
    if (index >= size())
    {
        throw std::out_of_range("JointDataManager: 索引超出范围");
    }

    auto it = m_jointDataList.begin();
    std::advance(it, index);
    return it->targetJoints;
}

// 获取指定位置标志位实现
int JointDataManager::getFlagAt(size_t index) const
{
    if (index >= size())
    {
        throw std::out_of_range("JointDataManager: 索引超出范围");
    }

    auto it = m_jointDataList.begin();
    std::advance(it, index);
    return it->flag0;
}

// 批量处理含-1标志位元素实现
// void JointDataManager::processNegativeOneFlags(
//     std::function<void(const std::array<float, 6> &, size_t)> processor) const
// {
//     for (const auto &it : m_negativeOneIterators)
//     {
//         if (it != m_jointDataList.end()) // 检查迭代器有效性
//         {
//             size_t index = std::distance(m_jointDataList.begin(), it);
//             processor(it->targetJoints, index);
//         }
//     }
// }

// 替换含-1标志位元素实现
void JointDataManager::replaceNegativeOneData(
    const std::vector<std::tuple<std::array<float, 6>, int, int, int>> &newData)
{
    std::vector<std::list<JointData>::iterator> validIters;

    for (const auto &it : m_negativeOneIterators)
    {
        if (it != m_jointDataList.end())
        {
            auto insertPos = it;
            m_jointDataList.erase(it);

            for (const auto &data : newData)
            {
                insertPos = m_jointDataList.emplace(
                    insertPos,
                    std::get<0>(data),
                    std::get<1>(data),
                    std::get<2>(data),
                    std::get<3>(data));
                ++insertPos;
            }
        }
    }

    // 重新收集含-1标志位的元素
    m_negativeOneIterators.clear();
    for (auto it = m_jointDataList.begin(); it != m_jointDataList.end(); ++it)
    {
        if (it->flag0 == -1 || it->flag1 == -1 || it->flag2 == -1)
        {
            m_negativeOneIterators.push_back(it);
        }
    }
}

int JointDataManager::replace_singular_Data(std::list<JointData> &org, std::list<JointData> &tmp, int start, int end, int dec)
{
    // 边界检查
    if (start < 0 || end < start || dec <= 0 || org.empty() || tmp.empty())
    {
        return 0;
    }

    // 第一步：清除 org 中 [start, end] 范围的元素
    auto it = org.begin();
    int index = 0;
    auto eraseBegin = org.end();
    auto eraseEnd = org.end();

    for (; it != org.end(); ++it, ++index)
    {
        if (index == start)
        {
            eraseBegin = it;
        }
        if (index == end + 1)
        {
            eraseEnd = it;
            break;
        }
    }

    if (eraseBegin != org.end())
    {
        org.erase(eraseBegin, eraseEnd);
    }


    std::cout << "关节空间插补点数" <<tmp.size();

    // ethcat周期在1ms，那么这个奇异点的过渡段就控制在1.5秒内
    if(tmp.size() > 3000)
    {
        // 向上取整
        dec = (tmp.size()  + 3000 -1) / 3000;
//        (a + b - 1) / b
    }

    // 第二步：插入 tmp 中每 dec 个取一个，插入到 org 的 start 位置
    auto insertPos = org.begin();
    std::advance(insertPos, start);
//        std::cout << "从 " << count;

    int count = 0;
    int tmpIndex = 0;
    for (auto tmpIt = tmp.begin(); tmpIt != tmp.end(); ++tmpIt, ++tmpIndex)
    {
        if (tmpIndex % dec == 0)
        {
            insertPos = org.insert(insertPos, *tmpIt);
            ++insertPos; // 插入后指向新插入位置的下一个位置
            count ++;
        }
    }
    std::cout << "共插入 " << count;

    return start + count;

}

int JointDataManager::replace_singular_Data(std::list<JointData> &org, std::list<JointData> &tmp, int start, int dec)
{
    // 边界检查：无效参数（start为负、dec非正）直接返回
    if (start < 0 || dec <= 0)
    {
        return 0;
    }

    size_t tmp_size = tmp.size();
    // 若tmp为空，无元素可插入，返回原start位置
    if (tmp_size == 0)
    {
        return start;
    }

    // 确定需要插入的元素数量：
    // - 若tmp元素数量 <= dec，全部插入
    // - 若tmp元素数量 > dec，均匀抽样到dec个
    size_t sample_count = (tmp_size <= static_cast<size_t>(dec)) ? tmp_size : static_cast<size_t>(dec);

    // 生成抽样索引（确定从tmp中选哪些元素）
    std::vector<size_t> sample_indices;
    sample_indices.reserve(sample_count);

    if (tmp_size <= static_cast<size_t>(dec))
    {
        // 情况1：tmp元素不足dec，全量插入（索引0到tmp_size-1）
        for (size_t i = 0; i < tmp_size; ++i)
        {
            sample_indices.push_back(i);
        }
    }
    else
    {
        // 情况2：tmp元素多于dec，均匀抽样到dec个
        // 计算步长（确保首尾元素被包含，中间均匀分布）
        if (dec == 1)
        {
            // 特殊处理：dec=1时只取第一个元素
            sample_indices.push_back(0);
        }
        else
        {
            double step = static_cast<double>(tmp_size - 1) / (dec - 1); // 避免除零
            for (size_t i = 0; i < dec; ++i)
            {
                // 四舍五入取整，保证均匀性
                size_t idx = static_cast<size_t>(std::round(i * step));
                sample_indices.push_back(idx);
            }
        }
    }

    // 收集需要插入的元素（按抽样索引从tmp中提取）
    std::vector<JointData> to_insert;
    to_insert.reserve(sample_count);
    auto tmp_it = tmp.begin();
    size_t current_idx = 0;  // 记录当前遍历到tmp的索引
    size_t idx_ptr = 0;      // 跟踪抽样索引的位置

    while (tmp_it != tmp.end() && idx_ptr < sample_indices.size())
    {
        if (current_idx == sample_indices[idx_ptr])
        {
            to_insert.push_back(*tmp_it); // 匹配抽样索引，加入待插入列表
            idx_ptr++;
        }
        ++tmp_it;
        current_idx++;
    }

    // 找到org中start位置的插入点
    auto insert_pos = org.begin();
    size_t org_size = org.size();
    if (start >= static_cast<int>(org_size))
    {
        // 若start超出org现有长度，插入到末尾
        insert_pos = org.end();
    }
    else
    {
        // 移动迭代器到start位置
        std::advance(insert_pos, start);
    }

    // 将待插入元素插入到org中
    int inserted_count = 0;
    for (const auto& data : to_insert)
    {
        insert_pos = org.insert(insert_pos, data); // 插入元素
        ++insert_pos; // 移动到新插入元素的下一个位置，保证插入顺序
        inserted_count++;
    }

    // 返回插入后的位置（原start + 插入数量）
    return start + inserted_count;
}

// 获取数据总数实现
size_t JointDataManager::size() const
{
    return m_jointDataList.size();
}

// 检查是否为空实现
bool JointDataManager::empty() const
{
    return m_jointDataList.empty();
}

void JointDataManager::print(const JointData &data) const
{
    // 打印关节数据（6个值）
    std::cout << "===== 关节数据 =====" << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "关节 " << i << ": " << data.targetJoints[i] << " ";
        // 每3个关节换一行，排版更清晰
        if ((i + 1) % 3 == 0)
        {
            std::cout << std::endl;
        }
    }

    // 打印标志位（突出显示值为 -1 的标志位）
    std::cout << "\n===== 标志位 =====" << std::endl;
    std::cout << "flag0: " << data.flag0;

    std::cout << "flag1: " << data.flag1;

    std::cout << "flag2: " << data.flag2;

    std::cout << std::endl;

    // 分隔线，区分不同 JointData 的输出
    std::cout << "-------------------------" << std::endl;
}
// 打印数据实现
void JointDataManager::print() const
{
    size_t index = 0;
    for (const auto &item : m_jointDataList)
    {
        std::cout << "Index: " << index << " Target Joints: ";
        for (int j = 0; j < 6; j++)
        {
            std::cout << item.targetJoints[j] << " ";
        }
        std::cout << "  Flag0: " << item.flag0
                  << "  Flag1: " << item.flag1
                  << "  Flag2: " << item.flag2 << std::endl;
        ++index;
    }
}
void JointDataManager::print_tmp() const
{
    size_t index = 0;
    for (const auto &item : tmp_jointDataList)
    {
        std::cout << "Index: " << index << " Target Joints: ";
        for (int j = 0; j < 6; j++)
        {
            std::cout << item.targetJoints[j] << " ";
        }
        std::cout << "  Flag0: " << item.flag0
                  << "  Flag1: " << item.flag1
                  << "  Flag2: " << item.flag2 << std::endl;
        ++index;
    }
    std::cout << "  结束 " << std::endl;
}

void JointDataManager::print_singular() const
{
    if (m_negativeOneIterators.empty())
    {
        std::cout << "m_negativeOneIterators 为空，无符合条件的元素" << std::endl;
        return;
    }

    std::cout << "===== 开始打印含 -1 标志位的元素（共 " << m_negativeOneIterators.size() << " 个） =====" << std::endl;
    size_t count = 0; // 计数：第几个符合条件的元素

    for (const auto &it : m_negativeOneIterators)
    {
        // 计算当前元素在 list 中的全局索引（从 0 开始）
        size_t global_index = std::distance(m_jointDataList.begin(), it);

        // 打印当前元素的基本信息
        std::cout << "[" << count << "] 全局索引: " << global_index << std::endl;
        std::cout << "  关节数据: ";
        for (int i = 0; i < 6; ++i)
        {
            std::cout << it->targetJoints[i] << " ";
        }
        std::cout << std::endl;

        // 打印标志位（突出显示为 -1 的标志位）
        std::cout << "  标志位: ";
        std::cout << "flag0=" << it->flag0;
        std::cout << ", flag1=" << it->flag1;
        std::cout << ", flag2=" << it->flag2;
        std::cout << std::endl
                  << "-------------------------" << std::endl;

        ++count;
    }

    std::cout << "===== 含 -1 标志位的元素打印完毕 =====" << std::endl;
}

// 清空数据实现
void JointDataManager::clear()
{

//    std::list<JointData> all_jointDataList;

//    // 主容器，在Aoid_moveL中先保存奇异点信息，后清空保存最后多段插补点信息
//    std::list<JointData> m_jointDataList;

//    // 用于保存奇异区域的起始点和终点
//    std::list<JointData> singular_pointsList;



//    // 用于跟踪当前读取位置的迭代器
//    std::list<JointData>::iterator m_currentIt;

//    std::list<JointData> tmp_jointDataList;
    all_jointDataList.clear();
    m_jointDataList.clear();
    tmp_jointDataList.clear();
    m_negativeOneIterators.clear();
    m_currentIt = m_jointDataList.begin();
    singular_pointsList.clear();

//    singular_pointsList.clear();
}
