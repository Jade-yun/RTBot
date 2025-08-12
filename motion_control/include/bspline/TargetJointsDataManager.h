#ifndef _TARGETJOINTSDATAMANAGER_H_
#define _TARGETJOINTSDATAMANAGER_H_

#include <list>
#include <array>
#include <functional>
#include <vector>
#include <tuple>
#include <utility>
#include <string>
#include "kinematics/Classic6dofKine.h"
#include "Parameters/SharedDataType.h"
// #include "Robot.h"


// class Robot; // 关键：仅声明类存在

/**
 * 关节数据管理类（单例模式）
 * 基于std::list实现，适合频繁在中间位置插入/删除数据
 * 固定管理6个关节的数据
 */
// 内部结构体，存储关节数据和标志位

class JointDataManager
{
public:
    struct JointData
    {
        std::array<float, 6> targetJoints;
        int flag0;
        int flag1;
        int flag2;
        JointData(const std::array<float, 6> &joints, int flagVal0, int flagVal1, int flagVal2);
    };

public:
    float speed;


    std::list<JointData> all_changed_jointDataList;

    // 保存所有原始插补点
    std::list<JointData> all_jointDataList;

    // 主容器，在Aoid_moveL中先保存奇异点信息，后清空保存最后多段插补点信息
    std::list<JointData> m_jointDataList;

    // 用于保存奇异区域的起始点和终点
    std::list<JointData> singular_pointsList;



    // 用于跟踪当前读取位置的迭代器
    std::list<JointData>::iterator m_currentIt;

    std::list<JointData> tmp_jointDataList;
    // 含-1标志位的元素迭代器
    std::vector<std::list<JointData>::const_iterator> m_negativeOneIterators;

    static JointDataManager *m_instance;

    // 单例模式：私有构造函数
    JointDataManager()
    {
        m_currentIt = m_jointDataList.begin();
    }

    // 禁用拷贝和赋值
    JointDataManager(const JointDataManager &) = delete;
    JointDataManager &operator=(const JointDataManager &) = delete;

public:
    // 单例模式：获取唯一实例
    static JointDataManager *getInstance();
    void set_speed(float x);

    // 筛选奇异区域
    bool is_delimiter(const JointData& data) const;
    std::pair<JointData, JointData> get_singular_area();
    bool collect_singular_area();

    void combine(int m_seconds);
    std::pair<std::vector<int>, int> filtering_solution(float interp_X,float interp_Y,float interp_Z,Kine6dSol q_sol,std::array<float, NUM_JOINTS> m_angleLimitMax,std::array<float, NUM_JOINTS> m_angleLimitMin,std::array<float, NUM_JOINTS> m_curJoints);



    // 保存CSV文件的函数
    void save_all_change_CSV(const std::string &filename) const;
    void save_all_CSV(const std::string &filename) const;
    void save_CSV(const std::string &filename) const;
    void singular_changed_save_CSV(const std::string &filename = "changed_singular_joint_data.csv") const;
    void singular_points_save_CSV(const std::string &filename = "singular_joint_data.csv") const;

    // 临时存储空间的函数
    void tmpaddData(const std::array<float, 6> &joints, int flag0, int flag1, int flag2);
    void tmpclear();
    std::tuple<JointData, JointData, int, int> find_singular_points() const;
    std::tuple<JointData, JointData, int, int> find_Mutation(float distance_max,int start_index); // 返回命令中突变的开始和结束值
    std::tuple<int,int> find_arm_point(float distance);
    // void moveL_SingularityAvoidance(Robot *robot);
    void moveL_run();
    void delete_arm_points(int first_index, int second_index);

    // 移动构造和赋值（单例模式下一般不使用，但保留默认实现）
    JointDataManager(JointDataManager &&) = default;
    JointDataManager &operator=(JointDataManager &&) = default;

    // 添加数据
    void addData(const std::array<float, 6> &joints, int flag0, int flag1, int flag2);
    void all_addData(const std::array<float, 6> &joints, int flag0, int flag1, int flag2);

    // 获取指定位置的关节数据
    const std::array<float, 6> &getJointsAt(size_t index) const;

    // 获取指定位置的标志位0
    int getFlagAt(size_t index) const;

    // 批量处理含-1标志位的元素
    // void processNegativeOneFlags(std::function<void(const std::array<float, 6> &, size_t)> processor) const;

    // 替换含-1标志位的元素
    void replaceNegativeOneData(const std::vector<std::tuple<std::array<float, 6>, int, int, int>> &newData);

    int  replace_singular_Data(std::list<JointData> &org, std::list<JointData> &tmp, int start, int end, int dec);
    int replace_singular_Data(std::list<JointData> &org, std::list<JointData> &tmp, int start, int dec);

    // 获取数据总数
    size_t size() const;

    // 检查是否为空
    bool empty() const;

    // 打印所有数据
    void print(const JointData &data) const;
    void print() const;
    void print_singular() const;
    void print_tmp() const;

    // 清空数据
    void clear();

    // 单例模式：析构函数（私有，确保只能通过实例销毁）
    ~JointDataManager() = default;
};

#endif
