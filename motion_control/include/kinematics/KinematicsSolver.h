// kinematics_solver.h
#pragma once
#include "CommonTypes.h"

class KinematicsSolver
{
public:
    bool setDHParameters(const std::vector<std::vector<double>>& dh_params); // 设定DH参数

    CartesianPose forwardKinematics(const JointVector& joints);
    bool inverseKinematics(const CartesianPose& pose, JointVector& joints_out);
private:
    // 保存DH参数
};
