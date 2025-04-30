#include "kinematics/DOF6Kinematic.h"

DOF6Kinematic::DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT)
    : armConfig{L_BS, D_BS, L_AM, L_FA, D_EW, L_WT},
      l_se_2(L_AM * L_AM),
      l_se(L_AM),
      l_ew_2(L_FA * L_FA + D_EW * D_EW),
      l_ew(std::sqrt(L_FA * L_FA + D_EW * D_EW)),
      atan_e(std::atan2(D_EW, L_FA))
{
    DH_matrix << 
        0.0f,             L_BS,    D_BS,   -M_PI_2,
       -M_PI_2,           0.0f,    L_AM,    0.0f,
        M_PI_2,           D_EW,    0.0f,    M_PI_2,
        0.0f,             L_FA,    0.0f,   -M_PI_2,
        0.0f,             0.0f,    0.0f,    M_PI_2,
        0.0f,             L_WT,    0.0f,    0.0f;

    L1_base << D_BS, -L_BS, 0.0f;
    L2_arm << L_AM, 0.0f, 0.0f;
    L3_elbow << -D_EW, 0.0f, L_FA;
    L6_wrist << 0.0f, 0.0f, L_WT;
}

void DOF6Kinematic::EulerAngleToRotMat(const Eigen::Vector3f& euler, Eigen::Matrix3f& rotMat)
{
    float ca = cos(euler.z());
    float cb = cos(euler.y());
    float cc = cos(euler.x());
    float sa = sin(euler.z());
    float sb = sin(euler.y());
    float sc = sin(euler.x());

    rotMat(0,0) = ca * cb;
    rotMat(0,1) = ca * sb * sc - sa * cc;
    rotMat(0,2) = ca * sb * cc + sa * sc;
    rotMat(1,0) = sa * cb;
    rotMat(1,1) = sa * sb * sc + ca * cc;
    rotMat(1,2) = sa * sb * cc - ca * sc;
    rotMat(2,0) = -sb;
    rotMat(2,1) = cb * sc;
    rotMat(2,2) = cb * cc;
}

void DOF6Kinematic::RotMatToEulerAngle(const Eigen::Matrix3f& rotMat, Eigen::Vector3f& euler)
{
    if (std::abs(rotMat(2,0)) >= 1.0f - 1e-4f)
    {
        if (rotMat(2,0) < 0)
        {
            euler.y() = M_PI_2;
            euler.x() = atan2(rotMat(0,1), rotMat(1,1));
            euler.z() = 0;
        }
        else
        {
            euler.y() = -M_PI_2;
            euler.x() = -atan2(rotMat(0,1), rotMat(1,1));
            euler.z() = 0;
        }
    }
    else
    {
        euler.y() = -asin(rotMat(2,0));
        euler.x() = atan2(rotMat(2,1), rotMat(2,2));
        euler.z() = atan2(rotMat(1,0), rotMat(0,0));
    }
}

bool DOF6Kinematic::SolveFK(const Joint6D_t& inputJoint6D, Pose6D_t& outputPose6D)
{
    Eigen::Matrix3f R_total = Eigen::Matrix3f::Identity();
    Eigen::Vector3f pos_total = Eigen::Vector3f::Zero();

    for (int i = 0; i < 6; ++i)
    {
        float theta = (inputJoint6D.a[i] * DEG_TO_RAD) + DH_matrix(i, 0);
        float d = DH_matrix(i, 1);
        float a = DH_matrix(i, 2);
        float alpha = DH_matrix(i, 3);

        Eigen::Matrix3f Rz = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Eigen::Matrix3f Rx = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()).toRotationMatrix();
        Eigen::Matrix3f R_i = Rz * Rx;

        R_total = R_total * R_i;

        Eigen::Vector3f trans(a, -d * sin(alpha), d * cos(alpha));
        pos_total += R_total * trans;
    }

    outputPose6D.X = pos_total.x() * 1000.0f;
    outputPose6D.Y = pos_total.y() * 1000.0f;
    outputPose6D.Z = pos_total.z() * 1000.0f;
    Eigen::Vector3f euler;
    RotMatToEulerAngle(R_total, euler);
    outputPose6D.A = euler.x() * RAD_TO_DEG;
    outputPose6D.B = euler.y() * RAD_TO_DEG;
    outputPose6D.C = euler.z() * RAD_TO_DEG;
    outputPose6D.R = R_total;
    outputPose6D.hasR = true;

    return true;
}

bool DOF6Kinematic::SolveIK(const Pose6D_t& inputPose6D, const Joint6D_t& lastJoint6D, IKSolves_t& outputSolves)
{
    if (!inputPose6D.hasR)
    {
        Eigen::Vector3f euler(inputPose6D.A * DEG_TO_RAD, inputPose6D.B * DEG_TO_RAD, inputPose6D.C * DEG_TO_RAD);
        EulerAngleToRotMat(euler, const_cast<Eigen::Matrix3f&>(inputPose6D.R)); // Remove constness carefully
    }

    Eigen::Vector3f pw(
        inputPose6D.X / 1000.0f,
        inputPose6D.Y / 1000.0f,
        inputPose6D.Z / 1000.0f
    );

    pw -= inputPose6D.R * L6_wrist;

    float d = sqrt(pw.x() * pw.x() + pw.y() * pw.y()) - L1_base.norm();
    float z = pw.z();

    float r = sqrt(d*d + z*z);
    float D = (r*r - l_se_2 - l_ew_2) / (2.0f * l_se * l_ew);

    if (std::abs(D) > 1.0f)
    {
        return false; // 超出机械臂结构可达范围
    }

    float q3_1 = atan2(-sqrt(1 - D*D), D);
    float q3_2 = atan2(sqrt(1 - D*D), D);

    std::array<float, 2> q3_candidates{ q3_1, q3_2 };

    int solution_idx = 0;
    for (auto q3 : q3_candidates)
    {
        float phi = atan2(z, d);
        float beta = atan2(l_ew * sin(q3), l_se + l_ew * cos(q3));
        float q2 = phi - beta;

        float q1 = atan2(pw.y(), pw.x()) - atan2(armConfig.D_BASE, armConfig.L_BASE);

        Eigen::Matrix3f R03;
        R03 = Eigen::AngleAxisf(q1, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
              Eigen::AngleAxisf(q2, Eigen::Vector3f::UnitY()).toRotationMatrix() *
              Eigen::AngleAxisf(q3 + atan_e, Eigen::Vector3f::UnitY()).toRotationMatrix();

        Eigen::Matrix3f R36 = R03.transpose() * inputPose6D.R;

        Eigen::Vector3f euler36;
        RotMatToEulerAngle(R36, euler36);

        for (int wrist_config = 0; wrist_config < 2; ++wrist_config)
        {
            Joint6D_t solution;
            solution.a[0] = q1 * RAD_TO_DEG;
            solution.a[1] = q2 * RAD_TO_DEG;
            solution.a[2] = q3 * RAD_TO_DEG;
            if (wrist_config == 0)
            {
                solution.a[3] = euler36.x() * RAD_TO_DEG;
                solution.a[4] = euler36.y() * RAD_TO_DEG;
                solution.a[5] = euler36.z() * RAD_TO_DEG;
            }
            else
            {
                solution.a[3] = (euler36.x() + M_PI) * RAD_TO_DEG;
                solution.a[4] = (-euler36.y()) * RAD_TO_DEG;
                solution.a[5] = (euler36.z() + M_PI) * RAD_TO_DEG;
            }

            outputSolves.config[solution_idx] = solution;
            outputSolves.solFlag[solution_idx] = {1, 1, 1};
            ++solution_idx;
        }
    }

    return solution_idx > 0;
}
