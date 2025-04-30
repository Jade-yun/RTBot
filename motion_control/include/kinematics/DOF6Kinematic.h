#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H

#include <Eigen/Dense>
#include <cmath>
#include <array>

class DOF6Kinematic
{
private:
    static constexpr float RAD_TO_DEG = 57.29577951308232f; // 180/pi
    static constexpr float DEG_TO_RAD = 0.017453292519943295f; // pi/180

    struct ArmConfig_t
    {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
    };
    ArmConfig_t armConfig;

    Eigen::Matrix<float, 6, 4> DH_matrix;
    Eigen::Vector3f L1_base, L2_arm, L3_elbow, L6_wrist;

    float l_se_2;
    float l_se;
    float l_ew_2;
    float l_ew;
    float atan_e;

public:
    struct Joint6D_t
    {
        Joint6D_t() = default;
        Joint6D_t(float a1, float a2, float a3, float a4, float a5, float a6)
            : a{a1, a2, a3, a4, a5, a6} {}

        std::array<float, 6> a;

        friend Joint6D_t operator-(const Joint6D_t& joints1, const Joint6D_t& joints2)
        {
            Joint6D_t result;
            for (int i = 0; i < 6; ++i)
                result.a[i] = joints1.a[i] - joints2.a[i];
            return result;
        }
    };

    struct Pose6D_t
    {
        Pose6D_t() = default;
        Pose6D_t(float x, float y, float z, float a, float b, float c)
            : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false) {}

        float X = 0, Y = 0, Z = 0;
        float A = 0, B = 0, C = 0;
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        bool hasR = false;
    };

    struct IKSolves_t
    {
        std::array<Joint6D_t, 8> config;
        std::array<std::array<char, 3>, 8> solFlag{};
    };

    DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);

    bool SolveFK(const Joint6D_t& inputJoint6D, Pose6D_t& outputPose6D);
    bool SolveIK(const Pose6D_t& inputPose6D, const Joint6D_t& lastJoint6D, IKSolves_t& outputSolves);

private:
    static void EulerAngleToRotMat(const Eigen::Vector3f& euler, Eigen::Matrix3f& rotMat);
    static void RotMatToEulerAngle(const Eigen::Matrix3f& rotMat, Eigen::Vector3f& euler);
};

#endif // DOF6_KINEMATIC_SOLVER_H
