#pragma once
#include <Eigen/Dense>
#include <vector>

// 基础定义
using JointVector = Eigen::VectorXd;     // n自由度关节向量
using CartesianPose = Eigen::Matrix4d;   // 齐次变换矩阵表示位姿
