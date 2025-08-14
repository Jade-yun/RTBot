#ifndef BSPLINE_SMOOTHING_H
#define BSPLINE_SMOOTHING_H

#include <vector>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <fstream>
#include <iomanip>

// 3D点结构体
struct Point3D {
    double x, y, z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x, double y, double z) : x(x), y(y), z(z) {}

    // 运算符重载
    Point3D operator+(const Point3D& other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D operator-(const Point3D& other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    // 计算向量长度
    double length() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    // 归一化向量
    Point3D normalize() const {
        double len = length();
        if (len < 1e-10) return Point3D(0, 0, 0);
        return Point3D(x/len, y/len, z/len);
    }
};

// 线段结构体
struct LineSegment {
    Point3D start, end;

    LineSegment() {}
    LineSegment(const Point3D& start, const Point3D& end) : start(start), end(end) {}

    // 获取线段方向向量
    Point3D getDirection() const {
        return end - start;
    }

    // 获取线段长度
    double getLength() const {
        return (end - start).length();
    }
};

// B样条平滑类（单例模式）
class BSplineSmoothing {
private:
    int degree_;                    // B样条次数
    double maximum_error_;          // 最大轮廓误差
    int max_iterations_;           // 最大迭代次数
    double tolerance_;             // 收敛容差
    std::vector<Point3D> smoothedCurve_;  // 存储生成的B样条曲线点

    // 缓存变量用于优化get_BSpline_point函数
    mutable double lastRatio_;              // 上一次查询的ratio
    mutable size_t lastSegmentIndex_;       // 上一次查询所在的线段索引
    mutable double lastAccumulatedLength_;  // 上一次查询时的累积弧长
    mutable bool cacheValid_;               // 缓存是否有效

    static BSplineSmoothing* instance_;  // 单例实例指针

    // 私有构造函数
    BSplineSmoothing(int degree = 3, double maximum_error = 0.01,
                     int max_iterations = 50, double tolerance = 1e-6);

    // 删除拷贝构造函数和赋值操作符
    BSplineSmoothing(const BSplineSmoothing&) = delete;
    BSplineSmoothing& operator=(const BSplineSmoothing&) = delete;

public:
    // 获取单例实例
    static BSplineSmoothing* getInstance(int degree = 3, double maximum_error = 0.01,
                                        int max_iterations = 50, double tolerance = 1e-6);

    // 销毁单例实例
    static void destroyInstance();

    // 析构函数
    ~BSplineSmoothing() = default;

    // 设置参数
    void setDegree(int degree);
    void setMaximumError(double maximum_error);
    void setMaxIterations(int max_iterations);
    void setTolerance(double tolerance);

    // 获取参数（内联函数）
    int getDegree() const { return degree_; }
    double getMaximumError() const { return maximum_error_; }
    int getMaxIterations() const { return max_iterations_; }
    double getTolerance() const { return tolerance_; }

    // 生成平滑B样条曲线
    void generateSmoothCurve(const LineSegment& seg1, const LineSegment& seg2,
                           int numSamples = 100);

    // 计算点到B样条曲线的最短距离
    double distanceToSpline(const Point3D& point, const std::vector<Point3D>& splinePoints);

    // 验证轮廓误差
    bool validateContourError(const Point3D& intersection, const std::vector<Point3D>& splinePoints);

    // 保存数据到文件
    bool saveDataToFile(const LineSegment& seg1, const LineSegment& seg2,
                       const std::vector<Point3D>& smoothedCurve,
                       const std::string& filename = "curve_data_class.txt");

    // 根据比例获取B样条曲线上的点（支持线性插值）
    Point3D get_BSpline_point(double ratio);

    // 获取当前存储的B样条曲线
    const std::vector<Point3D>& getSmoothedCurve() const { return smoothedCurve_; }

    // 获取B样条曲线点的数量
    size_t getCurvePointCount() const { return smoothedCurve_.size(); }

    // 获取B样条曲线的总路程（所有点之间距离的总和）
    double getCurveLength() const;

private:
    // 计算欧氏距离（内联函数）
    double distance(const Point3D& p1, const Point3D& p2) {
        return (p1 - p2).length();
    }

    // B样条基函数计算
    double basisFunction(int i, int k, double t, const std::vector<double>& knots);

    // 生成B样条节点向量
    std::vector<double> generateKnotVector(int numControlPoints);

    // 在参数t处计算B样条曲线上的点
    Point3D evaluateBSpline(double t, const std::vector<Point3D>& controlPoints,
                           const std::vector<double>& knots);

    // 寻找两条线段的交点
    bool findIntersection(const LineSegment& seg1, const LineSegment& seg2, Point3D& intersection);

    // 使用最大轮廓误差约束优化控制点
    std::vector<Point3D> optimizeControlPoints(const LineSegment& seg1, const LineSegment& seg2,
                                              const Point3D& intersection);

    // 生成B样条曲线
    std::vector<Point3D> generateBSplineCurve(const std::vector<Point3D>& controlPoints,
                                             int numSamples);
};

#endif // BSPLINE_SMOOTHING_H
