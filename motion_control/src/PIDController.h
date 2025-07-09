#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


class PIDController
{
public:
    PIDController(float kp, float ki, float kd, float dt);

    double compute(double target, double current);
    void reset();
private:
    double Kp, Ki, Kd;
    double dt; // 控制周期 (秒)

    double prev_error;
    double integral;
};

#endif // PIDCONTROLLER_H
