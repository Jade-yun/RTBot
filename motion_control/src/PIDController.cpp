#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float dt)
    : Kp(kp), Ki(ki), Kd(ki), dt(dt),
      prev_error(0.0), integral(0.0)
{

}

double PIDController::compute(double target, double current)
{
    double error = target - current;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;

    double output = Kp * error + Ki * integral + Kd * derivative;

//    if (output_limit > 0.0) {
//        if (output > output_limit) output = output_limit;
//        if (output < -output_limit) output = -output_limit;
//    }

    return output;

}

void PIDController::reset()
{
    prev_error = 0.0;
    integral = 0.0;

}
