// mclib
#include "mclib/pid.hpp"

#include "api.h"
#include "mclib/utils.hpp"

#include <cmath>
#include <limits>

PID::PID(double new_kp, double new_ki, double new_kd)
    : target(0), arrived(false), arrive(true), small_error_tolerance(1),
      big_error_tolerance(3), small_error_duration(100), big_error_duration(500),
      small_check_time(0), big_check_time(0), first_time(true), kp(new_kp),
      ki(new_ki), kd(new_kd), integral_range(0), integral_max(500),
      derivative_tolerance(std::numeric_limits<double>::infinity()),
      error_tolerance(0), current_error(0), previous_error(0), sum_error(0),
      proportional(0), integral(0), derivative(0), output(0)
{
}

void PID::setCoefficient(double new_kp, double new_ki, double new_kd)
{
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
}

void PID::setTarget(double new_target) { target = new_target; }

void PID::setSmallBigErrorTolerance(double new_small_error_tolerance, double new_big_error_tolerance)
{
    small_error_tolerance = new_small_error_tolerance;
    big_error_tolerance = new_big_error_tolerance;
}

void PID::setIntegralMax(double new_integral_max) { integral_max = new_integral_max; }

void PID::setIntegralRange(double new_integral_range) { integral_range = new_integral_range; }

void PID::clearSumError() { sum_error = 0; }

void PID::setDerivativeTolerance(double new_derivative_tolerance) { derivative_tolerance = new_derivative_tolerance; }

void PID::setSmallBigErrorDuration(double new_small_error_duration, double new_big_error_duration)
{
    small_error_duration = new_small_error_duration;
    big_error_duration = new_big_error_duration;
}

void PID::setArrive(bool new_arrive) { arrive = new_arrive; }

void PID::reset()
{
    arrived = false;
    first_time = true;
    small_check_time = 0;
    big_check_time = 0;
    current_error = 0;
    previous_error = 0;
    sum_error = 0;
    proportional = 0;
    integral = 0;
    derivative = 0;
    output = 0;
}

bool PID::targetArrived() { return arrived; }

double PID::getI() { return ki; }

double PID::getOutput() { return output; }

int PID::sign(double number)
{
    if (number > 0)
    {
        return 1;
    }
    else if (number < 0)
    {
        return -1;
    }
    return 0;
}

double PID::update(double input)
{
    current_error = target - input;
    if (first_time)
    {
        first_time = false;
        previous_error = current_error;
        sum_error = 0;
        small_check_time = pros::millis();
        big_check_time = pros::millis();
    }

    proportional = kp * current_error;
    const double error_delta = current_error - previous_error;
    derivative = kd * error_delta;
    previous_error = current_error;

    if (fabs(current_error) >= integral_range && integral_range != 0)
    {
        sum_error = 0;
    }
    else
    {
        sum_error += current_error;
        if (fabs(sum_error) * ki > integral_max && integral_max != 0)
        {
            sum_error = sign(sum_error) * integral_max / ki;
        }
    }

    if (sign(sum_error) != sign(current_error) || fabs(current_error) <= small_error_tolerance)
    {
        sum_error = 0;
    }

    integral = ki * sum_error;

    // Use raw error change for settling checks so kD tuning does not affect
    // arrival detection thresholds.
    if (arrive && fabs(current_error) <= small_error_tolerance && fabs(error_delta) <= derivative_tolerance)
    {
        if (pros::millis() - small_check_time >= small_error_duration)
        {
            arrived = true;
        }
    }
    else
    {
        small_check_time = pros::millis();
    }

    if (arrive && fabs(current_error) <= big_error_tolerance && fabs(error_delta) <= derivative_tolerance)
    {
        if (pros::millis() - big_check_time >= big_error_duration)
        {
            arrived = true;
        }
    }
    else
    {
        big_check_time = pros::millis();
    }

    if (arrived)
    {
        output = 0;
        return output;
    }

    output = proportional + integral + derivative;
    return output;
}
