// mclib
#pragma once

class PID {
public:
  PID(double new_kp, double new_ki, double new_kd);

  void setCoefficient(double new_kp, double new_ki, double new_kd);
  void setTarget(double new_target);
  void setSmallBigErrorTolerance(double new_small_error_tolerance,
                                 double new_big_error_tolerance);
  void setIntegralMax(double new_integral_max);
  void setIntegralRange(double new_integral_range);
  void clearSumError();
  void setDerivativeTolerance(double new_derivative_tolerance);
  void setSmallBigErrorDuration(double new_small_error_duration,
                                double new_big_error_duration);
  void setArrive(bool new_arrive);
  void reset();

  bool targetArrived();
  double getI();
  double getOutput();
  double update(double input);

private:
  static int sign(double number);

  double target;
  bool arrived;
  bool arrive;
  double small_error_tolerance;
  double big_error_tolerance;
  double small_error_duration;
  double big_error_duration;
  double small_check_time;
  double big_check_time;
  bool first_time;
  double kp;
  double ki;
  double kd;
  double integral_range;
  double integral_max;
  double derivative_tolerance;
  double error_tolerance;
  double current_error;
  double previous_error;
  double sum_error;
  double proportional;
  double integral;
  double derivative;
  double output;
};
