#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Kp_v, double Ki_v, double Kd_v) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  Kp_v_ = Kp_v;
  Ki_v_ = Ki_v;
  Kd_v_ = Kd_v;
  p_error_ = 0;
  i_error_ = 0;
  d_error_ = 0;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
  return p_error_+i_error_+d_error_;
}

