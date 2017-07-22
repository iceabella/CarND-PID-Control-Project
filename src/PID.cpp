#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_s, double Ki_s, double Kd_s, double Kp_v, double Ki_v, double Kd_v) {
  // PID gains steering and velocity (throttle)
  Kp_s_ = Kp_s;
  Ki_s_ = Ki_s;
  Kd_s_ = Kd_s;
  Kp_v_ = Kp_v;
  Ki_v_ = Ki_v;
  Kd_v_ = Kd_v;
  
  // PID error steering and velocity (throttle)
  p_error_s_ = 0;
  i_error_s_ = 0;
  d_error_s_ = 0;
  p_error_v_ = 0;
  i_error_v_ = 0;
  d_error_v_ = 0;
  
  // Save previous measurement and sum of all errors
  prev_meas_ = false;
  s_sum_cte_ = 0;
  v_sum_cte_ = 0;
  s_prev_cte_ = 0;
  v_prev_cte_ = 0;
  
}

void PID::UpdateError(double s_cte, double v_cte, double dt) {
  
  // *********************** PID CONTROLLER FOR STEERING ANGLE *************************        
  double diff_cte = s_cte - s_prev_cte_;
  s_prev_cte_ = s_cte; // save for next iteration
  s_sum_cte_ += s_cte;
  
  //update steering values
  p_error_s_ = -Kp_s_*s_cte;
  d_error_s_ = -Kd_s_*diff_cte/dt;
  i_error_s_ = -Ki_s_*s_sum_cte_;
    
  // *********************** PID CONTROLLER FOR THROTTLE ************************* 
  double v_diff_cte = v_cte - v_prev_cte_;
  v_prev_cte_ = v_cte; // save for next iteration
  v_sum_cte_ += v_cte;
  
  //update throttle values  
  p_error_v_ = -Kp_v_*v_cte;
  d_error_v_ = -Kd_v_*v_diff_cte/dt; 
  i_error_v_ = -Ki_v_*v_sum_cte_;      
                         
}

double PID::TotalSteeringError() {
  return p_error_s_+i_error_s_+d_error_s_;
}

double PID::TotalThrottleError() {
  return p_error_v_+i_error_v_+d_error_v_;
}

