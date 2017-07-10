#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error_s_;
  double i_error_s_;
  double d_error_s_;
  double p_error_v_;
  double i_error_v_;
  double d_error_v_;

  /*
  * Coefficients
  */ 
  double Kp_s_;
  double Ki_s_;
  double Kd_s_;
  double Kp_v_;
  double Ki_v_;
  double Kd_v_;
  
  /*
  * Previous data variables
  */ 
  bool prev_meas_;
  double s_sum_cte_;
  double v_sum_cte_;
  double s_prev_cte_;
  double v_prev_cte_;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_s, double Ki_s, double Kd_s, double Kp_v, double Ki_v, double Kd_v);

  /*
  * Update the PID error variables given cross track errors and time interval between measurements.
  */
  void UpdateError(double s_cte, double v_cte, double dt);

  /*
  * Calculate the total PID error for Steering.
  */
  double TotalSteeringError();
  
  /*
  * Calculate the total PID error for Steering.
  */
  double TotalThrottleError();
};

#endif /* PID_H */
