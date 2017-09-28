#ifndef PID_H
#define PID_H

#include <iostream>
#include <float.h>
#include <uWS/uWS.h>


class PID {
public:
  /*
  * Errors
  */
  double gains[3];

  double p_error;
  double i_error;
  double d_error;
  double prev_err;
  bool err_initialized;
  double err_squared_sum;
  double mse;

  /*
  * Coefficients
  */ 
  bool TWIDDLE;
  bool twiddle_init;
  bool increment_index;
  bool run;
  double d_twiddle[3];
  double err_sum;
  double best_err;
  int step_num;
  int max_steps;
  double thresh;
  int twiddle_index;
  int twiddle_iter;



  /*
  * Constructor
  */
  PID(double Kp, double Ki, double Kd, bool activate_twiddle);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void Twiddle(uWS::WebSocket<uWS::SERVER> ws);

  void Reset(uWS::WebSocket<uWS::SERVER> ws, bool increment_index);

  /*
  * Calculate the total PID error.
  */
  double GetSteeringInput(double cte, double max_angle, double min_angle);

  double GetMSE(int num_steps);
};

#endif /* PID_H */
