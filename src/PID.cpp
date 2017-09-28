#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd, bool activate_twiddle) {
    gains[0] = Kp;
    gains[1] = Ki;
    gains[2] = Kd;

    TWIDDLE = activate_twiddle;

    d_twiddle[0] = 1;
    d_twiddle[1] = 1;
    d_twiddle[2] = 1;

    err_initialized = false;
    err_sum = 0;
    max_steps = 2500;
    thresh = 0.035;

    twiddle_iter = 0;
    twiddle_init = false;
}

PID::~PID() {}

void PID::UpdateError(double err) {
    if (!err_initialized){
        prev_err = err;
        err_initialized = true;
    }

    p_error = -gains[0] * err;
    i_error = -gains[1] * err_sum;
    d_error = -gains[2] * (err - prev_err);

    prev_err = err;
    err_sum += err;
    err_squared_sum += err * err;

}

double PID::GetMSE(int num_steps){
    return err_squared_sum / num_steps;
}

double PID::GetSteeringInput(double cte, double max_angle, double min_angle) {
    UpdateError(cte);
    double controlEffort = p_error + d_error;

    if(controlEffort > max_angle){
        controlEffort = max_angle;
    } else if(controlEffort < min_angle){
        controlEffort = min_angle;
    }
    return controlEffort;
}

void PID::Reset(uWS::WebSocket<uWS::SERVER> ws, bool increment_index){
  err_initialized = false;
  err_squared_sum = 0;
  err_sum = 0;

  step_num = 0;
  twiddle_iter++;

  if (increment_index){
    twiddle_index = (twiddle_index + 1) % 3;
    //Ignore integral term. Only looking for PD controller.
    if (twiddle_index == 1){twiddle_index += 1;}
    gains[twiddle_index] += d_twiddle[twiddle_index];

    run = true;
  } else {
    run = false;
  }

  cout << "New params" << endl;
  cout << "Kp: " << gains[0] << endl;
  cout << "Ki: " << gains[1] << endl;
  cout << "Kd: " << gains[2] << endl;

  // Reset simulator
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

}

void PID::Twiddle(uWS::WebSocket<uWS::SERVER> ws) {
  step_num++;


  if(step_num >= max_steps){
    //init best_err
    if(!twiddle_init){
      cout << "initializing twiddle" << endl;
      twiddle_init = true;
      best_err = GetMSE(step_num);
      Reset(ws, true);
      return;
    }

    double mse = GetMSE(step_num);
    cout << "mse : " << mse << endl;
    cout << "Best: " << best_err << endl;

    if(mse < thresh){
      cout << "MSE is under the threshold. Twiddling Complete" << endl;
      cout << "Kp = " << gains[0] << "\n"
           << "Ki = " << gains[1] << "\n"
           << "Kd = " << gains[2] << "\n" << endl;
      ws.close();
    } else {

      if (mse < best_err){
        best_err = mse;
        d_twiddle[twiddle_index] *= 1.1;
        Reset(ws, true);
      } else {
          if(run){
            gains[twiddle_index] -= 2 * d_twiddle[twiddle_index];

            Reset(ws, false);
            return;
          } else {
            gains[twiddle_index] += d_twiddle[twiddle_index];
            d_twiddle[twiddle_index] *= (1.1);
            Reset(ws, true);
            return;
          }
      }
    }
  }
}


