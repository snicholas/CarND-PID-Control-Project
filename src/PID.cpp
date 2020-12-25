#include <uWS/uWS.h>
#include "PID.h"
#include <math.h>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  d_error = 0;
  p_error = 1;
  i_error = 1;
}

void PID::UpdateError(double cte)
{
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError()
{
  /**
   * TODO: Calculate and return the total error
   */
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  if (steer < -1)
  {
    steer = -1;
  }
  if (steer > 1)
  {
    steer = 1;
  }
  return steer;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
void PID::printparams()
{
  std::cout << "Optimized params" << std::endl;
  for (uint i = 0; i < 3; ++i)
  {
    std::cout << i << " - " << p[i] << std::endl;
  }
  std::cout << "**********************" << std::endl;
  // Kp = p[0];
  // Ki = p[1];
  // Kd = p[2];
}
void PID::Twiddle()
{
  if(cur_p_i==1){
    cur_p_i++;
  }
  double err = pow(TotalError(), 2);
  std::cout << cur_p_i << " " << p_phase[cur_p_i] << " "
            << best_err << " " << err << std::endl;

  if (p_phase[cur_p_i] == 0)
  {
    p[cur_p_i] += dp[cur_p_i];
    p_phase[cur_p_i] = 1;
  }
  else if (err < best_err && p_phase[cur_p_i] == 1)
  {
    best_err = err;
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    dp[cur_p_i] *= 1.1;
    p_phase[cur_p_i] = 0;
    cur_p_i = (cur_p_i + 1) % 3;
  }
  else
  {
    if (p_phase[cur_p_i] == 1)
    {
      p[cur_p_i] -= 2 * dp[cur_p_i];
      p_phase[cur_p_i] = 3;
    }
    else
    {
      if (err < best_err)
      {
        best_err = err;
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        dp[cur_p_i] *= 1.1;
        p_phase[cur_p_i] = 0;
        cur_p_i = (cur_p_i + 1) % 3;
      }
      else
      {
        p[cur_p_i] += dp[cur_p_i];
        // Kp = p[0];
        // Ki = p[1];
        // Kd = p[2];
        dp[cur_p_i] *= 0.9;
        p_phase[cur_p_i] = 0;
        cur_p_i = (cur_p_i + 1) % 3;
      }
    }
  }
  // p[cur_p_i] += dp[cur_p_i];

  // p_phase[cur_p_i] = (p_phase[cur_p_i] + 1) % 4;
  // if (p_phase[cur_p_i] == 0)
  // {
  //   cur_p_i = (cur_p_i + 1) % 3;
  // }
}
