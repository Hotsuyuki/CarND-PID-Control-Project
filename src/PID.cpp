#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  step_ = 1;
  one_lap_step_ = 3500;

  err_ = 0;
  best_err_ = std::numeric_limits<double>::max();

  is_Twiddle_ = true; //false;
  was_added_ = true;

  dp_ = {0.1*Kp_, 0.1*Ki_, 0.1*Kd_};
  param_idx_ = 0;

  cout << "Initial param index: param_idx_=" << param_idx_ << endl;
  cout << "Initial parameters: Kp_=" << Kp_ << ", Ki_=" << Ki_ << ", Kd_=" << Kd_ << endl;
  cout << "Initail deltas: dp_[0]=" << dp_[0] << ", dp_[1]=" << dp_[1] << ", dp_[2]=" << dp_[2] << endl << endl;
}


void PID::TwiddleParameter(double delta) {
  switch (param_idx_) {
    case 0:
      Kp_ += delta;
      return;

    case 1:
      Ki_ += delta;
      return;

    case 2:
      Kd_ += 0.1*delta;
      return;

    default:
      cout << "param_idx_ is more than the size of array" << endl;
      return;
  }
}

void PID::UpdateError(double cte) {
  // Update each error
  if (step_ == 1) {
    d_error_ = 0; // Cause there is no difference between previous and current CTE in first loop.
  } else {
    d_error_ = cte - p_error_; // Casue `p_error_` is previous CTE now.
  }
  p_error_ = cte; // `p_error_` is now current CTE.
  i_error_ += cte;

  // Update the total error for Twiddle after one fifth of the lap is passed
  if (step_%one_lap_step_ > one_lap_step_/5) {
    err_ += pow(cte, 2);
  }

  // Tuning the parameters after one lap is passed
  if (is_Twiddle_ && step_%one_lap_step_==0) {
    if (err_ < best_err_) { // If the error was improved,
      cout << "The best error was improved!!!" << endl;
      cout << "So try adding 1.1*dp[" << param_idx_ << "] in NEXT time" << endl;

      // Update the best error
      best_err_ = err_;

      // Try the new parameter in NEXT time
      dp_[param_idx_] *= 1.1;

      // Increment the parameter index
      param_idx_ = (param_idx_ + 1) % dp_.size();
      TwiddleParameter(+dp_[param_idx_]);
      was_added_ = true;

    } else { // If the error was not improved,

      if (was_added_) { // If the dp was added in the last step,
        cout << "The best error was not improved by adding 1.0*dp[" << param_idx_ << "]..." << endl;
        cout << "So try subtracting 1.0*dp[" << param_idx_ << "] in THIS time" << endl;

        // Try the new parameter in THIS time (by subtracting 2.0*dp)
        TwiddleParameter(-2*dp_[param_idx_]);
        was_added_ = false;
      } else { // If the dp was subtracted in the last step,
        cout << "The best error was not improved by subtracting 1.0*dp[" << param_idx_ << "]..." << endl;
        cout << "So try adding 0.9*dp[" << param_idx_ << "] in NEXT time" << endl;

        // Return the parameters back to origianl value by adding 1.0*dp
        TwiddleParameter(+dp_[param_idx_]);

        // Try the new parameter in NEXT time
        dp_[param_idx_] *= 0.9;

        // Increment the parameter index
        param_idx_ = (param_idx_ + 1) % dp_.size();
        TwiddleParameter(+dp_[param_idx_]);
        was_added_ = true;
      }
    }

    // Reset the total error
    err_ = 0;

    cout << "The " << step_/one_lap_step_ << "th lap" << endl;
    cout << "best_err_=" << best_err_ << endl;
    cout << "New param index: param_idx_=" << param_idx_ << endl;
    cout << "New parameters: Kp_=" << Kp_ << ", Ki_=" << Ki_ << ", Kd_=" << Kd_ << endl;
    cout << "New deltas: dp_[0]=" << dp_[0] << ", dp_[1]=" << dp_[1] << ", dp_[2]=" << dp_[2] << endl << endl;
  }

  step_++;
}


double PID::TotalError() {
  return -Kp_*p_error_ - Ki_*i_error_ - Kd_*d_error_;
}
