#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Steps of loop
  */
  unsigned int step_;
  unsigned int one_lap_step_;

  /*
  * Errors for Twiddle
  */
  double err_;
  double best_err_;

  /*
  * Flag for Twiddle
  */
  bool is_Twiddle_;
  bool was_added_;

  /*
  * Delta parameters for Twiddle
  */
  std::vector<double> dp_;
  int param_idx_;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Tuning the parameters.
  */
  void TwiddleParameter(double delta);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
