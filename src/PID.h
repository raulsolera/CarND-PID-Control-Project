#ifndef PID_H
#define PID_H

#include <vector>
using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  vector<double> Ks;
  

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
  void UpdateParams(vector<double>& Ks);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Update the params, to be used by a fine tuning method
   */
  void Init(double Kp, double Ki, double Kd);
  
  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
   * Parameter tunning.
   */
  void TwiddleParameters();
  
  double SumTwiddleParams();
  
};

#endif /* PID_H */
