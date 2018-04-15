#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  Ks.push_back(Kp);
  Ks.push_back(Ki);
  Ks.push_back(Kd);
  
  d_error = 0.0;
  p_error = 0.0;
  i_error = 0.0;
  
}

void PID::UpdateParams(vector<double>& Ks) {
  
  this->Kp = Ks[0];
  this->Ki = Ks[1];
  this->Kd = Ks[2];
  
}

void PID::UpdateError(double cte) {
  
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
}

double PID::TotalError() {
  
  double total_error = Kp * p_error + Ki * i_error + Kd * d_error;
  
  // normalization
  if(total_error < -1.0) total_error = -1.0;
  if(total_error >  1.0) total_error =  1.0;
  
  return total_error;
  
}

