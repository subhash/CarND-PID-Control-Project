#include "PID.h"
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;

  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;
}

void PID::UpdateError(double cte) {
  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;
}

double PID::TotalError() {
  double prop = this->Kp * this->p_error;
  double diff = this->Kd * this->d_error;
  double integ = this->Ki * this->i_error;
  double err =  - prop - diff - integ;
  return err;
}

