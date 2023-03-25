/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  //changing the implementation after first project review
  this->Kp = Kp;
  //Kp = Kpi;
  this->Ki = Ki;
  //Ki = Kii;
  this->Kd = Kd;
  //Kd = Kdi;
  this->output_lim_max = output_lim_max;
  //output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_max;
  //output_lim_min = output_lim_mini;
  //this->delta_t = 0.0;
  cte0 = 0;
  //total_cte = 0.0;
  //this->error_pi = 0.0;
  //difference_cte = 0.0;
  //this->error_ii = 0.0;
  //cte = 0.0;
  //this->error_di = 0.0;
  //is_initial = true;
  I = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  /*this->error_pi = cte;
  
  if(this->delta_t> 0.0){
    this->error_di = (cte-error_pi)/this->delta_t;
  }
  else{
    this->error_di = 0.0;
  }
  this->error_ii += cte*this->delta_t;*/
  //Changing after first project review
  /*if(is_initial){
    difference_cte = 0;
    is_initial = false;
  }
  else{
    if(std::abs(delta_t) < 0.0001)
      difference_cte = 0;
    else
      difference_cte = (cte_exist-cte)/delta_t*0.7 + 0.3*difference_cte;  
  }
  total_cte += cte_exist*delta_t;*/
  if(abs(delta_time) < 0.000001) return;
  double P = Kp * cte;
  I += Ki * cte * delta_time;
  double D = Kd * (cte - cte0) / delta_time;
  action = P + I + D;
  cte0 = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    //double control;
    /*control = ((this->Kp*this->error_pi) 
              +(this->Kd*this->error_di)
              +(this->Ki*this->error_ii));
  
    if (control > this->output_lim_max){
      return this->output_lim_max;
    }
    else if (control < this->output_lim_min){
      return this->output_lim_min;
    }
    else {
      return control;
    }*/
    //changing implementation after first project review
    /*control = -Kp*cte - Kd*difference_cte - Ki*total_cte;

    if(debugger){
      std::cout<< "CTE" << cte << " Control = " << control << " P = " << -Kp*cte << "I = " << -Ki*total_cte << " D = " << -Kd*difference_cte << " T = " << delta_t << std::endl;
    }

    control = max(control, output_lim_min);
    control = min(control, output_lim_max);*/
    double control = action;
    if (control < output_lim_min)	control = output_lim_min;
    if (control > output_lim_max)	control = output_lim_max;

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  //this->delta_t = new_delta_time;
  //return this->delta_t;
  //delta_t = new_delta_time;
  delta_time = new_delta_time;
  return delta_time;
}