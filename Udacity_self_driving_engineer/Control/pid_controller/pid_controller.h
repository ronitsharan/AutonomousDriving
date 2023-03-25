/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

   /**
   * TODO: Create the PID class
   **/
class PID {
public:
   /*
   * Errors
   */
   double cte0;
   //double difference_cte;
   //double total_cte;
   /*
   * Coefficients
   */
   double Kp;
   double Ki;
   double Kd;
   /*
   * Output limits
   */
   double action;
   double output_lim_max;
   double output_lim_min;
    /*
    * Delta time
    */
   //double delta_t;
   double delta_time, I;
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
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);
  

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    //double TotalError(bool debugger=false);
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);


};

#endif //PID_CONTROLLER_H


