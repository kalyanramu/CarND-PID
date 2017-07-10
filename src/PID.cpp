#include "PID.h"
#include <iostream>
#include <chrono> //http://www.compspace.org/how-to-measure-elapsed-time-in-cpp-code/

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {

}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = 0.1;
    this->Ki = 0.1;
    this->Kd = 0.1;

    p_error = 0;
    d_error = 0;
    i_error = 0;

    prev_time = std::chrono::system_clock::now();
}

void PID::UpdateError(double cte) {
    //cout << "Cross Track Error: " << cte << endl;
    current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = (current_time - prev_time);//current_time - prev_time;
    dt = elapsed_seconds.count();
    cout << "duration: " << dt << endl;

    p_error = cte;
    d_error = (cte - prev_cte)/dt;
    i_error += cte*dt;
    prev_cte = cte;

    prev_time = current_time;
}

double PID::TotalError() {
return p_error*Kp + d_error*Kd + i_error*Ki;
}

