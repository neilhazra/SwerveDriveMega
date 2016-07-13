#ifndef PID_h
#define PID_h

#include "Arduino.h"


class PID
{
public:
  PID(double* input,double* output,int setpoint, int sampleTime, double kp, double ki, double kd);
  void isEnabled(bool enabled);
  double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax);
  double getError();
  double getSetpoint();
  void setSetpoint(double setpoint);
  void setOutputLimits(double min, double max);
  int getSampleTime();
  void resetPID();
  void Compute();
private:
  double outputMax;
  double outputMin;
  bool enabled = true;
  int sampleTime;
  unsigned long prevTime;
  double* output;
  double error;
  double* input;
  double setpoint;
  double prevInput;
  double proportional;
  double integral;
  double derivative;
  double kp, ki, kd;
};
PID::PID(double* input, double* output,int setpoint, int sampleTime, double kp, double ki, double kd)  {
  PID::sampleTime = sampleTime;
  this->setpoint = setpoint;
  this->sampleTime = sampleTime;
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->input = input;
  this->output = output;
  prevTime = 0;
  outputMin = -1;
  outputMax = 1;
}
void PID::Compute()
{
  if(enabled) {
      if(millis()-prevTime >= sampleTime)  {
        error = setpoint-*input;
        integral += ki*error;
          if(integral>outputMax) integral = outputMax;
          else if(integral<outputMin) integral = outputMin;
        derivative = kd * (*input - prevInput);

        proportional = kp*error;

        *output = proportional + integral + derivative;
          if(*output>outputMax) *output = outputMax;
          if(*output<outputMin) *output = outputMin;
        prevTime = *input;
        prevTime = millis();
      }
  }
}
void PID::resetPID() {
  proportional = 0;
  integral = 0;
  derivative = 0;
}
void PID::setOutputLimits(double min, double max)  {
  outputMax = max;
  outputMin = min;
}
void PID::setSetpoint(double setpoint) {
  this->setpoint = setpoint;
  PID::resetPID();
}
double PID::getError() {
  return error;
}
int PID::getSampleTime() {
  return sampleTime;
}
void PID::isEnabled(bool enabled) {
  if(this->enabled != enabled)  {
    PID::resetPID();
  }
  this->enabled = enabled;
}

#endif
