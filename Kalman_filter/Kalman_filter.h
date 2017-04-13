

#ifndef Kalman_filter_h
#define Kalman_filter_h

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

#define RAD_TO_DEGREES 180.0/PI


class Kalman_filter{
public:
    Kalman_filter(float _dt);
    bool begin(sensors_event_t *accel_event,float offset_gyro[3]);
    bool update(sensors_event_t *accel_event,sensors_event_t *gyro_event,sensors_vec_t *orientation);
    void set_Q_angle(float _Q_angle);
    void set_Q_velocity(float _Q_velocity);
    void set_R(float _R);
private:
    //time between 2 update in seconde.
    float dt;
    
    //Calibration of the gyroscope
    float gyro_cal[3]={0,0,0};

    //Axis calulate with the accelerometer sensor
    //------------------------------
    float Accel_pitch = 0;
    
    float Accel_roll = 0;
    
    //Prediction Matrix
    //-----------------------------
    float Predicted_pitch = 0;
    float Velocity_pitch= 0;
    
    float Predicted_roll = 0;
    float Velocity_roll= 0;
    
    
    //noise in the covariance matrix
    //---------------------
    float Q_angle = 0.1;
    float Q_velocity = 0.1;
    
    //measurement noise
    //---------------------
    float R = 5;
    
    //Covariance Matrix
    //--------------------
    float PX00 = 0;
    float PX01 = 0;
    float PX10 = 0;
    float PX11 = 0;
    
    float PY00 = 0;
    float PY01 = 0;
    float PY10 = 0;
    float PY11 = 0;
    
    //Matrix of Kalman gain.
    //---------------
    float KX0, KX1;
    
    float KY0, KY1;
    
    void accelGetOrientation(sensors_event_t *accel_event);
};

#endif
