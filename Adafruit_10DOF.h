/***************************************************************************
  This is a library for the Adafruit 10DOF Breakout

  Designed specifically to work with the Adafruit 10DOF Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __ADAFRUIT_10DOF_H__
#define __ADAFRUIT_10DOF_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Wire.h>



/** Sensor axis */
typedef enum
{
  SENSOR_AXIS_X  = (1),
  SENSOR_AXIS_Y  = (2),
  SENSOR_AXIS_Z  = (3)
} sensors_axis_t;

/* Driver for the the 10DOF breakout sensors */
class Adafruit_10DOF
{
  public:
    Adafruit_10DOF(void);
    bool begin(void);
    
    bool  accelGetOrientation  ( sensors_event_t *event, sensors_vec_t *orientation );
    bool  magTiltCompensation  ( sensors_axis_t axis, sensors_event_t *mag_event, sensors_event_t *accel_event );
    bool  magGetOrientation    ( sensors_axis_t axis, sensors_event_t *event, sensors_vec_t *mag_orientation );
    bool  fusionGetOrientation ( sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation );

  private:
};

class Adafruit_10DOF_Kalman_Filter{
public:
    Adafruit_10DOF_Kalman_Filter(float _dt);
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
    float Q_angle = 0.01;
    float Q_velocity = 0.01;
    
    //measurement noise
    //---------------------
    float R = 0.03;
    
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
