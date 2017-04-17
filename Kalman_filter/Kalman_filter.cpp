

#include "Kalman_filter.h"


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/

/*!
 @brief  Put the pitch/roll fields in the class variable(Accel_pitch,Accel_roll)
 with the right angular data (in degree)
 
 @param  event         The sensors_event_t variable containing the
 data from the accelerometer
 */
/**************************************************************************/

void Kalman_filter::accelGetOrientation(sensors_event_t *accel_event){
    //Init angle.
    float signOfZ = accel_event->acceleration.z >= 0 ? 1.0F : -1.0F;
    
    
    /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
    /* pitch is positive and increasing when moving upwards                                     */
    /*                                                                                          */
    /*                                 x                                                        */
    /*            pitch = atan(-----------------)                                               */
    /*                          sqrt(y^2 + z^2)                                                 */
    /* where:  x, y, z are returned value from accelerometer sensor                             */
    
    Accel_pitch= accel_event->acceleration.y * accel_event->acceleration.y + accel_event->acceleration.z * accel_event->acceleration.z;
    Accel_pitch= atan2(accel_event->acceleration.x, signOfZ * sqrt(this->Accel_pitch)) * RAD_TO_DEGREES;
    
    
    /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -180<=roll<=180    */
    /* roll is positive and increasing when moving downward                                     */
    /*                                                                                          */
    /*                                 y                                                        */
    /*             roll = atan(-----------------)                                               */
    /*                          sqrt(x^2 + z^2)                                                 */
    /* where:  x, y, z are returned value from accelerometer sensor                             */
    
    Accel_roll= accel_event->acceleration.x * accel_event->acceleration.x + accel_event->acceleration.z * accel_event->acceleration.z;
    Accel_roll= atan2(accel_event->acceleration.y, sqrt(this->Accel_roll)) * RAD_TO_DEGREES;

}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates a new Kalman_filter class
 
 @param  _dt   time between 2 update in second
 */
/**************************************************************************/
Kalman_filter::Kalman_filter(float _dt):dt(_dt){
}


/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  initialise the kalaman filter.
 
 @param  accel_event    The sensors_event_t variable containing 
                        the data from the accelerometer
 
 @param offset_gyro     Contain the offset of each axis(pitch, roll, yaw)
 
 @return Returns true if the operation was successful, false if there
 was an error
 
 */
/**************************************************************************/

bool Kalman_filter::begin(sensors_event_t *accel_event,float offset_gyro[3]){
    int i=0;
    /* Make sure the input is valid, not null, etc. */
    if ( accel_event  == NULL) return false;

    this->accelGetOrientation(accel_event);
    
    /* The first prediction come from the accelemeter sensor.*/
    Predicted_roll=Accel_roll;
    Predicted_pitch=Accel_pitch;
    
    if(offset_gyro!=NULL){
        for (i=0; i<3; i++) {
            this->gyro_cal[i]=offset_gyro[i];
        }
    }
    
    return true;
}

/**************************************************************************/
/*!
 @brief  Set the angle noise in the covariance matrix
 */
/**************************************************************************/
void Kalman_filter::set_Q_angle(float _Q_angle){
    Q_angle=_Q_angle;
}

/**************************************************************************/
/*!
 @brief  Set the velocity noise in the covariance matrix
 */
/**************************************************************************/
void Kalman_filter::set_Q_velocity(float _Q_velocity){
    Q_velocity=_Q_velocity;
}

/**************************************************************************/
/*!
 @brief  Set measurement noise
 */
/**************************************************************************/
void Kalman_filter::set_R(float _R){
    R=_R;
}

/**************************************************************************/
/*!
 @brief  Populates the .pitch/.roll fields in the sensors_vec_t struct
 with the right angular data (in degree)
 
 @param  accel_event         The sensors_event_t variable containing the
 data from the accelerometer
 
 @param  gyro_event         The sensors_event_t variable containing the
 data from the gyroscope
 
 @param  orientation   The sensors_vec_t object that will have it's
 .pitch and .roll fields populated
 
 @return Returns true if the operation was successful, false if there
 was an error
 
 */
/**************************************************************************/

bool Kalman_filter::update(sensors_event_t *accel_event,sensors_event_t *gyro_event,sensors_vec_t *orientation){
    /* Make sure the input is valid, not null, etc. */
    if(accel_event == NULL) return false;
    if(gyro_event == NULL) return false;
    if(orientation == NULL) return false;
    
    
    //update of Prediction.
    //-------------------------------
    Predicted_pitch +=( ( (gyro_event->gyro.x-gyro_cal[0]) *RAD_TO_DEGREES) - Velocity_pitch )*dt;
    
    Predicted_roll +=( ( (gyro_event->gyro.y-gyro_cal[1]) *RAD_TO_DEGREES) - Velocity_roll ) *dt;
    
    
    this->accelGetOrientation(accel_event);
    
    //Predict of the Covariance matrix
    PX00+=dt*(dt*PX11-PX01-PX10+Q_angle);
    PX01-=dt*PX11;
    PX10-=dt*PX11;
    PX11+=Q_velocity*dt;
    
    PY00+=dt*(dt*PY11-PY01-PY10+Q_angle);
    PY01-=dt*PY11;
    PY10-=dt*PY11;
    PY11+=Q_velocity*dt;
    
    //update of the Kalman gain.
    //-------------------------------
    KX0=PX00/(PX00+R);
    KX1=PX01/(PX00+R);
    
    KY0=PY00/(PY00+R);
    KY1=PY01/(PY00+R);
    
    //Update of the Pitch and Roll
    //------------------------
    Predicted_pitch += (Accel_pitch - Predicted_pitch) * KX0;
    Velocity_pitch += KX1 * (Accel_pitch - Predicted_pitch);
    Predicted_roll += (Accel_roll - Predicted_roll) * KY0;
    Velocity_roll +=(Accel_roll - Predicted_roll) * KY1;
    
    orientation->pitch=Predicted_pitch;
    orientation->roll=Predicted_roll;
    
    //update of the covariance matrix
    //-------------------------------------
    PX00 -= KX0*PX00;
    PX01 -= KX0*PX01;
    PX10 -= KX1*PX00;
    PX11 -= KX1*PX01;
    
    PY00 -= KY0*PY00;
    PY01 -= KY0*PY01;
    PY10 -= KY1*PY00;
    PY11 -= KY1*PY01;
    
}
