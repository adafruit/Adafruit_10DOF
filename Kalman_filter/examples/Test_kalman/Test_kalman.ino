#include <Kalman_filter.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

/*init a kalman_filter class with dt equal to 0,5 seconde*/
Kalman_filter kalman=Kalman_filter(0.5);

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro=Adafruit_L3GD20_Unified(200);

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
   if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }

  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

void setup() {
  float offset_gyro[3]={0,0,0};
  int i;
  Serial.begin(9600);
  Serial.println("Kalman Filter Test."); Serial.println("");
  
  initSensors();
  
  /* Get a new gyro event  and just drop the first value*/ 
  sensors_event_t gyro_event; 
  gyro.getEvent(&gyro_event);
  delay(100);
  /*Do the calibration of the gyro*/
  Serial.println("Starting the calibration. Don't move the sensor during the calibration."); Serial.println("");
  for(i=0;i<20;i++){
      gyro.getEvent(&gyro_event);
      offset_gyro[0]+=gyro_event.gyro.x;
      offset_gyro[1]+=gyro_event.gyro.y;
      offset_gyro[2]+=gyro_event.gyro.z;
      delay(50);
  }
  for(i=0;i<3;i++){
    offset_gyro[i]/=20;
  }
  Serial.println("Calibration done."); Serial.println("");
  /* Get a new accel event */ 
  sensors_event_t accel_event;
  accel.getEvent(&accel_event);
  
  /*Initialise the kalman filter */ 
  if(kalman.begin(&accel_event,offset_gyro)<0){
    /* There was a problem detecting... check accel_event structur*/
    Serial.println(F("Ooops, accel_event is NULL... Check your accel_event structur!"));
    while(1);
  }
  delay(500);
}

void loop() {
  sensors_event_t gyro_event; 
  sensors_event_t accel_event;
  sensors_vec_t orientation;
  
   /* Get a new accel event */ 
  accel.getEvent(&accel_event);
  
  /* Get a new gyro event */ 
  gyro.getEvent(&gyro_event);
  
  /*update the pitch and the roll*/
  kalman.update(&accel_event, &gyro_event, &orientation);

  Serial.print("Pitch: "); Serial.print(orientation.pitch); Serial.print("\n");
  Serial.print("Roll: "); Serial.print(orientation.roll); Serial.print("\n\n");
  
  delay(500);
}

