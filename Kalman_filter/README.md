#Kalman filter for Adafruit 10DOF Library#

## About this Library ##

This library provide a simple kalman filter for the gyroscope and the accelerometer sensor. To use it,you just have to initialise it with your sensor frequency and use the function begin to give him the first value of the gyroscope and the accelerometer.Then in the loop use the update function.You can look at our example for more information.

## About Kalman filter ##

The Kalmnan filter is a mathematical tool you can use to estimate the states of a dynamical systems based on a series of data.It correct the data with noises wich might be very usefool in the case of this sensor if you use it with an embedded system.
