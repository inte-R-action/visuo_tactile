/******************************************************************
  ICP10xx library is used for the TDK InvenSense ICP-101xx
 series of barometric pressure sensors.

 Kalman filter was implemented to use roll and pitch values. 
 This code is ready to test with UR3 robot.
 
******************************************************************/
#include <ros.h>
#include <std_msgs/Float32.h>
#include<Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <icp101xx.h> 

//Set up the ros node and publisher
std_msgs::Float32 pressure_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);
ros::NodeHandle nh;

//long int pressure;
//int temp_2;

// Sensor is an ICP101xx object
ICP101xx sensor;

#define I2C_ADDR 0x68
#define PRE_ADDR 0x63


void setup() {
sensor.begin(); //force sensor
Wire.begin();
nh.initNode();
nh.advertise(pub_pressure);
//TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
setupICM();
}

long publisher_timer;

void loop() {
if (millis() > publisher_timer) {
  
//################## FORCE SENSOR #####################################
    // Optional: Measurement mode
    //    sensor.FAST: ~3ms
    //    sensor.NORMAL: ~7ms (default)
    //    sensor.ACCURATE: ~24ms
    //    sensor.VERY_ACCURATE: ~95ms
    sensor.measure(sensor.NORMAL);
    pressure_msg.data  = sensor.getPressurePa()/1000;
    pub_pressure.publish(&pressure_msg);

    publisher_timer = millis() + 10; //publish once a second

}

nh.spinOnce();
}


void setupICM(){
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM when AD0 is low
  Wire.write(0x6B); //Accessing the register 6B for Power Management
  Wire.write(0x01); //Setting the sleep register to 0.
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope configuration 
  Wire.write(0x00); //Setting the Gyro to full scale +/- 250 deg
  Wire.endTransmission();
  
  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer configuration
  Wire.write(0x00); //Setting the accel to  +/- 2 g
  Wire.endTransmission(); 

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x37); //Accessing register INT_PIN_CFG
  Wire.write(0x02); //Setting BAYPASS_EN to 1
  Wire.endTransmission();

  Wire.beginTransmission(I2C_ADDR); //I2C address of the ICM
  Wire.write(0x6A); //Accessing register USER_CTRL
  Wire.write(0x00); //Setting I2C_MST_EN (I2C_IF_DIS) to 0
  Wire.endTransmission(); 

 delay(100); // Wait for sensor to stabilize
  }
