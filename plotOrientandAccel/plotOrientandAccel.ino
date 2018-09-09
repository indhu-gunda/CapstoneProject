#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* My version to plot orientation
 * 
 *  This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
void displayAndWaitForCalibration(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  while (gyro != 3 || accel != 3 || mag != 3) {
  /* Display the individual values */
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.println(mag, DEC);
  bno.getCalibration(&system, &gyro, &accel, &mag);
  }
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.println(mag, DEC);
//
//  Serial.println("Calibration achieved");
}
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  int i;
// Turn off the on-board LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  Serial.begin(9600);
  bno.begin();
//  /* Initialise the sensor */
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
  delay(1000);
  /* Display some basic information on th
  is sensor */
 // displaySensorDetails();
 // displayAndWaitForCalibration();
 bno.setExtCrystalUse(true);
  // Blink LED when done and ready
  
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  for (i = 0; i < 5; i++){
    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(500);              // wait for a second
    digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW
    delay(500);              // wait for a second
  }
  
//
//  /* Optional: Display current status */
//  displaySensorStatus();
}
 /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */
void loop(void)
{
  float roll, pitch, yaw;
  float x_accel, y_accel, z_accel;
  
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event); // This is only Euler
  /* Send the floating point data. */
  /* Took this from the bunny example. */
  yaw = event.orientation.x;   // Was yaw
  pitch = event.orientation.y; // Was pitch
  roll = event.orientation.z;  // Was roll
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  x_accel = acceleration.x();
  y_accel = acceleration.y();
  z_accel = acceleration.z();
// Send yaw, pitch, roll
  Serial.write((byte *) &yaw, sizeof(yaw));
  Serial.write((byte *) &pitch, sizeof(pitch));
  Serial.write((byte *) &roll, sizeof(roll));
  Serial.flush();
 // Send acceleration
  Serial.write((byte *) &x_accel, sizeof(x_accel));
  Serial.write((byte *) &y_accel, sizeof(y_accel));
  Serial.write((byte *) &z_accel, sizeof(z_accel));
  Serial.flush(); // Wait for all data to send.
  
//  /* Optional: Display calibration status */
//  displayCalStatus();
//
//  /* Optional: Display sensor status (debug only) */
//  //displaySensorStatus();
//
//  /* New line for the next sample */
//  Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
