
// Will just be ESP32

/*
Parts/Sensor List:
-Sparkfun 9DoF IMU ICM-20948 (I2C & SPI available)
-HiLetgo Micro SD Card Adapter (SPI available)
-"BMP390 Precision Barometric Pressure and Altimeter Sensor Upgrade Version for BMP280 BMP388 I2C SPI Interface+SH1.0mm 4P Cable"
*/

// To Do:
/*
-Storing sensor data into  relevant variables so rocket logic works
-add functions to improve code readability (one for storing data from sensors in variables, the other writing them to a file)
-Recording sensor data into a file on the SD card (I assume we're still trying to do it like last semester)
*/

//Importing libraries
#include "ICM_20948.h" // IMU library
#include <Adafruit_Sensor.h> // Supporting library for pressure sensor
#include "Adafruit_BMP3XX.h" // Pressure sensor library
#include <SPI.h> // communication library
#include <Wire.h> // communication library
#include <TinyGPSPlus.h> 
#include <SoftwareSerial.h>
// GPS translation Library needs to go here
// All libraries (except SPI which is a default Arduino library) must be installed for this to function properly

// Setting up SPI
#define SPI_CS_BARO 17 // Chip select pin
#define SPI_CS_IMU 16 // Chip select pin
#define SPI_SCK 18 // Serial clock pin
#define SPI_CIPO 19 // Controller In Peripheral Out pin
#define SPI_COPI 23 // Controller Out Peripheral In pin
// Based off: https://www.reddit.com/r/esp32/comments/uai6xz/im_confused_about_the_spi_pins/, https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/hw-reference/esp32/get-started-devkitc.html

#define SEALEVELPRESSURE_HPA (1013.25)

#define SPI_PORT SPI

//Defining ports for flexability reasons
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define UART_TX 2
#define UART_RX 3
static const uint32_t GPSBaud = 4800;

// Variables
// Sensor objects
Adafruit_BMP3XX bmp; // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);  // Software SPI
ICM_20948_I2C myICM;
TinyGPSPlus gps;
SoftwareSerial ss(UART_TX, UART_RX);

//Need to figure out how to setup SPI



double gravity = 9.8; // acceleration due to gravity in m/s^2
double gravity_margin = 0.0; // same thing as time margin, but for gravity

// Sensor output variables
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0; // acceleration values
double accel_strength = 0.0; 
/*
data storage variable so that magnitude of acceleration function isn't called multiple times per loop,
which might result in slightly different values
*/
double altitude = 0.0, lat = 0.0, longi = 0.0;
float pressure = 0.0; // Note that this is in Celsius
float temperature = 0.0; // Note that this is in Pascals, uncorrected

// State of rocket variables
int rocket_state = 0; // 0 is on pad, 1 is in flight
unsigned long time_margin = 0UL; // just a fudge/margin factor, as unlikely the esp32 will directly measure the exact time of apogee
unsigned long time_since_launch = 0UL; // self explanatory
unsigned long apogee_time = 0UL; // the time of apogee as calculated by OpenRocket
unsigned long launch_time = 0UL; // self explanatory

void setup() {
  // put your setup code here, to run once:
  
  //Barometer set-up (following example found here: https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/arduino)
  Serial.begin(115200); // seems this is the same baud rate as the GPS library...that's nice
  while(!Serial);
  //hardware I2C https://randomnerdtutorials.com/getting-started-freenove-esp32-wrover-cam/
  if(!bmp.begin_I2C()){
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  /*IMU set-up: 
  https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide
  https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/PortableC/Example999_Portable/Example999_Portable.ino 
  https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/Arduino/Example1_Basics/Example1_Basics.ino 
  */
  //hopefully starting the wire here doesn't cause issues
  Wire.begin();
  Wire.setClock(400000);
  myICM.begin(Wire, 1);
  //bmp = 0x77 i2c address
  //imu = 0x69
  
  bool initialized = false;
  while(!initialized){
    myICM.begin(Wire, 1);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if(myICM.status != ICM_20948_Stat_Ok){
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

  //GPS setup
  ss.begin(GPSBaud);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Barometer data
  pressure = bmp.readPressure();
  temperature = bmp.readTemperature();

  //IMU Data
  if(myICM.dataReady()){
    myICM.getAGMT();
    delay(30);
    // https://community.sparkfun.com/t/sparkfun-9dof-imu-breakout-icm-20948-compass/47457/2 
    accel_x = myICM.accX();
    accel_y = myICM.accY();
    accel_z = myICM.accZ();
  }

  //GPS Data https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/KitchenSink/KitchenSink.ino 
  while(ss.available() > 0){
    gps.encode(ss.read());
  }
  if(gps.location.isUpdated()){
    lat = gps.location.lat();
    longi = gps.location.longi();
  }
  if(gps.altitude.isUpdated()){
    altitude = gps.altitude.meters();
  }
  
  switch
    case 0:
      accel_strength = magnitude_accel();
      /*
      Launch detection code.
      Determines if measured acceleration is equal to gravity +- an error margin.
      If it is outside this range, it is presumed that the acceleration not being roughly equal to gravity means the rocket
      is now accelerating under power, so rocket state variable is updated to switch to apogee detection logic.
      The launch time is also set; it is needed for calculations in case/rocket state 2.
      */
      if ( (accel_strength < (gravity - gravity_margin)) || (accel_strength > (gravity + gravity_margin)) ){
        launch_time = millis();
        rocket_state = 1;
      }
      break;
    case 1:
      accel_strength = magnitude_accel();
      time_since_launch = millis() - launch_time;
      /* 
      Apogee detection code.
      If statements were nested (instead of in line like the pseudo-code) to improve readability.
      At apogee, it is expected that the only acceleration measured will be gravity, and the time will be roughly 
      what OpenRocket predicted for apogee.
      */
      // Determines if measured acceleration falls within range of gravity +- an error margin.
      if ( (accel_strength >= (gravity - gravity_margin) && (accel_strength <= (gravity + gravity_margin)) ){

        // If acceleration is roughly equal to gravity, then checks if time falls within range of expected apogee time +- an error margin.
        if( (time_since_launch >= (apogee_time - time_margin)) && (time_since_launch <= (apogee_time + time_margin)) ){
          // trigger camera and parachute deployment mechanism
          rocket_state = 2;
        }
      }
      break;
    case 2:
      // this is a dummy case, so that the rocket does not execute any other code after triggering camera and parachute
      time_since_launch = millis();
      break;
  

}

double magnitude_accel(){
  return 0.0;
}
