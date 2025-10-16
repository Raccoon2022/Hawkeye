
// Will just be ESP32
// To Do:
/*
-Creating setup code for sensors
-Storing sensor data into  relevant variables so rocket logic works
-Recording sensor data into a file on the SD card (I assume we're still trying to do it like last semester)
*/

// Variables
int rocket_state = 0; // 0 is on pad, 1 is in flight

double gravity = 9.8; // acceleration due to gravity in m/s^2
double gravity_margin = 0.0; // same thing as time margin, but for gravity

double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0; // acceleration values
double accel_strength = 0.0; 
/*
data storage variable so that magnitude of acceleration function isn't called multiple times per loop,
which might result in slightly different values
*/

unsigned long time_margin = 0UL; // just a fudge/margin factor, as unlikely the esp32 will directly measure the exact time of apogee
unsigned long time_since_launch = 0UL; // self explanatory
unsigned long apogee_time = 0UL; // the time of apogee as calculated by OpenRocket
unsigned long launch_time = 0UL; // self explanatory

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
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
