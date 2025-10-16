
// Will just be ESP32
// to do:
/*


*/

// Variables
int rocket_state = 0; // 0 is on pad, 1 is in flight
double gravity = 9.8;
double gravity_margin = 0.0; // same thing as time margin, but for gravity
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0; // acceleration values
long time_margin = 0.0; // just a fudge/margin factor, as unlikely the esp32 will directly measure the exact time of apogee
long time_since_launch = 0.0; // self explanatory


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  switch
    case 0:
      if (magnitude_accel() != gravity){
        rocket_state = 1
      }
    case 1:
      
      if ((magnitude_accel()) == gravity && (t1 <= time_since_launch <= t2)){
        trigger camera and parachute pyrocharge
      }
  
  */
  magnitude_accel();

}

double magnitude_accel(){
  return 0.0;
}
