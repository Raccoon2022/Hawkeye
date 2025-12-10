// Need to confirm pin numbers on both ends
#define LORA_UART_TX 16
#define LORA_UART_RX 17

// Sensor output variables
/*
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0; 
double magno_x = 0.0, magno_y = 0.0, magno_z = 0.0;
double altitude = 0.0, lat = 0.0, longi = 0.0; 
float pressure = 0.0, temperature = 0.0;
unsigned long time_since_launch = 0UL;
*/

// Realized conversion to integer or floating point values is pointless b/c will be converting back to string again
// Also realized lessons learnt during live testing say that the printing function might be suffering errors too
String accel_x, accel_y, accel_z;
String gyro_x, gyro_y, gyro_z; 
String magno_x, magno_y, magno_z;
String altitude, lat, longi; 
String pressure, temperature;
String time_since_launch;

String msg;
String msg_values;
String txID;
String datalen;
String rssi;
String snr;
int keyPos[25];

void setup() {
  //Setting up wireless communication
  Serial2.begin(115200, SERIAL_8N1, LORA_UART_RX, LORA_UART_TX);
  //not adding delays here because why?
  Serial2.println("AT+ADDRESS=2");
  Serial2.println("AT+NETWORKID=5");
  Serial2.println("AT+BAND?");
  Serial2.println("AT+PARAMETER=5,9,1,12"); //This should maximize data throughput; expected data rate of 62.5kbps (kilobits per second)
  Serial2.println("AT+MODE?");

}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial2.available()){
    msg = Serial2.readString();
  }
  msg.trim();
  //msg_values = msg.substring(5); //Should only grab variable values, removing the +RCV= prefix
  //keyPos[0] = msg_values.indexOf(",");
  // 13 should be the number of commas, but doing 12 because already found one?
  Serial.println(msg_values);
  for(int i = 1; i <= 18; i++){
    keyPos[i] = msg.indexOf(",",keyPos[i-1]+1);
  }
  txID = msg.substring(5, keyPos[1]);
  datalen = msg.substring(keyPos[1]+1, keyPos[2]);
  //time_since_launch = msg.substring(keyPos[2]+1, keyPos[3]).toDouble();
  time_since_launch = msg.substring(keyPos[2]+1, keyPos[3]);
  //accel_x = msg.substring(keyPos[3]+1, keyPos[4]).toDouble();
  accel_x = msg.substring(keyPos[3]+1, keyPos[4]);
  //accel_y = msg.substring(keyPos[4]+1, keyPos[5]).toDouble();
  accel_y = msg.substring(keyPos[4]+1, keyPos[5]);
  //accel_z = msg.substring(keyPos[5]+1, keyPos[6]).toDouble();
  accel_z = msg.substring(keyPos[5]+1, keyPos[6]);
  //gyro_x = msg.substring(keyPos[6]+1, keyPos[7]).toDouble();
  gyro_x = msg.substring(keyPos[6]+1, keyPos[7]);
  //gyro_y = msg.substring(keyPos[7]+1, keyPos[8]).toDouble();
  gyro_y = msg.substring(keyPos[7]+1, keyPos[8]);
  //gyro_z = msg.substring(keyPos[8]+1, keyPos[9]).toDouble();
  gyro_z = msg.substring(keyPos[8]+1, keyPos[9]);
  //magno_x = msg.substring(keyPos[9]+1, keyPos[10]).toDouble();
  magno_x = msg.substring(keyPos[9]+1, keyPos[10]);
  //magno_y = msg.substring(keyPos[10]+1, keyPos[11]).toDouble();
  magno_y = msg.substring(keyPos[10]+1, keyPos[11]);
  //magno_z = msg.substring(keyPos[11]+1, keyPos[12]).toDouble();
  magno_z = msg.substring(keyPos[11]+1, keyPos[12]);
  //altitude = msg.substring(keyPos[12]+1, keyPos[13]).toDouble();
  altitude = msg.substring(keyPos[12]+1, keyPos[13]);
  //lat = msg.substring(keyPos[13]+1, keyPos[14]).toDouble();
  lat = msg.substring(keyPos[13]+1, keyPos[14]);
  //longi = msg.substring(keyPos[14]+1, keyPos[15]).toDouble();
  longi = msg.substring(keyPos[14]+1, keyPos[15]);
  //temperature = msg.substring(keyPos[15]+1, keyPos[16]).toFloat();
  temperature = msg.substring(keyPos[15]+1, keyPos[16]);
  //pressure = msg.substring(keyPos[16]+1, keyPos[17]).toFloat();
  pressure = msg.substring(keyPos[16]+1, keyPos[17]);
  rssi = msg.substring(keyPos[17]+1, keyPos[18]);
  snr = msg.substring(keyPos[18]+1);
  
  Serial.println(time_since_launch+"ms "+"a: "+accel_x+", "+accel_y+", "+accel_z+" (mg) "+"gyr: "+gyro_x+", "+gyro_y+", "+gyro_z+" (deg/s) "+"mag: "+magno_x+", "+magno_y+", "+magno_z+" (uT) "+"pressr: "+pressure+" (Pa) "+" temp: "+temperature+"°C"+"gps: "+altitude+"m"+", "+lat+"°N"+", "+longi+"°E");
  //Serial.println(String(time_since_launch)+"ms "+"a: "+String(accel_x)+", "+String(accel_y)+", "+String(accel_z) +" gy: "+String(gyro_x)+", "+String(gyro_y)+", "+String(gyro_z)+" mag: "+String(magno_x)+", "+String(magno_y)+", "+String(magno_z)+" "+"pressr: "+String(pressure)+" "+" temp: "+String(temperature)+" gps: "+String(altitude)+", "+String(lat)+", "+String(longi));
  

}
