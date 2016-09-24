#include <Wire.h>

 void setup() {
  //pinMode(53, INPUT);
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

}
signed short x,y,z;
float heading;
unsigned long timei=0;
void loop() {
  

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(0x1E);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(0x1E, 6);
  while(Wire.available()<6);
    x = int(Wire.read())<<8; //X msb
    x |= Wire.read(); //X lsb
    z = int(Wire.read())<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = int(Wire.read())<<8; //Y msb
    y |= Wire.read(); //Y lsb
  heading = (y!=0) ? atan2(x, y)/0.0174532925 : (x>0 ? 0.0 : 180.0);
  if(heading < 0) heading+=360;
  heading=360-heading; // N=0/360, E=90, S=180, W=270
  Serial.println(heading);
  timei = micros();  
  //Print out values of each axis
  /*Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);*/

}
