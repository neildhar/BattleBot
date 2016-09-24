#include <Wire.h>
#include "HMC5883L.h"

HMC5883L compass;
void setup(){
    Serial.begin(9600);
    while (!compass.begin()){
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(50);
    }
}

void loop(){
    Vector data  = compass.readNormalize();
    double bearing  = (data.YAxis!=0) ? atan2(data.XAxis, data.YAxis)/0.0174532925 : (data.XAxis>0 ? 0.0 : 180.0);
    Serial.println(bearing);
}

