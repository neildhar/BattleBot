#include <Wire.h>
#include "HMC5883L.h"

HMC5883L compass;

int XPos, YPos;

int getDir(int x, int y){
    int dx = x-xPos, dy = y-YPos;
    if(dy==0&&dx==0)
        return -1;
    else if(dy==0)
        return dx>0?90:-90;
    else if(dx==0)
        return dy>0?0:180;
    int basic = abs(atan2(dy,dx));
    if(dy>0 && dx >0)
        return 90-basic;
    else if(dx<0 && dy>0)
        return basic-90;
    else if(dx>0 && dy<0)
        return basic+90;
    else if(dx<0 && dy<0)
        return -90-basic;
}
void setup(){
    Serial.begin(9600);
    while (!compass.begin()){
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(50);
    }
}

void loop(){
    //READ DATA FROM RECEIVER HERE
    //CHECK FOR RED LINES AND USE RECEIVER DATA
    //SET COURSE
    Vector data  = compass.readNormalize();
    double bearing  = (data.YAxis!=0) ? atan2(data.XAxis, data.YAxis)/0.0174532925 : (data.XAxis>0 ? 0.0 : 180.0);
    Serial.println(bearing);
    Serial.println(data.YAxis);
}

