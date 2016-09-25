#include <Wire.h>
#include "HMC5883L.h"
#include "AFMotor.h"

HMC5883L compass;
AF_DCMotor leftMotor(3, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor rightMotor(4, MOTOR12_64KHZ); // create motor #2, 64KHz pwm


int XPos, YPos;
int targetX, targetY;
double trueBearing, relBearing, fieldBearing;
int compOffset = 0, targetBearingOffset = 0;
Vector data;

void updateBearings(){
    data  = compass.readNormalize();
    trueBearing  = (data.YAxis!=0) ? atan2(data.XAxis, data.YAxis)/0.0174532925 : (data.XAxis>0 ? 0.0 : 180.0);
    
    fieldBearing = trueBearing - compOffset;
    fieldBearing = fieldBearing<-180?360+fieldBearing:(fieldBearing>180?fieldBearing+360:fieldBearing);
    
    relBearing = fieldBearing - targetBearingOffset;
    relBearing = relBearing<-180?360+relBearing:(relBearing>180?relBearing+360:relBearing);
}

int getDir(int x, int y){
    int dx = x-XPos, dy = y-YPos;
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
    updateBearings();
    //targetBearingOffset = getDir(targetX, targetY);
    if(abs(relBearing)>10){
        int motSpeed = (double)1.41666*abs(relBearing);
        if(relBearing>0){
            motSpeed = 1.41666*relBearing;
            leftMotor.setSpeed(motSpeed);
            rightMotor.setSpeed(motSpeed);
            leftMotor.run(BACKWARD);
            rightMotor.run(FORWARD);
        }

        else if (relBearing<0){
            motSpeed = 1.41666*relBearing;
            leftMotor.setSpeed(motSpeed);
            rightMotor.setSpeed(motSpeed);
            leftMotor.run(FORWARD);
            rightMotor.run(BACKWARD);
        }
    }
    else if(XPos!=targetX || YPos != targetY){
        leftMotor.setSpeed(motSpeed-1.41666*relBearing);
        rightMotor.setSpeed(motSpeed+1.41666*relBearing);
        leftMotor.run(FORWARD);
        rightMotor.run(FORWARD);
    }
    else{
        leftMotor.run(RELEASE);
        rightMotor.run(RELEASE);
    }
}

