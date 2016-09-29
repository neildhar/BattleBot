#include <Wire.h>
#include "HMC5883L.h"
#include "AFMotor.h"
#define compKp 3
#define compKi 0.05
#define compKd 0.01


HMC5883L compass;
AF_DCMotor leftMotor(3, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor rightMotor(4, MOTOR12_64KHZ); // create motor #2, 64KHz pwm


int XPos=0, YPos=0, motSpeed, compAlignSpeed;
int targetX=0, targetY=0;
double trueBearing, relBearing, fieldBearing;
int compOffset = 0, targetBearingOffset = 0;
Vector data;
unsigned long long lastTime;
int lastError;
long long int errorTotal;
long int dt;

int sign(int n){
    return n==0?0:n/abs(n);
}

void updateBearings(){
    data  = compass.readNormalize();
    trueBearing  = (data.YAxis!=0) ? atan2(data.XAxis, data.YAxis)/0.0174532925 : (data.XAxis>0 ? 0.0 : 180.0);
    
    fieldBearing = trueBearing - compOffset;
    fieldBearing = fieldBearing<-180?360+fieldBearing:(fieldBearing>180?fieldBearing-360:fieldBearing);
    
    relBearing = fieldBearing - targetBearingOffset;
    relBearing = relBearing<-180?360+relBearing:(relBearing>180?relBearing-360:relBearing);
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
    compass.setSamples(HMC5883L_SAMPLES_4);
}

void loop(){
    //READ DATA FROM RECEIVER HERE
    //CHECK FOR RED LINES AND USE RECEIVER DATA
    //SET COURSE
    updateBearings();
    //targetBearingOffset = getDir(targetX, targetY);
    if(sign(relBearing)!=sign(lastError))
            errorTotal=0;
        dt = millis()-lastTime;
        errorTotal += relBearing*dt;
        compAlignSpeed = compKp*relBearing+compKi*errorTotal+compKd*((relBearing-lastError)/dt);
        lastTime = millis();
        lastError = relBearing;
        Serial.println(relBearing);
    if(abs(relBearing)>10){
        leftMotor.runWrapper(-compAlignSpeed);
        rightMotor.runWrapper(compAlignSpeed);
    }
    else if(XPos!=targetX || YPos != targetY){
        motSpeed = 200;
        leftMotor.runWrapper(motSpeed-compAlignSpeed);
        rightMotor.runWrapper(motSpeed+compAlignSpeed);
    }
    else{
        leftMotor.runWrapper(0);
        rightMotor.runWrapper(0);
    }
}

