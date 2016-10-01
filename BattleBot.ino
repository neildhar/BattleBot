#include <Wire.h>
#include "ChinaBee.h"
#include <SPI.h>  
#include "HMC5883L.h"
#include "AFMotor.h"

#define myTeamNumber 0 
#define compKp 1.4
#define compKi 0 //0.014
#define compKd 0 //.08
#define compXMax 280
#define compXMin -80
#define compYMax 180
#define compYMin -170


ChinaBee bee;
HMC5883L compass;
AF_DCMotor leftMotor(3, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor rightMotor(4, MOTOR12_64KHZ); // create motor #2, 64KHz pwm.

int lineSensors[4] = {A15, 56, 18, 19};
int triggeredSensor = -1; 
int botCoordinates[4][2];
int XPos=0, YPos=0, motSpeed, compAlignSpeed;
double targetX=-175, targetY=103;
double trueBearing, relBearing, fieldBearing;
int compOffset = 180, targetBearingOffset = 0;
Vector data;
unsigned long long lastTime;
int lastError;
long long int errorTotal;
long int dt;

int sign(int n){
    return n==0?0:n/abs(n);
}

void updateBearings(){
    //data  = compass.readRaw();
    //Serial.print(data.XAxis); Serial.print(" "); Serial.println(data.YAxis);
    data = compass.readNormalize();
    trueBearing  = -((data.YAxis!=0) ? atan2(data.XAxis, data.YAxis)/0.0174532925 : (data.XAxis>0 ? 0.0 : 180.0));
    
    fieldBearing = trueBearing - compOffset;
    fieldBearing = (fieldBearing<-180?360+fieldBearing:(fieldBearing>180?fieldBearing-360:fieldBearing));
    
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
    int basic = int(abs(atan2(dy,dx)/0.0174532925))%90;
    if(dy>0 && dx >0)
        return 90-basic;
    else if(dx<0 && dy>0)
        return basic-90;
    else if(dx>0 && dy<0)
        return basic+90;
    else if(dx<0 && dy<0)
        return -90-basic;
}

void updateCoordinates(){
    bee.update();
    for (int i=0; i<bee.get_num_teams(); i++) {
      team_status_t* stat = bee.get_status(i);
      if (stat->haveFound || true) {
 

        /*if (i == 0){
            Serial.print("Team ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(stat->x);
        Serial.print(" ");
        Serial.print(stat->y);
        Serial.print(" time since (ms): ");
        Serial.println(millis() - stat->timestamp); 
        }*/
        
        botCoordinates[i][0] = -stat->x;
        botCoordinates[i][1] = stat->y;  
        if (i == myTeamNumber){
           YPos = stat->y;
           XPos = -stat->x;
  
        }
      }
  }
}

//interupt functions for four line sensors
void onLine0() { triggeredSensor = 0; }
void onLine1() { triggeredSensor = 1; }
void onLine2() { triggeredSensor = 2; }
void onLine3() { triggeredSensor = 3; }

void setup(){
    Serial.begin(9600);
    while (!compass.begin()){
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(50);
    }
    compass.setOffset(-((compXMin+compXMax)/2), -((compYMin+compYMax)/2));
    compass.setScale((compXMax-compXMin)/2, (compYMax-compYMin)/2);
    compass.setSamples(HMC5883L_SAMPLES_4);
    bee.init(48, 49);
    
    attachInterrupt(digitalPinToInterrupt(lineSensors[0]), onLine0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lineSensors[1]), onLine1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lineSensors[2]), onLine2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lineSensors[3]), onLine3, CHANGE);

}

void loop(){


  
    updateCoordinates();
    targetBearingOffset = getDir(targetX, targetY);
    updateBearings();

    if (triggeredSensor != -1){
      //Serial.print("The bot is over the line at ");
      //Serial.println(triggeredSensor);
      triggeredSensor = -1;
    }
    //Serial.print(XPos); Serial.print(" "); Serial.println(YPos);
    Serial.print(trueBearing); Serial.print(" "); Serial.print(compOffset);Serial.print(" ");Serial.print(fieldBearing); Serial.print(" "); Serial.print(targetBearingOffset);Serial.print(" ");Serial.println(relBearing);
    if(sign(relBearing)!=sign(lastError))
            errorTotal=0;
    dt = millis()-lastTime;
    errorTotal += relBearing*dt;
    compAlignSpeed = compKp*relBearing+compKi*errorTotal+compKd*((relBearing-lastError)/dt);
/*    Serial.print(compKp*relBearing);
    Serial.print(" ");
    Serial.print(compKi*errorTotal);
    Serial.print(" ");

    Serial.println(compKd*((relBearing-lastError)/dt));*/

    lastTime = millis();
    lastError = relBearing;
    
    if(abs(relBearing)>20){
        leftMotor.runWrapper(compAlignSpeed);
        rightMotor.runWrapper(-compAlignSpeed);
    }
    else if(XPos!=targetX || YPos != targetY){
        motSpeed = 200;
        leftMotor.runWrapper(-motSpeed+compAlignSpeed);
        rightMotor.runWrapper(-motSpeed-compAlignSpeed);
    }
    else{
        leftMotor.runWrapper(0);
        rightMotor.runWrapper(0);
    }
}

