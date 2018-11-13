#include "Enes100Simulation.h"
#include "DFRTankSimulation.h"

#define abs(x) ((x)>0?(x):-(x))

Enes100Simulation enes;
DFRTankSimulation tank;

double xDistance;
double yDistance;
double angleDestination;

void turnleft(){
  tank.setLeftMotorPWM(-255);
  tank.setRightMotorPWM(255);
}

void turnright(){
  tank.setLeftMotorPWM(255);
  tank.setRightMotorPWM(-255);
}

void motorMove(){
  tank.setLeftMotorPWM(255);
  tank.setRightMotorPWM(255);
}

void motorStop(){
  tank.turnOffMotors();
}

void turningTotheAngle(double angle){
  if (abs(angle - enes.location.theta) > 0.05)
  {
    while(abs(angle - enes.location.theta) > 0.05)
    {
      turnleft();
      while (!enes.updateLocation());
      Serial.print(angle);
      Serial.print(" Current angle");
      Serial.println(enes.location.theta);
    }
  }
}

void avoidingObstacle(){
  if(enes.location.y > 1)
  { 
    turningTotheAngle(-1.57);
    motorStop();
    delay(500);
    enes.updateLocation();
    if(abs(enes.location.theta - 1.57) < 0.05)
    {
      while(enes.readDistanceSensor(9) < 0.4 || enes.readDistanceSensor(11) < 0.4){
      motorMove();
      Serial.println(enes.readDistanceSensor(9));
    } 
    }
  }
  else
  {
      turningTotheAngle(1.57);
      motorStop();
      delay(500);
      while(enes.readDistanceSensor(5) < 0.4 || enes.readDistanceSensor(3) < 0.4){
      motorMove();
      Serial.println(enes.readDistanceSensor(5));
  }
}
}

void setup() {

  tank.init();

  enes.println("Starting Navigation");

  while (!enes.retrieveDestination());

  while (!enes.updateLocation());

}

void loop(){

  //turn to face forward
  xDistance = enes.destination.x - enes.location.x;
  yDistance = enes.destination.y - enes.location.y;
  angleDestination = atan2(yDistance, xDistance);
  
  if(abs(xDistance <=0.05 && yDistance <=0.05))
  {
    motorStop();
    delay(200000);
  }
  else
  {
    if(yDistance > 0.1)
    {
      turningTotheAngle(1.57);
    }
    else if(yDistance < -0.1)
    {
      turningTotheAngle(-1.57);
    }
    
    if(abs(enes.location.theta)< 1.62 && abs(enes.location.theta) > 1.52)
    {
      while(abs(yDistance) >= 0.05){
        motorMove();
        while (!enes.updateLocation());
        xDistance = enes.destination.x - enes.location.x;
        yDistance = enes.destination.y - enes.location.y;
        Serial.println(yDistance);
      }
    }
      if(abs(yDistance) <= 0.05)
      {
        turningTotheAngle(0);
        while(abs(xDistance) >= 0.1){
          /*if(enes.readDistanceSensor(1) < 0.2 || enes.readDistanceSensor(0) < 0.2 || enes.readDistanceSensor(2) < 0.2)
        {
          avoidingObstacle();
          turningTotheAngle(0);
        }*/
        motorMove();
        while (!enes.updateLocation());
        xDistance = enes.destination.x - enes.location.x;
        yDistance = enes.destination.y - enes.location.y;
        Serial.println(xDistance);
        }
      }

      if(abs(yDistance) > 0.05 || abs(xDistance) > 0.05 && enes.location.x > 2)
      {
        enes.updateLocation();
        xDistance = enes.destination.x - enes.location.x;
        yDistance = enes.destination.y - enes.location.y;
        angleDestination = atan2(yDistance, xDistance);
        turningTotheAngle(angleDestination);
        while(abs(xDistance) > 0.05 || abs(yDistance) > 0.05)
        {
          motorMove();
          while (!enes.updateLocation());
          xDistance = enes.destination.x - enes.location.x;
          yDistance = enes.destination.y - enes.location.y;
        }
      }
    }
  }
