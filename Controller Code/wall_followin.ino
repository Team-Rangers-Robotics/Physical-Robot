#include <NewPing.h>

#define trig_1 30 //front
#define echo_1 32
#define trig_2 34 //side front
#define echo_2 36
#define trig_3 38  //side back
#define echo_3 40

#define RMotorIN1 31
#define RMotorIN2 33
#define RMotorPWM 2
#define RMotorEn 4

#define LMotorIN3 35
#define LMotorIN4 37
#define LMotorPWM 3
#define LMotorEn 5

#define MaxSpeed 80
#define MotorBaseSpeed 45

double max_distance=200;
float Kp = 30;
float Kd = 10;
float Ki = 0; 
double error = 0;
double setDistance=5;
double previousError=0;

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float distance1,distance2,distance3,currentDistance ,P,I,D;

NewPing sonar_1(trig_1,echo_1,max_distance);
NewPing sonar_2(trig_2,echo_2,max_distance);
NewPing sonar_3(trig_3,echo_3,max_distance);

void setup(){
  Serial.begin(9600);
  
  pinMode(RMotorIN1, OUTPUT);
  pinMode(RMotorIN2, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(LMotorIN3, OUTPUT);
  pinMode(LMotorIN4, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(trig_1, INPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, INPUT);
  pinMode(echo_2, INPUT);
  pinMode(trig_3, INPUT);
  pinMode(echo_3, INPUT);

  set_forward();
  }


  
void loop(){
  currentDistance = read_SONAR();

  //PID
  error=0;
   error = setDistance - currentDistance;

   P = error;
   I = I + error;
   D = error - previousError;

  previousError = error;

  speedAdjust = Kp*P + Ki*I + Kd*D;

  LMotorSpeed = MotorBaseSpeed + speedAdjust;
  RMotorSpeed = MotorBaseSpeed - speedAdjust;
  
  if (LMotorSpeed<0)  {LMotorSpeed = 0;}
  if (RMotorSpeed<0)  {RMotorSpeed = 0;}
  if (LMotorSpeed>MaxSpeed) {LMotorSpeed = MaxSpeed;}
  if (RMotorSpeed>MaxSpeed) {RMotorSpeed = MaxSpeed;}  
  set_speed();
  }


int read_SONAR(){
  distance2=sonar_2.ping_cm();
  distance3=sonar_3.ping_cm();
  return (distance2+distance3)/2;
  }

int read_FRONT_SONAR(){
  distance1=sonar_1.ping_cm(); //front distance
  return (distance1);
  }

void set_forward(){
  digitalWrite(RMotorEn , HIGH);
  digitalWrite(LMotorEn , HIGH);
  digitalWrite(RMotorIN1,HIGH);
  digitalWrite(RMotorIN2,LOW);
  digitalWrite(LMotorIN3,HIGH);
  digitalWrite(LMotorIN4,LOW); 
}


void set_speed(){
  analogWrite(RMotorPWM,RMotorSpeed);
  analogWrite(LMotorPWM,LMotorSpeed);
}
