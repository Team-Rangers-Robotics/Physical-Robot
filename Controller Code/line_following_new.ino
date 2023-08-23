#define IR1 42 //leftmost
#define IR2 43
#define IR3 44
#define IR4 45
#define IR5 46
#define IR6 47
#define IR7 48
#define IR8 49  //rightmost

#define RightIR 50
#define LeftIR 51

#define RMotorIN1 31 //motor2
#define RMotorIN2 33
#define RMotorPWM 2
#define RMotorEn 4

#define LMotorIN3 35 //motor 1  // inA
#define LMotorIN4 37           //inB
#define LMotorPWM 3
#define LMotorEn 5

#define MaxSpeed 100
#define MotorBaseSpeed 60

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[] = { -15, -8, -3, -1, 1, 3, 8, 15};
int RightIRval,LeftIRval;

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 15;
float Kd = 8;
float Ki = 0;



void read_IR();
void set_speed();
void set_forward();
void stop_motors();
void read_side_IRs();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);

  pinMode(RightIR, INPUT);
  pinMode(LeftIR, INPUT);

  pinMode(RMotorIN1, OUTPUT);
  pinMode(RMotorIN2, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(LMotorIN3, OUTPUT);
  pinMode(LMotorIN4, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  set_forward();
}


void loop() {
  //while (true){
  // put your main code here, to run repeatedly:
  read_IR();
  read_side_IRs();
 // stop_motors();
//  if ((RightIRval==1) && (LeftIRval==1)){
  //  stop_motors();
    //break;}
  // if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0){
  // stop_motors();
  //   while(1){}
  error = 0;
  for (int i = 0; i < 8; i++) {
    Serial.print(IR_val[i]);
    error += IR_weights[i] * !IR_val[i];
  }
 // Serial.print('-');
 // Serial.println(error);
//  delay(1000);
  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;

  LMotorSpeed = MotorBaseSpeed + speedAdjust;
  RMotorSpeed = MotorBaseSpeed - speedAdjust;

  if (LMotorSpeed < 0) {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MaxSpeed) {
    LMotorSpeed = MaxSpeed;
  }
  if (RMotorSpeed > MaxSpeed) {
    RMotorSpeed = MaxSpeed;
  }
  set_speed();
// }
}


void read_IR() {
  IR_val[0] = digitalRead(IR1);
  IR_val[1] = digitalRead(IR2);
  IR_val[2] = digitalRead(IR3);
  IR_val[3] = digitalRead(IR4);
  IR_val[4] = digitalRead(IR5);
  IR_val[5] = digitalRead(IR6);
  IR_val[6] = digitalRead(IR7);
  IR_val[7] = digitalRead(IR8);
}

void read_side_IRs(){
  RightIRval = digitalRead(RightIR);
  LeftIRval = digitalRead(LeftIR);
  }

void stop_motors() {
  digitalWrite(RMotorEn , LOW);
  digitalWrite(LMotorEn , LOW);
//  digitalWrite(RMotorIN1, LOW);
//  digitalWrite(RMotorIN2, LOW);
//  digitalWrite(LMotorIN3, LOW);
//  digitalWrite(LMotorIN4, LOW);
}

void set_forward() {
  digitalWrite(RMotorEn , HIGH);
  digitalWrite(LMotorEn , HIGH);
  digitalWrite(RMotorIN1, HIGH);
  digitalWrite(RMotorIN2, LOW);
  digitalWrite(LMotorIN3, HIGH);
  digitalWrite(LMotorIN4, LOW);
}

void set_speed() {
  analogWrite(RMotorPWM, RMotorSpeed);
  analogWrite(LMotorPWM, LMotorSpeed);
}
