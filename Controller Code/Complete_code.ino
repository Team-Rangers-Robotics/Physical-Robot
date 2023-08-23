#include <NewPing.h>

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

#define trig_1 30 //front
#define echo_1 32
#define trig_2 34 //side front
#define echo_2 36
#define trig_3 38  //side back
#define echo_3 40

#define REncoder 20
#define LEncoder 21

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

#define mode1 25
#define mode2 26
#define mode3 27
#define mode4 28
#define mode5 29

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[] = { -15, -8, -3, -1, 1, 3, 8, 15};
int RightIRval, LeftIRval;

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

//line following
float P_l, I_l, D_l, distance;
float error_l = 0;
float previousError_l = 0;
float Kp_l = 15;
float Kd_l = 8;
float Ki_l = 0;

//line maze
int REncoderVal = 0;
int LEncoderVal = 0;
int i = 0;
int TotalCounttoLeft = 430;
int TotalCounttoRight = 500;
int TotalCounttoMove = 50;
int TotalCounttoMoveBack = 100;
volatile unsigned int REncoderCount, LEncoderCount;

int path[] = {0, 0} ; //{what turn- left-1, right-2 , forward distance}
int distanceArray[] = {};
int pathArray[] = {};

//wall following
double max_distance = 100;
float Kp = 30;
float Kd = 10;
float Ki = 0;
double error = 0;
double setDistance = 7.5;
double previousError = 0;
float distance1, distance2, distance3, currentDistance , P, I, D;
bool traverse_;

//blindbox
double error_b = 0;
double previousError_b = 0;
float P_b, I_b, D_b;

bool maze_end = false;
bool left_path_detected = false;
bool right_path_detected = false;
bool straight_path_detected = false;

void read_IR();
void set_speed();
void set_forward();
void stop_motors();
void read_side_IRs();

bool traverse();
void IntEncoderCount();
void detachInt();
void REncoderCountFunction();
void LEncoderCountFunction();
void turn_left();
void stop_motors();
void set_backward();
void line_following();

void line_maze() ;
bool leftPath();
bool rightPath();
bool straight();
bool mazeEnd();
void turnLeft();
void turnRight() ;
void reverse() ;
void moveInch();
void backward();
void wall_following();
int read_FRONT_SONAR();
void blind_box();
void set_backward();
void turn_left();
void turn_right();
void stop_motors();
void movetoSquarecenter();

int read_SONAR();

bool end_task;

NewPing sonar_1(trig_1, echo_1, max_distance);
NewPing sonar_2(trig_2, echo_2, max_distance);
NewPing sonar_3(trig_3, echo_3, max_distance);



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
  pinMode(RMotorEn, OUTPUT);

  pinMode(LMotorIN3, OUTPUT);
  pinMode(LMotorIN4, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);
  pinMode(LMotorEn, OUTPUT);

  pinMode(trig_1, INPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, INPUT);
  pinMode(echo_2, INPUT);
  pinMode(trig_3, INPUT);
  pinMode(echo_3, INPUT);

  pinMode(mode1, INPUT_PULLUP); //all
  pinMode(mode2, INPUT_PULLUP); // line maze
  pinMode(mode3, INPUT_PULLUP); //wall following
  pinMode(mode4, INPUT_PULLUP); // blind box
  pinMode(mode5, INPUT_PULLUP); // line following

  set_forward();
  end_task = false;

  Serial.println("Set up Done");
  delay(1000);

 // line_maze();


if (digitalRead(mode1) == LOW) {
    line_maze();
    while (true) {
      wall_following();
      if (currentDistance > max_distance) {
        break;
      }
    }
    while (!end_task) {
      blind_box();
    }
  }
  else if (digitalRead(mode2) == LOW) {
    while (true) {
      line_maze();
    }
  }
  else if (digitalRead(mode3) == LOW) {
    while (true) {
      wall_following();
      if (currentDistance > max_distance) {
        break;
      }
    }
  }
  else if (digitalRead(mode4) == LOW) {
    while (true) {
      blind_box();
    }
  }
  else if (digitalRead(mode5) == LOW) {
    Serial.print(digitalRead(mode5));
    while (true) {
      line_following();
      if ((IR_val[1] == 0) && (IR_val[2] == 0) && (IR_val[3] == 0) && (IR_val[4] == 0) && (IR_val[5] == 0) && (IR_val[6] == 0) && (IR_val[7] == 0) && (IR_val[0] == 0)) {
        movetoSquarecenter();
        stop_motors();
        break;
      }

    }
  }
}
void loop() {
  //  line_maze();

  //  while (true) {
  //    wall_following();
  //    if (currentDistance > max_distance) {
  //      break;
  //    }
  //  }
  //
  //  while (!end_task) {
  //    blind_box();
  //  }
}

//////////////////////////////////////////////////////////////////////////

void line_following() {
  //  Serial.println("1");
  read_IR();
  Serial.print(IR_val[0]);
  Serial.print(IR_val[1]);
  Serial.print(IR_val[2]);
  Serial.print(IR_val[3]);
  Serial.print(IR_val[4]);
  Serial.print(IR_val[5]);
  Serial.print(IR_val[6]);
  Serial.println(IR_val[7]);
  read_side_IRs();
  error_l = 0;
  for (int i = 0; i < 8; i++) {
    //Serial.print(IR_val[i]);
    error_l += IR_weights[i] * !IR_val[i];
  }
  //  Serial.println("2");
  P_l = error_l;
  I_l = I_l + error_l;
  D_l = error_l - previousError_l;

  previousError_l = error_l;

  speedAdjust = Kp_l * P_l + Ki_l * I_l + Kd_l * D_l;

  //  Serial.println("3");
  LMotorSpeed = MotorBaseSpeed + speedAdjust;
  RMotorSpeed = MotorBaseSpeed - speedAdjust;

  //  Serial.print("Speed ");
  //  Serial.print(LMotorSpeed);
  //  Serial.print(" ");
  //  Serial.println(RMotorSpeed);

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
  //  Serial.println("4");
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

void read_side_IRs() {
  RightIRval = digitalRead(RightIR);
  LeftIRval = digitalRead(LeftIR);
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


/////////////////////////////////////////////////////////


void line_maze() {
  Serial.println("line maze");
  delay(100);
  while (true) {
    //exploring
    traverse_ = traverse();
    if (traverse_) break;

    pathArray[i] = path[0];
    distanceArray[i] = path[1];
    i++;
  }

  // reduce path
  // return
  // fast run
}

bool traverse() {
  Serial.println("traverse");
  delay(100);
  path[0] = 0;
  path[1] = 0;
  set_forward();
  line_following();
  read_IR();
  Serial.print(IR_val[0]);
  Serial.print(IR_val[1]);
  Serial.print(IR_val[2]);
  Serial.print(IR_val[3]);
  Serial.print(IR_val[4]);
  Serial.print(IR_val[5]);
  Serial.print(IR_val[6]);
  Serial.println(IR_val[7]);
  //  Serial.println("0");
  while (true) {
    //    Serial.println("In Loop");
    maze_end = mazeEnd();
    if (maze_end) {
      Serial.println("maze end");
      delay(100);
      stop_motors();
      return (true);
    }
    //    Serial.println("1");
    REncoderCount = 0;
    LEncoderCount = 0;
    IntEncoderCount();
    left_path_detected = leftPath();

    if (left_path_detected) {
      Serial.println("left detected");
      distance = (REncoderCount + LEncoderCount) / 2;
      path[0] = 1;
      path[1] = distance;//return 1 if left turn
      turnLeft();
      //      Serial.println("3");
      Serial.println("left");
      delay(100);
      return false;
    }
    else {
      //      Serial.println("4");
      //      moveInch() ;
      straight_path_detected = straight();
      if (!(straight_path_detected)) {
        Serial.println("4");
        right_path_detected = rightPath();
        if (right_path_detected) {
          distance = (REncoderCount + LEncoderCount) / 2;
          path[0] = 2;
          path[1] = distance;//return 2 if right turn
          turnRight();
          Serial.println("right");
          delay(100);
          return false;
        }

        else {
          Serial.println("5");
          reverse();
          path[0] = 3; // reverse-3
          Serial.println("u turn");
          delay(100);
          return false;
        }
      }
    }
    Serial.print("Encoder ");
    Serial.print(REncoderCount);
    Serial.print(" ");
    Serial.println(LEncoderCount);
    delay(100);
  }
}



void IntEncoderCount() {
  // output encoder count
  attachInterrupt(digitalPinToInterrupt(REncoder), REncoderCountFunction, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REncoder), LEncoderCountFunction, CHANGE);
}

void detachInt() {
  detachInterrupt(digitalPinToInterrupt(REncoder));
  detachInterrupt(digitalPinToInterrupt(LEncoder));
}


void REncoderCountFunction() {
  REncoderCount++;
}


void LEncoderCountFunction() {
  LEncoderCount++;
}


bool leftPath() {
  //if left path detected return true
  //  read_IR();
  if ((IR_val[0] == 0) && (IR_val[1] == 0) && (IR_val[2] == 0) ) {
    return (true);
  }
  else
    return false;
}

bool rightPath() {
  //if left path detected return true
  //  read_IR();
  if ((IR_val[5] == 0) && (IR_val[6] == 0) && (IR_val[7] == 0)) {
    return (true);
  }
  else return false;
}


bool straight() {
  //  read_IR();
  if ((IR_val[3] == 0) && (IR_val[4] == 0) ) {
    return (true);
  }
  else return false;
}


bool mazeEnd() {
  //  Serial.print(IR_val[0]);
  //  Serial.print(IR_val[1]);
  //  Serial.print(IR_val[2]);
  //  Serial.print(IR_val[3]);
  //  Serial.print(IR_val[4]);
  //  Serial.print(IR_val[5]);
  //  Serial.print(IR_val[6]);
  //  Serial.println(IR_val[7]);
  if ((IR_val[1] == 0) && (IR_val[2] == 0) && (IR_val[3] == 0) && (IR_val[4] == 0) && (IR_val[5] == 0) && (IR_val[6] == 0) &&  (IR_val[7] == 0) && (IR_val[0] == 0)) {
    moveInch();
    if ((IR_val[1] == 0) && (IR_val[2] == 0) && (IR_val[3] == 0) && (IR_val[4] == 0) && (IR_val[5] == 0) && (IR_val[6] == 0) &&  (IR_val[7] == 0) && (IR_val[0] == 0)) {
      return (true);
    }
    else
      return false;
  }
}



void turnLeft() {
  //turn left code
  stop_motors();
  delay(1000);
  REncoderCount = 0;
  LEncoderCount = 0;
  IntEncoderCount();
  turn_left();
  analogWrite(RMotorPWM, 80);
  analogWrite(LMotorPWM, 80);
  while (((REncoderCount + LEncoderCount) / 2 < TotalCounttoLeft)) {
  };
  detachInt();
  stop_motors();
  delay(500);
}


void turnRight() {
  //turn left code
  stop_motors();
  delay(1000);
  REncoderCount = 0;
  LEncoderCount = 0;
  IntEncoderCount();
  turn_right();
  analogWrite(RMotorPWM, 80);
  analogWrite(LMotorPWM, 80);
  while (((REncoderCount + LEncoderCount) / 2 < TotalCounttoRight)) {
  };
  detachInt();
  stop_motors();
  delay(500);
}


void reverse() {
  turnLeft();
  turnLeft();
}


void moveInch() {
  //to move a little forward
  //turn left code
  REncoderCount = 0;
  LEncoderCount = 0;
  IntEncoderCount();
  digitalWrite(RMotorEn, HIGH);
  digitalWrite(LMotorEn, HIGH);
  digitalWrite(RMotorIN1, HIGH);
  digitalWrite(RMotorIN2, LOW);
  digitalWrite(LMotorIN3, HIGH);
  digitalWrite(LMotorIN4, LOW);
  analogWrite(RMotorPWM, 80);
  analogWrite(LMotorPWM, 80);
  while (((REncoderCount + LEncoderCount) / 2 < TotalCounttoMove)) {
  };
  detachInt();
  analogWrite(LMotorPWM, 0);
  analogWrite(RMotorPWM, 0);
  digitalWrite(RMotorEn, LOW);
  digitalWrite(LMotorEn, LOW);
  delay(2000);
}


void backward() {
  //to move backward
  REncoderCount = 0;
  LEncoderCount = 0;
  IntEncoderCount();
  digitalWrite(RMotorEn, HIGH);
  digitalWrite(LMotorEn, HIGH);
  digitalWrite(RMotorIN1, LOW);
  digitalWrite(RMotorIN2, HIGH);
  digitalWrite(LMotorIN3, LOW);
  digitalWrite(LMotorIN4, HIGH);
  analogWrite(RMotorPWM, 80);
  analogWrite(LMotorPWM, 80);
  while (((REncoderCount + LEncoderCount) / 2 < TotalCounttoMoveBack)) {
  };
  detachInt();
  analogWrite(LMotorPWM, 0);
  analogWrite(RMotorPWM, 0);
  digitalWrite(RMotorEn, LOW);
  digitalWrite(LMotorEn, LOW);
  delay(2000);
}

///////////////////////////////////////////////////////////////////////////
//wall following

void wall_following() {
  currentDistance = read_SONAR();

  //PID
  error = 0;
  error = setDistance - currentDistance;

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;

  LMotorSpeed = MotorBaseSpeed + speedAdjust;
  RMotorSpeed = MotorBaseSpeed - speedAdjust;

  if (LMotorSpeed < 0)  {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0)  {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MaxSpeed) {
    LMotorSpeed = MaxSpeed;
  }
  if (RMotorSpeed > MaxSpeed) {
    RMotorSpeed = MaxSpeed;
  }
  set_speed();
}


int read_SONAR() {
  distance2 = sonar_2.ping_cm();
  distance3 = sonar_3.ping_cm();
  return (distance2 + distance3) / 2;
}

int read_FRONT_SONAR() {
  distance1 = sonar_1.ping_cm(); //front distance
  return (distance1);
}

////////////////////////
//blindbox

void blind_box() {
  currentDistance = read_SONAR();

  //PID
  error_b = setDistance - currentDistance;

  P_b = error_b;
  I_b = I_b + error_b;
  D_b = error_b - previousError_b;

  previousError_b = error_b;

  speedAdjust = Kp * P_b + Ki * I_b + Kd * D_b;

  LMotorSpeed = MotorBaseSpeed + speedAdjust;
  RMotorSpeed = MotorBaseSpeed - speedAdjust;

  if (LMotorSpeed < 0)  {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0)  {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MaxSpeed) {
    LMotorSpeed = MaxSpeed;
  }
  if (RMotorSpeed > MaxSpeed) {
    RMotorSpeed = MaxSpeed;
  }
  set_speed();

  set_forward();
  distance1 = sonar_1.ping_cm();
  if (distance1 < 20) {
    turnRight();
  }

  if (currentDistance > max_distance) {
    delay(50);
    turnLeft();
    set_forward();
    delay(1000);
    stop_motors();
    read_IR();
    if ((IR_val[1] == 0) || (IR_val[2] == 0) || (IR_val[3] == 0) || (IR_val[4] == 0) || (IR_val[5] == 0) || (IR_val[6] == 0) ||  (IR_val[7] == 0) || (IR_val[0] == 0)) {
      while (true) {
        line_following();
        if ((IR_val[1] == 0) && (IR_val[2] == 0) && (IR_val[3] == 0) && (IR_val[4] == 0) && (IR_val[5] == 0) && (IR_val[6] == 0) &&  (IR_val[7] == 0) && (IR_val[0] == 0)) {
          stop_motors();
          end_task = true;
          break;
        }
      }
    }
    else {
      set_backward();
      delay(1000);
      stop_motors();
      delay(50);
      turnRight();
      set_forward();
      delay(1000);
    }
  }
}



void set_backward() {
  digitalWrite(RMotorEn , HIGH);
  digitalWrite(LMotorEn , HIGH);
  digitalWrite(RMotorIN1, LOW);
  digitalWrite(RMotorIN2, HIGH);
  digitalWrite(LMotorIN3, LOW);
  digitalWrite(LMotorIN4, HIGH);
}

void turn_left() {
  digitalWrite(RMotorEn , HIGH);
  digitalWrite(LMotorEn , HIGH);
  digitalWrite(RMotorIN1, HIGH);
  digitalWrite(RMotorIN2, LOW);
  digitalWrite(LMotorIN3, LOW);
  digitalWrite(LMotorIN4, HIGH);
}

void turn_right() {
  digitalWrite(RMotorEn , HIGH);
  digitalWrite(LMotorEn , HIGH);
  digitalWrite(RMotorIN1, LOW);
  digitalWrite(RMotorIN2, HIGH);
  digitalWrite(LMotorIN3, HIGH);
  digitalWrite(LMotorIN4, LOW);
}

void stop_motors() {
  digitalWrite(RMotorEn , LOW);
  digitalWrite(LMotorEn , LOW);
  digitalWrite(RMotorIN1, LOW);
  digitalWrite(RMotorIN2, LOW);
  digitalWrite(LMotorIN3, LOW);
  digitalWrite(LMotorIN4, LOW);
}
