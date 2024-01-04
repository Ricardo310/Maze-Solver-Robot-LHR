#include <NewPing.h>
// version a presentar en el primer congreso venezolano de robotica 6/12/23    https://github.com/Ricardo310/Maze-Solver-Robot-LHR   
#define TRIGGER_PINL  A3  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A2  // Arduino pin tied to echo pin on ping sensor.

int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4



float P = 5.5 ; //2.8
float D = 9; //1.5
float I = 0.8 ; //0.4
float oldErrorP ;
float totalError ;
int offset = 0 ; // era 5//////////

int wall_threshold = 27 ; // la distancia para detectar pared
int front_threshold = 17.5 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;

int izq=0;
int der=0;
int u=0;

int en1 = 3 ;
int en2 = 2 ;

int en3 = 4 ;
int en4 = 5 ;

int enA = 11 ;
int enB = 10 ;


int baseSpeed = 110 ; // 0-250    
int distanceFromWall=12;// distancia entre el sensor y la pared a seguir(izquierda) //11.8


int RMS ;
int LMS ;



NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 25; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
float errorP=0;
float errorD=0;
float errorI=0;

//int TestNUM = 1  ;



void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.

  for (int i = 2; i <= 13; i++) //For Motor Shield
    pinMode(i, OUTPUT);
}

void loop() {


  //========================================START========================================//

  ReadSensors();

  walls();
  
  
  PID();
  //vuelta en U
  if (leftwall == true && rightwall == true && frontwall == true && frontwall != 0 && rightSensor != 0) //callejon sin salida
  {
    u++;
    if(u>=15){ //verifica varias veces 
      PID();
      vueltaU();
      u=0;
    }
  } else u=0;
  //cruzar a la derecha
  if (leftwall == true && rightwall == false && frontwall == true && frontwall != 0) { //hueco derecha parde adelante y izquierda
    der++;
    if(der>=10){ //verifica varias veces 
      PID();
      turnright();
      der=0;
    }
  }else{der=0;}
  //cruzar a la izquirda 
  if (leftwall == false ) { //hueco izquierda
    izq++;
    if(izq>=1){ //verifica varias veces 
      PID() ;
      turnleft();
      izq=0;
    }
  }else{izq=0;}

  
  if ( (leftSensor == 0 || leftSensor > 100 )&& (rightSensor == 0 || rightSensor > 100) && (frontSensor == 0 || frontSensor > 100) ) {
    setDirection(STOP);
  }


  if (RMS < 0) {
    RMS = map(RMS , 0 , -255, 0, 255);
    setDirection(RIGHT);
  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);
    setDirection(LEFT);
  }
  else {
    setDirection(FORWARD);
  }


  // damos velocidad a los motores   
  analogWrite(enA , RMS);
  analogWrite(enB , (LMS*1.13)); // el coeficiente es porque el motor izquierdo es menos potente que el derecho, 
  

  // read sensors & print the result to the serial monitor //
  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  //measure error & print the result to the serial monitor
  Serial.print("error=");
  Serial.println(totalError);

}

//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, LOW );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

  //leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
  //rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
  //frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

 // mas rapido
  leftSensor = sonarLeft.convert_cm(leftSensor);
  rightSensor = sonarRight.convert_cm(rightSensor);
  frontSensor = sonarFront.convert_cm(frontSensor);
///////////
  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();

/*
  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;
*/
  leftSensor = (0.6*lSensor + 0.4*oldLeftSensor); //average distance between old & new readings to make the change smoother
  rightSensor = (0.6*rSensor + 0.4*oldRightSensor); // aqui le damos mayor importancia a la ultima lectura
  frontSensor = (0.6*fSensor + 0.4*oldFrontSensor);

  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

//---------------------------------------------------------------------------//


//--------------------------------- control ---------------------------------//

//----------------------------- wall follow  control -------------------------------//

void PID() {
  errorP= leftSensor - distanceFromWall;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3) * errorI + errorP ;
  totalError = P * errorP + D * errorD + I * errorI ;
  oldErrorP = errorP ;

  RMS = baseSpeed + (baseSpeed/100)*totalError ;
  LMS = baseSpeed - (baseSpeed/100)*totalError ;

  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;         //
  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;         //

  if (RMS < 0) {
    RMS = map(RMS , 0 , -255, 0, 255);
    setDirection(RIGHT);
  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);
    setDirection(LEFT);
  }
  else {
    setDirection(FORWARD);
  }
      
  /////
  Serial.println(RMS);
  Serial.println(LMS);
  /////
}

//--------------------------- wall detection --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold && leftSensor!= 0) {
    leftwall = true ;     
  }
  else {
    leftwall = false ;
    Serial.println(" sin left");
  }
  if ( rightSensor < wall_threshold && rightSensor != 0 ) {
    rightwall = true ;
  }
  else {
    Serial.println(" sin righ");
    rightwall = false ;
  } if ( frontSensor < front_threshold  &&  frontSensor!= 0) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
    Serial.println(" sin fron");
  }
}



//---------------------------------------------------------------------------//

void turnright() {
  LMS=baseSpeed*1.2;
  RMS=-18*baseSpeed/frontSensor- baseSpeed/leftSensor ;
  //RMS = baseSpeed - (3* baseSpeed/leftSensor + baseSpeed / 1.5) ;
  //RMS = LMS * rightSensor / ( rightSensor + 70 ) ;
  setDirection(RIGHT);
  analogWrite(enA , RMS);
  analogWrite(enB , LMS);
  delay(360);
  Serial.println(" >>>>");

}

//---------------------------------------------------------------------------//

void turnleft() {
  RMS=baseSpeed*1.2;
  LMS= baseSpeed/2.6;
  //LMS = baseSpeed - (5* baseSpeed/rightSensor + baseSpeed / 1.5) ;
  //LMS = RMS * leftSensor / ( leftSensor + 70 ) ;
  analogWrite(enA , RMS);
  analogWrite(enB , LMS);
  //delay(700);
  Serial.println("<<<<");
}

//---------------------------------------------------------------------------//

void vueltaU(){
  RMS=baseSpeed*1.45; //pero al contrario
  LMS= baseSpeed*0.8;
  setDirection(RIGHT);
  analogWrite(enA , RMS);
  analogWrite(enB , LMS);
  delay(750);
   Serial.println("uuuuuuuuuuuuuuuuuuuu");
}
/*
void prueba(){
  setDirection(FORWARD);
  analogWrite(enA , baseSpeed);
  analogWrite(enB , baseSpeed);
  delay(180);
  turnleft();
}
*/
/*
void circulo(){
  setDirection(FORWARD);
  RMS=baseSpeed*1.2;
  LMS= baseSpeed/2.5;
  analogWrite(enA , RMS);
  analogWrite(enB , LMS);
  delay(10000);

  LMS=baseSpeed*1.2;
  RMS= baseSpeed/2.5;
  analogWrite(enA , RMS);
  analogWrite(enB , LMS);
  delay(10000);
  }
*/

//---------------------------------------------------------------------------//
