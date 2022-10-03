#include <Servo.h>
#include <math.h>
#include <Arduino_NineAxesMotion.h>
#define pwm_min 1300 // Add the minimum value of the PWM (in microseconds )
#define pwm_max 1700 // Add the maximum value of the PWM (in microseconds )
#include <Wire.h>
#define echoPin 29 // Echo Pin
#define trigPin 25 // Trigger Pin

#define v 0.16 // vitesse du robot en m.s


unsigned long dep_time = 0;
unsigned long stop_time = 0;

byte pin_motor_left = 5;
byte pin_motor_right = 6;
Servo motorL , motorR ;
Servo motorFront ;

NineAxesMotion mySensor ; // Object that for the sensor
unsigned long lastStreamTime = 0; // To store the last streamed timestamp
const int streamPeriod = 20;

unsigned long Go() {
  motorL.writeMicroseconds (1700) ; 
  motorR.writeMicroseconds (1300) ;
  return millis();
}

unsigned long Stop() {
  motorL.writeMicroseconds (1500) ; 
  motorR.writeMicroseconds (1500) ; 
  return millis(); // temps total roulé millisecondes
}

void RightTurn(int& axis){
  long mem = GetYaw();
  motorL.writeMicroseconds (1700) ;
  motorR.writeMicroseconds (1700) ;
  delay(720);
  motorL.writeMicroseconds (1500) ;
  motorR.writeMicroseconds (1500) ;

  if(axis==1){axis=-2;}
  else if(axis==-2){axis=-1;}
  else if(axis==-1){axis=2;}
  else if(axis==2){axis=1;}

}

void LeftTurn(int& axis) {
  long mem = GetYaw();
  motorL.writeMicroseconds (1300) ;
  motorR.writeMicroseconds (1300) ;
  delay(730);
  motorL.writeMicroseconds (1500) ;
  motorR.writeMicroseconds (1500) ;

  if(axis==1){axis=2;}
  else if(axis==2){axis=-1;}
  else if(axis==-1){axis=-2;}
  else if(axis==-2){axis=1;}
}


double detect(){
  digitalWrite ( trigPin , HIGH ) ;
  delayMicroseconds (10) ;
  digitalWrite ( trigPin , LOW ) ;
  long roundTripTime = pulseIn(echoPin, HIGH); 
  double distance = roundTripTime/2*340/10000;
  return distance;
}


long GetYaw(){
  if (( millis () - lastStreamTime ) >= streamPeriod ) {
  lastStreamTime = millis () ;
  mySensor . updateEuler () ; // Update the Euler data into the structure of the object
  Serial.println ( mySensor.readEulerHeading() ) ;
  return ( mySensor.readEulerHeading() ) ; // Heading data
  }
}

void edit_position(double travel_time, int axis, double& x, double& y, double xt, double yt){
    if (axis == 1 || axis == -1){                   // changer la valeur de x
      x = x +  axis * (v * travel_time) * pow(10,2);
    }
    if (axis == 2 || axis == -2){                   // changer la valeur de y
      y = y + axis/2 * (v * travel_time) * pow(10,2);
    }
    Serial.print("je suis maintenant a  x = ");
    Serial.print(x);
    Serial.print(" et y = ");
    Serial.println(y);
    if ( (xt-10 < x) && (x < xt + 10) && (yt - 10 < y) && (y < yt + 10)){ // arrivé
      Serial.print("je me suis arrete fdp");
      exit(0) ;
    }
    Serial.println("je suis dans edit");
}

void check_distance(float& d_right, float& d_left){
  Serial.println("je suis dans check distance");
  motorFront.write(0);
  d_right = detect();
  motorFront.write(180);
  Serial.println("j'y suis toujours");
  d_left = detect();
  motorFront.write(90);
}


void setup() {
  Serial.begin (9600) ;
  Wire1.begin ();
  motorL.attach ( pin_motor_left  , pwm_min , pwm_max ) ;
  motorR.attach ( pin_motor_right , pwm_min , pwm_max ) ;

  motorFront.attach(3 , 530 , 2900) ; // for the hs65hb servomotor
  motorFront.write(90);

  pinMode ( trigPin , OUTPUT );
  pinMode ( echoPin , INPUT ) ;

  mySensor.initSensor () ; // Sensor Initialization
  mySensor.setOperationMode (OPERATION_MODE_NDOF) ; //9 degrees of Freedom Sensor Fusion
  mySensor.setUpdateMode (MANUAL) ;

  motorFront.write(90);
  dep_time=Go();
}

double x_target = 17;
double y_target = 17;
double x = 422.0 ;
double y = 14.0 ;
int axis = -1;
float d_right=0;
float d_left=0;
double travel_time = 0.0;

void loop() {
/*
  RightTurn(axis);
  delay(1000);
  LeftTurn(axis);
  delay(1000);
  */


  motorFront.write(180);
  delay(500);
  if(detect()>30){
    delay(500);
    travel_time = (Stop()-dep_time) * pow(10,-3) ;
    edit_position(travel_time , axis, x , y , x_target, y_target);
    Serial.print("j'ai roulé");
    Serial.print(travel_time);
    Serial.println("sec");
    LeftTurn(axis);
    Serial.print("je suis sur l'axe ");
    Serial.println(axis);
    dep_time=Go();
    delay(500);
    
  }
  motorFront.write(90);
  delay(500);
  if(detect()<15){ 
    Serial.print("j'ai roulé");
    Serial.print(travel_time);
    Serial.println("sec");
    travel_time = (Stop()-dep_time) * pow(10,-3) ;
    edit_position(travel_time , axis, x , y , x_target, y_target);
    RightTurn(axis);
    Serial.print("je suis sur l'axe ");
    Serial.println(axis);
    dep_time=Go();
  }
}
