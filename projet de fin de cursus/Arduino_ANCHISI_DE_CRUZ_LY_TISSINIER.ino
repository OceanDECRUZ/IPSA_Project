//
// Import lib
//

# include <Servo.h>

# define pwm_min 1300 // Add the minimum value of the PWM (in microseconds )
# define pwm_max 1700 // Add the maximum value of the PWM (in microseconds )

//
// Iniate Variables
//

String msg;
byte pin_motor_left = 5;
byte pin_motor_right = 6;

Servo motorL , motorR , motorFront;

//
// Setup
//

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  motorFront.attach(3, 530, 2500);
  motorL . attach ( pin_motor_left , pwm_min , pwm_max ) ;
  motorR . attach ( pin_motor_right , pwm_min , pwm_max ) ;
  
}

//
// Function
//

void forward(){
  motorL.writeMicroseconds(1550);
  motorR.writeMicroseconds(1450);
}

void backward(){
  motorL.writeMicroseconds(1450);
  motorR.writeMicroseconds(1550);
}

void turnL(){
  motorL.writeMicroseconds (1485) ;
  motorR.writeMicroseconds (1485) ;
}

void turnR(){
  motorL.writeMicroseconds (1515) ;
  motorR.writeMicroseconds (1515) ;
}

void stopRobot(){
  motorL.writeMicroseconds (1500) ;
  motorR.writeMicroseconds (1500) ;
}

void avance(){
  forward();
  delay(1000);
  stopRobot();
}

void recul(){
  backward();
  delay(1000);
  stopRobot();
}

void TurnLeft(){
    turnL();
    delay(600);
    stopRobot();
}

void TurnRight(){
    turnR();
    delay(600);
    stopRobot();
}

//
// Loop
//

void loop() {

  // get message from raspberry 
  
  msg = "";
  if (Serial1.available()) {
    delay(10);
    while (Serial1.available() > 0) {
      msg += (char)Serial1.read();
    }
    //Serial1.flush();
  }
  
  // message recu sous la forme  (Detection_ball  0(non)/1(oui) ;Distance 0(rien)/1(avance)/2(recul); Angle 0(rien)/1(gauche)/2(droite))
  // reecriture du message
  
  if(msg=="avance"){
    avance();
  }
  if(msg=="recul"){
    recul();
  }
  if(msg=="left"){
    TurnLeft();
  }
  if(msg=="right"){
    TurnRight();
  }
  if(msg=="right1"){
    TurnRight();
    avance();
  }
}
