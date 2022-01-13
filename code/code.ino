/*
 * This code is for controlling the speed and angle of rotation a DC motor with 95 rpm speed using Arduino Mega and PID controller
 * and an incoder with 612 pulse for a full rotation
 * 
 * Note:
 * I found that the maximum speed of the motor is 75 rpm although it is supposed to be 95 rpm!!!
*/



#define channelA 2 //pin 1 of motor incoder
#define channelB 24 //pin 2 of motor incoder

#define PWM 3 //PWM pin of the motor
#define INR0 30 //IN1 pin of the motor driver L298
#define INR1 32 //IN2 pin of the motor driver L298

#define pot A0 //Analog pin for tunning the gains

//PID gains for speed control
volatile double Kp_0 = 2;
volatile double Ki_0 = 6; 
volatile double Kd_0 = 0;

//PID gains for angle control
volatile double Kp_1 = 15; 
volatile double Ki_1 = 2; 
volatile double Kd_1 = 0.001;

//variables for the PID controllers
volatile double sampleTime = 0.1;
volatile double kpPart_0, kiPart_0, kdPart_0;
volatile double kpPart_1, kiPart_1, kdPart_1;
volatile double curError_0, preError_0, errorSum_0;
volatile double curError_1, preError_1, errorSum_1;
volatile int16_t PIDOUT, motorPower = 127;
const double maxISpeed = 84; //upper limit of the PID integrator for speed control
const double maxIAngle = 21; //upper limit of the PID integrator for angle control



//variables to calculate the motor speed and angle of rotation
const double targetPos = 3.14 * 2; //Set the target angle in radians
const double targetPosDed = (targetPos * 180) / PI;
const double stepPerPul = 0.0103; // 2 * PI / number of pulses per revolution = 2 * PI / 612 = 0.01026664265
volatile double targetSpeed = 30; //This value will be set by the angle controller now
volatile bool bState; //to check the direction of the motor rotation
volatile double angleDeg;
volatile double curSpeed = 0;
volatile double curPos = 0;
volatile unsigned long pulsesCnt = 0;
volatile int8_t motorDir;
const uint16_t pulsesPerRev = 612; //number of incoder pulses for a full rotation
const double desiredNumofRev = targetPos / 3.14; //get the number of full rotations
const double errorPerRot = 0.0698; //error in radian for a full rotation, 8 degrees for every 360 degree, hence the error is (+ or - 1%)
const double allowedErrAngle = desiredNumofRev * errorPerRot; //the allowed error
volatile bool reachedDes = false;

//variables for reading the pot value
int16_t potRead, potShift = 0;
double potAns;

//Initialize timer 1
void INIT_TIMER(void)
{
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 100 ms
  OCR1A = 24999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 bits for prescaling by 64
  TCCR1B |= (1 << CS11) | (1 << CS10); 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() {
  //Initialize external interrupt for the incoder pin
  attachInterrupt(digitalPinToInterrupt(channelA), POSITION_CONTROL, RISING);

  //Initialize serial moniter and output pins
  Serial.begin(38400);
  pinMode(PWM, OUTPUT);
  pinMode(INR0, OUTPUT);
  pinMode(INR1, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  //Call the timer initialization code
  INIT_TIMER();

//  if(desiredNumofRev < 1)
//    desiredNumofRev = 1;
}

void loop() 
{
  //calculate the angle of rotation in degrees
  angleDeg = (curPos * 180) / PI;
  
  //Print the current speed and target speed in the arduino plotter to see the 
  PLOTTER_SHOW();

  //You can comment the PLOTTER_SHOW function to see the SERIAL_MONITER_SHOW function output
  
  //Print the current error and PID controller parts, output, and the current motor speed
//  SERIAL_MONITER_SHOW();
}

void PLOTTER_SHOW(void)
{
  Serial.print(targetPosDed);
  Serial.print(" ");
  Serial.print(angleDeg);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(90);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  Serial.print(270);
  Serial.print(" ");
  Serial.println(360);
}

void SERIAL_MONITER_SHOW(void)
{
  if(reachedDes)
  {
    Serial.print("Destination Reached!!!!!!");
    Serial.print('\t');
    Serial.print("targetPos = ");
    Serial.print(targetPos);
    Serial.print('\t');
    Serial.print("curPos = ");
    Serial.println(curPos);
  }
  else
  { 
    Serial.print(curError_1);
    Serial.print('\t');
    Serial.print(kpPart_1);
    Serial.print('\t');
    Serial.print(kiPart_1);
    Serial.print('\t');
    Serial.print(kdPart_1);
    Serial.print('\t');  
    Serial.print(targetSpeed);
    Serial.print('\t');  
    Serial.print(curPos);
  
    Serial.print('\t');  
    Serial.print("||");
    Serial.print('\t');  
  
    Serial.print(curError_0);
    Serial.print('\t');
    Serial.print(kpPart_0);
    Serial.print('\t');
    Serial.print(kiPart_0);
    Serial.print('\t');
    Serial.print(kdPart_0);
    Serial.print('\t');  
    Serial.print(PIDOUT);
    Serial.print('\t');  
    Serial.println(curSpeed);
  }
}

double READ_POT(void)
{
  potRead = analogRead(pot);
  potRead = map(potRead, 0, 1023, 0, 75);
  potAns = (double)potRead / 1;
  potAns = potAns + potShift;
  return potAns;
}



void POSITION_CONTROL()
{
  //read the other incoder pin to know the direction of rotation
  //bState = HIGH => clockwise rotation
  //bState = LOW => counter-clockwise rotation
  bState = digitalRead(channelB);
  pulsesCnt++;
  if(bState == HIGH)
  {
    curPos = curPos + stepPerPul;
  }
  else
  {
    curPos = curPos - stepPerPul;
  }

  //set current angle to zero when finishing a full rotation
//  if(curPos > 2 * PI)
//    curPos = 0;
//  else if(curPos < (-2 * PI))
//    curPos = 0;
}

double myAbs(double A)
{
  if(A < 0)
    return -A;
  else
    return A;
}

ISR(TIMER1_COMPA_vect)
{
  //Calculate the angle error
  curError_1 = targetPos - curPos;

  //Turn on the PID controller if the error is greater than the allowed error value
  if(myAbs(curError_1) > allowedErrAngle)
  {  
//  Calculate the speed error
    errorSum_1 = errorSum_1 + curError_1;
//  Limit the error sum value
    if(errorSum_1 > maxIAngle)
      errorSum_1 = maxIAngle;
//  Calculate KP_part, KI_part, KD_part
    kpPart_1 = Kp_1*curError_1;
    kiPart_1 = Ki_1*errorSum_1*sampleTime;
    kdPart_1 = (Kd_1*(curError_1 - preError_1)) / sampleTime;
    targetSpeed = kpPart_1 + kiPart_1 + kdPart_1;
    preError_1 = curError_1;
    reachedDes = false;
  }
  else
  {
    targetSpeed = 0;
    reachedDes = true;   
  }

//Set the direction of the motor
  if(targetSpeed < 0)
  {
    motorDir = 0;
    targetSpeed = -targetSpeed;
  }
  else if(targetSpeed > 0)
  {
    motorDir = 1;
  }
  else
  {
     motorDir = 2;
  }

//Calculate the speed of the motor  
  curSpeed = (double)pulsesCnt * (1 / sampleTime); //number of pulses per second.
  curSpeed = curSpeed / pulsesPerRev; // rps = (counted pulses until now / total number of pulses per revolution)
  curSpeed = curSpeed * 60;
  pulsesCnt = 0; 
    
  if(targetSpeed == 0)
  {
    errorSum_0 = 0;
    errorSum_1 = 0;
    PIDOUT = 0;
  }
  else
  { 
//  Calculate the speed error
    curError_0 = targetSpeed - curSpeed;

//  Calculate the error sum
    errorSum_0 = errorSum_0 + curError_0;

//  Limit the error sum value
    if(errorSum_0 > maxISpeed)
      errorSum_0 = maxISpeed;

//  Calculate KP_part, KI_part, KD_part
    kpPart_0 = Kp_0*curError_0;
    kiPart_0 = Ki_0*errorSum_0*sampleTime;
    kdPart_0 = (Kd_0*(curError_0 - preError_0)) / sampleTime;

//  Calculate the PID output     
    motorPower = kpPart_0 + kiPart_0 + kdPart_0;

//  Save the previous value of the error
    preError_0 = curError_0;
    
    PIDOUT = (int16_t) motorPower;  
  }

//Determine the max limits and min limits of the PID controller output  
  if(PIDOUT > 255)
    PIDOUT = 255;
  else if(PIDOUT < 0)
    PIDOUT = 0;

//Set the motor direction
  if(motorDir == 1)
  {
    digitalWrite(INR0, HIGH);
    digitalWrite(INR1, LOW);  
  }
  else if(motorDir == 0)
  {
    digitalWrite(INR0, LOW);
    digitalWrite(INR1, HIGH);  
  }
  else
  {
    digitalWrite(INR0, HIGH);
    digitalWrite(INR1, HIGH);  
  }
  analogWrite(PWM,PIDOUT);
}
