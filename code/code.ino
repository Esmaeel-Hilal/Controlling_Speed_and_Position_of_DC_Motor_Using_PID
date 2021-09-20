#define channelA 2
#define channelB 24 

#define PWM 3 //pwm for the left motor
#define INR0 30 //output 1 for right motor driver
#define INR1 32 //output 2 for right motor driver

#define Kp_0  25
#define Ki_0  0.001
#define sampleTime  0.005
//#define sampleTime  0.01

volatile bool bState;
volatile double angle, angleDeg, preAngle;
volatile double stepPerPul = 0.01026664265; // 2 * PI / number of pulses per revolution
volatile double error_0, errorSum_0;
volatile double motorPower = 127, curSpeed;
volatile double targetSpeed = 50; 
double fullRotation = 6.2831853;
volatile unsigned long pulsesCnt, prePulsesCnt;
volatile int n;
volatile bool motorDir;

int motorPower_1 = 150;

void INIT_TIMER(void)
{
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
//  OCR1A = 19999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(channelA), POSITION_CONTROL, RISING);  
  Serial.begin(9600);
  pinMode(PWM, OUTPUT);
  pinMode(INR0, OUTPUT);
  pinMode(INR1, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  
  

  INIT_TIMER();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  angleDeg = (angle * 360) / (2 * PI);
//  Serial.print(motorDir);
//  Serial.print('\t');
//  Serial.print(angleDeg);
//  Serial.print('\t');
//  Serial.print(curSpeed);
//  Serial.print('\t');
//  Serial.print(pulsesCnt);
//  Serial.print('\t');
//  Serial.println(motorPower);

  Serial.println(errorSum_0);
//

//




  digitalWrite(INR0, HIGH);
  digitalWrite(INR1, LOW);
//  analogWrite(PWM,motorPower_1);
//  Serial.println(curSpeed);
  analogWrite(PWM,motorPower);
//  Serial.print(curSpeed);
//  Serial.print(" ");
//  Serial.println(targetSpeed);  
}

void POSITION_CONTROL()
{
//  interrEnter = HIGH;
  bState = digitalRead(channelB);
  pulsesCnt++;
  if(bState == LOW)
  {
    angle = angle + stepPerPul;
  }
  else
  {
    angle = angle - stepPerPul;
  }
  if(angle > 2 * PI)
    angle = 0;
  else if(angle < (-2 * PI))
  {
    angle = 0;
  }
//  if(pulsesCnt > 482) //477
//  {
//    digitalWrite(INR0, LOW);
//    digitalWrite(INR1, LOW);
//    analogWrite(RPWM, 0);
//  }
}

ISR(TIMER1_COMPA_vect)
{
//  digitalWrite(22, HIGH);
  if(n > 9)
  {
//    digitalWrite(22, !digitalRead(22));
    curSpeed = pulsesCnt / 0.05; // number of pulses per second = (pulses / sec)
    curSpeed = curSpeed / 612; // rps = (counted pulses until now / total number of pulses per revolution)
    curSpeed = curSpeed * 60;
    pulsesCnt = 0;  
    n = 0;
    error_0 = targetSpeed - curSpeed;
    errorSum_0 = errorSum_0 + error_0;  
//  errorSum_0 = constrain(errorSum_0, -100, 100);
  //calculate output from KP, KI
    motorPower = Kp_0*(error_0) + Ki_0*(errorSum_0)*sampleTime*10;
    motorPower = constrain(motorPower, 0, 255);
  }
  n++;
//  if(motorPower < 0)
//  {
//    motorDir = 0;
//    motorPower = -motorPower;
//  }
//  else
//    motorDir = 1;
//  digitalWrite(22, LOW);
}
