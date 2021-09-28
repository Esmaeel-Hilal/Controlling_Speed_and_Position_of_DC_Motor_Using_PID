#define channelA 2
#define channelB 24 

#define PWM 3 //pwm for the left motor
#define INR0 30 //output 1 for right motor driver
#define INR1 32 //output 2 for right motor driver

#define pot A0

volatile double Kp_0 = 2; 
volatile double Ki_0 = 6;
volatile double Kd_0 = 0;

volatile double Kp_1 = 0; 
volatile double Ki_1 = 0;
volatile double Kd_1 = 0;

volatile double sampleTime = 0.1;
volatile double kpPart_0, kiPart_0, kdPart_0;
volatile double kpPart_1, kiPart_1, kdPart_1;
volatile int16_t PIDOUT;
//volatile uint8_t doubleSampleTime = 20;

volatile bool bState;
volatile double angle, angleDeg, preAngle;
volatile double stepPerPul = 0.01026664265; // 2 * PI / number of pulses per revolution
volatile double curError_0, preError_0, errorSum_0;
volatile double curError_1, preError_1, errorSum_1;
volatile double motorPower = 127, curSpeed;
volatile double curPos = 0;
volatile double targetSpeed = 30; 
double fullRotation = 6.2831853;
volatile unsigned long pulsesCnt, prePulsesCnt;
volatile int8_t n;
volatile bool motorDir;



int16_t potRead, potShift = 0;
double potAns;



uint8_t motorPower_1 = 255;

void INIT_TIMER(void)
{
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 24999;
//  OCR1A = 9999;    
//  OCR1A = 19999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 64
  TCCR1B |= (1 << CS11) | (1 << CS10); 
// Set CS11 bit for prescaling by 8
//    TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(channelA), POSITION_CONTROL, RISING);  
  Serial.begin(38400);
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
  
//  motorPower_1 = READ_POT();
//  Serial.print(motorPower_1);
//  Serial.print('\t');
//  Serial.println(error_0);

  
//  Serial.println(errorSum_0);

//  Serial.println(curError_0 - preError_0);

  
  digitalWrite(INR0, HIGH);
  digitalWrite(INR1, LOW);
//  analogWrite(PWM,motorPower_1);
//  Serial.println(curSpeed);
  analogWrite(PWM,PIDOUT);
  
  Serial.print(curSpeed);
  Serial.print(" ");
  Serial.print(targetSpeed);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
//  Serial.print(20);
//  Serial.print(" ");
//  Serial.print(40);
//  Serial.print(" ");
//  Serial.print(60);
//  Serial.print(" ");
//  Serial.print(80);
//  Serial.print(" ");
  Serial.println(100);

//  Serial.print(curError_0);
//  Serial.print('\t');
//  Serial.print(kpPart);
//  Serial.print('\t');
//  Serial.print(kiPart);
//  Serial.print('\t');
//  Serial.print(kdPart);
//  Serial.print('\t');  
//  Serial.print(PIDOUT);
//  Serial.print('\t');  
//  Serial.println(curSpeed);

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
}

ISR(TIMER1_COMPA_vect)
{
    curSpeed = (double)pulsesCnt / sampleTime;
    curSpeed = curSpeed / 612; // rps = (counted pulses until now / total number of pulses per revolution)
    curSpeed = curSpeed * 60;
    pulsesCnt = 0;  
    curError_0 = targetSpeed - curSpeed;
    errorSum_0 = errorSum_0 + curError_0;  
//    errorSum_0 = constrain(errorSum_0, -200, 200);
//  calculate output from KP, KI, KD
    kpPart_0 = Kp_0*curError_0;
    kiPart_0 = Ki_0*errorSum_0*sampleTime;
    kdPart_0 = (Kd_0*(curError_0 - preError_0)) / sampleTime;
    motorPower = kpPart_0 + kiPart_0 + kdPart_0;
    preError_0 = curError_0;
    PIDOUT = (int16_t) motorPower;
    if(PIDOUT > 255)
      PIDOUT = 255;
    else if(PIDOUT < 0)
      PIDOUT = 0;
}
