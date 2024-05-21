#include <rxwheel.h>

#define SOL_PIN 4

float controllerbandwidth = 40; //Hz
float omegaBandwidth = 40;
float alphaBandwidth = 1;
float Iwheel = 0.0019098;
float Isat = 0.00042853;

typedef enum{
  IDLE,
  RISE,
  WAITFALL,
  DWELL,
  DETECTED,
  FALL,
}state_t;

typedef enum{
  WAIT_A0,
  LOW_A0,
  WAIT_A1,
  LOW_A1,
  SOL_ON,
  SOL_OFF,
} solenoid_state_t;

SerialComms comms;
CmdVals_t cmdVals{0, 0, 0, 0, 0};
//MPU6050 imu;
PID_control c0, c1, c2, c3, c4;
Differentiator wheeldiff(0.01, 1 / (omegaBandwidth * 2 * PI));
Differentiator acceldiff(0.01, 1 / (alphaBandwidth * 2 * PI));

unsigned long cur, prev, prevsm, prevmotor;
unsigned long dt = 10000;
unsigned long debounce_dt = 10000;
int counter = 0;
bool flip = false;
float power, thrusterPower;
int pwm, thrusterPwm;
int32_t count = 0;
float N = 1000;
float wDesired, thetaDesired, wActual, thetaActual;
state_t state;

// IR PHOTO DIODE THRESHOLDS
int high_thresh = 700;
int low_thresh = 650;

// Solenoid vars
unsigned long sol_on_time=0;
solenoid_state_t sol_state;

// Pendulum vars
int32_t swing_count=0;

void setup() {
  state = IDLE;
  Serial.begin(115200);
  pinMode(SOL_PIN, OUTPUT);
  
  digitalWrite(SOL_PIN, LOW);
  // put your setup code here, to run once:
//  imu.begin();
//  imu.calibrate();
  motorSetup();
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), doA, RISING);

  //default c0 for a moment (motor vel)
  c0.setGains(0.824674, 9.132689, -0.000339689);
  c0.setTimeParameters(0.01, 1 / (controllerbandwidth * 2 * PI));
  c0.setLimits(-12, 12);
  c0.setDeadbands(-2, 2);
  c0.antiWindupEnabled = true;
  //Sat Omega controller (inner loop)
  c1.setGains(0.005, 0.00001, 0);
  c1.setTimeParameters(0.01, 1 / (controllerbandwidth * 2 * PI));
  c1.setLimits(-15, 15);
  c1.antiWindupEnabled = true;
  //  c1.setDeadbands(-3.6, 3.6);
  //Sat Theta controller (outer loop)
  c2.setGains(50, 0.1, 0);
  c2.setTimeParameters(0.01, 1 / (controllerbandwidth * 2 * PI));
  c2.setLimits(-10, 10);
  c2.antiWindupEnabled = true;
  //motor acceleration control
  c3.setGains(0.1, 0.4, 0);
  c3.setTimeParameters(0.01, 1 / (controllerbandwidth * 2 * PI));
  c3.setLimits(-12, 12);
  c3.antiWindupEnabled = true;
  c3.setDeadbands(-2, 2);

  comms.writeData=true;
}

void loop() {
  cur = micros();

  comms.handleCommand();
  if (cur - prev > dt) {
    //Run feedback control
      //Estimate motor velocity 
      wActual = wheeldiff.differentiate(2 * PI * count / (N));
//      thetaDesired = 1*sin(2*PI*cur/1000000);
      thetaDesired = -0.25*swing_count;
      thetaActual = thetaActual + wActual * (dt / 1000000.0); // Integrate wSat to get theta (approximate

      // Outer loop of sat (sat theta)
      wDesired = c2.pid(thetaDesired, thetaActual);
//              wDesired = 1.5;

      //Motor Velocity feedback control (using c0)
      power = c0.pid(wDesired, wActual);

    Serial.print(thetaDesired);
    Serial.print(',');
    Serial.print(thetaActual);;
    Serial.print(',');
    Serial.print(swing_count);;
    Serial.print(',');
    Serial.print(count);;
    Serial.print(',');
    Serial.print(power);
    Serial.print(',');
    Serial.print(analogRead(A0));
    Serial.print(',');
    Serial.print(analogRead(A1));
    Serial.println();
//      comms.sendData((cur - prev) / 1000000.0, cmdVals.thetaSetpoint, thetaActual, 0.0, wActual);
    prev = cur;
  }
    //photo diode state machine
    switch(state)
    {
      case IDLE:
        if(analogRead(A0) > high_thresh){ state = RISE; }
        break;
      case RISE:
        if(cur - prevsm > debounce_dt){
          if(analogRead(A0) > high_thresh){
            state = DETECTED;
          }
          else{
            state = IDLE;
          }
          prevsm=cur;
        }
        break;
      case DETECTED:
        swing_count++;
        state = DWELL;
        prevmotor=micros();
        if(count <= -3000){ swing_count = 0; }
        break;
      case DWELL:
        if(cur - prevmotor >= debounce_dt){state = WAITFALL;}
        break;
      case WAITFALL:
        if(analogRead(0) < low_thresh){ state =IDLE; }
    }
  

  switch(sol_state)
  {
    case WAIT_A0:
      if(analogRead(A0) < low_thresh)  { sol_state = LOW_A0; }
      break;
    case LOW_A0:
      if(analogRead(A0) >= high_thresh) { sol_state= WAIT_A1; }
      break;
    case WAIT_A1:
      if(analogRead(A1) < low_thresh) { sol_state= LOW_A1; }
      break;
    case LOW_A1:
      if(analogRead(A1) >= high_thresh){ sol_state = SOL_ON; sol_on_time = micros(); }
      break;
    case SOL_ON:
      digitalWrite(SOL_PIN, HIGH);
      if(cur - sol_on_time >= 350000) { sol_state = SOL_OFF; }
      break;
    case SOL_OFF:
      digitalWrite(SOL_PIN, LOW);
      sol_state= WAIT_A0;
      break;
  }

  //Write motor power and direction (convert from voltage to pwm)
  pwm = int(-1 * power * 255 / 7.5);
  thrusterPwm = int(thrusterPower * 255 / 7.5);
  rawMotorCtrl(constrain(pwm, -255, 255), 0);

}

void doA()
{
  int B = digitalRead(3);
  if (B == HIGH) {
    count++;
  }
  else {
    count--;
  }
}
