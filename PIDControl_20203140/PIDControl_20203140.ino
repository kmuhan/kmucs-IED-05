#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_LED 9
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.1

//Servo range
#define _DUTY_MIN 650
#define _DUTY_NEU 1300
#define _DUTY_MAX 1950

// Servo speed control
#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 700

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

//PID parameters
#define _ITERM_MAX 15.0
#define _KP 1.03
#define _KD 36.5L
#define _KI 0.0019

// Filter
#define DELAY_MICROS  1500
#define EMA_ALPHA 0.25

//////////////////////
// global variables //
//////////////////////

float dist_min, dist_max;

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;
const float coE[] = {0.0000097, 0.0006014, 0.6475733, 59.6953395}; // ir_filtered
int a, b; // unit: mm

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;
int duty_neu, duty_max, duty_min;

//PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//Filter
float ema_dist=0; 
float samples_num = 3;
float filtered_dist;

void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
   
  // initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA;

  duty_neu = _DUTY_NEU;
  duty_max = _DUTY_MAX;
  duty_min = _DUTY_MIN;

  iterm, dterm, pterm = 0;

  duty_target = duty_curr = _POS_START;
  
  myservo.writeMicroseconds(_DUTY_NEU);
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX-_DUTY_MIN)*((float)_SERVO_SPEED/180)*((float)_INTERVAL_SERVO/1000);
  
  // initialize serial port
  Serial.begin(57600);
  
  a = 69.06;
  b = 232.12;
  
  // initialize last sampling time
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
}

void loop() {

  /////////////////////
  // Event generator //
  /////////////////////
  
  if(millis() >= last_sampling_time_dist +  _INTERVAL_DIST)event_dist = true;
  if(millis() >= last_sampling_time_servo + _INTERVAL_SERVO)event_servo = true;
  if(millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)event_serial = true;

  ////////////////////
  // Event handlers //
  ////////////////////
  
  if(event_dist){
    event_dist = false;
    filtered_dist = filtered_ir_distance();
    float x = filtered_dist;
    dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
    //dist_raw = 100 + 300.0 / (b-a) * (x - a);
//    dist_ema = alpha*dist_raw + (1-alpha)*dist_ema;
//    dist_raw = dist_ema;
    
    // PID control logic
    error_curr = dist_target - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    if(abs(iterm) > _ITERM_MAX)iterm = 0;
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;
    
    //Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit

    // update error_prev
    error_prev = error_curr;
    
    last_sampling_time_dist = millis(); 
  }

  if(event_servo){
    event_servo = false;
    
    if (duty_curr < duty_target) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    
    myservo.writeMicroseconds(duty_curr);
    
    last_sampling_time_servo = millis();  
  }

  if(event_serial) {
   event_serial = false;
   Serial.print("IR:");
   Serial.print(dist_raw);
   Serial.print(",T:");
   Serial.print(dist_target);
   Serial.print(",P:");
   Serial.print(map(pterm,-1000,1000,510,610));
   Serial.print(",D:");
   Serial.print(map(dterm,-1000,1000,510,610));
   Serial.print(",I:");
   Serial.print(map(iterm,-1000,1000,510,610));
   Serial.print(",DTT");;
   Serial.print(map(duty_target,1000,2000,410,510));
   Serial.print(",DTC:");
   Serial.print(map(duty_curr,1000,2000,410,510));
   Serial.println(",-G:245,+G:265,m:0,M:800");
   
   last_sampling_time_serial = millis();
  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
