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
#define _DIST_ALPHA 0.4

//Servo range
#define _DUTY_MIN 540
#define _DUTY_NEU 1400
#define _DUTY_MAX 2260

// Servo speed control
#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)
#define _SERVO_ANGLE 50
#define _SERVO_SPEED 300

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 20

//PID parameters
#define _KP 1.0

//////////////////////
// global variables //
//////////////////////

float dist_min, dist_max;

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;
int a, b;
const float coE[] = {-0.0000090, 0.0078906, -0.4842492, 103.1485335}; // ir_filtered

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;
int duty_neu, duty_max, duty_min;

//PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

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

  duty_target = duty_curr = _POS_START;
  
  myservo.writeMicroseconds(_DUTY_NEU);
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX-_DUTY_MIN)*((float)_SERVO_SPEED/180)*((float)_INTERVAL_SERVO/1000);
  
  // initialize serial port
  Serial.begin(57600);
  
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
    float x = ir_distance();
    dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
    dist_ema = alpha*dist_raw + (1-alpha)*dist_ema;
    dist_raw = dist_ema;
    
    // PID control logic
    error_curr = dist_target - dist_raw;
    pterm = error_curr*_KP;
    control = pterm;
    
    // duty_target = f(duty_neutral, control)
    if (dist_raw < dist_target) {
      duty_target = duty_neu + duty_neu*(control/150);
    }
    else {
      duty_target = duty_neu + duty_neu*(control/175);
    }
    
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
   Serial.print("dist_ir:");
   Serial.print(dist_raw);
   Serial.print(",pterm:");
   Serial.print(map(pterm,-1000,1000,510,610));
   Serial.print(",duty_target:");
   Serial.print(duty_target);
   Serial.print(map(duty_target,1000,2000,410,510));
   Serial.print(",duty_curr:");
   Serial.print(map(duty_curr,1000,2000,410,510));
   Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
   
   last_sampling_time_serial = millis();
  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
