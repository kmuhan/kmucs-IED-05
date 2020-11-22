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
#define _DUTY_MIN 1000
#define _DUTY_NEU 1400
#define _DUTY_MAX 2000

// Servo speed control
#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 60

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

//////////////////////
// global variables //
//////////////////////

float dist_min, dist_max, event_dist ;

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;
int a, b;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

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

  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX-_DUTY_MIN)*((float)_SERVO_SPEED/180)*((float)_INTERVAL_SERVO/1000);
  
  // initialize serial port
  Serial.begin(57600);

  a = 69.06;
  b = 357.76;
  
  // initialize last sampling time
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}


void loop() {

  //if(millis() < last_sampling_time_dist +  _INTERVAL_DIST) return;
  //if(millis() < last_sampling_time_servo + _INTERVAL_SERVO) return;
  //if(millis() < last_sampling_time_serial + _INTERVAL_SERIAL) return;
  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  float dist_ema = alpha*dist_cali + (1-alpha)*dist_cali;
  
  if (dist_ema < dist_target) {
    duty_curr += duty_chg_per_interval;
    if (dist_ema > dist_target) dist_ema = dist_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if (dist_ema < dist_target) dist_ema = dist_target;
  }
  
  myservo.writeMicroseconds(duty_curr);

  Serial.print("min:0,max:500,dist_ema:");
  Serial.println(dist_ema);

  last_sampling_time_dist +  _INTERVAL_DIST;
  last_sampling_time_servo + _INTERVAL_SERVO;
  last_sampling_time_serial + _INTERVAL_SERIAL;

}
