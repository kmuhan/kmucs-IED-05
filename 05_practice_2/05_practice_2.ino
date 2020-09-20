#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  count = toggle = 0;
  digitalWrite(PIN_LED, 1);
}


void loop() {   
  if (count == 5) {
  digitalWrite(PIN_LED, 1);
  while(1) {
    ;
    }
  }
  if (count == 0) {
    digitalWrite(PIN_LED, 0);
    delay(1000);
  }
 
  digitalWrite(PIN_LED, 1);
  delay(100);
  digitalWrite(PIN_LED, 0);
  delay(100);
  ++count;
}
