
#include <Servo.h>
//#include <Keyboard.h>

// defining global variables
int max_pulse = 2000;
int min_pulse = 1000;
int esc_pin = 12;
int potValue = 0;
//char ctrlKey = KEY_LEFT_CTRL;

Servo ESC; 

void setup() {
  // attaching esc with global variables:
  pinMode(esc_pin, OUTPUT);
  ESC.attach(esc_pin);
  Serial.begin(9600);
  //Keyboard.begin();
  
}

void loop() {
  potValue = map(analogRead(A3), 0, 1024, 940, 1700);
// if (Serial.available() > 0){
//    potValue = potValue + Serial.parseInt();
//  }
//
  ESC.write(potValue);
  Serial.println();
  Serial.print("Throttle Command: ");
  Serial.print(potValue);
}

  
