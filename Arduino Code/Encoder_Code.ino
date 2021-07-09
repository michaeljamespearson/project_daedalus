#define outputA 13
#define outputB 11
#include <Servo.h>

int counter = 0;
int count = 0;
int aState;
int aLastState;
int pos = 0;
Servo servoD;
unsigned long time;


void setup() {
  // put your setup code here, to run once:
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  servoD.attach(9);
  pinMode(9, OUTPUT);

  Serial.begin(230400);

  aLastState = digitalRead(outputA);

}

void loop() {
  aState = digitalRead(outputA);
  if (aState != aLastState) {
    count++;
    if (count % 10) {
      if (digitalRead(outputB) != aState) {
        counter++;
      }
      else {
        counter --;
      }
      Serial.print("Position: ");
      Serial.println(counter);
    }
  }
  aLastState = aState;

  //  if (counter > 5) {
  //    Serial.println("Braking");
  //    brake();
  //  }

}

