/*
 * Made by Michael Pearson
 * 
 */
#include <Servo.h>

int val;
int encoder0PinA = 13;
int encoder0PinB = 11;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
double rev = 0.0;
int counter = 0;
Servo servoD;
int count = 0;

void setup() {
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  Serial.begin(57600);
  pinMode(9, OUTPUT);
  servoD.attach(9);
  Serial.println(servoD.read());
}

void loop() {
  if (count == 0) {
    servoD.write(180);
    Serial.println("setting to 180");
    count++;
  }
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    //  if (digitalRead(encoder0PinB) == LOW) {
    //  encoder0Pos--; q
    //} else {
    encoder0Pos++;
    //}

    Serial.print("Rev: ");
    Serial.print(rev, DEC);
    Serial.print("     Angle: ");
    Serial.print(encoder0Pos);
    Serial.print("     Servo is at: ");
    Serial.println(servoD.read());
  }
  encoder0PinALast = n;

  rev = (double) encoder0Pos / 200;

  if (rev > 1 && counter == 0) {
    servoD.write(73);
    Serial.println("Braking");
    counter++;
  }
}
