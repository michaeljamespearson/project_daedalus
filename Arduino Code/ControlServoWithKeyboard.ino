#include <Servo.h>

int incomingByte = 0;   // for incoming serial data
Servo s;
int pos;
void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pos = s.read();
  s.attach(9);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    digitalWrite(10,LOW);
    digitalWrite(8,LOW);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    // read the incoming byte:
    incomingByte = Serial.read();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
    if (incomingByte == 108) { //l
      pos += 10;
      if (pos >= 180) {
        s.write(180);
        Serial.println("Max value of 180 reached");
      }
      else {
        s.write(pos);
        Serial.println("Adding 10 to the degrees");
        Serial.print("Current position is ");
        Serial.print(pos);
        Serial.println();
      }
      digitalWrite(6,HIGH);
    }
    else if (incomingByte == 106)  { //j
      pos -= 10;
      if (pos <= 0) {
        s.write(0);
        Serial.println("Min value of 0 reached");
      }
      else {
        s.write(pos);
        digitalWrite(10, LOW);
        Serial.println("Subtracting 10 to the degrees");
        Serial.print("Current position is ");
        Serial.print(pos);
        Serial.println();
      }
      digitalWrite(7,HIGH);
    }
    else if (incomingByte == 100) { //d
      digitalWrite(10, HIGH);
      pos += 1;
      if (pos >= 180) {
        s.write(180);
        Serial.println("Max value of 180 reached");
      }
      else {
        s.write(pos);
        Serial.println("Adding 5 to the degrees");
        Serial.print("Current position is ");
        Serial.print(pos);
        Serial.println();
      }
      digitalWrite(10,HIGH);
    }
    else if (incomingByte == 97) { //a
      pos -= 1;
      if (pos <= 0) {
        s.write(0);
        Serial.println("Min value of 0 reached");
      }
      else {
        s.write(pos);
        digitalWrite(10, LOW);
        Serial.println("Subtracting 5 to the degrees");
        Serial.print("Current position is ");
        Serial.print(pos);
        Serial.println();
      }
      digitalWrite(8,HIGH);
    }
  }
}
