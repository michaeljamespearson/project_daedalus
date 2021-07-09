#include <Servo.h>

int incomingByte = 0;   // for incoming serial data
Servo s;
int pos;
void setup() {
        Serial.begin(9600);
        pinMode(10,OUTPUT);
        pos = 0;
}

void loop() {

        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
                if(incomingByte == 108) { //l
                digitalWrite(10,HIGH);
                pos +=5;
                s.write(pos);
                Serial.println("Adding 5 to the degrees");
                Serial.print("Current position is ");
                Serial.print(pos);
                Serial.println();
                }
                else if(incomingByte == 106)  { //j
                  pos -=5;
                  s.write(pos);
                  digitalWrite(10,LOW);
                  Serial.println("Subtracting 5 to the degrees");
                  Serial.print("Current position is ");
                Serial.print(pos);
                Serial.println();
                }
                else if(incomingByte == 100) { //d
                digitalWrite(10,HIGH);
                pos +=1;
                s.write(pos);
                Serial.println("Adding 1 to the degrees");
                Serial.print("Current position is ");
                Serial.print(pos);
                Serial.println();
                  
                }
                else if(incomingByte == 97) { //a
                  pos -=1;
                  s.write(pos);
                  digitalWrite(10,LOW);
                  Serial.println("Subtracting 1 to the degrees");
                  Serial.print("Current position is ");
                Serial.print(pos);
                Serial.println();
                }
        }
}
