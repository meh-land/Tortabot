/*
ABOUT: TEST LIMIT SWITCH
DATE : 9/5/2023

*/

#define SWITCH PB6

void setup() {

  pinMode(SWITCH, INPUT_PULLUP);


}

void loop() {
  Serial.print("SWITCH: ");
  Serial.println(digitalRead(SWITCH));

}
