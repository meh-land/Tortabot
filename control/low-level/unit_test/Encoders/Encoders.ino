// STM32F1 (Bluee Pill) code using Official Arduino Core
//#define __STM32F1__

#define PIN_ENCODER_A PA7
#define PIN_ENCODER_B PA6


long long counter = 0 ;

void setup()
{

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_A, encoderPinChangeA, CHANGE);
  attachInterrupt(PIN_ENCODER_B, encoderPinChangeB, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  Serial.print("Counter : ");
  Serial.println(counter);

}



void encoderPinChangeB(void)
{
  counter += digitalRead(PIN_ENCODER_A) == digitalRead(PIN_ENCODER_B) ? -1 : 1;
}
void encoderPinChangeA(void)
{
  counter += digitalRead(PIN_ENCODER_A) != digitalRead(PIN_ENCODER_B) ? -1 : 1;
}
