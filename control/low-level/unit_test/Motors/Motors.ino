#define MOTOR0_EN  PA3
#define MOTOR0_IN1 PA5
#define MOTOR0_IN2 PA4

#define MOTOR1_EN PA1
#define MOTOR1_IN1 PA2
#define MOTOR1_IN2 PA0

void move_forward(int motor_speed);
void move_backward(int motor_speed);
void move_clockwise(int motor_speed);


void setup() {
  //Seting the modes of the speed pins  
  pinMode(MOTOR0_EN, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);

  //Setting the modes of the direction pins
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  analogWriteResolution(16);
}

void loop() {
  move_backward(60000);
}

void move_forward(int motor_speed)
{
  analogWrite(MOTOR0_EN, motor_speed);
  analogWrite(MOTOR1_EN, motor_speed);

  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  
}

void move_backward(int motor_speed)
{
  analogWrite(MOTOR0_EN, motor_speed);
  analogWrite(MOTOR1_EN, motor_speed);

  digitalWrite(MOTOR0_IN1, LOW);
  digitalWrite(MOTOR0_IN2, HIGH);

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  
}

void move_clockwise(int motor_speed)
{
  analogWrite(MOTOR0_EN, motor_speed);
  analogWrite(MOTOR1_EN, motor_speed);

  digitalWrite(MOTOR0_IN1, LOW);
  digitalWrite(MOTOR0_IN2, HIGH);

  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  
}
