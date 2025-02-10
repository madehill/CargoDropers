#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

// Пины для шаговых двигателей
#define MOTOR1_STEP 2
#define MOTOR1_DIR 3
#define MOTOR2_STEP 4
#define MOTOR2_DIR 5
#define MOTOR3_STEP 6
#define MOTOR3_DIR 7
#define MOTOR4_STEP 8
#define MOTOR4_DIR 9

// Пины для датчиков линии
#define LINE_SENSOR1 A0
#define LINE_SENSOR2 A1
#define LINE_SENSOR3 A2
#define LINE_SENSOR4 A3
#define LINE_SENSOR5 A4

// Пин для сервопривода
#define SERVO_PIN 10

// Пины для сонара
#define SONAR_TRIG 11
#define SONAR_ECHO 12

// Создание объекта для датчика цвета
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Создание объекта для сервопривода
Servo gripperServo;

// Переменные для хранения данных с датчиков
int lineSensorValues[5];
int objectDistance = 0;
bool objectDetected = false;
uint16_t r, g, b, c;

// Переменная для хранения цвета объекта
enum ObjectColor { RED, BLUE, GREEN, UNKNOWN };
ObjectColor detectedColor = UNKNOWN;

void setup() {
  // Инициализация шаговых двигателей
  pinMode(MOTOR1_STEP, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_STEP, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_STEP, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_STEP, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  // Инициализация датчиков линии
  pinMode(LINE_SENSOR1, INPUT);
  pinMode(LINE_SENSOR2, INPUT);
  pinMode(LINE_SENSOR3, INPUT);
  pinMode(LINE_SENSOR4, INPUT);
  pinMode(LINE_SENSOR5, INPUT);

  // Инициализация сонара
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);

  // Инициализация сервопривода
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(90); // Исходное положение клешни

  // Инициализация датчика цвета
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  Serial.begin(9600);
}

void loop() {
  // Чтение данных с датчиков линии
  readLineSensors();

  // Следование по линии
  followLine();
  handleObject();

  // Поиск объекта с помощью сонара
  objectDistance = getDistance();
  if (objectDistance < 10) { // Если объект ближе 10 см
    objectDetected = true;
    stopMotors();
    detectColor();
    handleObject();
  }
}

void readLineSensors() {
  lineSensorValues[0] = analogRead(LINE_SENSOR1);
  lineSensorValues[1] = analogRead(LINE_SENSOR2);
  lineSensorValues[2] = analogRead(LINE_SENSOR3);
  lineSensorValues[3] = analogRead(LINE_SENSOR4);
  lineSensorValues[4] = analogRead(LINE_SENSOR5);
}

void followLine() {
  // Простейший алгоритм следования по линии
  if (lineSensorValues[2] > 500) { // Если центральный датчик на линии
    moveForward();
  } else if (lineSensorValues[1] > 500) { // Если левый датчик на линии
    turnLeft();
  } else if (lineSensorValues[3] > 500) { // Если правый датчик на линии
    turnRight();
  }
}

void moveForward() {
  digitalWrite(MOTOR1_DIR, HIGH);
  digitalWrite(MOTOR2_DIR, HIGH);
  digitalWrite(MOTOR3_DIR, HIGH);
  digitalWrite(MOTOR4_DIR, HIGH);
  for (int i = 0; i < 100; i++) {
    digitalWrite(MOTOR1_STEP, HIGH);
    digitalWrite(MOTOR2_STEP, HIGH);
    digitalWrite(MOTOR3_STEP, HIGH);
    digitalWrite(MOTOR4_STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(MOTOR1_STEP, LOW);
    digitalWrite(MOTOR2_STEP, LOW);
    digitalWrite(MOTOR3_STEP, LOW);
    digitalWrite(MOTOR4_STEP, LOW);
    delayMicroseconds(1000);
  }
}

void turnLeft() {
  digitalWrite(MOTOR1_DIR, LOW);
  digitalWrite(MOTOR2_DIR, HIGH);
  digitalWrite(MOTOR3_DIR, LOW);
  digitalWrite(MOTOR4_DIR, HIGH);
  for (int i = 0; i < 50; i++) {
    digitalWrite(MOTOR1_STEP, HIGH);
    digitalWrite(MOTOR2_STEP, HIGH);
    digitalWrite(MOTOR3_STEP, HIGH);
    digitalWrite(MOTOR4_STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(MOTOR1_STEP, LOW);
    digitalWrite(MOTOR2_STEP, LOW);
    digitalWrite(MOTOR3_STEP, LOW);
    digitalWrite(MOTOR4_STEP, LOW);
    delayMicroseconds(1000);
  }
}

void turnRight() {
  digitalWrite(MOTOR1_DIR, HIGH);
  digitalWrite(MOTOR2_DIR, LOW);
  digitalWrite(MOTOR3_DIR, HIGH);
  digitalWrite(MOTOR4_DIR, LOW);
  for (int i = 0; i < 50; i++) {
    digitalWrite(MOTOR1_STEP, HIGH);
    digitalWrite(MOTOR2_STEP, HIGH);
    digitalWrite(MOTOR3_STEP, HIGH);
    digitalWrite(MOTOR4_STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(MOTOR1_STEP, LOW);
    digitalWrite(MOTOR2_STEP, LOW);
    digitalWrite(MOTOR3_STEP, LOW);
    digitalWrite(MOTOR4_STEP, LOW);
    delayMicroseconds(1000);
  }
}

void stopMotors() {
  digitalWrite(MOTOR1_STEP, LOW);
  digitalWrite(MOTOR2_STEP, LOW);
  digitalWrite(MOTOR3_STEP, LOW);
  digitalWrite(MOTOR4_STEP, LOW);
}

int getDistance() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);
  long duration = pulseIn(SONAR_ECHO, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void detectColor() {
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("R: "); Serial.print(r); Serial.print(" ");
  Serial.print("G: "); Serial.print(g); Serial.print(" ");
  Serial.print("B: "); Serial.print(b); Serial.print(" ");
  Serial.print("C: "); Serial.print(c); Serial.println(" ");

  // Логика определения цвета
  if (r > g && r > b) {
    detectedColor = RED;
    Serial.println("Detected RED object");
  } else if (b > r && b > g) {
    detectedColor = BLUE;
    Serial.println("Detected BLUE object");
  } else if (g > r && g > b) {
    detectedColor = GREEN;
    Serial.println("Detected GREEN object");
  } else {
    detectedColor = UNKNOWN;
    Serial.println("Unknown color");
  }
}

void handleObject() {
  switch (detectedColor) {
    case RED:
      avoidObject(); // Объехать красный объект
      break;
    case BLUE:
      grabObject();
      deliverObjectToSide(); // Отвезти синий объект в сторону
      break;
    case GREEN:
      grabObject();
      deliverObjectToFinish(); // Отвезти зеленый объект на финиш
      break;
    default:
      // Неизвестный цвет, просто продолжить движение
      break;
  }
  objectDetected = false;
}

void avoidObject() {
  // Логика объезда объекта
  Serial.println("Avoiding RED object");
  moveBackward();
  delay(500);
  turnRight();
  delay(1000);
  moveForward();
  delay(1000);
  turnLeft();
  delay(1000);
  moveForward();
}

void grabObject() {
  gripperServo.write(0); // Смыкание клешни
  delay(1000);
}

void deliverObjectToSide() {
  Serial.println("Delivering BLUE object to the side");
  moveForward();
  delay(1000);
  turnRight();
  delay(1000);
  moveForward();
  delay(1000);
  gripperServo.write(90); // Размыкание клешни
  delay(1000);
  moveBackward();
  delay(1000);
  turnLeft();
  delay(1000);
  moveForward();
}

void deliverObjectToFinish() {
  Serial.println("Delivering GREEN object to the finish");
  moveForward();
  delay(500);
  moveBackward();
  delay(2000);
  turnLeft();
  delay(1000);
  followLine();
  delay(68000);
  gripperServo.write(90); // Размыкание клешни
  delay(1000);
  moveBackward();
  delay(1000);
}

void moveBackward() {
  digitalWrite(MOTOR1_DIR, LOW);
  digitalWrite(MOTOR2_DIR, LOW);
  digitalWrite(MOTOR3_DIR, LOW);
  digitalWrite(MOTOR4_DIR, LOW);
  for (int i = 0; i < 100; i++) {
    digitalWrite(MOTOR1_STEP, HIGH);
    digitalWrite(MOTOR2_STEP, HIGH);
    digitalWrite(MOTOR3_STEP, HIGH);
    digitalWrite(MOTOR4_STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(MOTOR1_STEP, LOW);
    digitalWrite(MOTOR2_STEP, LOW);
    digitalWrite(MOTOR3_STEP, LOW);
    digitalWrite(MOTOR4_STEP, LOW);
    delayMicroseconds(1000);
  }
}
