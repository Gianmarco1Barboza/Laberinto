#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>  
const int ledVerde = 6;
const int ledRojo = 7;
const int ledArduino = 13;  
const int trigPin = 23;  
const int echoPin = 22;  

long duration;
int distance;

MPU6050 mpu;

double setpoint = 90;  
double input, output;
double Kp = 1, Ki = 0, Kd = 0;  

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double gz_offset = 0;   
long sum_gz = 0;  
int sampleCount = 100;  

bool isTurning = false;  
void setup() {
  Serial.begin(115200);
  
  
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);
  pinMode(ledArduino, OUTPUT);  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  
  Wire.begin();
  mpu.initialize();
  
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error al conectar el MPU6050.");
    while(1); 
  }
  
  
  calibrarGiroscopio();
  
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);  
  
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledArduino, LOW); 
}
void loop() {
  
  distanciaUltrasonico();
  
  if (distance < 15 && !isTurning) {
    
    digitalWrite(ledArduino, HIGH);  
    
    
    digitalWrite(ledRojo, LOW);  
    digitalWrite(ledVerde, HIGH);  
    Serial.println("Obstáculo detectado. Girando a la derecha.");
    girarDerecha(90);  
    isTurning = true;  
    digitalWrite(ledVerde, LOW);  
    digitalWrite(ledArduino, LOW);  
  } else {
   
    digitalWrite(ledRojo, LOW);  
    digitalWrite(ledVerde, LOW); 
    digitalWrite(ledArduino, LOW);  
    Serial.println("En línea recta");
  }
  delay(100);  
}
void distanciaUltrasonico() {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
 
  duration = pulseIn(echoPin, HIGH);
  
  
  distance = duration * 0.0344 / 2;  
}
void girarDerecha(int grados) {
  int16_t gx, gy, gz;
  double desired_angle = grados;  
  
  while (abs(input) < desired_angle) {
    
    mpu.getRotation(&gx, &gy, &gz);
    
    
    gz -= gz_offset;  
    
    
    input = gz;
    
    
    myPID.Compute();
    
    
    if (input > setpoint) {
      digitalWrite(ledRojo, HIGH);  
      digitalWrite(ledVerde, LOW);  
    }
    
    else if (input < setpoint) {
      digitalWrite(ledVerde, HIGH);  
      digitalWrite(ledRojo, LOW);    
    }
    delay(100);  
  }
  
  digitalWrite(ledVerde, LOW);  
}
void calibrarGiroscopio() {
  sum_gz = 0;  
  for (int i = 0; i < sampleCount; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum_gz += gz;
    delay(10);  
  }
  
  
  gz_offset = sum_gz / sampleCount;
  Serial.print("Calibración completada. Offset de gz: ");
  Serial.println(gz_offset);
}
