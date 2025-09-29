#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Pines
const int ledRojo = 7;
const int ledVerde = 6;
const int ledamarillo = 5;
const int trig = 23;
const int echo = 22;

// Variables MPU
int16_t gx, gy, gz;
float anguloZ = 0;
float anguloOffset = 0;
unsigned long tiempoPrevio = 0;

// Variables PID
float anguloDeseado = 0;
float error, errorAnterior = 0, integral = 0;
float Kp = 1.5;
float Ki = 0.01;
float Kd = 0.3;

bool girando = false;

// Variables para control híbrido
int contadorEstable = 0;
const int CICLOS_ESTABILIDAD = 5; // 5 ciclos ≈ 250ms
const float UMBRAL_VELOCIDAD = 3.0; // 3°/s - velocidad máxima para considerar "detenido"
const float UMBRAL_ERROR = 3.0; // ±3° de error aceptable

void setup() {
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledamarillo, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  Serial.begin(9600);
  Serial.println("🤖 SISTEMA DE NAVEGACIÓN ROBOT");
  Serial.println("================================");
  
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("✅ MPU6050 - CONECTADO");
  } else {
    Serial.println("❌ MPU6050 - FALLO");
    while(1);
  }
  
  calibrarGiroscopio();
  Serial.println("🎯 SISTEMA LISTO - Buscando obstáculos...");
  Serial.println();
}

void loop() {
  // Actualizar ángulo continuamente
  actualizarAngulo();
  
  // Solo procesar ultrasonico si NO estamos girando
  if (!girando) {
    float distancia = medirDistancia();
    
    // Mostrar siempre la distancia y ángulo
    Serial.print("📏 Distancia: ");
    Serial.print(distancia);
    Serial.print(" cm");
    
    Serial.print(" | 🎯 Angulo: ");
    Serial.print(anguloZ, 1);
    Serial.println("°");
    
    // 🚨 DETECCIÓN DE OBSTÁCULO
    if (distancia < 15.0 && distancia > 2.0) {
      Serial.println();
      Serial.println("🚨 ¡OBSTÁCULO DETECTADO! Iniciando giro de 90°...");
      Serial.println();
      iniciarGiro90Grados();
    }
  }
  
  delay(100);
}

void actualizarAngulo() {
  mpu.getRotation(&gx, &gy, &gz);
  
  unsigned long tiempoActual = millis();
  float deltaT = (tiempoActual - tiempoPrevio) / 1000.0;
  
  // Evitar deltaT muy grandes
  if (deltaT > 0.1) deltaT = 0.01;
  
  tiempoPrevio = tiempoActual;
  
  // ⭐ CORRECCIÓN: Usar el factor de escala correcto para el MPU6050
  float velocidadZ = (gz / 131.0) - anguloOffset;
  
  // Solo integrar si hay movimiento significativo
  if (abs(velocidadZ) > 0.3) {
    anguloZ += velocidadZ * deltaT;
  }
}

void calibrarGiroscopio() {
  Serial.println("🔧 Calibrando MPU6050... NO MOVER!");
  delay(2000);
  
  long suma = 0;
  int lecturas = 500;
  
  for(int i = 0; i < lecturas; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    suma += gz;
    delay(5);
  }
  
  // Calcular offset en las mismas unidades que usaremos
  anguloOffset = (float)suma / lecturas / 131.0;
  anguloZ = 0;
  tiempoPrevio = millis();
  
  Serial.print("📊 Offset calculado: ");
  Serial.println(anguloOffset, 6);
  Serial.println("✅ Calibración completada");
}

float medirDistancia() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duracion = pulseIn(echo, HIGH, 30000);
  if (duracion == 0) return 999;
  return duracion * 0.034 / 2;
}

void iniciarGiro90Grados() {
  girando = true;
  
  // Guardar ángulo inicial
  float anguloInicial = anguloZ;
  
  // Definir objetivo: -90 grados desde la posición actual
  anguloDeseado = anguloInicial - 90.0;
  
  // Resetear variables PID y control híbrido
  integral = 0;
  errorAnterior = 0;
  contadorEstable = 0;
  
  Serial.println("🔄 INICIANDO SECUENCIA DE GIRO");
  Serial.println("===============================");
  Serial.print("🎯 Angulo inicial: ");
  Serial.print(anguloInicial, 1);
  Serial.print("° | Angulo objetivo: ");
  Serial.print(anguloDeseado, 1);
  Serial.println("°");
  Serial.println();
  
  while (girando) {
    actualizarAngulo();
    
    error = anguloDeseado - anguloZ;
    
    // 📊 MOSTRAR INFORMACIÓN MEJORADA
    Serial.print("Actual: ");
    Serial.print(anguloZ, 1);
    Serial.print("° | Objetivo: ");
    Serial.print(anguloDeseado, 1);
    Serial.print("° | Error: ");
    Serial.print(error, 1);
    Serial.print("° | Giro: ");
    
    // Calcular PID
    integral += error;
    // Limitar integral para evitar windup
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    
    float derivativo = error - errorAnterior;
    float salidaPID = (Kp * error) + (Ki * integral) + (Kd * derivativo);
    errorAnterior = error;
    
    // Controlar LEDs y mostrar indicador visual (umbrales más sensibles)
    if (salidaPID < -0.5) {
      // Necesita girar MÁS a la DERECHA
      if(error>-10 && error<0){
        digitalWrite(ledamarillo, HIGH);\
        digitalWrite(ledRojo, HIGH);
        digitalWrite(ledVerde, LOW);
        Serial.println("🔴 DERECHA");
      }
      else{
        digitalWrite(ledamarillo, LOW);
        digitalWrite(ledRojo, HIGH);
        digitalWrite(ledVerde, LOW);
        Serial.println("🔴 DERECHA");
      }
    } 
    else if (salidaPID > 0.5) {
      // Se PASÓ, necesita corregir a la IZQUIERDA
      if(error<10 && error>0){
        digitalWrite(ledamarillo, HIGH);
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledVerde, HIGH);
        Serial.println("🟢 IZQUIERDA");
      }
      else{
        digitalWrite(ledamarillo, LOW);
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledVerde, HIGH);
        Serial.println("🟢 IZQUIERDA");
      }
    }
    else {
      // Muy cerca del objetivo
      digitalWrite(ledamarillo, LOW);
      digitalWrite(ledRojo, LOW);
      digitalWrite(ledVerde, LOW);
      Serial.println("⚪ OK");
    }
    
    // ⭐⭐ ESTRATEGIA HÍBRIDA MEJORADA - Control de posición + velocidad + histéresis
    float velocidadAngular = gz / 131.0;
    bool enPosicion = (abs(error) <= UMBRAL_ERROR);
    bool velocidadBaja = (abs(velocidadAngular) < UMBRAL_VELOCIDAD);
    
    if (enPosicion && velocidadBaja) {
      contadorEstable++;
      Serial.print(" [Estable: ");
      Serial.print(contadorEstable);
      Serial.print("/");
      Serial.print(CICLOS_ESTABILIDAD);
      Serial.print("]");
      
      if (contadorEstable >= CICLOS_ESTABILIDAD) {
        Serial.println();
        Serial.println("✅ GIRO COMPLETADO Y ESTABILIZADO!");
        Serial.print("📊 Giro realizado: ");
        Serial.print(anguloZ - anguloInicial, 1);
        Serial.println("°");
        
        girando = false;
        
        // ⭐ RESETEAR ÁNGULO A CERO después del giro
        anguloZ = 0;
        break;
      }
    } else {
      // Resetear contador si se mueve o sale de posición
      contadorEstable = 0;
    }
    
    Serial.println(); // Nueva línea para cada ciclo
    delay(50); // Control loop rate
  }
  
  // Asegurar que LEDs estén apagados al final
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledamarillo, LOW);
  
  Serial.println();
  Serial.println("🔄 Volviendo a modo navegación...");
  Serial.println();
  delay(1000);
}
