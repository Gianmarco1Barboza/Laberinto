#include <Wire.h>
#include <MPU6050.h>

// Pines
const int ledRojo = 6;
const int ledVerde = 5;
const int ledAmarillo = 7;
const int trigFrontal = 23;
const int echoFrontal = 22;
const int trigIzquierdo = 24;
const int echoIzquierdo = 25;
const int trigDerecho = 27;
const int echoDerecho = 26;

// Variables MPU
MPU6050 mpu;
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

// Variables de control
int contadorEstable = 0;
const int CICLOS_ESTABILIDAD = 5;
const float UMBRAL_VELOCIDAD = 3.0;
const float UMBRAL_ERROR = 3.0;

void setup() {
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(trigFrontal, OUTPUT);
  pinMode(echoFrontal, INPUT);
  pinMode(trigIzquierdo, OUTPUT);
  pinMode(echoIzquierdo, INPUT);
  pinMode(trigDerecho, OUTPUT);
  pinMode(echoDerecho, INPUT);
  
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

  // Solo procesar sensores si NO estamos girando
  if (!girando) {
    // Medir distancias de los sensores
    float distanciaFrontal = medirDistancia(trigFrontal, echoFrontal);
    float distanciaIzquierdo = medirDistancia(trigIzquierdo, echoIzquierdo);
    float distanciaDerecho = medirDistancia(trigDerecho, echoDerecho);

    // Mostrar las distancias y el ángulo en el monitor serial
    Serial.print("📏 F:");
    Serial.print(distanciaFrontal);
    Serial.print("cm | I:");
    Serial.print(distanciaIzquierdo);
    Serial.print("cm | D:");
    Serial.print(distanciaDerecho);
    Serial.print("cm | 🎯 Ángulo:");
    Serial.print(anguloZ, 1);
    Serial.println("°");

    // Evaluar las condiciones de los sensores
    if (distanciaFrontal < 15.0 && distanciaIzquierdo < 15.0 && distanciaDerecho < 15.0) {
      // Camino cerrado: los 3 sensores detectan obstáculos
      Serial.println("🚨 ¡CAMINO CERRADO! Retrocediendo...");
      caminoCerrado();
    }
    else if (distanciaFrontal < 15.0 && distanciaIzquierdo > 15.0 && distanciaDerecho > 15.0) {
      // Solo frontal detecta obstáculo - decidir según espacio disponible
      if (distanciaIzquierdo > distanciaDerecho) {
        Serial.println("🔄 Girando a la IZQUIERDA (más espacio)");
        iniciarGiro(90);
      } else {
        Serial.println("🔄 Girando a la DERECHA (más espacio)");
        iniciarGiro(-90);
      }
    }
    else if (distanciaFrontal < 15.0 && distanciaIzquierdo < 15.0) {
      // Frontal y izquierdo detectan pared, girar a la derecha
      Serial.println("🚨 Pared Frontal e Izquierda - Girando a la DERECHA...");
      iniciarGiro(-90);
    }
    else if (distanciaFrontal < 15.0 && distanciaDerecho < 15.0) {
      // Frontal y derecho detectan pared, girar a la izquierda
      Serial.println("🚨 Pared Frontal y Derecha - Girando a la IZQUIERDA...");
      iniciarGiro(90);
    }
    else if (distanciaIzquierdo < 15.0 && distanciaDerecho < 15.0) {
      // 📏 AVANZAR EN LÍNEA RECTA - Paredes a los lados
      Serial.println("📍 Avanzando en línea recta");
      digitalWrite(ledAmarillo, HIGH);
      delay(100);
      digitalWrite(ledAmarillo, LOW);
    }
    else {
      // Cuando no hay obstáculos cercanos
      Serial.println("🟢 Camino libre, avanzando...");
      digitalWrite(ledAmarillo, HIGH);
      delay(500);
      digitalWrite(ledAmarillo, LOW);
    }
  }
  
  delay(100);
}

void caminoCerrado() {
  // Parpadeo rápido de los 3 LEDs
  for(int i = 0; i < 10; i++) {
    digitalWrite(ledRojo, HIGH);
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledAmarillo, HIGH);
    delay(100);
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledAmarillo, LOW);
    delay(100);
    
    // Re-evaluar sensores laterales durante el retroceso
    float distanciaIzquierdo = medirDistancia(trigIzquierdo, echoIzquierdo);
    float distanciaDerecho = medirDistancia(trigDerecho, echoDerecho);
    
    if (distanciaIzquierdo >= 15.0 || distanciaDerecho >= 15.0) {
      Serial.println("✅ Camino lateral disponible - Terminando retroceso");
      break;
    }
  }
  
  // Decidir giro después del retroceso
  float distanciaIzquierdo = medirDistancia(trigIzquierdo, echoIzquierdo);
  float distanciaDerecho = medirDistancia(trigDerecho, echoDerecho);
  
  if (distanciaIzquierdo >= 15.0 && distanciaDerecho >= 15.0) {
    // Ambos lados disponibles - prioridad derecha
    Serial.println("🔄 Ambos lados disponibles - Girando DERECHA");
    iniciarGiro(-90);
  } else if (distanciaIzquierdo >= 15.0) {
    Serial.println("🔄 Girando IZQUIERDA");
    iniciarGiro(90);
  } else if (distanciaDerecho >= 15.0) {
    Serial.println("🔄 Girando DERECHA");
    iniciarGiro(-90);
  }
}

void iniciarGiro(float anguloRelativo) {
  girando = true;
  float anguloInicial = anguloZ;
  anguloDeseado = anguloInicial + anguloRelativo;
  
  // Ajustar ángulo deseado para que esté en el rango [-180, 180]
  if (anguloDeseado > 180) anguloDeseado -= 360;
  if (anguloDeseado < -180) anguloDeseado += 360;
  
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
    
    // Ajustar error para el camino más corto
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // 📊 MOSTRAR INFORMACIÓN DEL PID
    Serial.print("Actual: ");
    Serial.print(anguloZ, 1);
    Serial.print("° | Objetivo: ");
    Serial.print(anguloDeseado, 1);
    Serial.print("° | Error: ");
    Serial.print(error, 1);
    Serial.print("° | Giro: ");
    
    // Calcular PID
    integral += error;
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    
    float derivativo = error - errorAnterior;
    float salidaPID = (Kp * error) + (Ki * integral) + (Kd * derivativo);
    errorAnterior = error;
    
    // Controlar LEDs según el error y dirección
    if (salidaPID < -0.5) {
      // Necesita girar MÁS a la DERECHA
      if(error > -10 && error < 0){
        digitalWrite(ledAmarillo, HIGH);
        digitalWrite(ledRojo, HIGH);
        digitalWrite(ledVerde, LOW);
        Serial.println("🟡 DERECHA (ajuste fino)");
      } else {
        digitalWrite(ledAmarillo, LOW);
        digitalWrite(ledRojo, HIGH);
        digitalWrite(ledVerde, LOW);
        Serial.println("🔴 DERECHA");
      }
    } 
    else if (salidaPID > 0.5) {
      // Se PASÓ, necesita corregir a la IZQUIERDA
      if(error < 10 && error > 0){
        digitalWrite(ledAmarillo, HIGH);
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledVerde, HIGH);
        Serial.println("🟡 IZQUIERDA (ajuste fino)");
      } else {
        digitalWrite(ledAmarillo, LOW);
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledVerde, HIGH);
        Serial.println("🟢 IZQUIERDA");
      }
    }
    else {
      // Muy cerca del objetivo
      digitalWrite(ledAmarillo, LOW);
      digitalWrite(ledRojo, LOW);
      digitalWrite(ledVerde, LOW);
      Serial.println("⚪ OK");
    }
    
    // ⭐⭐ ESTRATEGIA HÍBRIDA - Control de posición + velocidad
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
        anguloZ = 0; // ⭐ REINICIAR ÁNGULO A CERO
        break;
      }
    } else {
      contadorEstable = 0;
    }
    
    Serial.println(); // Nueva línea para cada ciclo
    delay(50); // Control loop rate
  }
  
  // Asegurar que LEDs estén apagados al final
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarillo, LOW);
  
  Serial.println();
  Serial.println("🔄 Volviendo a modo navegación...");
  Serial.println();
  delay(1000);
}

void actualizarAngulo() {
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long tiempoActual = millis();
  float deltaT = (tiempoActual - tiempoPrevio) / 1000.0;
  
  if (deltaT > 0.1) deltaT = 0.01;
  tiempoPrevio = tiempoActual;

  float velocidadZ = (gz / 131.0) - anguloOffset;

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
  
  anguloOffset = (float)suma / lecturas / 131.0;
  anguloZ = 0;
  tiempoPrevio = millis();
  
  Serial.print("📊 Offset calculado: ");
  Serial.println(anguloOffset, 6);
  Serial.println("✅ Calibración completada");
}

float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracion = pulseIn(echoPin, HIGH, 30000);
  if (duracion == 0) return 999;
  return duracion * 0.034 / 2;
}
