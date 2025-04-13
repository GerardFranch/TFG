#include <PID_v1.h>

// Definición de pines para el driver L298N
#define enA 11          // Pin de habilitación para el motor izquierdo
#define in1 13         // Pin de control 1 para el motor izquierdo
#define in2 12         // Pin de control 2 para el motor izquierdo
#define in3 7          // Pin de control 1 para el motor derecho
#define in4 8          // Pin de control 2 para el motor derecho
#define enB 9         // Pin de habilitación para el motor derecho

// Definición de pines para los encoders
#define left_enc_A 2   // Canal A del encoder izquierdo (interrupción)
#define left_enc_B 4   // Canal B del encoder izquierdo
#define right_enc_A 3  // Canal A del encoder derecho (interrupción)
#define right_enc_B 5  // Canal B del encoder derecho

// Variables para medición de velocidad y estado de los encoders
unsigned int left_encoder_counter = 0;  // Contador de pulsos del encoder izquierdo
unsigned int right_encoder_counter = 0; // Contador de pulsos del encoder derecho
String left_encoder_sign = "p";         // Dirección del movimiento izquierdo (p=positivo, n=negativo)
String right_encoder_sign = "p";        // Dirección del movimiento derecho (p=positivo, n=negativo)
double left_wheel_vel = 0.0;            // Velocidad rueda izquierda en rad/s
double right_wheel_vel = 0.0;           // Velocidad rueda derecha en rad/s

// Variables para el procesamiento de comandos seriales
bool is_left_wheel_cmd = false;         // Indica si el comando actual es para la rueda izquierda
bool is_right_wheel_cmd = false;        // Indica si el comando actual es para la rueda derecha
char value[] = "00.00";                 // Buffer para almacenar el valor de velocidad recibido
uint8_t value_idx = 0;                  // Índice para el buffer de valores
bool is_cmd_complete = false;           // Indica si el comando está completo
bool is_right_wheel_forward = true;     // Dirección actual de la rueda derecha
bool is_left_wheel_forward = true;      // Dirección actual de la rueda izquierda
double right_wheel_cmd_vel = 0.0;       // Velocidad objetivo para la rueda derecha
double left_wheel_cmd_vel = 0.0;        // Velocidad objetivo para la rueda izquierda

// Variables para control de tiempo
unsigned long last_millis = 0;          // Tiempo de la última actualización
const unsigned long interval = 100;     // Intervalo de actualización (100ms)

// Variables para salida del controlador PID
double right_wheel_cmd = 0.0;           // Señal PWM para el motor derecho (0-255)
double left_wheel_cmd = 0.0;            // Señal PWM para el motor izquierdo (0-255)

// Parámetros del controlador PID para el motor derecho
double Kp_r = 11.5;                     // Ganancia proporcional
double Ki_r = 7.5;                      // Ganancia integral
double Kd_r = 0.1;                      // Ganancia derivativa

// Parámetros del controlador PID para el motor izquierdo
double Kp_l = 12.8;                     // Ganancia proporcional
double Ki_l = 8.3;                      // Ganancia integral
double Kd_l = 0.1;                      // Ganancia derivativa

// Inicialización de los controladores PID
PID rightMotor(&right_wheel_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Configuración de pines de salida para los motores
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Configuración de pines de entrada para los encoders
  pinMode(left_enc_B, INPUT);
  pinMode(right_enc_B, INPUT);

  // Configuración de interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(left_enc_A), leftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(right_enc_A), rightEncoderCallback, RISING);

  // Inicialización de la dirección de los motores (ambos hacia adelante)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Activación de los controladores PID
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  // Inicialización de la comunicación serial
  Serial.begin(115200);
}

void loop() {
  // Procesar comandos recibidos por el puerto serial
  if(Serial.available())
  {
    char chr = Serial.read();
    // CORRECCIÓN: Uso de comillas simples para caracteres
    if(chr == 'r')  // Si el comando es para la rueda derecha
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == 'l')  // Si el comando es para la rueda izquierda
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p')  // Dirección positiva (adelante)
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // Invertir estado actual de los pines para cambiar dirección
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        is_right_wheel_forward = true;
      }
      else if (is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n')  // Dirección negativa (atrás)
    {
      if (is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        is_right_wheel_forward = false;
      }
      else if (is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        is_left_wheel_forward = false;
      }
    }
    else if(chr == ',')  // Fin del comando, procesar el valor recibido
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);  // Convertir string a float
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reiniciar el buffer de valor
      value_idx = 0;
      // CORRECCIÓN: Uso de comillas simples para caracteres
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else  // Almacenar dígitos del valor de velocidad
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Actualización periódica de velocidades y control PID
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    // Cálculo de velocidades reales a partir de pulsos del encoder
    // Factor de conversión: (pulsos/100ms) * 10 * (60.0/824.0) * 0.10472
    // 824 pulsos = 1 revolución, 0.10472 rad/s = 1 rpm
    left_wheel_vel = 10 * left_encoder_counter * (60.0/824.0) * 0.10472;
    right_wheel_vel = 10 * right_encoder_counter * (60.0/824.0) * 0.10472;

    // Actualización de los controladores PID
    rightMotor.Compute();
    leftMotor.Compute();

    // Si la velocidad objetivo es cero, apagar el motor
    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    // Aplicar señales PWM a los motores
    analogWrite(enA, left_wheel_cmd);   // Motor izquierdo usa enA
    analogWrite(enB, right_wheel_cmd);  // Motor derecho usa enB
    
    // Enviar datos de encoder por serial
    String encoder_read = "r" + right_encoder_sign + String(right_wheel_vel) + ",l" + left_encoder_sign + String(left_wheel_vel) + ",";
    Serial.println(encoder_read);
    
    // Reiniciar contadores y actualizar tiempo
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
  }
}

// Función de interrupción para el encoder izquierdo
void leftEncoderCallback() {
  left_encoder_counter++;  // Incrementar contador de pulsos

  // Determinar dirección basada en el estado del canal B
  if(digitalRead(left_enc_B) == HIGH) {
    left_encoder_sign = "p";  // Movimiento positivo
  }
  else {
    left_encoder_sign = "n";  // Movimiento negativo
  }
}

// Función de interrupción para el encoder derecho
void rightEncoderCallback() {
  right_encoder_counter++;  // Incrementar contador de pulsos

  // Determinar dirección basada en el estado del canal B
  // Nota: La lógica está invertida respecto al encoder izquierdo
  if(digitalRead(right_enc_B) == HIGH) {
    right_encoder_sign = "n";  // Movimiento negativo
  }
  else {
    right_encoder_sign = "p";  // Movimiento positivo
  }
}
