#include <PID_v1.h>

#define enA 9
#define in1 12
#define in2 13
#define in3 7
#define in4 8
#define enB 11

#define left_enc_A 2
#define left_enc_B 4
#define right_enc_A 3
#define right_enc_B 5

unsigned int left_encoder_counter = 0; //número de pulsos
unsigned int right_encoder_counter = 0; //número de pulsos
String left_encoder_sign = "p";  //direcció moviment  
String right_encoder_sign = "p";  //direcció moviment    
double left_wheel_vel = 0.0;   //rad/s
double right_wheel_vel = 0.0;   //rad/s
bool is_left_wheel_cmd = false;
bool is_right_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;

double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;

PID rightMotor(&right_wheel_vel,&right_wheel_cmd,&right_wheel_cmd_vel);
PID leftMotor(&left_wheel_vel,&left_wheel_cmd,&left_wheel_cmd_vel);

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(left_enc_B, INPUT);
  pinMode(right_enc_B, INPUT);


  attachInterrupt(digitalPinToInterrupt(left_enc_A),leftEncoderCallback,RISING);
  attachInterrupt(digitalPinToInterrupt(right_enc_A),rightEncoderCallback,RISING);

  
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  Serial.begin(115200);

}

void loop() {
  if(Serial.available())
  {
    char chr = Serial.read();
    if(chr == "r")
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == "l")
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == "p")
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(in1, HIGH - digitalRead(in1));
        digitalWrite(in2, HIGH - digitalRead(in2));
        is_right_wheel_forward = true;

      }
      else if (is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(in3, HIGH - digitalRead(in3));
        digitalWrite(in4, HIGH - digitalRead(in4));
        is_left_wheel_forward = true;
      }
    }
    else if(chr == "n")
    {
      if (is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(in1, HIGH - digitalRead(in1));
        digitalWrite(in2, HIGH - digitalRead(in2));
        is_right_wheel_forward = false;

      }
      else if (is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(in3, HIGH - digitalRead(in3));
        digitalWrite(in4, HIGH - digitalRead(in4));
        is_left_wheel_forward = false;
      }
    }
    else if(chr ==",")
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0]="0";
      value[1]="0";
      value[2]=".";
      value[3]="0";
      value[4]="0";
      value[5]="\0";

    }
    else
    {
      if(value_idx <5)
      {
        value[value_idx]=chr;
        value_idx++;
      }
    }
  }

  unsigned long current_millis = millis();
  if(current_millis-last_millis >= interval)
  {
    left_wheel_vel = 10*left_encoder_counter + (60.0/824.0)*0.10472; //11senayals per volta x 78 ratio reduccio = 858 (en realitat 824)---- 1rpm = 0.10472 rad/s
    right_wheel_vel = 10*right_encoder_counter + (60.0/824.0)*0.10472;
    String encoder_read = "r"+ right_encoder_sign + String(right_wheel_vel) + ",l"+ left_encoder_sign + String(left_wheel_vel)+",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
  }
  
  analogWrite(enA, 100);

}


void leftEncoderCallback() {

  left_encoder_counter++;

  if(digitalRead(left_enc_B)==HIGH){ //Si se detecta un flanco de subida en A y B ya esta HIGH...
    
    left_encoder_sign="p"; //Movimiento positivo
  }
  else{
    
    left_encoder_sign="n"; //Movimiento negativo
  }
}

void rightEncoderCallback() {

  right_encoder_counter++;

  if(digitalRead(right_enc_B)==HIGH){ //Si se detecta un flanco de subida en A y B ya esta HIGH...
    
    right_encoder_sign="n"; //Movimiento positivo
  }
  else{
    
    right_encoder_sign="p"; //Movimiento negativo
  }
}
