#define enA 9
#define in1 12
#define in2 13
#define left_enc_A 2
#define left_enc_B 4

unsigned int left_encoder_counter = 0; //número de pulsos
String left_encoder_sign = "p";  //direcció moviment    
double left_wheel_vel = 0.0;   //rad/s

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(left_enc_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_enc_A),leftEncoderCallback,RISING);
  
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW );

  Serial.begin(115200);

}

void loop() {
  left_wheel_vel = 10*left_encoder_counter + (60.0/858.0)*0.10472; //11senayals per volta x 78 ratio reduccio = 858 ---- 1rpm = 0.10472 rad/s
  String encoder_read = left_encoder_sign + String(left_wheel_vel);
  Serial.println(encoder_read);
  analogWrite(enA, 100);

  left_encoder_counter = 0;
  delay(100);
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
