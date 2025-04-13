#define enA 9
#define in1 12
#define in2 13
#define left_enc_A 2
#define left_enc_B 4

unsigned int left_encoder_counter = 0; //número de pulsos
String left_encoder_sign = "p";  //direcció moviment    
double left_wheel_vel = 0.0;   //rad/s

const double dt = 0.1; // tiempo entre lecturas en segundos
const int PPR = 824;


void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(left_enc_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_enc_A),leftEncoderCallback,RISING);
  
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);

  Serial.begin(115200);

}

void loop() {
  
  double left_wheel_rev_s = left_encoder_counter / (PPR * dt);  //  1rpm = 0.10472 rad/s
  String encoder_read = left_encoder_sign + String(left_wheel_rev_s, 3);

  
  Serial.println(encoder_read);
  analogWrite(enA, 210);

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
