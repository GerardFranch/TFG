#define enA 9
#define in1 12
#define in2 13

float cmd = 0.0;

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW );

  Serial.begin(115200);

}

void loop() {

  if(Serial.available())
  {
    cmd = Serial.readString().toFloat();
    Serial.println(cmd);
  }  
  analogWrite(enA, cmd);
}
