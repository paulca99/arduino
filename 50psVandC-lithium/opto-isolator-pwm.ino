//Initializing PWM Pin
int led_pin = 7;
void setup() {
  //Declaring PWM pin output
  pinMode(led_pin, OUTPUT);
}

void loop() {

  for(int i=0; i<255; i++){
    analogWrite(led_pin, i);
    delay(5);
  }
  for(int i=255; i>0; i--){
    analogWrite(led_pin, i);
    delay(5);
  }
}