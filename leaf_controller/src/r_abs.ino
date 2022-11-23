const int stepPin = 5; 
const int dirPin = 2; 
const int enPin = 8;

void setup() {
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  
}

void loop() {
  //Brake
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  for(int x = 0; x < 15000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(50); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(50); 
  }
   delay(1000); // One second delay
   
   // Acceleration
//  digitalWrite(dirPin,LOW); //Changes the direction of rotation
//  for(int x = 0; x < 15000; x++) {
//     digitalWrite(stepPin,HIGH);
//     delayMicroseconds(50);
//     digitalWrite(stepPin,LOW);
//     delayMicroseconds(500);
//   }
//  delay(1000); 
}
