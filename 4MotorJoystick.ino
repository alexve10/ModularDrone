#include<Servo.h>

//Definir lo pines de los 4 Motores
#define ESC_PIN 3
#define ESC_PIN2 5
#define ESC_PIN3 6
#define ESC_PIN4 9

//Definir los 4 Motores Brushless
Servo esc;
Servo esc2;
Servo esc3;
Servo esc4;

void setup() 
{
  Serial.begin(9600);
  esc.attach(ESC_PIN,  1000, 2000);
  esc.write(0);
  delay(2000);
  
  esc2.attach(ESC_PIN2,  1000, 2000);
  esc2.write(0);
  delay(2000);

  esc3.attach(ESC_PIN3,  1000, 2000);
  esc3.write(0);
  delay(2000);

  esc4.attach(ESC_PIN4,  1000, 2000);
  esc4.write(0);
  delay(2000);

}

void loop() 
{
  int joystickValue = analogRead(A0);
  joystickValue = constrain(joystickValue, 550, 1023);  //Read upper half of joystick value from center.
  int mmotorSpeed = map(joystickValue, 550, 1023, 0, 180);
  esc.write(mmotorSpeed);  esc2.write(mmotorSpeed);  esc3.write(mmotorSpeed);  esc4.write(mmotorSpeed); 
  Serial.print("Velocidad Motor: ");Serial.print(mmotorSpeed);Serial.print("\n");
  Serial.print("Valor Joystick: ");Serial.print(joystickValue);Serial.print("\n");
 
}
