#include <PS2X_lib.h>  //for v1.6
#include <VL53L0X.h> //Tof Sensor
#include <SoftwareSerial.h> //GPS
#include<Servo.h> //Motores
#include "I2Cdev.h" // Tof e IMU
#include "MPU6050.h" //IMU
#include "Wire.h" // Tof e IMU
#include "DFRobot_BNO055.h"

///////////////////// MPU ////////////////////////

//Definir el MPU
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Valor de los offsets
int ax_o,ay_o,az_o;
int gx_o,gy_o,gz_o;

/////////////////////////// TOF ////////////////////////
VL53L0X tofsensor;

////////////////// MOTORS ///////////////

//Definir lo pines de los 4 Motores
#define ESC_PIN1 3
#define ESC_PIN2 5
#define ESC_PIN3 6
#define ESC_PIN4 9

//Definir los 4 Motores Brushless
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int pwmL2, pwmL1, pwmR3, pwmR4;

//////////// PID /////////////
float ang_deseado_y = 0;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

long tiempo_prev, time;
float elapsedTime;
float dt;
float dif_tiempo;

float error_y, previous_error_y;
float PID_y;
float pid_p_y, pid_i_y, pid_d_y;
float kp_y = 1;
float ki_y = 0;
float kd_y = 1;

////////////////////// CONTROL PS2 /////////////////////////////////

#define PS2_DAT        12  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        8 //17

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;

/////////// BNO ////////
typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********

BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:    Serial.println("everything ok"); break;
  case BNO::eStatusErr:   Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}


void setup(){
  Serial.begin(9600);
  Wire.begin();           //Iniciando I2C  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it

  /////////////////////////////// MPU /////////////////////////////////
  sensor.initialize();
  if (sensor.testConnection()) Serial.println("Sensor IMU iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  // Leer los offset los offsets anteriores
  ax_o=sensor.getXAccelOffset();
  ay_o=sensor.getYAccelOffset();
  az_o=sensor.getZAccelOffset();
  gx_o=sensor.getXGyroOffset();
  gy_o=sensor.getYGyroOffset();
  gz_o=sensor.getZGyroOffset();
  
  Serial.println("Offsets:");
  Serial.print(ax_o); Serial.print("\t"); 
  Serial.print(ay_o); Serial.print("\t"); 
  Serial.print(az_o); Serial.print("\t"); 
  Serial.print(gx_o); Serial.print("\t"); 
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.print("\t");
  
  ///////////////////////////// TOF //////////////////////////////////

  if (tofsensor.init()) {
    Serial.println("Sensor TOF iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor TOF");
  }

  ///////////////////////// CONTROL PS2 ///////////////////////////
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);


  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
	  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

  /////////////////////////// MOTORES ///////////////////////////
  //Calibrando los ESC-Motores
  esc1.attach(ESC_PIN1,  1000, 2000);
  esc1.write(0);
  delay(500);
  
  esc2.attach(ESC_PIN2,  1000, 2000);
  esc2.write(0);
  delay(500);

  esc3.attach(ESC_PIN3,  1000, 2000);
  esc3.write(0);
  delay(500);

  esc4.attach(ESC_PIN4,  1000, 2000);
  esc4.write(0);
  delay(500); 

  Serial.println("Motores Calibrados \n");

  /////////////////// BNO ////////////////////
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
  
  delay(500);
  Serial.println("Programa Listo");
}

void loop() {

      /////  INICIO

      ////////////////////////////////// BNO ///////////////////////

      BNO::sEulAnalog_t   sEul;
      sEul = bno.getEul();
      Serial.print("pitch:");
      Serial.print(sEul.pitch, 3);
      Serial.print(" ");
      Serial.print("roll:");
      Serial.print(sEul.roll, 3);
      Serial.print(" ");
      Serial.print("yaw:");
      Serial.print(sEul.head, 3);
      Serial.println(" ");
      
      //////////////////////////// TOF ////////////////////
      
      /*int distance = tofsensor.readRangeSingleMillimeters();
      distance = distance /10;
      Serial.print("TOF Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      Serial.println("\n");*/

      ///////////////////////// PS2 ///////////////////////

      if(error == 1) //skip loop if no controller found
        return; 

      else { //DualShock Controller
        ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed 

        //if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
          if(ps2x.Button(PSB_L3))
            Serial.println("L3 pressed");       
        }    

        int joystickvalue = ps2x.Analog(PSS_LY);
        joystickvalue = constrain(joystickvalue, 0,127); 
        Serial.print("Stick LY value: ");
        Serial.println(joystickvalue);
        int mmotorSpeed = map(joystickvalue, 127, 0, 0, 50);
        
        //mmotorSpeed = 10;

        if (mmotorSpeed>100){
          mmotorSpeed = 100;
          }       

        Serial.print("\n");
        Serial.print("Motor Speed value: ");
        Serial.println(mmotorSpeed);   

        esc1.write(mmotorSpeed); esc2.write(mmotorSpeed); esc3.write(mmotorSpeed); esc4.write(mmotorSpeed); 


      delay(10);      
      }
  
}
 