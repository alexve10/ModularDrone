#include <PS2X_lib.h>        //for v1.6
#include <VL53L0X.h>         //Tof Sensor
#include <SoftwareSerial.h>  //GPS
#include <Servo.h>           //Motores
#include "I2Cdev.h"          // Tof e IMU
#include "Wire.h"            // Tof e IMU
#include "DFRobot_BNO055.h"

///// MATLAB //////////
// Create a union to easily convert float to byte
typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t send1;   // Throttle
FLOATUNION_t send2;   // Ang_Pitch
FLOATUNION_t send3;   // Ang_Roll
FLOATUNION_t send4;   // Ang_Yaw
FLOATUNION_t send5;   // Ang_Pitch_Des
FLOATUNION_t send6;   // Ang_Roll_Des
FLOATUNION_t send7;   // Err_Pitch
FLOATUNION_t send8;   // Err_Roll
FLOATUNION_t send9;   // P_pitch
FLOATUNION_t send10;  // I_pitch
FLOATUNION_t send11;  // D_pitch
FLOATUNION_t send12;  // PID_pitch
FLOATUNION_t send13;  // P_roll
FLOATUNION_t send14;  // I_roll
FLOATUNION_t send15;  // D_roll
FLOATUNION_t send16;  // PID_roll
FLOATUNION_t send17;  // PWM_R1
FLOATUNION_t send18;  // PWM_R2
FLOATUNION_t send19;  // PWM_L1
FLOATUNION_t send20;  // PWM_L2

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

int pwmL2, pwmL1, pwmR1, pwmR2;

////////////////////// CONTROL PS2 /////////////////////////////////

#define PS2_DAT 12  //14
#define PS2_CMD 11  //15
#define PS2_SEL 10  //16
#define PS2_CLK 8   //17

//#define pressures   true
#define pressures false
//#define rumble      true
#define rumble false

PS2X ps2x;  // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;

/////////// BNO ////////
typedef DFRobot_BNO055_IIC BNO;  // ******** use abbreviations instead of full names ********

BNO bno(&Wire, 0x28);  // input TwoWire interface and IIC address

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus) {
  switch (eStatus) {
    case BNO::eStatusOK: Serial.println("everything ok"); break;
    case BNO::eStatusErr: Serial.println("unknow error"); break;
    case BNO::eStatusErrDeviceNotDetect: Serial.println("device not detected"); break;
    case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
    case BNO::eStatusErrDeviceStatus: Serial.println("device internal status error"); break;
    default: Serial.println("unknow status"); break;
  }
}

////////////////////// PID ////////////////////////
float ang_roll_des, ang_pitch_des, ang_yaw_des;
float ang_roll_prev, ang_pitch_prev, ang_yaw_prev;

float err_roll, err_pitch, err_yaw;
float err_roll_prev, err_pitch_prev, err_yaw_prev;

float tiempo_prev, time, dt;

float kp_roll = 0.5, ki_roll = 0.1, kd_roll = 0.5;
float kp_pitch = 0.5, ki_pitch = 0.1, kd_pitch = 0.5;
float kp_yaw, ki_yaw, kd_yaw;
double err_pitch_sum = 0, err_roll_sum = 0;

float p_roll, i_roll, d_roll;
float p_pitch, i_pitch, d_pitch;
float p_yaw, i_yaw, d_yaw;

float pid_roll, pid_pitch, pid_yaw;


void setup() {
  Serial.begin(115200);
  Wire.begin();  //Iniciando I2C
  delay(300);    //added delay to give wireless ps2 module some time to startup, before configuring it

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
  switch (type) {
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
  esc1.attach(ESC_PIN1, 1000, 2000);
  esc1.write(0);
  delay(500);

  esc2.attach(ESC_PIN2, 1000, 2000);
  esc2.write(0);
  delay(500);

  esc3.attach(ESC_PIN3, 1000, 2000);
  esc3.write(0);
  delay(500);

  esc4.attach(ESC_PIN4, 1000, 2000);
  esc4.write(0);
  delay(500);

  Serial.println("Motores Calibrados \n");

  /////////////////// BNO ////////////////////
  bno.reset();
  while (bno.begin() != BNO::eStatusOK) {
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

  BNO::sEulAnalog_t sEul;
  sEul = bno.getEul();
  float ang_pitch = sEul.pitch;
  float ang_roll = sEul.roll;
  float ang_yaw = sEul.head;

  //////////////////////////// TOF ////////////////////

  /*int distance = tofsensor.readRangeSingleMillimeters();
      distance = distance /10;
      Serial.print("TOF Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      Serial.println("\n");*/

  ///////////////////////// PS2 ///////////////////////

  if (error == 1)  //skip loop if no controller found
    return;

  else {                                //DualShock Controller
    ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed

    if (ps2x.Button(PSB_PAD_UP)) {  //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");
    }

    if (ps2x.ButtonPressed(PSB_CIRCLE))  //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if (ps2x.NewButtonState(PSB_CROSS))  //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if (ps2x.ButtonReleased(PSB_SQUARE))  //will be TRUE if button was JUST released
      Serial.println("Square just released");

    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {  //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC);  //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }

    int JS1_Y_throttle = ps2x.Analog(PSS_LY);
    int JS2_Y_pitch = ps2x.Analog(PSS_RY);
    int JS2_X_roll = ps2x.Analog(PSS_RX);
    int JS1_X_yaw = ps2x.Analog(PSS_LX);

    JS1_Y_throttle = constrain(JS1_Y_throttle, 0, 127);
    int throttle = map(JS1_Y_throttle, 127, 0, 0, 25);
    int ang_pitch_des = map(JS2_Y_pitch, 0, 254, -30, 30);
    int ang_roll_des = map(JS2_X_roll, 0, 254, -30, 30);

    //Borrar luego!!!!!
    ang_roll_des = 0;
    ang_pitch_des = 0;

    if (throttle > 100) {
      throttle = 100;
    }

    //////////// PID //////////////////////////////////////
    err_pitch = ang_pitch - ang_pitch_des;
    err_roll = ang_roll - ang_roll_des;

    dt = 1;
    const float SampleTime = dt / 100.0;

    // Pitch
    p_pitch = err_pitch * kp_pitch;
    err_pitch_sum += ((err_pitch + err_pitch_prev) * SampleTime) / 2.0;
    i_pitch = err_pitch_sum * ki_pitch;
    i_pitch = constrain(i_pitch, -120, 120);
    d_pitch = kd_pitch * ((err_pitch - err_pitch_prev) / dt);


    /*if (abs(err_pitch>2)){
          i_pitch = 0;
        }*/

    pid_pitch = p_pitch + i_pitch + d_pitch;

    err_pitch_prev = err_pitch;

    // Roll
    p_roll = err_roll * kp_roll;
    err_roll_sum += ((err_roll + err_roll_prev) * SampleTime) / 2.0;

    i_roll = err_roll_sum * ki_roll;
    i_roll = constrain(i_roll, -120, 120);
    d_roll = kd_roll * ((err_roll - err_roll_prev) / dt);

    /*if (abs(err_roll) >2){
          i_roll = 0;
        }*/

    pid_roll = p_roll + i_roll + d_roll;

    err_roll_prev = err_roll;
  
    pwmR1 = throttle + pid_roll - pid_pitch;
    pwmR2 = throttle + pid_roll + pid_pitch;
    pwmL2 = throttle - pid_roll + pid_pitch;
    pwmL1 = throttle - pid_roll - pid_pitch;

    if (pwmL1 < 0) { pwmL1 = 0; }
    if (pwmL2 < 0) { pwmL2 = 0; }
    if (pwmR1 < 0) { pwmR1 = 0; }
    if (pwmR2 < 0) { pwmR2 = 0; }

    if (pwmL1 > 120) { pwmL1 = 120; }
    if (pwmL2 > 120) { pwmL2 = 120; }
    if (pwmR1 > 120) { pwmR1 = 120; }
    if (pwmR2 > 120) { pwmR2 = 120; }


    esc1.write(pwmR1);     esc2.write(pwmR2);     esc3.write(pwmL2);     esc4.write(pwmL1);

    //Serial.println("------------------------------------------------");
    Serial.print("M: ");
    Serial.print(throttle);
    Serial.print("  P: ");
    Serial.print(ang_pitch, 3);
    Serial.print("  R: ");
    Serial.print(ang_roll, 3);
    Serial.print("  Y: ");
    Serial.print(ang_yaw, 3);
    
    Serial.print(" ||| P: ");
    Serial.print(p_pitch, 3); 
    Serial.print("  ");
    Serial.print(i_pitch, 3); 
    Serial.print("  ");
    Serial.print(d_pitch, 3); 
    Serial.print("  ");
    Serial.print(pid_pitch, 3); 
    
    Serial.print(" ||| R: ");
    Serial.print(p_roll, 3); 
    Serial.print("  ");
    Serial.print(i_roll, 3); 
    Serial.print("  ");
    Serial.print(d_roll, 3); 
    Serial.print("  ");
    Serial.print(pid_roll, 3); 
    Serial.print(" ||| PWM: ");
    Serial.print(pwmR1); 
    Serial.print("  ");
    Serial.print(pwmL1); 
    Serial.print("  ");
    Serial.print(pwmR2); 
    Serial.print("  ");
    Serial.println(pwmL2); 
    
    //////////////// Enviar Datos MATLAB ////////////////////

    /*send1.number = throttle;
        send2.number = ang_pitch;
        send3.number = ang_roll;
        send4.number = ang_yaw;
        send5.number = ang_pitch_des;
        send6.number = ang_roll_des;
        send7.number = err_pitch;
        send8.number = err_roll;
        send9.number = p_pitch;
        send10.number = i_pitch;
        send11.number = d_pitch;
        send12.number = pid_pitch;
        send13.number = p_roll;
        send14.number = i_roll;
        send15.number = d_roll;
        send16.number = pid_roll;
        send17.number = pwmR1;
        send18.number = pwmR2;
        send19.number = pwmL1;
        send20.number = pwmL2;

        Serial.write('A'); 
  
        // Print float data
        for (int i=0; i<4; i++){
          Serial.write(send1.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send2.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send3.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send4.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send5.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send6.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send7.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send8.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send9.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send10.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send11.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send12.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send13.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send14.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send15.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send16.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send17.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send18.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send19.bytes[i]); 
        }
        for (int i=0; i<4; i++){
          Serial.write(send20.bytes[i]); 
        }
        // Print terminator
        Serial.print('\n');*/

    delay(10);
  }
}

