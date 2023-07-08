#include <PS2X_lib.h>        //for v1.6
#include <VL53L0X.h>         //Tof Sensor
#include <SoftwareSerial.h>  //GPS
#include <Servo.h>           //Motores
#include "I2Cdev.h"          // Tof e IMU
#include "Wire.h"            // Tof e IMU
#include "DFRobot_BNO055.h"

/////////////////////////// TOF ////////////////////////
VL53L0X tofsensor;

////////////////// MOTORS ///////////////

//Definir lo pines de los 4 Motores
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define ESC_PIN1 4
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
float kp_roll = 0.2, ki_roll = 0.001, kd_roll = 0.2;
float kp_pitch = 0.2, ki_pitch = 0.001, kd_pitch = 0.5;
float kp_yaw = 1, ki_yaw = 0.01, kd_yaw = 0;
double err_pitch_sum = 0, err_roll_sum = 0;

float p_roll, i_roll, d_roll;
float p_pitch, i_pitch, d_pitch;
float p_yaw, i_yaw, d_yaw;

float pid_roll, pid_pitch, pid_yaw;

/////////////////////////// BATERIA ///////////////

int battery_voltaje;

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

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
  ControlConectado();

  /////////////////////////// MOTORES ///////////////////////////
  //Calibrando los ESC-Motores
  esc1.attach(ESC_PIN1);
  esc2.attach(ESC_PIN2);
  esc3.attach(ESC_PIN3);
  esc4.attach(ESC_PIN4);
  delay(500);

  esc1.writeMicroseconds(MAX_SIGNAL);
  esc2.writeMicroseconds(MAX_SIGNAL);
  esc3.writeMicroseconds(MAX_SIGNAL);
  esc4.writeMicroseconds(MAX_SIGNAL);
  delay(500);

  esc1.writeMicroseconds(MIN_SIGNAL);
  esc2.writeMicroseconds(MIN_SIGNAL);
  esc3.writeMicroseconds(MIN_SIGNAL);
  esc4.writeMicroseconds(MIN_SIGNAL);
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

  int distance = tofsensor.readRangeSingleMillimeters();
  distance = distance / 10;
  /*Serial.print("TOF Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.println("\n");*/

  ///////////////////////// PS2 ///////////////////////

  if (error == 1)  //skip loop if no controller found
    return;

  else {                                //DualShock Controller
    ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed
    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
    }

    int JS1_Y_throttle = ps2x.Analog(PSS_LY);
    int JS2_Y_pitch = ps2x.Analog(PSS_RY);
    int JS2_X_roll = ps2x.Analog(PSS_RX);
    int JS1_X_yaw = ps2x.Analog(PSS_LX);

    JS1_Y_throttle = constrain(JS1_Y_throttle, 0, 127);
    int throttle = map(JS1_Y_throttle, 127, 0, 0, 15);
    int ang_pitch_des = map(JS2_Y_pitch, 0, 254, -30, 30);
    int ang_roll_des = map(JS2_X_roll, 0, 254, -30, 30);

    //Borrar luego!!!!!
    ang_roll_des = -1.5;
    ang_pitch_des = 0;

    if (throttle > 100) {
      throttle = 100;
    }

    ///////////////////// PID //////////////////////////////////////
    err_pitch = ang_pitch - ang_pitch_des;
    err_roll = ang_roll - ang_roll_des;
    err_yaw = ang_yaw - ang_yaw_des;

    dt = 1;
    const float SampleTime = dt / 1000.0;

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

    if (abs(ang_pitch) <= 2.5) {
      pid_pitch = 0;
    }

    /*if (abs(ang_roll) <= ) {
      pid_roll = 0;
    }*/

    if (abs(err_roll)<= 2.5){
      pid_roll = 0;
    }
    // Combinación de motores
    pwmR1 = throttle + pid_roll - pid_pitch;
    pwmR2 = throttle + pid_roll + pid_pitch;
    pwmL2 = throttle - pid_roll + pid_pitch;
    pwmL1 = throttle - pid_roll - pid_pitch;

    pwmL1 = limit_PWM(pwmL1);
    pwmL2 = limit_PWM(pwmL2);
    pwmR1 = limit_PWM(pwmR1);
    pwmR2 = limit_PWM(pwmR2);

    if (throttle == 0) {
      pwmL1 = 0;
      pwmR1 = 0;
      pwmL2 = 0;
      pwmR2 = 0;
    }

    int var = 12;

    if (throttle != 0) {
      if (pwmL1 < var) {
        pwmL1 = var;
      }

      if (pwmL2 < var) {
        pwmL2 = var;
      }

      if (pwmR1 < var) {
        pwmR1 = var;
      }

      if (pwmR2 < var) {
        pwmR2 = var;
      }
    }

    int Valor_pwmR1 = (pwmR1 * 10 + 1000);
    int Valor_pwmR2 = (pwmR2 * 10 + 1000);
    int Valor_pwmL1 = (pwmL1 * 10 + 1000);
    int Valor_pwmL2 = (pwmL2 * 10 + 1000);

    esc1.writeMicroseconds(Valor_pwmR1);
    esc2.writeMicroseconds(Valor_pwmR2);
    esc3.writeMicroseconds(Valor_pwmL2);
    esc4.writeMicroseconds(Valor_pwmL1);

    //////////////////////// IMPRIMIR DATOS ///////////////////////
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

    //delay(10);
  }
}

float limit_PWM(float output) {
  if (output > 120) output = 120;
  else if (output < 0) output = 0;
  return output;
}


void ControlConectado() {
  Serial.println("Esperando Mando de Control...");
  int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  while (error == 1) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("No conectado: ");
    Serial.println(error);
    if (error == 0) {
      break;
    }
  }

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
      return error;
  }
}