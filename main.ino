/*
 * Reference : http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
 */

#include <Servo.h>

#define FS_GR3E_PWM_CH1_SERVO D7
#define FS_GR3E_PWM_CH2_DC_MOTOR D6
#ifdef FS_GR3E_PWM_CH2_DC_MOTOR
  #define FS_GR3E_PWM_CH2_ESC FS_GR3E_PWM_CH2_DC_MOTOR
#elif
  #define FS_GR3E_PWM_CH2_ESC D6
#endif
#define FS_GR3E_PWM_CH3_EX D5

#define SERVO_SIG D4
#define DC_MOTOR_PWM_SIG D3
#ifdef DC_MOTOR_PWM_SIG
  #define ESC_PWM_SIG DC_MOTOR_PWM_SIG
#elif
  #define ESC_PWM_SIG D3
#endif

#define DEFAULT_ANGLE_CALIBRATION (100)

void setup()
{
  Serial.begin(115200);

  pinMode(FS_GR3E_PWM_CH1_SERVO, INPUT);
  pinMode(FS_GR3E_PWM_CH2_DC_MOTOR, INPUT);
  pinMode(FS_GR3E_PWM_CH3_EX, INPUT);
  
  servo.attach(SERVO_SIG);
  pinMode(DC_MOTOR_PWM_SIG, OUTPUT);
  
  servo.writeMicroseconds(1500 + DEFAULT_ANGLE_CALIBRATION); //default Angle
}

void loop()
{

  
  
  
}
