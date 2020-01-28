#include <Arduino.h>
#include <Wire.h>


#define BAUD_RATE 115200

/* ### PIN CONNECTIONS ### */

// Optical Encoder
static const uint8_t  OPTICAL_ENCODER_A0_pin =  A7;
static const uint8_t  OPTICAL_ENCODER_D0_pin =  2;

// Rotary Encoder
static const uint8_t  ROTARY_ENCODER_CLK_pin =  3;
static const uint8_t  ROTARY_ENCODER_DW_pin  =  4;
static const uint8_t  ROTARY_ENCODER_SW_pin  =  7; 

// Motor Bridge
static const uint8_t  ENABLE_MOTOR_PWD_pin     =  11;
static const uint8_t  MOTOR_CLOCKWISE_pin      =  9;
static const uint8_t  MOTOR_ANTICLOCKWISE_pin  =  10; 

// Ultrasound sensor
static const byte SENSOR_ADDRESS  = 0x70;
static const byte RANGE_COMMAND   = 0x51;
static const uint8_t SENSOR_READING_PERIOD = 100;    // Time between readings to allow for acoustic dissipation
static const uint8_t COMMAND_TO_READING_TIME = 80;   // Time between range command and range retrieval



/* ### CONSTANTS ### */

static const uint8_t  MOTOR_START_PWM_SPEED = 50;

// PID settle time, waits this long to start taking ultrasound measurements
static const float  MOTOR_PID_SETTLE_TIME = 5000;

// Motor desired speed in beats/min 
// static const float  MOTOR_BEATS_MIN = 65;

// Motor desired speed in degrees/sec
// static const float  MOTOR_SET_ANGULAR_SPEED = MOTOR_BEATS_MIN*(360/(60*1));
static const float  MOTOR_SET_ANGULAR_SPEED = 313;

// Period to measure speed in ms
static const uint32_t  SPEED_MEASUREMENT_PERIOD =  250;

// PID parameters
static const float  K_p = 0.02;   // Proportional
static const float  K_i = 0.00001;  // Integral
static const float  K_d = 0.009;  // Differential



/* ### GLOBAL VARIABLES ### */

volatile uint32_t change_counter_optical_encoder  = 0;
volatile bool state_optical_encoder      = false;
volatile bool old_state_optical_encoder  = false;

volatile uint32_t change_counter_rotary_encoder   = 0;
volatile bool state_rotary_encoder_DW    = false;
volatile bool state_rotary_encoder_CLK   = false;
volatile bool old_state_rotary_encoder   = false;



/* ### FUNCTION HEADERS ### */

void set_motor_rotation_speed(int motor_new_PWM_speed);
void ISR_optical_encoder();
void ISR_rotary_encoder();
int PID_motor_speed(float current_angular_speed);
uint16_t ultrasound_distance_reading();
void ultrasound_range_command();
void establish_serial_connection();
void send_serial_data(struct serial_data *serial_data);


/* ### DATA STRUCTURE ### */

struct serial_data { 
    uint16_t distance;
    uint16_t angular_position;
    float current_angular_speed;
    int motor_pwm_value;
}; 
  
typedef struct serial_data serial_data; 


void setup() {

  // Turn off interrupts while setting up
  noInterrupts();

  // # Pin Configuration #
  // Arduino on-board led
  pinMode(LED_BUILTIN, OUTPUT);
  // Optical Encoder
  pinMode(OPTICAL_ENCODER_D0_pin, INPUT);
  // Rotary Encoder
  pinMode(ROTARY_ENCODER_CLK_pin, INPUT);
  pinMode(ROTARY_ENCODER_DW_pin, INPUT);
  pinMode(ROTARY_ENCODER_SW_pin, INPUT_PULLUP);
  // Motor Bridge
  pinMode(ENABLE_MOTOR_PWD_pin, OUTPUT);
  pinMode(MOTOR_CLOCKWISE_pin, OUTPUT);
  pinMode(MOTOR_ANTICLOCKWISE_pin, OUTPUT);


  // Start serial communication
  Serial.begin(BAUD_RATE);
  Serial.flush();
  // establish_serial_connection();

  // Start I2C communication
  Wire.begin();
  

  // Set clockwise rotation mode
  digitalWrite(MOTOR_CLOCKWISE_pin, LOW);
  digitalWrite(MOTOR_ANTICLOCKWISE_pin, HIGH);


  // Setup interrupts for optical and rotary encoder
  attachInterrupt(digitalPinToInterrupt(2), ISR_optical_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), ISR_rotary_encoder, CHANGE);


  // Intialize encoders state variables
  state_rotary_encoder_DW = (bool) digitalRead(ROTARY_ENCODER_DW_pin);
  state_rotary_encoder_CLK = (bool) digitalRead(ROTARY_ENCODER_CLK_pin);

  state_optical_encoder = (bool) digitalRead(OPTICAL_ENCODER_D0_pin);


  // Re-enable interrupts after setup
  interrupts();

}

void loop() {

  // ### Local variables to loop ###
  static uint32_t old_change_counter_rotary_encoder   = 0;
  static uint32_t old_time_millis = millis();
  static uint32_t old_time_millis_ultrasound = millis();
  static float angular_speed_rotary = 0;
  static bool read_write_ultrasound = true;     // true for writing ultrasound sensor, false for reading

  struct serial_data serial_data;

  // speed variables (in units of changes/ms)


  // Calculate speed every SPEED_MEASUREMENT_PERIOD
  if ( (millis()-old_time_millis) > SPEED_MEASUREMENT_PERIOD ) {
    
    noInterrupts();
    // speed variables in units of changes/ms
    angular_speed_rotary = (float)(change_counter_rotary_encoder - old_change_counter_rotary_encoder)/((float)(SPEED_MEASUREMENT_PERIOD));
    old_change_counter_rotary_encoder = change_counter_rotary_encoder;
    interrupts();

    serial_data.current_angular_speed = angular_speed_rotary;
    serial_data.motor_pwm_value = PID_motor_speed(angular_speed_rotary*1000*9);

    old_time_millis = millis();
  }

  if ( millis() > MOTOR_PID_SETTLE_TIME ) {

    if ( read_write_ultrasound == true ) {  // Write ultrasound

      if ( (millis()-old_time_millis_ultrasound) > (SENSOR_READING_PERIOD) ) {

        serial_data.distance = 0;
        ultrasound_range_command();
        read_write_ultrasound = false;    // set to read ultrasound
        old_time_millis_ultrasound = millis();

      }

    } else if ( read_write_ultrasound == false ) {  // Read ultrasound
      if ( (millis()-old_time_millis_ultrasound) > COMMAND_TO_READING_TIME ){

        serial_data.distance = ultrasound_distance_reading();
        read_write_ultrasound = true;     // set to write ultrasound
        old_time_millis_ultrasound = millis();
        serial_data.angular_position = (change_counter_rotary_encoder%((uint32_t)40))*((uint32_t)9);
        send_serial_data(&serial_data);

      }

    }

  }

}

void ultrasound_range_command() {

  Wire.beginTransmission(SENSOR_ADDRESS); //Start addressing
  Wire.write(RANGE_COMMAND); //send range command
  Wire.endTransmission(); //Stop and do something else now

}

uint16_t ultrasound_distance_reading() {

  static uint16_t range = 0;

  Wire.requestFrom(SENSOR_ADDRESS, 2);

  if (Wire.available() > 1) { //Sensor responded
    // Sensor sends back two bytes, the first sent is the high byte and the second sent is the low byte
    range = ((Wire.read() << 8) | Wire.read());
    return range;
  }
  else {
    return 0; //Else nothing was received, return 0
  }

}

int PID_motor_speed(float current_angular_speed){
  // Does PID control over the motor speed and change PWM output value accordingly
  // Input: motor current angular speed in degrees/sec


  // ### Local variables to PID_motor_speed ###

  // Error signal
  static float error           = 0;
  static float old_error       = 0;
  // Integral and previous integral values
  static float integral        = 0;
  static float old_integral    = 0;
  // Differential 
  static float differential    = 0;
  // PID output signal  
  static float control_signal  = 0;
  // Time
  static float time_interval   = 0;
  static float old_time_millis = millis();

  // Motor speed PWM
  static int motor_pwm_value = 0;


  time_interval = (millis()-old_time_millis)/1000;
  error = current_angular_speed - MOTOR_SET_ANGULAR_SPEED;  // Proportional
  integral = time_interval*error + old_integral;            // Integral
  differential = (error - old_error)/time_interval;         // Differential

  control_signal = K_p*error + K_i*integral + K_d*differential;

  motor_pwm_value = round(motor_pwm_value + control_signal);
  analogWrite(ENABLE_MOTOR_PWD_pin, abs(motor_pwm_value));

  old_error = error;
  old_integral = integral;
  old_time_millis = millis();

  return abs(motor_pwm_value);

}

void set_motor_rotation_speed(int motor_new_PWM_speed) {

  static uint8_t motor_current_PWM_speed = motor_new_PWM_speed;

  noInterrupts(); // Deactivate counting of encoders while the speed of the motor reaches the assigned value
  
  if (motor_new_PWM_speed > motor_current_PWM_speed) {
    
    while (motor_new_PWM_speed > motor_current_PWM_speed)
    {
      analogWrite(ENABLE_MOTOR_PWD_pin, motor_current_PWM_speed);
      motor_current_PWM_speed++;
      delay(100);
    }

  } else if (motor_new_PWM_speed < motor_current_PWM_speed) {
    
    while (motor_new_PWM_speed < motor_current_PWM_speed)
    {
      analogWrite(ENABLE_MOTOR_PWD_pin, motor_current_PWM_speed);
      motor_current_PWM_speed--;
      delay(100);
    }

  } else if (motor_new_PWM_speed == motor_current_PWM_speed) {
    analogWrite(ENABLE_MOTOR_PWD_pin, motor_current_PWM_speed);
    delay(100);
  }
  
  interrupts();

}

void ISR_optical_encoder() {

  state_optical_encoder = (bool) digitalRead(OPTICAL_ENCODER_D0_pin);

  if ( state_optical_encoder != old_state_optical_encoder ){
    change_counter_optical_encoder++;
    old_state_optical_encoder = state_optical_encoder;
  }

}

void ISR_rotary_encoder() {
  state_rotary_encoder_DW = (bool) digitalRead(ROTARY_ENCODER_DW_pin);

  if ( state_rotary_encoder_DW != old_state_rotary_encoder ){
    change_counter_rotary_encoder++;
    old_state_rotary_encoder = state_rotary_encoder_DW;
  }

}

void establish_serial_connection() {

  while (Serial.available() <= 0)
  {
    Serial.write('A'); // send a capital A
    delay(300);
  }

}

void send_serial_data(struct serial_data *serial_data){

  byte* byteData1 = (byte*) (&(serial_data->distance));
  byte* byteData2 = (byte*) (&(serial_data->angular_position));
  byte* byteData3 = (byte*) (&(serial_data->current_angular_speed));
  byte* byteData4 = (byte*) (&(serial_data->motor_pwm_value));
  byte buf[10] = {byteData1[0], byteData1[1],
                 byteData2[0], byteData2[1],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1]};
  Serial.write(buf, 10);

}