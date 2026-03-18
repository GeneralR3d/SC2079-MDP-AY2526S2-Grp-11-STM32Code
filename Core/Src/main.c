#include "main.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// UART IT receive state
static volatile uint8_t uart_rx_byte = 0;
static volatile uint8_t uart_rx_ready = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) {
    uart_rx_ready = 1;
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_rx_byte, 1);
  }
}

// VARIABLES FOR TASK 2
#define TASK2_PWM 3000
#define TASK2_SPRINT_PWM 5000
#define SNAP_WAIT_MS 8000 // 8 seconds to retry once

// TASK 2 - OBS1
const float TASK2_obs_1_clearance_distance = 42.5f;
const float TASK2_obs_1_start_side_distance = 45.0f;

// TASK 2 - OBS2
const float TASK2_distance_from_back_of_first_obs = 30.0f;
const float TASK2_obs_2_clearance_distance = 25.0f;

// TASK 2 - RETURN TO START
const float TASK2_distance_from_back_of_second_obs = 10.0f;
// Safe threshold for vertical distance travelled before arcing
const float TASK2_vertical_dist_return_arc_buffer = 50.0f;
// Threshold for skipping reverse motion and makes final turn immediately
const float TASK2_obstacle2_too_short_threshold = 40.0f;
// Threshold for checking if carpark side is clear
const float TASK2_carpark_side_IR_distance_threshold = 40.0f;
// Final brake ultrasonic distance threshold
const float TASK2_carpark_wall_clearance_distance = 17.5f;

typedef enum {
  TASK2_SURFACE_HPL,
  TASK2_SURFACE_OUTSIDE,
} TASK2_Surface_t;
TASK2_Surface_t TASK2_current_surface = TASK2_SURFACE_HPL;

volatile float TASK2_vertical_dist_now = 0;
volatile float TASK2_horizontal_dist_now = 0;

//------------------------------------------
// Global variables
//------------------------------------------

// average distance using the calibrated scales
// if actual > measured, then COUNTS_PER_CM_L and COUNTS_PER_CM_R should be
// smaller if actual < measured, then COUNTS_PER_CM_L and COUNTS_PER_CM_R should
// be larger

// const float COUNTS_PER_CM_L = 74.0467f;
// const float COUNTS_PER_CM_R = 70.6355f;
// const float COUNTS_PER_CM_L = 75.5f;
// const float COUNTS_PER_CM_R = 75.5f;

const float COUNTS_PER_CM_L = 73.7f;
const float COUNTS_PER_CM_R = 77.5f;

const float COUNTS_PER_CM_L_REVERSE = 74.73f;
const float COUNTS_PER_CM_R_REVERSE = 74.89f;

// Global servo center position (can be tweaked)
volatile int32_t SERVO_CENTER_US = 1477;
// 1475 is left
// 1480 is right

// HPL
// float GYRO_LEFT_BIAS = 131.33f;
// float GYRO_RIGHT_BIAS = 130.959f;

float GYRO_LEFT_BIAS = 131.33f;
float GYRO_RIGHT_BIAS = 130.959f;

// Ackermann differential steering constants (measure your car!)
#define WHEELBASE_CM 14.5f   // distance from front axle to rear axle
#define TRACK_WIDTH_CM 16.2f // distance between the two rear wheels

float yaw_angle = 0; // global or static variable
uint32_t last_time = 0;
float gyro_gz_filtered = 0.0f; // Global to allow reset between turns

#define CMD_BUF_LEN 64
#define PI 3.141592653589f
char cmd_buf[CMD_BUF_LEN];
int cmd_index = 0;

// IR global variables
volatile uint16_t raw6, raw7;
volatile uint32_t mv6, mv7;
volatile float dist6, dist7;
volatile float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
// --- ICM20948 address / WHO_AM_I ---
#define WHO_AM_I 0x00
#define WHO_AM_I_VAL 0xEA
#define REG_BANK_SEL 0x7F

#define ICM_ADDR_68 (0x68 << 1)         // AD0 = 0
#define ICM_ADDR_69 (0x69 << 1)         // AD0 = 1
static uint16_t ICM_ADDR = ICM_ADDR_69; // will be auto-detected
char buf[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);

uint32_t counter = 0; // Timer 2 counter
int16_t count = 0;    // Convert counter to signed value
int16_t no_of_tick =
    50;            // number of tick used in SysTick to calculate speed, in msec
int16_t speed = 0; // speed in term of number of edges detected per Systick
int16_t rpm = 0;   // speed in rpm number of count/sec * 60 sec  divide by 260
                   // count per round
int start = 0;     // use to start stop the motor
int32_t pwmVal = 0;     // pwm value to control motor speed
int32_t pwmVal_raw = 0; // pwm value before clamping - for debugging
const int16_t pwmMax =
    (7200 - 200); // Maximum PWM value = 7200 keep the maximum value to 7000
const int16_t pwmMin = 500; // offset value to compensate for deadzone
int err;                    // status for checking return

int encoder_A = 0; // encoders reading of Drive A (from complement of TIM2->CNT)
int encoder_D = 0; // encoders reading of Drive D (from TIM5->CNT)

int16_t position = 0;  // position of the motor (1 rotation = 260 count)
extern int16_t oldpos; // // see SysTick_Handler in stm32f4xx_it.c
int16_t angle =
    0; // angle of rotation, in degree resolution = 360 degree/260 tick
int16_t target_angle = 0; // target angle of rotation,
int16_t position_target;  // target position
int16_t direction;        // motor direction 0 or 1
int16_t error;            // error between target and actual
int32_t error_area =
    0; // area under error - to calculate I for PI implementation
int32_t error_old, error_change, error_rate; // to calculate D for PID control
int32_t millisOld, millisNow, dt; // to calculate I and D for PID control

static inline int32_t left_ticks_forward(void);
static inline int32_t right_ticks_forward(void);
static inline int32_t left_ticks_reverse(void);
static inline int32_t right_ticks_reverse(void);

void Motor_stop(void);
void Drive_Forward_ToCM(float target_cm, int base_pwm); // function prototype
void Drive_Reverse_ToCM(float target_cm, int base_pwm);
void task_two_clear_first_obs_alternate(int pwm, float first_obs_dist,
                                        char direction);
void Drive_Forward_Until_Obstacle(int pwm, float obstacle_clearance_distance);
void Turn_Car(float target_deg, int pwmVal, int steer_angle, float target_cm);
void Turn_Car_Reverse(float target_deg, int pwmVal, int steer_angle,
                      float target_cm);
uint16_t Servo_SetAngle_Safe(int16_t angle_deg, uint8_t gradual);
void task_two();
void task_two_return_to_start(char direction_obs1, char previousDirection);
void UART_arm();
char UART_receive();

/*Purpose: Directly sets the PWM pulse width in microseconds

Safety: Clamps values between 500-2500μs to prevent servo damage
Hardware: Uses TIM12, Channel 2 for PWM generation
Range: Standard servo range (500μs = 0°, 1500μs = 90°, 2500μs = 180°)
LOW LEVEL FUNCTION DO NOT CALL DIRECTLY, call Steering_ToUS() instead*/

static inline void _Servo_WriteUS(uint16_t us) {
  if (us < 961)
    us = 961;
  if (us > 2380)
    us = 2380;
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}

/**
 * @brief Map steering direction (deg) to servo PWM microsecond value
 *
 * User input range:
 *   -45 = Full Left  (950 µs)
 *     0 = Straight   (SERVO_CENTER_US µs)
 *   +45 = Full Right (2380 µs)
 *
 * @param steer_angle Steering angle in degrees [-45 to +45]
 * No delay. Delay should be handled by the caller
 * @return uint16_t Pulse widinth in microseconds
 */
uint16_t Steering_ToUS(int16_t steer_angle) {
  if (steer_angle < -45)
    steer_angle = -45;
  if (steer_angle > 45)
    steer_angle = 45;

  // Linear interpolation using the dynamic center
  // but we want exact 0° = SERVO_CENTER_US, so adjust baseline

  int32_t us;
  if (steer_angle >= 0) {
    // Positive range: 0 to +45 deg -> SERVO_CENTER_US to 2380
    // Slope = (2380 - SERVO_CENTER_US) / 45
    us = SERVO_CENTER_US + (int32_t)steer_angle * (2380 - SERVO_CENTER_US) / 45;
  } else {
    // Negative range: 0 to -45 deg -> SERVO_CENTER_US to 950
    // Slope = (SERVO_CENTER_US - 961) / 45
    // Note: steer_angle is negative, so we add (negative * positive_slope)
    // which subtracts
    us = SERVO_CENTER_US + (int32_t)steer_angle * (SERVO_CENTER_US - 961) / 45;
  }

  _Servo_WriteUS((uint16_t)us);
  return (uint16_t)us;
}

void send_message_over(const char *input) {
  char msg[64];
  snprintf(msg, sizeof(msg), input);
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  counter = __HAL_TIM_GET_COUNTER(htim);
  count = (int16_t)counter;
  position = count / 2; // x2 encoding
  angle = count / 2;    // x2 encoding
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // see EXTI0_IRQHandler() in stm32f4xx_it.c for interrupt
  if (GPIO_Pin == USER_PB_Pin) {
    // toggle LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); // LED - A12
    if (start == 0) {
      start = 1;
      // reset all value to Zero
      TIM2->CNT = 0; // Timer Counter Value
      speed = 0;
      position = 0; // see SysTick_Handler in stm32f4xx_it.c
      oldpos = 0;   // see SysTick_Handler in stm32f4xx_it.c
      angle = 0;
      pwmVal = 0;
    } else
      start = 0;
  }
}

void MotorDrive_enable(void) {
  // Enable PWM through TIM4-CH1/CH4 to drive the DC motor - Rev D board
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // on Motor drive A interface
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // on Motor drive A interface
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // on Motor drive D interface
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // on Motor drive D interface
}

void Motor_stop(void) {
  // Set both IN1 and IN2 pins = '1'
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
}

// simple motor forward code - no pid control
//  Accepts separate PWM for left and right motors (Ackermann support)
void Motor_forward_simple(int pwmValL, int pwmValR) {
  // Motor A (left): PWM on CH3, CH4 = 0
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmValL);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);

  // Motor D (right): PWM on CH4, CH3 = 0 (inverted because wired opposite)
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmValR);

  sprintf(buf, "PWM=%4d/%4d ", pwmValL, pwmValR);
  OLED_ShowString(0, 10, buf);
}

void Motor_reverse_simple(int pwmValL, int pwmValR) {
  // Motor A: PWM on CH3, CH4 = 0
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmValL);

  // Motor D: PWM on CH4, CH3 = 0 (inverted because wired opposite)
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmValR);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

  sprintf(buf, "PWM= %4d/%4d ", pwmValL, pwmValR);
  OLED_ShowString(0, 10, buf);
}

// THIS IS THE MOTOR FORWARD THAT IMPLEMENTS THE PID CONTROL
static uint8_t motor_forward_needs_reset = 0;

void Motor_forward_reset_heading(void) { motor_forward_needs_reset = 1; }

void Motor_forward(int pwmVal) {
  // --- Static variables persist across calls ---
  static uint32_t last_time = 0;
  static float heading = 0.0f;
  static float target_heading = 0.0f;
  static uint8_t initialized = 0;

  static float integral = 0.0f;
  static float last_error = 0.0f;
  static float gz_filtered = 0.0f;

  // --- Timing ---
  uint32_t now = HAL_GetTick();
  float dt = (now - last_time) / 1000.0f; // ms -> seconds
  if (dt <= 0)
    dt = 0.001f; // protect against div by 0
  last_time = now;

  if (!initialized || motor_forward_needs_reset) {
    target_heading = heading; // lock current heading as new target
    integral = 0.0f;
    last_error = 0.0f;
    last_time = HAL_GetTick(); // prevent dt spike after a turn
    initialized = 1;
    motor_forward_needs_reset = 0;
  }

  // --- Filter gyro Z-axis (yaw rate) ---
  float alpha =
      0.6f; // weight of NEW value: lower = more smoothing, higher = faster
  gz_filtered = alpha * gz_dps + (1.0f - alpha) * gz_filtered;

  // --- Update heading from filtered gyro ---
  heading += gz_filtered * dt; // integrate deg/s over time

  // --- Compute heading error ---
  float heading_error = target_heading - heading;
  integral += heading_error * dt;                       // I term
  float derivative = (heading_error - last_error) / dt; // D term
  last_error = heading_error;

  // --- PID controller gains ---
  float Kp_h = 30.0f; // proportional gain
  float Ki_h = 4.0f;  // integral gain
  float Kd_h = 3.5f;  // derivative gain

  // --- Control law (PID) ---
  int correction =
      (int)(Kp_h * heading_error + Ki_h * integral + Kd_h * derivative);

  // --- Clamp correction ---
  if (correction > 2000)
    correction = 2000;
  if (correction < -2000)
    correction = -2000;

  // --- Motor offset compensation (baseline bias) ---

  int left_offset = -324;

  if (TASK2_current_surface == TASK2_SURFACE_OUTSIDE) {
    left_offset = -350;
  }

  // int left_offset = -324;
  //  int right_offset = -300;
  int right_offset = 0;

  /* -350, 0 -> significant left drift
   * -325, 0 -> very slight right drift
   * -337, 0 -> slight left
   * -332, 0 -> a lot left
   * -327
   * -325
   * -320 too much right
   */

  // --- Apply correction + offsets ---
  int left_pwm = pwmVal + left_offset + correction;
  int right_pwm = pwmVal + right_offset - correction;

  // Clamp to valid PWM range
  // if (left_pwm > pwmMax)
  //   left_pwm = pwmMax;
  // if (left_pwm < pwmMin)
  //   left_pwm = pwmMin;
  // if (right_pwm > pwmMax)
  //   right_pwm = pwmMax;
  // if (right_pwm < pwmMin)
  //   right_pwm = pwmMin;

  // --- Send to motors ---
  // Motor A (left)
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, left_pwm);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);

  // Motor D (right)
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, right_pwm);

  // --- Debug (optional) ---
  // sprintf(buf, "Err=%.2f Corr=%d L=%d R=%d\r\n",
  //         heading_error, correction, left_pwm, right_pwm);
  // HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void Motor_reverse(int pwmVal) {
  // --- Static variables persist across calls ---
  static uint32_t last_time = 0;
  static float heading = 0.0f;
  static float target_heading = 0.0f;
  static uint8_t initialized = 0;

  static float integral = 0.0f;
  static float last_error = 0.0f;
  static float gz_filtered = 0.0f;

  // --- Timing ---
  uint32_t now = HAL_GetTick();
  float dt = (now - last_time) / 1000.0f; // ms -> seconds
  if (dt <= 0)
    dt = 0.001f; // protect against div by 0
  last_time = now;

  if (!initialized) {
    target_heading = heading; // lock current heading
    initialized = 1;
  }

  // --- Filter gyro Z-axis (yaw rate) ---
  float alpha =
      0.1f; // weight of NEW value: lower = more smoothing, higher = faster
  gz_filtered = alpha * gz_dps + (1.0f - alpha) * gz_filtered;

  // --- Update heading from filtered gyro ---
  heading += gz_filtered * dt; // integrate deg/s over time

  // --- Compute heading error ---
  float heading_error = target_heading - heading;
  integral += heading_error * dt;                       // I term
  float derivative = (heading_error - last_error) / dt; // D term
  last_error = heading_error;

  // --- PID controller gains ---
  float Kp_h = 30.0f; // proportional gain
  float Ki_h = 4.0f;  // integral gain
  float Kd_h = 3.5f;  // derivative gain

  // --- Control law (PID) ---
  int correction =
      (int)(Kp_h * heading_error + Ki_h * integral + Kd_h * derivative);

  // --- Clamp correction ---
  if (correction > 2000)
    correction = 2000;
  if (correction < -2000)
    correction = -2000;

  // --- Motor offset compensation (baseline bias) ---
  int left_offset = 0;
  int right_offset = -75;

  // --- Apply correction + offsets ---
  int left_pwm = pwmVal + left_offset - correction;
  int right_pwm = pwmVal + right_offset + correction;

  // Clamp to valid PWM range
  if (left_pwm > pwmMax)
    left_pwm = pwmMax;
  if (left_pwm < pwmMin)
    left_pwm = pwmMin;
  if (right_pwm > pwmMax)
    right_pwm = pwmMax;
  if (right_pwm < pwmMin)
    right_pwm = pwmMin;

  // --- Send to motors ---
  // Motor A (left) reverse
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, left_pwm);

  // Motor D (right) reverse
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, right_pwm);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

  // --- Debug (optional) ---
  // sprintf(buf, "PWM = %4dR ", pwmVal);
  // OLED_ShowString(0, 20, buf);
}

void serial_uart() {
  // send various values to serial port @ usart 3 for display
  angle = (int)(position * 360 /
                265); // Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 =
                      // 260 pulses per revolution
                      //  measured value = 265 pulses per revolution
  sprintf(buf, "%5d", angle);
  OLED_ShowString(60, 10, buf);
  // also send to serial port
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';                                      // comma separator
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", target_angle);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", pwmVal);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port
  OLED_ShowString(40, 20, buf);

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_area);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_change);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_rate);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%4d ", speed); // RPM speed of the DC motor
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port
  OLED_ShowString(40, 30, buf);
  OLED_Refresh_Gram();

  buf[0] = '\r';
  buf[1] = '\n'; // move to next line on serial port
  HAL_UART_Transmit(&huart3, buf, 2, HAL_MAX_DELAY); // Send through USB port
}

int _write(int file, char *ptr, int len) { // redirect printf to USART
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  for (int i = 0; i < len; i++) {
    ITM_SendChar(ptr[i]);
  }
  return len;
}

/* ---------------------- Encoder utilities ------------------------- */
/* TIM2: PA15/PB3  (Left), TIM5: PA0/PA1 (Right) */
static uint16_t L0 = 0, R0 = 0; // baselines captured at reset

static inline uint32_t tim_arr(TIM_TypeDef *t) {
  return (t->ARR ? t->ARR : 0xFFFFu);
}

// Forward ticks on LEFT wheel since reset (>=0) with wrap handling
static inline int32_t left_ticks_forward(void) {
  uint32_t arrp1 = tim_arr(TIM2) + 1u;
  int32_t d = (int32_t)((uint32_t)TIM2->CNT - (uint32_t)L0);
  if (d < -(int32_t)(arrp1 / 2))
    d += (int32_t)arrp1;
  else if (d > (int32_t)(arrp1 / 2))
    d -= (int32_t)arrp1;
  return d;
}

// Forward ticks on RIGHT wheel since reset (>=0); flip sense (R reversed)
static inline int32_t right_ticks_forward(void) {
  uint32_t arrp1 = tim_arr(TIM5) + 1u;
  int32_t d = (int32_t)((uint32_t)R0 - (uint32_t)TIM5->CNT);
  if (d < -(int32_t)(arrp1 / 2))
    d += (int32_t)arrp1;
  else if (d > (int32_t)(arrp1 / 2))
    d -= (int32_t)arrp1;
  return d;
}

// Reverse ticks on LEFT wheel since reset (>=0); counts when wheel turns
// backward
static inline int32_t left_ticks_reverse(void) { return -left_ticks_forward(); }

// Reverse ticks on RIGHT wheel since reset (>=0); flip sense (R reversed)
static inline int32_t right_ticks_reverse(void) {
  return -right_ticks_forward();
}

static inline void reset_encoders(void) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  HAL_Delay(2);             // small settle
  L0 = (uint16_t)TIM2->CNT; // capture baselines
  R0 = (uint16_t)TIM5->CNT;
}

static float cm_travelled_forward(void) {
  float cmL = (float)left_ticks_forward() / COUNTS_PER_CM_L;
  float cmR = (float)right_ticks_forward() / COUNTS_PER_CM_R;
  return 0.5f * (cmL + cmR);
}

static float cm_travelled_reverse(void) {
  float cmL = (float)left_ticks_reverse() / COUNTS_PER_CM_L_REVERSE;
  float cmR = (float)right_ticks_reverse() / COUNTS_PER_CM_R_REVERSE;
  return 0.5f * (cmL + cmR);
}

static uint16_t adc_read_channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // stable for GP2Y0A21YK
  HAL_ADC_ConfigChannel(hadc, &sConfig);

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
  uint16_t raw_v = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);

  // Implement a first-order Low Pass Filter (Exponential Moving Average) per
  // channel
  static float filtered_val[19] = {0};
  static uint8_t initialized[19] = {0};

  // Extract channel index assuming values like ADC_CHANNEL_6 are 0-18
  uint32_t ch_idx = channel & 0x1F;

  if (ch_idx < 19) {
    if (!initialized[ch_idx]) {
      filtered_val[ch_idx] = (float)raw_v;
      initialized[ch_idx] = 1;
    } else {
      // Alpha controls weight of NEW value: lower = more smoothing / slower
      // response
      const float alpha = 0.6f;
      filtered_val[ch_idx] =
          alpha * (float)raw_v + (1.0f - alpha) * filtered_val[ch_idx];
    }
    return (uint16_t)filtered_val[ch_idx];
  }

  return raw_v; // Fallback
}

/* *
 * Uses piecewise linear interpolation over 13 measured (mv, cm) pairs.
 * This is more accurate than a single analytical formula because the
 * sensor response has near-field and far-field roll-off that no simple
 * curve fits well across the full 5–70 cm range.
 *
 * Calibration data (measured):
 *   mv : 3080  3030  2680  2520  2453  1650  1295  1080  840  630  510  435 337
 *   cm :    5    7     8     9     10    15    20    25   30   40   50   60 70
 *
 * For any mv strictly between two table entries the output is linearly
 * interpolated.  Inputs outside [220, 3079] mV are clamped to [5, 70] cm.
 */
static inline float dist_cm_from_mv_6(uint32_t mv) {
  /* Calibration table — mv sorted descending (close→far) */
  static const uint32_t mv_lut[] = {3080, 3030, 2680, 2520, 2453, 1650, 1295,
                                    1080, 840,  630,  510,  435,  337};
  static const float cm_lut[] = {5.0f,  7.0f,  8.0f,  9.0f,  10.0f,
                                 15.0f, 20.0f, 25.0f, 30.0f, 40.0f,
                                 50.0f, 60.0f, 70.0f};
  const int N = 13;

  /* Clamp out-of-range inputs */
  if (mv >= mv_lut[0])
    return cm_lut[0];
  if (mv <= mv_lut[N - 1])
    return cm_lut[N - 1];

  /* Find the segment [mv_lut[i], mv_lut[i+1]] that contains mv */
  for (int i = 0; i < N - 1; i++) {
    if (mv >= mv_lut[i + 1]) {
      /* t = 0 at mv_lut[i] (cm_lut[i]), t = 1 at mv_lut[i+1] (cm_lut[i+1]) */
      float t = (float)(mv_lut[i] - mv) / (float)(mv_lut[i] - mv_lut[i + 1]);
      return cm_lut[i] + t * (cm_lut[i + 1] - cm_lut[i]);
    }
  }
  return cm_lut[N - 1]; /* unreachable, satisfies compiler */
}

/* *
 * Uses piecewise linear interpolation over 8 measured (mv, cm) pairs.
 * This is more accurate than a single analytical formula because the
 * sensor response has near-field and far-field roll-off that no simple
 * curve fits well across the full 5–70 cm range.
 *
 * Calibration data (measured):
 *   mv : 3079  3045  2770  2545  2480  1750  1365  1115  800  610  510  415 220
 *   cm :    5    7     8     9     10    15    20   25   30   40   50   60   70
 *
 * For any mv strictly between two table entries the output is linearly
 * interpolated.  Inputs outside [220, 3079] mV are clamped to [5, 70] cm.
 */
static inline float dist_cm_from_mv_7(uint32_t mv) {
  /* Calibration table — mv sorted descending (close→far) */
  static const uint32_t mv_lut[] = {3079, 3045, 2770, 2545, 2480, 1750, 1365,
                                    1115, 800,  610,  510,  415,  220};
  static const float cm_lut[] = {5.0f,  7.0f,  8.0f,  9.0f,  10.0f,
                                 15.0f, 20.0f, 25.0f, 30.0f, 40.0f,
                                 50.0f, 60.0f, 70.0f};
  const int N = 13;

  /* Clamp out-of-range inputs */
  if (mv >= mv_lut[0])
    return cm_lut[0];
  if (mv <= mv_lut[N - 1])
    return cm_lut[N - 1];

  /* Find the segment [mv_lut[i], mv_lut[i+1]] that contains mv */
  for (int i = 0; i < N - 1; i++) {
    if (mv >= mv_lut[i + 1]) {
      /* t = 0 at mv_lut[i] (cm_lut[i]), t = 1 at mv_lut[i+1] (cm_lut[i+1]) */
      float t = (float)(mv_lut[i] - mv) / (float)(mv_lut[i] - mv_lut[i + 1]);
      return cm_lut[i] + t * (cm_lut[i + 1] - cm_lut[i]);
    }
  }
  return cm_lut[N - 1]; /* unreachable, satisfies compiler */
}

// IMU configuration
static void ICM20948_SelectBank(uint8_t bank) {
  uint8_t d[2] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY);
}

static void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val) {
  ICM20948_SelectBank(bank);
  uint8_t d[2] = {reg, val};
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY);
}

static void ICM20948_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *data,
                              uint8_t len) {
  ICM20948_SelectBank(bank);
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c2, ICM_ADDR, data, len, HAL_MAX_DELAY);
}

// quick raw read (no bank select) used only by detector
static HAL_StatusTypeDef icm_read_raw(uint16_t addr, uint8_t reg, uint8_t *data,
                                      uint8_t len) {
  HAL_StatusTypeDef s;
  s = HAL_I2C_Master_Transmit(&hi2c2, addr, &reg, 1, 100);
  if (s != HAL_OK)
    return s;
  return HAL_I2C_Master_Receive(&hi2c2, addr, data, len, 100);
}

// try 0x68 then 0x69, set ICM_ADDR if WHO_AM_I == 0xEA
int ICM20948_Detect(void) {
  uint8_t who = 0;

  if (icm_read_raw(ICM_ADDR_68, WHO_AM_I, &who, 1) == HAL_OK &&
      who == WHO_AM_I_VAL) {
    ICM_ADDR = ICM_ADDR_68;
    return 0;
  }
  if (icm_read_raw(ICM_ADDR_69, WHO_AM_I, &who, 1) == HAL_OK &&
      who == WHO_AM_I_VAL) {
    ICM_ADDR = ICM_ADDR_69;
    return 0;
  }
  return -1; // not found on this I2C bus
}

// Init IMU
int ICM20948_Init(void) {
  uint8_t whoami;
  ICM20948_ReadRegs(0, WHO_AM_I, &whoami, 1);
  if (whoami != 0xEA)
    return -1;

  // Reset device
  ICM20948_WriteReg(0, 0x06, 0x80); // DEVICE_RESET=1
  HAL_Delay(100);
  ICM20948_WriteReg(0, 0x06, 0x01); // wake + clock select

  // Enable accel + gyro
  ICM20948_WriteReg(0, 0x07, 0x00);
  ICM20948_WriteReg(0, 0x05, 0x00); // disable cycle mode

  // Set accel = ±2g, gyro = ±250 dps
  ICM20948_WriteReg(2, 0x14, 0x00); // accel config (no filter by default)

  // Gyro Config 1:
  // [2:1] GYRO_FS_SEL = 00b (±250 dps)
  // [0] GYRO_FCHOICE = 1b (Enable Gyro DLPF)
  ICM20948_WriteReg(2, 0x01, 0x01); // gyro config (fs=250dps, DLPF=enabled)

  // Gyro Config 2: (GYRO_DLPFCFG)
  // Options:
  // 0 = 196.6 Hz bandwidth, 3.1 ms latency
  // 1 = 151.8 Hz bandwidth, 3.4 ms latency
  // 2 = 119.5 Hz bandwidth, 4.1 ms latency
  // 3 = 51.2 Hz bandwidth, 7.3 ms latency
  // 4 = 23.9 Hz bandwidth, 14.5 ms latency
  // 5 = 11.6 Hz bandwidth, 28.5 ms latency
  // 6 = 5.7 Hz bandwidth, 56.4 ms latency
  ICM20948_WriteReg(2, 0x02, 0x03); // Setting DLPFCFG = 3 (51.2Hz BW)

  return 0;
}

float get_IR_distance_right() {
  // IR6 PA6 = ADC1_IN6 (right sensor)

  raw6 = adc_read_channel(&hadc1, ADC_CHANNEL_6);
  mv6 = (uint32_t)raw6 * 3300u / 4095u;
  dist6 = dist_cm_from_mv_6(mv6);
  return dist6;
}

float get_IR_distance_left() {
  // IR7 PA7 = ADC1_IN7 (left sensor)

  raw7 = adc_read_channel(&hadc1, ADC_CHANNEL_7);
  mv7 = (uint32_t)raw7 * 3300u / 4095u;
  dist7 = dist_cm_from_mv_7(mv7);
  return dist7;
}

// Read raw accel/gyro
void ICM20948_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                      int16_t *gy, int16_t *gz) {
  uint8_t d[12];
  ICM20948_ReadRegs(0, 0x2D, d, 12);
  *ax = (d[0] << 8) | d[1];
  *ay = (d[2] << 8) | d[3];
  *az = (d[4] << 8) | d[5];
  *gx = (d[6] << 8) | d[7];
  *gy = (d[8] << 8) | d[9];
  *gz = (d[10] << 8) | d[11];
}

// Add this after IMU initialization
float calibrate_gyro_bias() {
  float bias_sum_raw = 0;
  const int samples = 200; // Increased samples for better stability
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    bias_sum_raw += (float)gz;
    HAL_Delay(5); // Smaller delay, more samples
  }
  return bias_sum_raw / (float)samples; // Return RAW bias
}

// Global variable
float gyro_z_bias = 0.0f;
// HCSR04 (Ultrasonic Sensor) Reading Function
uint32_t HCSR04_Read(void) {
  uint32_t start_tick, stop_tick, pulse_length;
  uint32_t timeout;

  // --- 1. Send 10 µs pulse on TRIG ---
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  for (volatile int i = 0; i < 300; i++)
    ; // ~10 µs delay @168 MHz
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  // --- 2. Wait for ECHO rising edge ---
  timeout = 1000000;
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) {
    if (--timeout == 0)
      return 999; // Error safety timeout, pretend obstacle is far away
  }

  start_tick = DWT->CYCCNT;

  // --- 3. Wait for ECHO falling edge ---
  timeout = 1000000;
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
    if (--timeout == 0)
      return 999;
  }

  stop_tick = DWT->CYCCNT;

  // --- 4. Compute pulse length ---
  pulse_length = stop_tick - start_tick;

  // Convert to µs (SystemCoreClock = 168 MHz → 1 tick = 1/168 MHz = 5.95 ns)
  uint32_t time_us = pulse_length / (SystemCoreClock / 1000000);

  // Distance (cm) = (time_us * 0.0343) / 2
  return (uint32_t)((time_us * 346) /
                    20000); // 346 m/s = speed of sound at 25°C
}

#define STRAIGHT_KP 0.8f  // Proportional gain for steering correction
#define STRAIGHT_KI 0.05f // Integral gain for steering correction
#define MIN_PWM_DIFF 50   // Minimum PWM difference to apply correction

void Drive_Forward_ToCM(float target_cm, int base_pwm) {
  reset_encoders();
  Motor_forward_reset_heading();

  if (base_pwm < pwmMin)
    base_pwm = pwmMin;
  if (base_pwm > pwmMax)
    base_pwm = pwmMax;
  const float STOP_TOL_CM = 0.0f;

  while (1) {
    //     Emergency stop for obstacles
    // if (HCSR04_Read() <= 15) {
    //   // Motor_stop();
    //   OLED_ShowString(0, 30, "Obstacle detected!");
    //   HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
    //   HAL_Delay(1000);
    //   HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
    //   // break;
    // }

    float cm_now = cm_travelled_forward();
    float cm_left = target_cm - cm_now;
    if (cm_left <= STOP_TOL_CM)
      break;

    // Speed ramping (from your friend's code)
    int pwm = base_pwm;
    if (cm_left > 30.0f)
      pwm = base_pwm;
    else if (cm_left > 10.0f)
      pwm = (int)(base_pwm * 0.60f);
    else if (cm_left > 3.0f)
      pwm = (int)(base_pwm * 0.35f);
    else
      pwm = (int)(base_pwm * 0.25f);

    if (pwm < pwmMin)
      pwm = pwmMin;

    Motor_forward(pwm); // feb 23

    // Display progress
    // snprintf(buf, sizeof(buf), "Dist: %.1f/%.1fcm", cm_now, target_cm);
    // OLED_ShowString(0, 10, (uint8_t *)buf);
    // // show left encoder ticks for debugging
    // int32_t l = left_ticks_forward();
    // int32_t r = right_ticks_forward();
    // snprintf(buf, sizeof(buf), "L:%ld R:%ld", (long)l, (long)r);
    // OLED_ShowString(0, 20, (uint8_t *)buf);

    // OLED_Refresh_Gram();

    // 4600 ticks approximately for left for 62cm, so 74 for left!?
    // 457_ for 57 cm, left, so 80.31
  }
  Motor_reverse_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  HAL_Delay(1000);
}

void Drive_Reverse_ToCM(float target_cm, int base_pwm) {
  reset_encoders();
  Motor_forward_reset_heading();

  if (base_pwm < pwmMin)
    base_pwm = pwmMin;
  if (base_pwm > pwmMax)
    base_pwm = pwmMax;
  const float STOP_TOL_CM = 0.0f;

  while (1) {
    float cm_now = cm_travelled_reverse();
    float cm_left = target_cm - cm_now;
    if (cm_left <= STOP_TOL_CM)
      break;

    // Speed ramping (same as forward)
    int pwm = base_pwm;
    if (cm_left > 30.0f)
      pwm = base_pwm;
    else if (cm_left > 10.0f)
      pwm = (int)(base_pwm * 0.60f);
    else if (cm_left > 3.0f)
      pwm = (int)(base_pwm * 0.35f);
    else
      pwm = (int)(base_pwm * 0.25f);

    if (pwm < pwmMin)
      pwm = pwmMin;

    //     Motor_reverse_advanced(pwm);
    // Motor_reverse(pwm);
    Motor_reverse_simple(pwm, pwm);

    // Display progress
    // snprintf(buf, sizeof(buf), "R Dist: %.1f/%.1fcm", cm_now, target_cm);
    // OLED_ShowString(0, 30, (uint8_t *)buf);
    // // // show left encoder ticks for debugging
    // int32_t l = left_ticks_reverse();
    // int32_t r = right_ticks_reverse();
    // snprintf(buf, sizeof(buf), "L:%ld R:%ld", (long)l, (long)r);
    // OLED_ShowString(0, 40, (uint8_t *)buf);

    // Display progress
    // snprintf(buf, sizeof(buf), "Rev: %.1f/%.1fcm", cm_now, target_cm);
    // OLED_ShowString(0, 10, (uint8_t *)buf);
    // OLED_Refresh_Gram();
  }
  Motor_forward_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  HAL_Delay(1000);
}

void Drive_Forward_ToCM_Set_Delay(float target_cm, int base_pwm,
                                  uint32_t delay_ms) {
  reset_encoders();
  Motor_forward_reset_heading();

  if (base_pwm < pwmMin)
    base_pwm = pwmMin;
  if (base_pwm > pwmMax)
    base_pwm = pwmMax;
  const float STOP_TOL_CM = 0.0f;

  while (1) {

    float cm_now = cm_travelled_forward();
    float cm_left = target_cm - cm_now;
    if (cm_left <= STOP_TOL_CM)
      break;

    // Speed ramping (from your friend's code)
    int pwm = base_pwm;
    if (cm_left > 30.0f)
      pwm = base_pwm;
    else if (cm_left > 10.0f)
      pwm = (int)(base_pwm * 0.60f);
    else if (cm_left > 3.0f)
      pwm = (int)(base_pwm * 0.35f);
    else
      pwm = (int)(base_pwm * 0.25f);

    if (pwm < pwmMin)
      pwm = pwmMin;

    Motor_forward(pwm); // feb 23
  }
  Motor_reverse_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  HAL_Delay(delay_ms);
}

void Drive_Reverse_ToCM_Set_Delay(float target_cm, int base_pwm,
                                  uint32_t delay_ms) {

  reset_encoders();
  Motor_forward_reset_heading();

  if (base_pwm < pwmMin)
    base_pwm = pwmMin;
  if (base_pwm > pwmMax)
    base_pwm = pwmMax;
  const float STOP_TOL_CM = 0.0f;

  while (1) {
    float cm_now = cm_travelled_reverse();
    float cm_left = target_cm - cm_now;
    if (cm_left <= STOP_TOL_CM)
      break;

    // Speed ramping (same as forward)
    int pwm = base_pwm;
    if (cm_left > 30.0f)
      pwm = base_pwm;
    else if (cm_left > 10.0f)
      pwm = (int)(base_pwm * 0.60f);
    else if (cm_left > 3.0f)
      pwm = (int)(base_pwm * 0.35f);
    else
      pwm = (int)(base_pwm * 0.25f);

    if (pwm < pwmMin)
      pwm = pwmMin;

    Motor_reverse_simple(pwm, pwm);
  }
  Motor_forward_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  HAL_Delay(delay_ms);
}

void Drive_Forward_Until_Obstacle(int pwm, float obstacle_clearance_distance) {
  Motor_forward_reset_heading();

  float current_pwm = pwm;

  // forward for some distance
  while (1) {

    float cm_left = HCSR04_Read() - obstacle_clearance_distance;

    if (cm_left <= 0)
      break;

    // Speed ramping
    if (cm_left < 10) {
      current_pwm = (int)(pwm * 0.5f); // Final crawl


    if (current_pwm < pwmMin)
      current_pwm = pwmMin;

    Motor_forward(current_pwm);
    HAL_Delay(50); // Delay between sensor triggers to avoid overlap/freezing
  }

  Motor_stop();
}
void Reset_Yaw_Integration(void) {
  yaw_angle = 0.0f;
  gyro_gz_filtered = 0.0f;
  // We also need to reset the previous reading so the trapezoidal integration
  // doesn't spike if the previous turn ended abruptly at a high rotation speed.
  extern float gyro_gz_prev;
  gyro_gz_prev = 0.0f;
}

float gyro_gz_prev = 0.0f;

void Update_Yaw(void) {
  static float gz_dps_corrected = 0.0f;
  const float alpha =
      0.6f; // weight of NEW value: higher = faster response / less lag

  // INCREASE DEADZONE: Vibration from the motors natively adds noise to the
  // gyro. When the robot is at a standstill or "final crawl", the motors
  // humming causes micro-vibrations which will be integrated as yaw drift if
  // the deadzone is too small.
  const float GZ_DEADZONE = 1.5f;

  uint32_t now = HAL_GetTick();
  float dt = (now - last_time) / 1000.0f;
  if (dt <= 0)
    dt = 0.001f;
  last_time = now;

  int16_t ax, ay, az, gx, gy, gz;
  ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);

  // Subtract hardware offset FIRST before determining turn direction
  float gz_raw = (float)gz;
  float gz_corrected_raw = gz_raw - gyro_z_bias;

  // Now determine turn direction based on actual motion
  if (gz_corrected_raw > 0) {
    // Math: 131.0 * (OLED_Reported_Left / Real_Life_Degrees)
    // 131.0 * (85.0 / 90.0) = 123.7
    gz_dps_corrected = gz_corrected_raw / GYRO_LEFT_BIAS;
  } else {
    // Math: 131.0 * (OLED_Reported_Right / Real_Life_Degrees)
    // 131.0 * (96.0 / 90.0) = 139.7
    gz_dps_corrected = gz_corrected_raw / GYRO_RIGHT_BIAS;
  }

  if (fabsf(gz_dps_corrected) < GZ_DEADZONE) {
    gz_dps_corrected = 0.0f;
  }

  // Use a lighter filter for display/damping, but integrate the raw corrected
  // signal
  gyro_gz_filtered =
      alpha * gz_dps_corrected + (1.0f - alpha) * gyro_gz_filtered;

  // Trapezoidal integration for better precision
  yaw_angle += (gyro_gz_prev + gz_dps_corrected) * 0.5f * dt;
  gyro_gz_prev = gz_dps_corrected;
}

/**
 * @brief Set servo angle with enhanced safety features
 * @param angle_deg: Angle in degrees [-45 to +45]
 * @param gradual: If true, move gradually to prevent sudden movements
 * @return uint16_t: Actual pulse width in microseconds sent to servo
 */
uint16_t Servo_SetAngle_Safe(int16_t angle_deg, uint8_t gradual) {
  static int16_t current_angle = 0;

  // Safety limits
  if (angle_deg < -45)
    angle_deg = -45;
  if (angle_deg > 45)
    angle_deg = 45;

  if (gradual) {
    // Move gradually to prevent stress on servo and mechanics
    int16_t step = (angle_deg > current_angle) ? 1 : -1;

    while (current_angle != angle_deg) {
      current_angle += step;
      Steering_ToUS(current_angle);
      HAL_Delay(20); // 20ms delay between steps
    }
  } else {
    // Immediate movement
    current_angle = angle_deg;
    Steering_ToUS(angle_deg);
  }

  return Steering_ToUS(current_angle);
}

/**
 * @brief Turns the robot to a target relative angle using gyroscope feedback.
 *
 * This function sets the steering to a specified angle and drives the motors
 * until the integrated yaw angle reaches the target. It includes local gyro
 * bias calibration, speed ramping as the target is approached, an obstacle
 * detection safety check, and a 15-second timeout.
 *
 * @param target_deg The relative angle to rotate by (degrees).
 * @param pwmVal The base PWM speed for the turn.
 * @param steer_angle The steering servo angle during the turn [-45 to 45].
 * @param target_cm Optional distance limit (>=0). Set to 0 to ignore.
 */

void Turn_Car(float target_deg, int pwmVal, int steer_angle, float target_cm) {
  float target_deg_abs = fabsf(target_deg);
  // Safety check on steering angle
  if (steer_angle < -45)
    steer_angle = -45;
  if (steer_angle > 45)
    steer_angle = 45;

  Reset_Yaw_Integration();
  gyro_gz_filtered = 0; // reset filter memory to prevent drift inheritance
  reset_encoders();     // reset odometry when starting the turn

  float target_cm_abs = fabsf(target_cm);
  float stop_tol_cm = 0.0f;
  uint8_t use_distance = (target_cm_abs > 0.0f) ? 1u : 0u;
  float cm_now = 0.0f;

  // Set steering angle gradually for safety
  Servo_SetAngle_Safe(steer_angle, 0); // gradual movement
  HAL_Delay(100);                      // let servo settle

  // Capture time JUST before loop starts to exclude servo movement time
  last_time = HAL_GetTick();
  uint32_t start_time = HAL_GetTick();
  uint32_t last_slow_tick = 0;
  const uint32_t timeout_ms = 8000;

  while (1) {
    uint32_t loop_tick = HAL_GetTick();
    if (loop_tick - start_time > timeout_ms)
      break;

    Update_Yaw();
    float abs_yaw = fabsf(yaw_angle);
    if (use_distance) {
      cm_now = cm_travelled_forward();
    }

    const float OVERSHOOT_OFFSET = 0.0f;
    uint8_t angle_reached = (target_deg_abs > OVERSHOOT_OFFSET) &&
                            (abs_yaw >= (target_deg_abs - OVERSHOOT_OFFSET));
    uint8_t distance_reached =
        use_distance && (cm_now >= (target_cm_abs - stop_tol_cm));
    if (angle_reached || distance_reached) {
      break;
    }

    // Apply gyro bias correction
    // yaw_angle -= gyro_bias * (HAL_GetTick() - last_time) / 1000.0f;
    // NOTE: Update_Yaw() already integrates (gz - gyro_z_bias).
    // DO NOT subtract bias again here.

    // Drive with controlled speed (reduce speed as we approach target)
    float angle_progress =
        (target_deg_abs > 0.0f) ? abs_yaw / target_deg_abs : 0.0f;
    if (angle_progress > 1.0f)
      angle_progress = 1.0f;
    float dist_progress = (use_distance && target_cm_abs > 0.0f)
                              ? (cm_now / target_cm_abs)
                              : 0.0f;
    if (dist_progress > 1.0f)
      dist_progress = 1.0f;
    float progress = fmaxf(angle_progress, dist_progress);
    int current_pwm = pwmVal;

    // if (progress > 0.90f) {
    //   current_pwm = (int)(pwmVal * 0.5f);
    // }

    if (progress > 0.85f) {
      current_pwm = (int)(pwmVal * 0.4f);
    } else if (progress > 0.7f) {
      current_pwm = (int)(pwmVal * 0.6f);
    }

    // --- Ackermann differential: slow the inner wheel ---
    int pwm_left = current_pwm;
    int pwm_right = current_pwm;

    float steer_rad = fabsf((float)steer_angle) * PI / 180.0f;
    if (steer_rad > 0.01f) { // avoid division by zero for near-straight
      float R_inner = WHEELBASE_CM / tanf(steer_rad);
      float R_outer = R_inner + TRACK_WIDTH_CM;
      float ratio = R_inner / R_outer; // < 1.0

      int pwm_inner = (int)(current_pwm * ratio);
      if (pwm_inner < pwmMin)
        pwm_inner = pwmMin;

      if (steer_angle > 0) {
        // Turning right -> right wheel is inner
        pwm_right = pwm_inner;
      } else {
        // Turning left  -> left wheel is inner
        pwm_left = pwm_inner;
      }
    }

    // pwm clamping
    // if (pwm_left < pwmMin)
    //   pwm_left = pwmMin;
    // if (pwm_left > pwmMax)
    //   pwm_left = pwmMax;
    // if (pwm_right < pwmMin)
    //   pwm_right = pwmMin;
    // if (pwm_right > pwmMax)
    //   pwm_right = pwmMax;

    Motor_forward_simple(pwm_left, pwm_right);

    // Run non-critical slow tasks (US sensor, OLED) only every 50ms
    // if (loop_tick - last_slow_tick > 50) {
    //   last_slow_tick = loop_tick;
    //   if (HCSR04_Read() <= OBSTACLE_THRESHOLD_CM) {
    //     HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
    //     // Motor_stop();
    //     HAL_Delay(1000);
    //     HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
    //     // break;
    //   }

    // Debug output
    //            if (use_distance) {
    //            snprintf(buf, sizeof(buf), "Yaw %.1f/%.1f Dist %.1f/%.1f",
    //                     abs_yaw, target_deg_abs, cm_now, target_cm_abs);
    //            } else {
    //            snprintf(buf, sizeof(buf), "Yaw: %.1f° Target: %.1f°",
    //            yaw_angle, target_deg_abs);
    //            }
    // snprintf(buf, sizeof(buf), "Yaw %.1f/%.1f Dist %.1f/%.1f", abs_yaw,
    //          target_deg_abs, cm_now, target_cm_abs);
    // OLED_ShowString(0, 30, (uint8_t *)buf);
    // OLED_Refresh_Gram();
    // }

    HAL_Delay(5); // Faster loop frequency (200Hz)
  }
  Motor_reverse_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  Servo_SetAngle_Safe(0, 0); // gradual return to center
  HAL_Delay(50);

  // Final position feedback
  //    if (use_distance) {
  //        cm_now = cm_travelled_forward();
  //        snprintf(buf, sizeof(buf), "Final Y:%.1f° D:%.1fcm", yaw_angle,
  //        cm_now);
  //    } else {
  //        snprintf(buf, sizeof(buf), "Final: %.1f°", yaw_angle);
  //    }
  // cm_now = cm_travelled_forward();
  // snprintf(buf, sizeof(buf), "Final Y:%.1f° D:%.1fcm", yaw_angle, cm_now);
  // OLED_ShowString(0, 30, (uint8_t *)buf);
  // OLED_Refresh_Gram();
}

void Turn_Car_Reverse(float target_deg, int pwmVal, int steer_angle,
                      float target_cm) {
  float target_deg_abs = fabsf(target_deg);
  // Safety check on steering angle
  if (steer_angle < -45)
    steer_angle = -45;
  if (steer_angle > 45)
    steer_angle = 45;

  Reset_Yaw_Integration();
  gyro_gz_filtered = 0; // reset filter memory to prevent drift inheritance
  reset_encoders();     // reset odometry when starting the turn

  float target_cm_abs = fabsf(target_cm);
  float stop_tol_cm = 0.0f;
  uint8_t use_distance = (target_cm_abs > 0.0f) ? 1u : 0u;
  float cm_now = 0.0f;

  //     float gyro_bias = 0;
  // for(int i = 0; i < 10; i++) {
  //     int16_t ax, ay, az, gx, gy, gz;
  //     ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
  //     gyro_bias += gz / 131.0f; // Convert to dps
  //     HAL_Delay(10);
  // }
  // gyro_bias /= 10.0f;
  // Calibrate gyro bias handled globally in main/init
  // If you want to recalibrate here, you should update the GLOBAL `gyro_z_bias`
  // but typically one calibration at startup is enough.
  // For now, we will rely on Update_Yaw using the global bias.

  // Set steering angle gradually for safety
  Servo_SetAngle_Safe(steer_angle, 1); // gradual movement
  HAL_Delay(100);                      // let servo settle

  // Capture time JUST before loop starts to exclude servo movement time
  last_time = HAL_GetTick();
  uint32_t start_time = HAL_GetTick();
  uint32_t last_slow_tick = 0;
  const uint32_t timeout_ms = 8000;

  while (1) {
    uint32_t loop_tick = HAL_GetTick();
    if (loop_tick - start_time > timeout_ms)
      break;

    Update_Yaw();
    float abs_yaw = fabsf(yaw_angle);
    if (use_distance) {
      cm_now = cm_travelled_reverse();
    }

    const float OVERSHOOT_OFFSET = 0.0f;
    uint8_t angle_reached = (target_deg_abs > OVERSHOOT_OFFSET) &&
                            (abs_yaw >= (target_deg_abs - OVERSHOOT_OFFSET));
    uint8_t distance_reached =
        use_distance && (cm_now >= (target_cm_abs - stop_tol_cm));
    if (angle_reached || distance_reached) {
      break;
    }

    // Apply gyro bias correction
    // yaw_angle -= gyro_bias * (HAL_GetTick() - last_time) / 1000.0f;
    // NOTE: Update_Yaw() already integrates (gz - gyro_z_bias).
    // DO NOT subtract bias again here.

    // Drive with controlled speed (reduce speed as we approach target)
    float angle_progress =
        (target_deg_abs > 0.0f) ? abs_yaw / target_deg_abs : 0.0f;
    if (angle_progress > 1.0f)
      angle_progress = 1.0f;
    float dist_progress = (use_distance && target_cm_abs > 0.0f)
                              ? (cm_now / target_cm_abs)
                              : 0.0f;
    if (dist_progress > 1.0f)
      dist_progress = 1.0f;
    float progress = fmaxf(angle_progress, dist_progress);
    int current_pwm = pwmVal;

    if (progress > 0.95f) {
      current_pwm = pwmMin; // Final crawl
    } else if (progress > 0.85f) {
      current_pwm = (int)(pwmVal * 0.3f);
    } else if (progress > 0.7f) {
      current_pwm = (int)(pwmVal * 0.6f);
    }

    // --- Ackermann differential: slow the inner wheel ---
    int pwm_left = current_pwm;
    int pwm_right = current_pwm;

    float steer_rad = fabsf((float)steer_angle) * PI / 180.0f;
    if (steer_rad > 0.02f) { // avoid division by zero for near-straight
      float R_inner = WHEELBASE_CM / tanf(steer_rad);
      float R_outer = R_inner + TRACK_WIDTH_CM;
      float ratio = R_inner / R_outer; // < 1.0

      int pwm_inner = (int)(current_pwm * ratio);
      if (pwm_inner < pwmMin)
        pwm_inner = pwmMin;

      if (steer_angle > 0) {
        // Turning right -> right wheel is inner
        pwm_right = pwm_inner;
      } else {
        // Turning left  -> left wheel is inner
        pwm_left = pwm_inner;
      }
    }

    // pwm clamping
    if (pwm_left < pwmMin)
      pwm_left = pwmMin;
    if (pwm_left > pwmMax)
      pwm_left = pwmMax;
    if (pwm_right < pwmMin)
      pwm_right = pwmMin;
    if (pwm_right > pwmMax)
      pwm_right = pwmMax;

    Motor_reverse_simple(pwm_left, pwm_right);

    // Run non-critical slow tasks (US sensor, OLED) only every 50ms
    if (loop_tick - last_slow_tick > 50) {
      last_slow_tick = loop_tick;

      // Debug output
      //            if (use_distance) {
      //            snprintf(buf, sizeof(buf), "Yaw %.1f/%.1f Dist %.1f/%.1f",
      //                     abs_yaw, target_deg_abs, cm_now, target_cm_abs);
      //            } else {
      //            snprintf(buf, sizeof(buf), "Yaw: %.1f° Target: %.1f°",
      //            yaw_angle, target_deg_abs);
      //            }

      snprintf(buf, sizeof(buf), "Yaw %.1f/%.1f Dist %.1f/%.1f", abs_yaw,
               target_deg_abs, cm_now, target_cm_abs);
      OLED_ShowString(0, 30, (uint8_t *)buf);
      OLED_Refresh_Gram();
    }

    HAL_Delay(5); // Faster loop frequency (200Hz)
  }
  Motor_forward_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  Servo_SetAngle_Safe(0, 0); // gradual return to center
  HAL_Delay(100);

  //    // Final position feedback
  //    if (use_distance) {
  //        cm_now = cm_travelled_reverse();
  //        snprintf(buf, sizeof(buf), "Final Y:%.1f° D:%.1fcm", yaw_angle,
  //        cm_now);
  //    } else {
  //        snprintf(buf, sizeof(buf), "Final: %.1f°", yaw_angle);
  //    }
  cm_now = cm_travelled_reverse();
  snprintf(buf, sizeof(buf), "Final Y:%.1f° D:%.1fcm", yaw_angle, cm_now);
  OLED_ShowString(0, 30, (uint8_t *)buf);
  OLED_Refresh_Gram();
}

void task_two_second_obs_check() {
  // find the acceptable distance to the 2nd obstacle
  reset_encoders();
  float front_dist = HCSR04_Read();
  HAL_Delay(100);

  // SEND to rpi for 2nd obstacle while moving
  // task_two_uart()

  // Back up / move forward until 2nd obstacle is within clearance distance
  if (front_dist < TASK2_obs_2_clearance_distance) {
    Drive_Reverse_ToCM_Set_Delay(TASK2_obs_2_clearance_distance - front_dist,
                                 TASK2_PWM, 100);
    TASK2_vertical_dist_now -= cm_travelled_reverse();
  } else if (front_dist >= TASK2_obs_2_clearance_distance) {
    Drive_Forward_Until_Obstacle(TASK2_PWM, TASK2_obs_2_clearance_distance);
    TASK2_vertical_dist_now += cm_travelled_forward();
  }
}

void task_two() {

  send_message_over("ACK\n");

  // Reset tracked distances
  TASK2_vertical_dist_now = 0;
  TASK2_horizontal_dist_now = 0;

  // listen to rpi for 1st obstacle
  char direction_obs1 = '<'; // task_two_uart()

  // drive around 1st obstacle
  task_two_clear_first_obs_alternate(TASK2_PWM, TASK2_obs_1_clearance_distance,
                                     direction_obs1);

  char direction_obs2 = '<'; // task_two_uart()

  // Reverse and move forward until 2nd obstacle is within clearance distance
  task_two_second_obs_check();

  // Uncomment to test first obstacle only
  //  Motor_stop();
  //  return;

  // Turn according to picture (arrow)
  if (direction_obs2 == '<') {

    // Turn left to be parallel with obstacle 2
    Turn_Car(90, TASK2_PWM, -45, 0);
    reset_encoders();
    Motor_forward_reset_heading();

    // use right IR to clear side of obstacle 2
    do {
      Motor_forward(TASK2_PWM);
      HAL_Delay(30);
    } while (get_IR_distance_right() < 50.0f);

    // Turn behind obstacle2
    switch (TASK2_current_surface) {
    case TASK2_SURFACE_HPL: {
      if (direction_obs1 == '>')
        Turn_Car(173, TASK2_PWM, 45, 0);
      else
        Turn_Car(175, TASK2_PWM, 45, 0);
    } break;
    case TASK2_SURFACE_OUTSIDE: {
      if (direction_obs1 == '>')
        Turn_Car(165, TASK2_PWM, 45, 0);
      else
        Turn_Car(173, TASK2_PWM, 45, 0);
    } break;
    }

    Motor_forward_reset_heading();
    reset_encoders();

    // Failsafe to prevent undershooting wall when turning 180
    Motor_forward(TASK2_PWM);
    HAL_Delay(750);

    // use right IR
    do {
      Motor_forward(TASK2_PWM);
      HAL_Delay(30);
    } while (get_IR_distance_right() < 60.0f);

  } else if (direction_obs2 == '>') {

    // Turn right to be parallel with obstacle 2
    Turn_Car(90, TASK2_PWM, 45, 0);
    reset_encoders();
    Motor_forward_reset_heading();

    // use left IR to clear side of obstacle 2
    do {
      Motor_forward(TASK2_PWM);
      HAL_Delay(30);
    } while (get_IR_distance_left() < 50.0f);

    // Turn behind obstacle2
    switch (TASK2_current_surface) {
    case TASK2_SURFACE_HPL: {
      if (direction_obs1 == '>')
        Turn_Car(175, TASK2_PWM, -45, 0);
      else
        Turn_Car(175, TASK2_PWM, -45, 0);
    } break;
    case TASK2_SURFACE_OUTSIDE: {
      if (direction_obs1 == '>')
        Turn_Car(173, TASK2_PWM, -45, 0);
      else
        Turn_Car(173, TASK2_PWM, -45, 0);
    } break;
    }

    Motor_forward_reset_heading();

    // Failsafe to prevent undershooting wall when turning 180
    Motor_forward(TASK2_PWM);
    HAL_Delay(750);

    // use left IR
    do {
      Motor_forward(TASK2_PWM);
      HAL_Delay(30);
    } while (get_IR_distance_left() < 60.0f);
  }

  TASK2_horizontal_dist_now = cm_travelled_forward();
  Motor_stop();
  task_two_return_to_start(direction_obs1, direction_obs2 == '<' ? '>' : '<');

  // Inform RPI end of task 2
  Motor_stop();
  send_message_over("END\n");
}

void task_two_print_stats() {
  char buf[100];
  snprintf(buf, sizeof(buf), "V:%.1f H:%.1f", TASK2_vertical_dist_now,
           TASK2_horizontal_dist_now);
  OLED_ShowString(0, 30, (uint8_t *)buf);
  OLED_Refresh_Gram();
  HAL_Delay(1000);
}

// This is the entry point for returning to the start
// IR sensor detects that second obstacle is cleared
void task_two_return_to_start(char direction_obs1,
                              char initial_carpark_direction) {

  TASK2_vertical_dist_now +=
      TASK2_obs_2_clearance_distance + TASK2_distance_from_back_of_second_obs;
  task_two_print_stats();

  // Drive forward slightly to get away from the carpark wall
  Motor_forward_simple(TASK2_PWM, TASK2_PWM);

  // Do not increase this further - needs 50 cm clearance
  HAL_Delay(125);
  Motor_stop();

  // Calculate turn angle to face carpark general direction, based on current
  // surface
  int turn_angle;

  switch (TASK2_current_surface) {
  case TASK2_SURFACE_HPL: {
    if (initial_carpark_direction == '>')
      turn_angle = 90;
    else
      turn_angle = 85;
  } break;

    // OUTDATED
  case TASK2_SURFACE_OUTSIDE:
    if (initial_carpark_direction == '>')
      turn_angle = 85;
    else if (direction_obs1 == '>')
      turn_angle = 82; // was 72
    else
      turn_angle = 78;
    break;
  default:
    turn_angle = 75;
    break;
  }

  // Carpark is on the right, turn right
  if (initial_carpark_direction == '>') {
    Turn_Car(turn_angle, TASK2_PWM, 45, 0);
    // Carpark is on the left, turn left
  } else if (initial_carpark_direction == '<') {
    Turn_Car(turn_angle + 7, TASK2_PWM, -45, 0);
  }
  // Move straight until clear
  reset_encoders();
  Motor_forward_reset_heading();
  HAL_Delay(500);

  // The car is now facing back at the carpark wall
  int return_dist =
      TASK2_vertical_dist_now - TASK2_vertical_dist_return_arc_buffer;

  // Drive forward until arc point
  Drive_Forward_ToCM_Set_Delay(return_dist, TASK2_SPRINT_PWM, 100);

  // Turn in perpencidular to carpark
  if (initial_carpark_direction == '>') {
    Turn_Car(turn_angle + 5, TASK2_PWM, 45, 0);
  } else if (initial_carpark_direction == '<') {
    Turn_Car(turn_angle, TASK2_PWM, -45, 0);
  }

  // Based on length of obstacle2, first reverse then move forward until the
  // carpark wall is found If obstacle2 is too short, it will skip this step
  // and assume that carpark wall is already clear
  if (TASK2_horizontal_dist_now > TASK2_obstacle2_too_short_threshold) {

    const float alignment_pwm = 2500;

    Motor_reverse_simple(alignment_pwm, alignment_pwm);
    HAL_Delay(1000);
    Motor_stop();

    // Move forward until the carpark wall is found
    do {
      Motor_forward_simple(alignment_pwm, alignment_pwm);
      HAL_Delay(30); // polling rate for IR sensor
    } while (
        (initial_carpark_direction == '>' &&
         get_IR_distance_left() > TASK2_carpark_side_IR_distance_threshold) ||
        (initial_carpark_direction == '<' &&
         get_IR_distance_right() > TASK2_carpark_side_IR_distance_threshold));
    Motor_stop();

    // Move forward boost to clear carpark wall
    Motor_forward_simple(TASK2_SPRINT_PWM, TASK2_SPRINT_PWM);
    switch (TASK2_current_surface) {
    case TASK2_SURFACE_HPL: {
      if (initial_carpark_direction == '>')
        HAL_Delay(275);
      else
        HAL_Delay(325);
    } break;
    case TASK2_SURFACE_OUTSIDE:
      HAL_Delay(275);
      break;
    }
    Motor_stop();
  }

  // Turn into carpark bay in the final stretch
  if (initial_carpark_direction == '>') {
    // Left turn
    Turn_Car(turn_angle - 10, TASK2_PWM, -45, 0);
  } else if (initial_carpark_direction == '<') {
    // Right turn
    Turn_Car(turn_angle, TASK2_PWM, 45, 0);
  }

  // Return straight to start position, stopping based on ultrasonic
  reset_encoders();
  Motor_forward_reset_heading();
  Drive_Forward_Until_Obstacle(TASK2_SPRINT_PWM,
                               TASK2_carpark_wall_clearance_distance);
  Motor_stop();

  // Motor_forward_simple(TASK2_SPRINT_PWM, TASK2_SPRINT_PWM);
  //  if(TASK2_current_surface == TASK2_SURFACE_OUTSIDE) {
  //	  HAL_Delay(100);
  //  }else{
  //	  HAL_Delay(150);
  //  }

  // Only accelerate if front right/left sensor doesn't see obstacle 1
  //  if((current_direction=='>' && get_IR_distance_right() >
  //  TASK2_carpark_side_IR_distance_threshold)
  //		  || (current_direction == '<' && get_IR_distance_left() >
  // TASK2_carpark_side_IR_distance_threshold)) {
  //	  // Monitor
  //	  Motor_forward_simple(TASK2_SPRINT_PWM, TASK2_SPRINT_PWM);
  //	  if(TASK2_current_surface == TASK2_SURFACE_OUTSIDE) {
  //		  HAL_Delay(100);
  //	  }else{
  //		  HAL_Delay(150);
  //	  }
  //  }

  // Reverse until carpark wall is found
  //  do {
  //    Motor_reverse_simple(2500, 2500);
  //    HAL_Delay(30); // polling rate for IR sensor
  //  } while (
  //      (current_direction == '>' &&
  //       get_IR_distance_left() > TASK2_carpark_side_IR_distance_threshold)
  //       ||
  //      (current_direction == '<' &&
  //       get_IR_distance_right() >
  //       TASK2_carpark_side_IR_distance_threshold));
  //  Motor_stop();

  // Go forward again until carpark wall is gone
}

// Arms the UART3 IT receiver for one character. Call this before doing other
// work so the byte is captured in the background. Safe to call multiple times.
void UART_arm() {
  uart_rx_ready = 0;
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_rx_byte, 1);
}

// Waits for a single direction character ('<' or '>') over UART3.
// Assumes UART_arm() has already been called. Retries once on timeout.
char UART_receive() {

  for (int attempt = 0; attempt < 2; ++attempt) {

    uint32_t t0 = HAL_GetTick();

    while (HAL_GetTick() - t0 < SNAP_WAIT_MS) {
      if (uart_rx_ready) {
        uart_rx_ready = 0;
        char ch = uart_rx_byte;
        if (ch == '<' || ch == '>') {
          send_message_over("ACK\n");
          return ch;
        }
        // Ignore any other byte (e.g. '\n', '\r', junk) and keep waiting
      }
    }
  }

  send_message_over("timeout\n");
  return 0;
}

/**
Updates: TASK2_vertical_dist_now SIDE AFFECT
 */

void task_two_clear_first_obs_alternate(int pwm,
                                        float obstacle_clearance_distance,
                                        char direction) {
  reset_encoders();

  Drive_Forward_Until_Obstacle(pwm, obstacle_clearance_distance);
  // no stop, turn and continue, HARDCODED
  if (direction == '<') {

    Turn_Car(-45, pwm, -45, 0);
    Turn_Car(35, pwm, 45, 0);

    // Track start side of obstacle 1
    do {
      Motor_forward_simple(pwm, pwm);
      HAL_Delay(30);
    } while (get_IR_distance_right() > TASK2_obs_1_start_side_distance);

    // Track end side of obstacle 1
    do {
      Motor_forward_simple(pwm, pwm);
      HAL_Delay(30);
    } while (get_IR_distance_right() < TASK2_obs_1_start_side_distance);

    Motor_stop();

    // Possible to send snapshot here to RPI
    // send_message_over("snap\n");

    Turn_Car(75, pwm, 45, 0);
    Motor_reverse_simple(pwm, pwm);
    HAL_Delay(250);
    Turn_Car(60, pwm, -45, 0);
    // Motor_forward_simple(pwm, pwm);
    // HAL_Delay(100);
    Motor_stop();

  } else if (direction == '>') {

    Turn_Car(-45, pwm, 45, 0);
    Turn_Car(35, pwm, -45, 0);

    // Track start side of obstacle 1
    do {
      Motor_forward_simple(pwm, pwm);
      HAL_Delay(30);
    } while (get_IR_distance_left() > TASK2_obs_1_start_side_distance);

    // Track end side of obstacle 1
    do {
      Motor_forward_simple(pwm, pwm);
      HAL_Delay(30);
    } while (get_IR_distance_left() < TASK2_obs_1_start_side_distance);

    Motor_stop();

    // Possible to send snapshot here to RPI
    // send_message_over("snap\n");

    Turn_Car(75, pwm, -45, 0);
    Motor_reverse_simple(pwm, pwm);
    HAL_Delay(150);
    Turn_Car(60, pwm, 45, 0);
    Motor_forward_simple(pwm, pwm);
    HAL_Delay(100);
    Motor_stop();
  }
  Motor_reverse_simple(1000, 1000);
  HAL_Delay(50);
  Motor_stop();
  TASK2_vertical_dist_now += cm_travelled_forward() +
                             obstacle_clearance_distance + 10 +
                             TASK2_distance_from_back_of_first_obs;
}


void front_back_test() {
  int delay_btw_front_back = 5000;
  int delay_after_front_back = 8000;

  HAL_Delay(2000);

  int front_back_distance = 50.0f;

  Drive_Forward_ToCM(front_back_distance, 3000);
  Motor_stop();

  HAL_Delay(delay_btw_front_back);

  Drive_Reverse_ToCM(front_back_distance, 3000);
  Motor_stop();

  HAL_Delay(delay_after_front_back);

  front_back_distance = 100.0f;

  Drive_Forward_ToCM(front_back_distance, 3000);
  Motor_stop();

  HAL_Delay(delay_btw_front_back);

  Drive_Reverse_ToCM(front_back_distance, 3000);
  Motor_stop();

  HAL_Delay(delay_after_front_back);
}

void testing() {
  // Turn_Car_Reverse(90,3000,45,0);
  // HAL_Delay(10000);
  // Turn_Car_Reverse(90,3000,-45,0);
  // front_back_test();
  // turning_test();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  uint8_t *oled_buf; // buffer to store value to be display on OLED
  uint8_t i, status; // status for checking return

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();
  // Enable DWT Cycle Counter for precise timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0; // reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  // gyro_z_bias = calibrate_gyro_bias(); // MOVED DOWN after init
  /* USER CODE BEGIN 2 */

  MotorDrive_enable(); // enable PWM needed to drive MotroDrive A and D
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // For servo
  // start TIM2/TIM5-Encoder to read Motor rotation in interrupt mode
  // Hall sensors produce 13 ticks/counts per turn, gear ratio = 20
  // 260 count per rotation of output (wheel)
  // 360 degree = 260 ticks/counts
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Left encoder (TIM2)
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // Right encoder (TIM5)
  // rpm = (int)((1000/no_of_tick) * 60/260 * 1/dt);  // For calculating motor
  // rpm - by multiplying it with speed value rpm = (1000.0f /
  // (float)no_of_tick) * (60.0f / 260.0f);
  OLED_Init();
  //  OLED_ShowString(10, 5, "SC2104/CE3002"); // show message on OLED display
  //  at line 5) OLED_ShowString(40, 30, "Lab 4");        // show message on
  //  OLED display at line 30) oled_buf = "Motor Control";              //
  //  anther way to show message through buffer OLED_ShowString(10, 50,
  //  oled_buf);       // another message at line 50

  uint8_t sbuf[] = "SC2104\n\r"; // send to serial port
  HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf),
                    HAL_MAX_DELAY); // Send through Serial Port @115200
  HAL_UART_Transmit(&huart2, sbuf, sizeof(sbuf),
                    HAL_MAX_DELAY); // Send through BT @9600

  OLED_Refresh_Gram();
  HAL_Delay(3000); // pause for 3 second to show message
  OLED_Clear();    // get display ready

  if (ICM20948_Detect() == 0) {
    sprintf(buf, "ICM @0x%02X", (unsigned)(ICM_ADDR >> 1));
  } else {
    sprintf(buf, "ICM NOT FOUND");
  }
  OLED_ShowString(0, 0, (uint8_t *)buf);
  OLED_Refresh_Gram();
  HAL_Delay(500);

  if (ICM20948_Init() == 0) {
    sprintf(buf, "ICM OK");
    // Calibrate AFTER initialization
    HAL_Delay(2000);
    gyro_z_bias = calibrate_gyro_bias();
    // sprintf(buf, "Bias: %.2f", gyro_z_bias);
    // OLED_ShowString(0, 20, (uint8_t *)buf);
  } else {
    sprintf(buf, "ICM FAIL");
  }
  OLED_ShowString(0, 10, (uint8_t *)buf);
  OLED_Refresh_Gram();
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //  OLED_ShowString(15, 40, "Press User");    // show message on OLED display
  //  at line 40) OLED_ShowString(0, 50, "button to stop"); // show message on
  //  OLED display at line 50)
  OLED_Refresh_Gram();

  //  if (target_angle > 0) // Determine rotation direction)
  //    direction = 0;
  //  else
  //    direction = 1;

  start = 1;                 // do a step response upon reset and power up
  MotorDrive_enable();       // enable PWM needed to drive MotroDrive A and D
  millisOld = HAL_GetTick(); // get time value before starting - for PID

  Servo_SetAngle_Safe(0, 0);
  HAL_Delay(1000);
  OLED_Clear();

  /********************************our testing*** */

  testing();

  /****************************START************START*************START**********************
   */
  task_two();

  // uint32_t distance = HCSR04_Read();

  // sprintf(buf, "Dist: %lu cm", distance);
  // OLED_ShowString(0, 50, (uint8_t*)buf);
  // if (distance <= 15) {
  //     Motor_stop();

  //     // Optional: buzzer/LED feedback
  //     HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
  // } else {
  //     // Example: keep moving forward at pwm=3000
  //     //Motor_forward(3000);    //to make the robot move forward when the
  //     object is removed from infront of US

  //     HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
  // }

  /******** IR Sensor Code *******************************************/
  /******** IR Sensor Code END ************************/

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // while

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_6; // just as placeholder during set up
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // In MX_TIM2_Init() and MX_TIM5_Init():
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Instead of TIM_ENCODERMODE_TI1
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // In MX_TIM2_Init() and MX_TIM5_Init():
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Instead of TIM_ENCODERMODE_TI1
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED4_Pin | OLED3_Pin | OLED2_Pin | OLED1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin | LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED4_Pin OLED3_Pin OLED2_Pin OLED1_Pin */
  GPIO_InitStruct.Pin = OLED4_Pin | OLED3_Pin | OLED2_Pin | OLED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin | LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_PB_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin | IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // --- Ultrasonic HCSR04 Pins ---
  // Trig -> PB14

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Echo -> PC9
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //  GPIO_InitStruct.Pin = GPIO_PIN_15;
  //  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  //  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void OLED_show(void *argument, int y, int x) // display message on OLED panel
{
  //uint8_t hello[20]="Hello World";
  OLED_Init();
  OLED_Display_On();
//	OLED_ShowString(10,10,argument);
  OLED_ShowString(y, x, argument);
  OLED_Refresh_Gram();
}
*/

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
