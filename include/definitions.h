/*******************************************************************************
** I/O pin assignment
*******************************************************************************/

#define FRONT_RIGHT_ENCODER_A 39
#define FRONT_RIGHT_ENCODER_B 36
#define FRONT_LEFT_ENCODER_A 33
#define FRONT_LEFT_ENCODER_B 32

// #define REAR_RIGHT_ENCODER_A 35
// #define REAR_RIGHT_ENCODER_B 34
// #define REAR_LEFT_ENCODER_A 26
// #define REAR_LEFT_ENCODER_B 25

#define FRONT_LEFT_PWM_FORWARD  23
#define FRONT_LEFT_PWM_REVERSE  22
#define FRONT_RIGHT_PWM_FORWARD 16
#define FRONT_RIGHT_PWM_REVERSE 13

// #define REAR_LEFT_PWM_FORWARD  18
// #define REAR_LEFT_PWM_REVERSE  17
// #define REAR_RIGHT_PWM_FORWARD 21
// #define REAR_RIGHT_PWM_REVERSE 19

#define IMU_SDA 27
#define IMU_SCL 4

/*******************************************************************************
** Timeout Values
*******************************************************************************/

#define MOTOR_TIMEOUT   70   // milliseconds
#define ODOM_TIMEOUT    10   // milliseconds
#define IMU_TIMEOUT     10   // milliseconds
#define CMDVEL_TIMEOUT  10   // milliseconds

/*******************************************************************************
** PWM Related parameters
*******************************************************************************/

#define WHEEL_RADIUS                0.21   // meters
#define WHEEL_TRACK                 0.85   // meters
#define WHEEL_BASE                  0.75   // meters
#define WHEEL_ALLOWABLE_VARIATION   0.02   // percentage

/*******************************************************************************
** Encoder related parameters
*******************************************************************************/

#define ENCODER_REDUCTION 1
#define ENCODER_PULSES_PER_ROTATION 2400	// Phase count x pulses per rotation in spec sheet

/*******************************************************************************
** PID related parameters
*******************************************************************************/

#define K_P 1
#define K_I 0.5
#define K_D 0
#define ITERATIONS_PER_LOOP 10
#define REAL_WORLD_FACTOR 1

/*******************************************************************************
** constants for ros messages
*******************************************************************************/

#define ODOM_COVARIANCE 0.005
#define IMU_COVARIANCE 0.0
#define EARTH_GRAVITY_MS2 9.80665  // m/s2

/*******************************************************************************
** status checking
*******************************************************************************/

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
