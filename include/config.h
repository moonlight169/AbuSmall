#define MotorPinFL_A 17
#define MotorPinFL_B 18

#define MotorPinFR_A 16
#define MotorPinFR_B 4

#define MotorPinRL_A 33
#define MotorPinRL_B 32

#define MotorPinRR_A 23
#define MotorPinRR_B 19

#define MAX_RPM 200

// #define K_P 1.5  // P constant
// #define K_I 0.25 // I constant
// #define K_D 0.4  // D constant

#define K_P 0.8  // ลด P constant สำหรับระบบที่ไม่มี encoder feedback
#define K_I 0.3  // ลด I constant
#define K_D 0.02 // ลด D constant

// #define WHEEL_DIAMETER 0.153
#define WHEEL_DIAMETER 0.1524

#define FR_WHEELS_DISTANCE 0.440
#define LR_WHEELS_DISTANCE 0.560
// #define PWM_BITS 8
#define PWM_BITS 10

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX