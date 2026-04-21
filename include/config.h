//master
#define MotorPinFL_A 17
#define MotorPinFL_B 18

#define MotorPinFR_A 16
#define MotorPinFR_B 4

#define MotorPinRL_A 33
#define MotorPinRL_B 32

#define MotorPinRR_A 23
#define MotorPinRR_B 19

//master2
#define MotorPinFL_A 23
#define MotorPinFL_B 19

#define MotorPinFR_A 16
#define MotorPinFR_B 4

#define MotorPinRL_A 17
#define MotorPinRL_B 18

#define MotorPinRR_A 33
#define MotorPinRR_B 32

// #define MotorPinFL_A 17
// #define MotorPinFL_B 18

// #define MotorPinFR_A 16
// #define MotorPinFR_B 4

// #define MotorPinRL_A 33
// #define MotorPinRL_B 32

// #define MotorPinRR_A 23
// #define MotorPinRR_B 19

//slave1
#define MotorPinS1FL_A 17
#define MotorPinS1FL_B 18

#define MotorPinS1FR_A 16
#define MotorPinS1FR_B 4

#define MotorPinS1RL_A 33
#define MotorPinS1RL_B 32

#define MotorPinS1RR_A 23
#define MotorPinS1RR_B 19

//slave2
#define MotorPinS2FL_A 17
#define MotorPinS2FL_B 18

#define MotorPinS2FR_A 16
#define MotorPinS2FR_B 4

#define MotorPinS2RL_A 33
#define MotorPinS2RL_B 32

#define MotorPinS2RR_A 23
#define MotorPinS2RR_B 19

//slave3
#define MotorPinS3FL_A 17
#define MotorPinS3FL_B 18

#define MotorPinS3FR_A 16
#define MotorPinS3FR_B 4

#define MotorPinS3RL_A 33
#define MotorPinS3RL_B 32

#define MotorPinS3RR_A 23
#define MotorPinS3RR_B 19

#define M_MAX_RPM 200
#define M2_MAX_RPM 170
#define S1_MAX_RPM 200
#define S2_MAX_RPM 200
#define S3_MAX_RPM 200

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

//master2
#define WHEEL_DIAMETER_M2 0.090
#define FR_WHEELS_DISTANCE_M2 0.550
#define LR_WHEELS_DISTANCE_M2 0.790