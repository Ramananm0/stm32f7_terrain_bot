/**
 * app.c  —  STM32F746G-DISCO  Main Application
 *
 * ── Hardware ─────────────────────────────────────────────────────────
 *   ICM-20948 IMU      : I2C1  SCL=PB8  SDA=PB9          @ 102 Hz
 *   BTS7960 #1 (LEFT)  : TIM12 CH1(PH6/D6)=RPWM  CH2(PB15/D11)=LPWM
 *                        Controls FL + RL motors in parallel
 *   BTS7960 #2 (RIGHT) : TIM1  CH1(PA8/D10)=RPWM
 *                        TIM13 CH1(PA6/Morpho)=LPWM
 *                        Controls FR + RR motors in parallel
 *   Encoder FL         : TIM5  PA0(CH1) + PA1(CH2)
 *   Encoder FR         : TIM2  PA15(CH1) + PB3(CH2)
 *   Encoder RL         : TIM4  PB6(CH1) + PB7(CH2)
 *   Encoder RR         : TIM3  PB4(CH1) + PB5(CH2)
 *   micro-ROS (RPi)    : USART6 PG14(TX) PG9(RX) @ 2 Mbaud
 *   LCD display        : LTDC built-in 480×272
 *
 * ── Drive mode ───────────────────────────────────────────────────────
 *   Skid-steer (tank drive):
 *     Left  side (FL+RL) ← same velocity target
 *     Right side (FR+RR) ← same velocity target
 *   2WD now:  1 motor per BTS (FL on BTS#1, FR on BTS#2)
 *   4WD later: add RL to BTS#1 output, RR to BTS#2 output — no code change
 *
 * ── Velocity feedback ────────────────────────────────────────────────
 *   Left  side velocity  = average of FL + RL encoder readings
 *   Right side velocity  = average of FR + RR encoder readings
 *   (In 2WD: only FL/FR encoders active — RL/RR read 0, average still works)
 *
 * ── Safety layer (paper §III) ────────────────────────────────────────
 *   IMU inclination + shake → risk R → speed_scale = (1−R)²
 *   Hard E-stop: pitch>30° or roll>40°
 *
 * ── micro-ROS topics ─────────────────────────────────────────────────
 *   Pub: /imu/data          sensor_msgs/Imu           @ 100 Hz
 *   Pub: /wheel_ticks       std_msgs/Int32MultiArray  @  50 Hz
 *   Pub: /wheel_velocity    std_msgs/Float32MultiArray@  50 Hz
 *   Sub: /cmd_vel           geometry_msgs/Twist
 */

#include "app.h"
#include "icm20948.h"
#include "encoder.h"
#include "motor.h"
#include "madgwick.h"
#include "microros_transport.h"
#include "lcd_display.h"
#include "main.h"

/* micro-ROS */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <uxr/client/transport.h>

#include <math.h>
#include <string.h>

/* ── HAL handles (declared by CubeMX in main.c) ─────────────────── */
extern I2C_HandleTypeDef  hi2c1;    /* ICM-20948 : SCL=PB8 SDA=PB9       */
extern TIM_HandleTypeDef  htim5;    /* Encoder FL: PA0(CH1) + PA1(CH2)   */
extern TIM_HandleTypeDef  htim2;    /* Encoder FR: PA15(CH1) + PB3(CH2)  */
extern TIM_HandleTypeDef  htim4;    /* Encoder RL: PB6(CH1) + PB7(CH2)   */
extern TIM_HandleTypeDef  htim3;    /* Encoder RR: PB4(CH1) + PB5(CH2)   */
extern TIM_HandleTypeDef  htim12;   /* LEFT  BTS7960: RPWM=CH1 LPWM=CH2  */
extern TIM_HandleTypeDef  htim1;    /* RIGHT BTS7960: RPWM=CH1            */
extern TIM_HandleTypeDef  htim13;   /* RIGHT BTS7960: LPWM=CH1            */
extern UART_HandleTypeDef huart6;   /* micro-ROS: TX=PG14  RX=PG9  2 Mbaud */

/* ── Loop timing ─────────────────────────────────────────────────── */
#define IMU_PERIOD_MS        10    /* 100 Hz — IMU read + Madgwick + safety */
#define ENC_PERIOD_MS        10    /* 100 Hz — encoder read + motor PID     */
#define IMU_PUB_PERIOD_MS    10    /* 100 Hz — publish /imu/data            */
#define ENC_PUB_PERIOD_MS    20    /*  50 Hz — publish /wheel_ticks + vel   */
#define LCD_PUB_PERIOD_MS   100    /*  10 Hz — LCD refresh                  */
#define CALIB_SAMPLES       200    /*   ~2 s  — gyro bias calibration       */
#define MADGWICK_BETA       0.033f

/* ── Safety thresholds (paper §III) ─────────────────────────────── */
#define SAFETY_W_PITCH       0.7f
#define SAFETY_W_ROLL        0.3f
#define SAFETY_THETA_SAFE    10.0f
#define SAFETY_THETA_CRIT    30.0f
#define SAFETY_W_INCL        0.7f
#define SAFETY_W_SHAKE       0.3f
#define SAFETY_LPF_BETA      0.90f
#define SAFETY_ESTOP_PITCH   30.0f
#define SAFETY_ESTOP_ROLL    40.0f
#define SAFETY_FLIP_RATE     (30.0f * 0.017453293f)
#define SAFETY_FLIP_PITCH    (15.0f * 0.017453293f)

/* ── cmd_vel watchdog ────────────────────────────────────────────── */
#define CMD_VEL_TIMEOUT_MS   500

/* ── Motor PI gains ──────────────────────────────────────────────── */
#define MOTOR_KP     0.5f
#define MOTOR_KI     0.1f

/* ── IMU noise covariance (ICM-20948 datasheet) ─────────────────── */
#define GY_COV   (0.00262f * 0.00262f)
#define AC_COV   (0.02256f * 0.02256f)
#define OR_COV    0.0001f

#define UROS_OK(fn)  do { if ((fn) != RCL_RET_OK) Error_Handler(); } while(0)

/* ── micro-ROS objects ───────────────────────────────────────────── */
static rcl_allocator_t  alloc;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_publisher_t    pub_imu;
static rcl_publisher_t    pub_ticks;
static rcl_publisher_t    pub_vel;
static rcl_subscription_t sub_cmd_vel;

static sensor_msgs__msg__Imu              msg_imu;
static std_msgs__msg__Int32MultiArray     msg_ticks;
static std_msgs__msg__Float32MultiArray   msg_vel;
static geometry_msgs__msg__Twist          msg_cmd_vel;

static int32_t  tick_buf[ENC_NUM];
static float    vel_buf [ENC_NUM];
static char     frame_id_buf[] = "imu_link";

/* ── Sensor state ────────────────────────────────────────────────── */
static ICM20948_Data  imu;
static ICM20948_Calib calib;
static Madgwick_t     ahrs;

/* ── Safety state ────────────────────────────────────────────────── */
static float g_risk_filtered = 0.0f;
static float g_safety_scale  = 1.0f;

/* ── Motor / cmd_vel state ───────────────────────────────────────── */
/* Per-side: [0]=LEFT [1]=RIGHT */
static float    g_target_mmps[SIDE_NUM] = {0};
static float    g_pid_integral[SIDE_NUM] = {0};
static uint32_t g_cmd_last_ms = 0;

/* ── Helper: average encoder velocity for one side ──────────────── */
static float side_vel_mmps(Motor_Side side)
{
    if (side == SIDE_LEFT)
        return (Encoder_VelMmps(ENC_FL) + Encoder_VelMmps(ENC_RL)) * 0.5f;
    else
        return (Encoder_VelMmps(ENC_FR) + Encoder_VelMmps(ENC_RR)) * 0.5f;
}

/* ═══════════════════════════════════════════════════════════════════
 *  cmd_vel callback — differential drive kinematics → per-side targets
 * ═══════════════════════════════════════════════════════════════════ */
static void cmd_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *tw =
        (const geometry_msgs__msg__Twist *)msg_in;

    float vx = (float)tw->linear.x  * 1000.0f;   /* m/s → mm/s */
    float wz = (float)tw->angular.z;              /* rad/s      */
    float hL = ENC_WHEELBASE_MM * 0.5f;

    g_target_mmps[SIDE_LEFT]  = vx - wz * hL;
    g_target_mmps[SIDE_RIGHT] = vx + wz * hL;
    g_cmd_last_ms = HAL_GetTick();
}

/* ═══════════════════════════════════════════════════════════════════
 *  Safety layer — paper §III
 * ═══════════════════════════════════════════════════════════════════ */
static void update_safety(void)
{
    float pitch_deg = ahrs.pitch * (180.0f / 3.14159265f);
    float roll_deg  = ahrs.roll  * (180.0f / 3.14159265f);
    float pitch_abs = fabsf(pitch_deg);
    float roll_abs  = fabsf(roll_deg);

    /* Hard E-stop check first (minimum latency) */
    float pitch_rate = fabsf(imu.gy - calib.gy_bias);
    if (pitch_abs >= SAFETY_ESTOP_PITCH ||
        roll_abs  >= SAFETY_ESTOP_ROLL  ||
        (pitch_rate > SAFETY_FLIP_RATE && ahrs.pitch > SAFETY_FLIP_PITCH))
    {
        g_risk_filtered = 1.0f;
        g_safety_scale  = 0.0f;
        Motor_StopAll();
        return;
    }

    /* Eq.5: weighted inclination */
    float theta_w = SAFETY_W_PITCH * pitch_abs + SAFETY_W_ROLL * roll_abs;

    /* Eq.9: normalize */
    float r_incl = (theta_w - SAFETY_THETA_SAFE) /
                   (SAFETY_THETA_CRIT - SAFETY_THETA_SAFE);
    if (r_incl < 0.0f) r_incl = 0.0f;
    if (r_incl > 1.0f) r_incl = 1.0f;

    /* Eq.6-7: terrain shake */
    float a_mag = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
    float g_ref = (calib.gravity > 9.0f) ? calib.gravity : 9.80665f;
    float r_shake = fabsf(a_mag - g_ref) / g_ref;
    if (r_shake > 1.0f) r_shake = 1.0f;

    /* Eq.8: combined risk */
    float r_raw = SAFETY_W_INCL * r_incl + SAFETY_W_SHAKE * r_shake;
    if (r_raw > 1.0f) r_raw = 1.0f;

    /* Eq.10: low-pass filter */
    g_risk_filtered = SAFETY_LPF_BETA * g_risk_filtered +
                      (1.0f - SAFETY_LPF_BETA) * r_raw;

    /* Eq.11: nonlinear scale */
    float s = 1.0f - g_risk_filtered;
    g_safety_scale = s * s;
}

/* ═══════════════════════════════════════════════════════════════════
 *  Motor PI — runs per side at 100 Hz
 * ═══════════════════════════════════════════════════════════════════ */
static void motor_pid_update(float dt_s)
{
    for (int s = 0; s < SIDE_NUM; s++) {
        float target   = g_target_mmps[s] * g_safety_scale;
        float measured = side_vel_mmps((Motor_Side)s);
        float error    = target - measured;

        g_pid_integral[s] += error * dt_s;
        if (g_pid_integral[s] >  MOTOR_ILIMIT) g_pid_integral[s] =  MOTOR_ILIMIT;
        if (g_pid_integral[s] < -MOTOR_ILIMIT) g_pid_integral[s] = -MOTOR_ILIMIT;

        float cmd = target
                  + MOTOR_KP * error
                  + MOTOR_KI * g_pid_integral[s];

        Motor_SetSide((Motor_Side)s, cmd);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  micro-ROS setup
 * ═══════════════════════════════════════════════════════════════════ */
static void uros_setup(void)
{
    rmw_uros_set_custom_transport(
        true, NULL,
        transport_open, transport_close,
        transport_write, transport_read);

    alloc = rcutils_get_default_allocator();

    while (rmw_uros_ping_agent(200, 10) != RMW_RET_OK)
        HAL_Delay(200);

    UROS_OK(rclc_support_init(&support, 0, NULL, &alloc));
    UROS_OK(rclc_node_init_default(&node, "stm32f7_node", "terrain_bot", &support));

    UROS_OK(rclc_publisher_init_best_effort(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
    UROS_OK(rclc_publisher_init_best_effort(
        &pub_ticks, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/wheel_ticks"));
    UROS_OK(rclc_publisher_init_best_effort(
        &pub_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_velocity"));

    UROS_OK(rclc_subscription_init_best_effort(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

    UROS_OK(rclc_executor_init(&executor, &support.context, 1, &alloc));
    UROS_OK(rclc_executor_add_subscription(
        &executor, &sub_cmd_vel, &msg_cmd_vel,
        cmd_vel_callback, ON_NEW_DATA));

    /* Message buffer wiring */
    msg_ticks.data.data     = tick_buf;
    msg_ticks.data.size     = ENC_NUM;
    msg_ticks.data.capacity = ENC_NUM;
    msg_vel.data.data       = vel_buf;
    msg_vel.data.size       = ENC_NUM;
    msg_vel.data.capacity   = ENC_NUM;

    msg_imu.header.frame_id.data     = frame_id_buf;
    msg_imu.header.frame_id.size     = 8;
    msg_imu.header.frame_id.capacity = 9;

    memset(msg_imu.orientation_covariance,         0, sizeof(double)*9);
    memset(msg_imu.angular_velocity_covariance,    0, sizeof(double)*9);
    memset(msg_imu.linear_acceleration_covariance, 0, sizeof(double)*9);
    msg_imu.orientation_covariance[0]         = OR_COV;
    msg_imu.orientation_covariance[4]         = OR_COV;
    msg_imu.orientation_covariance[8]         = OR_COV;
    msg_imu.angular_velocity_covariance[0]    = GY_COV;
    msg_imu.angular_velocity_covariance[4]    = GY_COV;
    msg_imu.angular_velocity_covariance[8]    = GY_COV;
    msg_imu.linear_acceleration_covariance[0] = AC_COV;
    msg_imu.linear_acceleration_covariance[4] = AC_COV;
    msg_imu.linear_acceleration_covariance[8] = AC_COV;
}

static void publish_imu(void)
{
    uint64_t ns = (uint64_t)HAL_GetTick() * 1000000ULL;
    msg_imu.header.stamp.sec     = (int32_t)(ns / 1000000000ULL);
    msg_imu.header.stamp.nanosec = (uint32_t)(ns % 1000000000ULL);
    msg_imu.orientation.w = ahrs.q0;
    msg_imu.orientation.x = ahrs.q1;
    msg_imu.orientation.y = ahrs.q2;
    msg_imu.orientation.z = ahrs.q3;
    msg_imu.angular_velocity.x = imu.gx - calib.gx_bias;
    msg_imu.angular_velocity.y = imu.gy - calib.gy_bias;
    msg_imu.angular_velocity.z = imu.gz - calib.gz_bias;
    msg_imu.linear_acceleration.x = imu.ax;
    msg_imu.linear_acceleration.y = imu.ay;
    msg_imu.linear_acceleration.z = imu.az;
    rcl_publish(&pub_imu, &msg_imu, NULL);
}

static void publish_encoders(void)
{
    for (int i = 0; i < ENC_NUM; i++) {
        tick_buf[i] = Encoder_Ticks((uint8_t)i);
        vel_buf[i]  = Encoder_VelMmps((uint8_t)i);
    }
    rcl_publish(&pub_ticks, &msg_ticks, NULL);
    rcl_publish(&pub_vel,   &msg_vel,   NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 *  App entry point — never returns
 * ═══════════════════════════════════════════════════════════════════ */
void App_Run(void)
{
    /* 1. LCD */
    LCD_Display_Init();

    /* 2. ICM-20948 IMU */
    if (ICM20948_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    /* 3. All 4 encoders */
    Encoder_Init(&htim5, &htim2, &htim4, &htim3);

    /* 4. BTS7960 motor drivers */
    Motor_Init(&htim12, &htim1, &htim13);

    /* 5. Madgwick AHRS filter */
    Madgwick_Init(&ahrs, MADGWICK_BETA);

    /* 6. micro-ROS — blocks until Raspberry Pi agent responds */
    uros_setup();

    /* 7. Gyro bias calibration (~2 s) — keep robot still */
    memset(&calib, 0, sizeof(calib));
    while (!calib.done) {
        LCD_Display_Update(&ahrs, 0);
        if (ICM20948_Read(&hi2c1, &imu) == HAL_OK)
            ICM20948_Calibrate(&imu, &calib, CALIB_SAMPLES);
        HAL_Delay(IMU_PERIOD_MS);
    }

    /* 8. Seed Madgwick with accel-derived roll/pitch */
    {
        float ax = imu.ax, ay = imu.ay, az = imu.az;
        float rn = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= rn; ay *= rn; az *= rn;
        float r = atan2f(ay, az);
        float p = atan2f(-ax, sqrtf(ay*ay + az*az));
        ahrs.q0 = cosf(r/2) * cosf(p/2);
        ahrs.q1 = sinf(r/2) * cosf(p/2);
        ahrs.q2 = cosf(r/2) * sinf(p/2);
        ahrs.q3 = -sinf(r/2) * sinf(p/2);
        ahrs.initialised = true;
    }

    /* 9. Arm cmd_vel watchdog */
    g_cmd_last_ms = HAL_GetTick();

    /* 10. Main loop */
    uint32_t t_imu     = HAL_GetTick();
    uint32_t t_enc     = HAL_GetTick();
    uint32_t t_pub_imu = HAL_GetTick();
    uint32_t t_pub_enc = HAL_GetTick();
    uint32_t t_lcd     = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();

        /* IMU + Madgwick + safety @ 100 Hz */
        if (now - t_imu >= IMU_PERIOD_MS) {
            float dt = (float)(now - t_imu) * 0.001f;
            t_imu = now;

            if (ICM20948_Read(&hi2c1, &imu) == HAL_OK) {
                float gx = imu.gx - calib.gx_bias;
                float gy = imu.gy - calib.gy_bias;
                float gz = imu.gz - calib.gz_bias;

                if (imu.mx != 0.0f || imu.my != 0.0f || imu.mz != 0.0f)
                    Madgwick_UpdateMARG(&ahrs, gx, gy, gz,
                                        imu.ax, imu.ay, imu.az,
                                        imu.mx, imu.my, imu.mz, dt);
                else
                    Madgwick_UpdateIMU(&ahrs, gx, gy, gz,
                                       imu.ax, imu.ay, imu.az, dt);
                update_safety();
            }
        }

        /* Encoders + motor PI @ 100 Hz */
        if (now - t_enc >= ENC_PERIOD_MS) {
            float dt = (float)(now - t_enc) * 0.001f;
            t_enc = now;
            Encoder_Update();

            if (now - g_cmd_last_ms > CMD_VEL_TIMEOUT_MS) {
                memset(g_target_mmps,  0, sizeof(g_target_mmps));
                memset(g_pid_integral, 0, sizeof(g_pid_integral));
                Motor_StopAll();
            } else {
                motor_pid_update(dt);
            }
        }

        /* Publish IMU @ 100 Hz */
        if (now - t_pub_imu >= IMU_PUB_PERIOD_MS) {
            t_pub_imu = now;
            publish_imu();
        }

        /* Publish encoders @ 50 Hz */
        if (now - t_pub_enc >= ENC_PUB_PERIOD_MS) {
            t_pub_enc = now;
            publish_encoders();
        }

        /* LCD @ 10 Hz */
        if (now - t_lcd >= LCD_PUB_PERIOD_MS) {
            t_lcd = now;
            LCD_Display_Update(&ahrs, 1);
        }

        /* Process incoming /cmd_vel */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
    }
}
