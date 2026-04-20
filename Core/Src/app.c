/**
 * app.c — STM32F746G-DISCO Terrain Bot Main Application
 *
 * ── Hardware (Arduino header only) ──────────────────────────────────
 *
 *   LEFT  BTS7960 (#1 → FL+RL motors):
 *     RPWM ← A4 (PF7  TIM11_CH1)   forward PWM
 *     LPWM ← A5 (PF6  TIM10_CH1)   reverse PWM
 *     R_EN, L_EN → 3.3V (always enabled)
 *
 *   RIGHT BTS7960 (#2 → FR+RR motors):
 *     RPWM ← A2 (PF9  TIM14_CH1)   forward PWM
 *     LPWM ← A3 (PF8  TIM13_CH1)   reverse PWM
 *     R_EN, L_EN → 3.3V (always enabled)
 *
 *   ENCODERS (1 pin per motor, interrupt on CH_A rising edge):
 *     FL → A0  (PA0)  EXTI0
 *     FR → D3  (PB4)  EXTI4
 *     RL → D9  (PA15) EXTI15
 *     RR → D2  (PG6)  EXTI6
 *
 *   IMU ICM-20948:
 *     SCL → D15 (PB8)  I2C1_SCL
 *     SDA → D14 (PB9)  I2C1_SDA
 *     Connect via CN2 connector (8-pin I2C extension)
 *
 *   UART to Raspberry Pi:
 *     TX  → D1  (PC6)  USART6_TX → white wire on TTL cable
 *     RX  → D0  (PC7)  USART6_RX → green wire on TTL cable
 *     GND → GND        → black wire on TTL cable
 *     Baud: 2,000,000
 *
 * ── CubeMX peripherals needed ────────────────────────────────────────
 *   I2C1    : PB8(SCL) PB9(SDA)  400kHz Fast Mode
 *   USART6  : PC6(TX)  PC7(RX)   2000000 baud 8N1
 *   TIM10   : PWM CH1  PF6(A5)   PSC=0 ARR=10799 (20kHz)
 *   TIM11   : PWM CH1  PF7(A4)   PSC=0 ARR=10799
 *   TIM13   : PWM CH1  PF8(A3)   PSC=0 ARR=10799
 *   TIM14   : PWM CH1  PF9(A2)   PSC=0 ARR=10799
 *   PA0     : GPIO_EXTI0   (FL encoder)  Rising edge, Pull-Down
 *   PB4     : GPIO_EXTI4   (FR encoder)  Rising edge, Pull-Down
 *   PA15    : GPIO_EXTI15  (RL encoder)  Rising edge, Pull-Down
 *   PG6     : GPIO_EXTI6   (RR encoder)  Rising edge, Pull-Down
 *   Enable NVIC for EXTI0, EXTI4, EXTI9_5, EXTI15_10
 *
 * ── Drive topology ───────────────────────────────────────────────────
 *   Skid-steer (tank drive):
 *   v_left  = linear.x(mm/s) - angular.z(rad/s) × wheelbase/2
 *   v_right = linear.x(mm/s) + angular.z(rad/s) × wheelbase/2
 *
 * ── Safety layer (paper §III) ────────────────────────────────────────
 *   Risk R from IMU pitch/roll/shake → speed_scale = (1-R)²
 *   Hard E-stop: pitch>30° or roll>40°
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
extern I2C_HandleTypeDef  hi2c1;    /* ICM-20948: SCL=PB8 SDA=PB9   */
extern TIM_HandleTypeDef  htim10;   /* LEFT  LPWM: A5 (PF6)         */
extern TIM_HandleTypeDef  htim11;   /* LEFT  RPWM: A4 (PF7)         */
extern TIM_HandleTypeDef  htim13;   /* RIGHT LPWM: A3 (PF8)         */
extern TIM_HandleTypeDef  htim14;   /* RIGHT RPWM: A2 (PF9)         */
extern UART_HandleTypeDef huart6;   /* RPi UART: TX=PC6 RX=PC7      */

/* ── Timing ──────────────────────────────────────────────────────── */
#define IMU_PERIOD_MS       10    /* 100 Hz IMU + Madgwick + safety  */
#define ENC_PERIOD_MS      100    /*  10 Hz encoder speed update     */
#define PID_PERIOD_MS       10    /* 100 Hz motor PID                */
#define IMU_PUB_PERIOD_MS   10    /* 100 Hz /imu/data publish        */
#define ENC_PUB_PERIOD_MS   20    /*  50 Hz /wheel_velocity publish  */
#define LCD_PERIOD_MS      100    /*  10 Hz LCD refresh              */
#define CALIB_SAMPLES      200    /*   ~2 s gyro bias calibration    */
#define MADGWICK_BETA      0.033f

/* ── Safety thresholds (paper §III) ─────────────────────────────── */
#define SAFETY_W_PITCH      0.7f
#define SAFETY_W_ROLL       0.3f
#define SAFETY_THETA_SAFE   10.0f
#define SAFETY_THETA_CRIT   30.0f
#define SAFETY_W_INCL       0.7f
#define SAFETY_W_SHAKE      0.3f
#define SAFETY_LPF_BETA     0.90f
#define SAFETY_ESTOP_PITCH  30.0f
#define SAFETY_ESTOP_ROLL   40.0f
#define SAFETY_FLIP_RATE    (30.0f * 0.017453293f)
#define SAFETY_FLIP_PITCH   (15.0f * 0.017453293f)

/* ── cmd_vel watchdog ────────────────────────────────────────────── */
#define CMD_VEL_TIMEOUT_MS  500

/* ── Motor PI gains ──────────────────────────────────────────────── */
#define MOTOR_KP   0.5f
#define MOTOR_KI   0.1f

/* ── IMU noise covariance ────────────────────────────────────────── */
#define GY_COV  (0.00262f * 0.00262f)
#define AC_COV  (0.02256f * 0.02256f)
#define OR_COV   0.0001f

#define UROS_OK(fn) do { if((fn) != RCL_RET_OK) Error_Handler(); } while(0)

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
static float    g_target_mmps[SIDE_NUM] = {0};
static float    g_pid_integral[SIDE_NUM] = {0};
static uint32_t g_cmd_last_ms = 0;

/* ── Side velocity feedback (average of 2 encoders per side) ─────── */
static float side_vel_mmps(Motor_Side side)
{
    if (side == SIDE_LEFT)
        return (Encoder_VelMmps(ENC_FL) + Encoder_VelMmps(ENC_RL)) * 0.5f;
    else
        return (Encoder_VelMmps(ENC_FR) + Encoder_VelMmps(ENC_RR)) * 0.5f;
}

/* ═══════════════════════════════════════════════════════════════════
 *  cmd_vel callback
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

    /* Hard E-stop first */
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

    /* Eq.8: combined */
    float r_raw = SAFETY_W_INCL * r_incl + SAFETY_W_SHAKE * r_shake;
    if (r_raw > 1.0f) r_raw = 1.0f;

    /* Eq.10: LPF */
    g_risk_filtered = SAFETY_LPF_BETA * g_risk_filtered +
                      (1.0f - SAFETY_LPF_BETA) * r_raw;

    /* Eq.11: scale */
    float s = 1.0f - g_risk_filtered;
    g_safety_scale = s * s;
}

/* ═══════════════════════════════════════════════════════════════════
 *  Motor PI — 100 Hz
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

    /* Message buffers */
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
        tick_buf[i] = (int32_t)Encoder_Ticks((uint8_t)i);
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

    /* 3. Encoders (interrupt-based) */
    Encoder_Init();

    /* 4. BTS7960 motor drivers */
    Motor_Init(&htim10, &htim11, &htim13, &htim14);

    /* 5. Madgwick AHRS */
    Madgwick_Init(&ahrs, MADGWICK_BETA);

    /* 6. micro-ROS — blocks until RPi agent responds */
    uros_setup();

    /* 7. Gyro calibration (~2s) — keep robot still! */
    memset(&calib, 0, sizeof(calib));
    while (!calib.done) {
        LCD_Display_Update(&ahrs, 0, 0.0f);
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
    uint32_t t_pid     = HAL_GetTick();
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

        /* Encoder speed update @ 10 Hz */
        if (now - t_enc >= ENC_PERIOD_MS) {
            t_enc = now;
            Encoder_Update();
        }

        /* Motor PID @ 100 Hz */
        if (now - t_pid >= PID_PERIOD_MS) {
            float dt = (float)(now - t_pid) * 0.001f;
            t_pid = now;

            if (now - g_cmd_last_ms > CMD_VEL_TIMEOUT_MS) {
                /* Watchdog: stop if no cmd_vel */
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
        if (now - t_lcd >= LCD_PERIOD_MS) {
            t_lcd = now;
            LCD_Display_Update(&ahrs, 1, g_risk_filtered);
        }

        /* Process /cmd_vel */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  EXTI GPIO Callback — called by HAL from EXTI ISR
 *  Add this to stm32f7xx_it.c or keep here if using weak override
 * ═══════════════════════════════════════════════════════════════════ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_0:   Encoder_PulseISR(ENC_FL); break;  /* A0  PA0  */
        case GPIO_PIN_4:   Encoder_PulseISR(ENC_FR); break;  /* D3  PB4  */
        case GPIO_PIN_15:  Encoder_PulseISR(ENC_RL); break;  /* D9  PA15 */
        case GPIO_PIN_6:   Encoder_PulseISR(ENC_RR); break;  /* D2  PG6  */
        default: break;
    }
}
