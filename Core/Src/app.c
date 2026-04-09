/**
 * app.c  —  STM32F746G-DISCO  Main Application
 *
 * ── What this board does ────────────────────────────────────────────
 *   1. Reads ICM-20948 (9-DOF) at 102 Hz via I2C1
 *        I2C1: SCL = PB8 (Arduino SCL pin)
 *              SDA = PB9 (Arduino SDA pin)
 *
 *   2. Runs Madgwick AHRS filter (Madgwick et al., IEEE ICORR 2011)
 *        - Full quaternion orientation (no gimbal lock)
 *        - MARG mode: fuses accel + gyro + mag → heading observable
 *        - IMU mode:  fuses accel + gyro only  (mag disturbed)
 *        - β = 0.033  (tuned for ICM-20948 @ 102 Hz)
 *
 *   3. Reads 2× RMCS-3070 encoder timers at 100 Hz
 *        LEFT  : TIM5   A0(PA0=CH1)        + A1(PA1=CH2)
 *        RIGHT : TIM2   D9(PA15=CH1)       + CN2pin15(PB3=CH2)
 *
 *   4. Publishes via micro-ROS over USART6 (D1=TX, D0=RX) at 2 Mbaud:
 *        /imu/data               sensor_msgs/Imu          @ 100 Hz
 *          ├ orientation         quaternion [w,x,y,z]    ← NEW (Madgwick)
 *          ├ angular_velocity    [gx,gy,gz] bias-removed
 *          └ linear_acceleration [ax,ay,az]
 *        /wheel_ticks            std_msgs/Int32MultiArray  @  50 Hz
 *        /wheel_velocity         std_msgs/Float32MultiArray @  50 Hz
 *
 * ── Research references ─────────────────────────────────────────────
 *   [1] Madgwick, S.O.H., et al. (2011). "Estimation of IMU and MARG
 *       orientation using a gradient descent algorithm." IEEE ICORR.
 *       doi:10.1109/ICORR.2011.5975346
 *   [2] Mahony, R., et al. (2008). "Nonlinear complementary filters on
 *       the special orthogonal group." IEEE Trans. Autom. Control 53(5).
 *
 * ── How to integrate into CubeMX project ───────────────────────────
 *   Files to add: app.c  icm20948.c  encoder.c  microros_transport.c
 *                 madgwick.c
 *   Headers:      app.h  icm20948.h  encoder.h  microros_transport.h
 *                 madgwick.h
 *   CubeMX peripherals: I2C1  USART6  TIM2  TIM5
 *   Add to bottom of main():   App_Run();
 */

#include "app.h"
#include "icm20948.h"
#include "encoder.h"
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
#include <uxr/client/transport.h>

#include <math.h>
#include <string.h>

/* ── HAL handles (declared in CubeMX-generated main.c) ──────────── */
extern I2C_HandleTypeDef  hi2c1;    /* SCL=PB8 (Arduino SCL)  SDA=PB9 (Arduino SDA) */
extern TIM_HandleTypeDef  htim5;    /* LEFT  encoder: A0(PA0=CH1) + A1(PA1=CH2)      */
extern TIM_HandleTypeDef  htim2;    /* RIGHT encoder: D9(PA15=CH1) + CN2p15(PB3=CH2) */
extern UART_HandleTypeDef huart6;   /* D1(PC6=TX)  D0(PC7=RX)  2 Mbaud               */

/* ── Timing ─────────────────────────────────────────────────────── */
#define LCD_PUB_PERIOD_MS   100   /*  10 Hz  — LCD refresh            */
#define IMU_PERIOD_MS       10    /* 100 Hz  — Madgwick update        */
#define ENC_PERIOD_MS       10    /* 100 Hz  — encoder polling        */
#define IMU_PUB_PERIOD_MS   10    /* 100 Hz  — /imu/data publish      */
#define ENC_PUB_PERIOD_MS   20    /*  50 Hz  — /wheel_ticks publish   */
#define CALIB_SAMPLES      200    /*  ~2 s at 100 Hz — gyro bias      */

/* Madgwick β for ICM-20948 @ 102 Hz
 * β = sqrt(3/4) × ω_err,  ω_err = 0.015dps/√Hz × √100 × π/180
 * β ≈ 0.0026 × 0.866 ≈ 0.033  (per Madgwick 2011, §IV-C)          */
#define MADGWICK_BETA       0.033f

/* ── micro-ROS objects ──────────────────────────────────────────── */
static rcl_allocator_t  alloc;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_publisher_t  pub_imu;
static rcl_publisher_t  pub_ticks;
static rcl_publisher_t  pub_vel;

static sensor_msgs__msg__Imu               msg_imu;
static std_msgs__msg__Int32MultiArray      msg_ticks;
static std_msgs__msg__Float32MultiArray    msg_vel;

static int32_t  tick_buf[ENC_NUM];
static float    vel_buf [ENC_NUM];
static char     frame_id_buf[] = "imu_link";

/* ── Sensor state ───────────────────────────────────────────────── */
static ICM20948_Data  imu;
static ICM20948_Calib calib;
static Madgwick_t     ahrs;   /* Madgwick filter — replaces simple CF */

/* ── Noise covariance constants (ICM-20948 datasheet) ──────────── */
/* Gyro  : 0.015 dps/√Hz @ 100 Hz → σ = 0.015 × √100 × π/180 = 0.00262 rad/s */
/* Accel : 230 µg/√Hz   @ 100 Hz → σ = 230e-6 × √100 × 9.807 = 0.02256 m/s²  */
#define GY_SIGMA  0.00262f
#define AC_SIGMA  0.02256f
#define GY_COV   (GY_SIGMA * GY_SIGMA)
#define AC_COV   (AC_SIGMA * AC_SIGMA)
/* Orientation from Madgwick — estimated covariance based on β tuning */
#define OR_COV   0.0001f

#define UROS_OK(fn) do { if ((fn) != RCL_RET_OK) Error_Handler(); } while(0)

/* ── micro-ROS setup ────────────────────────────────────────────── */
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

    UROS_OK(rclc_executor_init(&executor, &support.context, 1, &alloc));

    msg_ticks.data.data     = tick_buf;
    msg_ticks.data.size     = ENC_NUM;
    msg_ticks.data.capacity = ENC_NUM;
    msg_vel.data.data       = vel_buf;
    msg_vel.data.size       = ENC_NUM;
    msg_vel.data.capacity   = ENC_NUM;

    msg_imu.header.frame_id.data     = frame_id_buf;
    msg_imu.header.frame_id.size     = 8;
    msg_imu.header.frame_id.capacity = 9;

    /* Covariance matrices — set once, reused every publish */
    memset(msg_imu.orientation_covariance,           0, sizeof(double)*9);
    memset(msg_imu.angular_velocity_covariance,      0, sizeof(double)*9);
    memset(msg_imu.linear_acceleration_covariance,   0, sizeof(double)*9);
    /* Diagonal entries only (off-diagonal are negligible) */
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

/* ── Publish /imu/data ──────────────────────────────────────────── */
static void publish_imu(void)
{
    uint64_t ns = (uint64_t)HAL_GetTick() * 1000000ULL;
    msg_imu.header.stamp.sec     = (int32_t)(ns / 1000000000ULL);
    msg_imu.header.stamp.nanosec = (uint32_t)(ns % 1000000000ULL);

    /* ── Orientation — full quaternion from Madgwick AHRS ── */
    msg_imu.orientation.w = ahrs.q0;
    msg_imu.orientation.x = ahrs.q1;
    msg_imu.orientation.y = ahrs.q2;
    msg_imu.orientation.z = ahrs.q3;

    /* ── Angular velocity — bias removed ── */
    msg_imu.angular_velocity.x = imu.gx - calib.gx_bias;
    msg_imu.angular_velocity.y = imu.gy - calib.gy_bias;
    msg_imu.angular_velocity.z = imu.gz - calib.gz_bias;

    /* ── Linear acceleration — raw (gravity included, per REP-145) ── */
    msg_imu.linear_acceleration.x = imu.ax;
    msg_imu.linear_acceleration.y = imu.ay;
    msg_imu.linear_acceleration.z = imu.az;

    rcl_publish(&pub_imu, &msg_imu, NULL);
}

/* ── Publish encoders ───────────────────────────────────────────── */
static void publish_encoders(void)
{
    tick_buf[ENC_LEFT]  = Encoder_Ticks(ENC_LEFT);
    tick_buf[ENC_RIGHT] = Encoder_Ticks(ENC_RIGHT);
    vel_buf[ENC_LEFT]   = Encoder_VelMmps(ENC_LEFT);
    vel_buf[ENC_RIGHT]  = Encoder_VelMmps(ENC_RIGHT);
    rcl_publish(&pub_ticks, &msg_ticks, NULL);
    rcl_publish(&pub_vel,   &msg_vel,   NULL);
}

/* ── App entry point ────────────────────────────────────────────── */
void App_Run(void)
{
    /* 1. Init LCD */
    LCD_Display_Init();

    /* 2. Init ICM-20948 */
    if (ICM20948_Init(&hi2c1) != HAL_OK)
        Error_Handler();   /* check wiring — see PIN_CONNECTIONS.txt */

    /* 3. Init encoders */
    Encoder_Init(&htim5, &htim2);

    /* 4. Init Madgwick filter  (β=0.033 per Madgwick 2011 §IV-C) */
    Madgwick_Init(&ahrs, MADGWICK_BETA);

    /* 5. Init micro-ROS */
    uros_setup();

    /* 6. Gyro bias calibration — keep robot completely still */
    memset(&calib, 0, sizeof(calib));
    while (!calib.done) {
        LCD_Display_Update(&ahrs, 0);   /* show CALIB status on LCD */
        if (ICM20948_Read(&hi2c1, &imu) == HAL_OK)
            ICM20948_Calibrate(&imu, &calib, CALIB_SAMPLES);
        HAL_Delay(IMU_PERIOD_MS);
    }

    /* 6. Seed Madgwick with accel-derived orientation */
    {
        float ax = imu.ax, ay = imu.ay, az = imu.az;
        float rn = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= rn; ay *= rn; az *= rn;
        /* Approximate initial roll/pitch from gravity vector */
        float r = atan2f(ay, az);
        float p = atan2f(-ax, sqrtf(ay*ay + az*az));
        ahrs.q0 = cosf(r/2)*cosf(p/2);
        ahrs.q1 = sinf(r/2)*cosf(p/2);
        ahrs.q2 = cosf(r/2)*sinf(p/2);
        ahrs.q3 = -sinf(r/2)*sinf(p/2);
        ahrs.initialised = true;
    }

    /* 7. Main loop */
    uint32_t t_imu     = HAL_GetTick();
    uint32_t t_enc     = HAL_GetTick();
    uint32_t t_pub_imu = HAL_GetTick();
    uint32_t t_pub_enc = HAL_GetTick();
    uint32_t t_lcd     = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();

        /* ── Read + filter IMU @ 100 Hz ── */
        if (now - t_imu >= IMU_PERIOD_MS) {
            float dt = (float)(now - t_imu) * 0.001f;
            t_imu = now;

            if (ICM20948_Read(&hi2c1, &imu) == HAL_OK) {
                float gx = imu.gx - calib.gx_bias;
                float gy = imu.gy - calib.gy_bias;
                float gz = imu.gz - calib.gz_bias;

                /* Use MARG when mag is fresh; fall back to IMU-only */
                if (imu.mx != 0.0f || imu.my != 0.0f || imu.mz != 0.0f) {
                    Madgwick_UpdateMARG(&ahrs,
                        gx, gy, gz,
                        imu.ax, imu.ay, imu.az,
                        imu.mx, imu.my, imu.mz,
                        dt);
                } else {
                    Madgwick_UpdateIMU(&ahrs,
                        gx, gy, gz,
                        imu.ax, imu.ay, imu.az,
                        dt);
                }
            }
        }

        /* ── Update encoders @ 100 Hz ── */
        if (now - t_enc >= ENC_PERIOD_MS) {
            t_enc = now;
            Encoder_Update();
        }

        /* ── Publish IMU @ 100 Hz ── */
        if (now - t_pub_imu >= IMU_PUB_PERIOD_MS) {
            t_pub_imu = now;
            publish_imu();
        }

        /* ── Publish encoders @ 50 Hz ── */
        if (now - t_pub_enc >= ENC_PUB_PERIOD_MS) {
            t_pub_enc = now;
            publish_encoders();
        }

        /* ── Refresh LCD @ 10 Hz ── */
        if (now - t_lcd >= LCD_PUB_PERIOD_MS) {
            t_lcd = now;
            LCD_Display_Update(&ahrs, 1);
        }

        /* Spin micro-ROS (non-blocking) */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
    }
}
