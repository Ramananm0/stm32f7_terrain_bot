/**
 * app.c  —  Main application  (STM32F746G-DISCO)
 *
 * What this board does:
 *   1. Reads ICM-20948 IMU at 102 Hz via I2C1  (SDA=PB9, SCL=PB8)
 *   2. Runs complementary filter for roll / pitch
 *   3. Reads 2× RMCS-3070 encoder timers at 100 Hz
 *        LEFT  : TIM5  A0(PA0=CH1)   + A1(PA1=CH2)
 *        RIGHT : TIM2  D9(PA15=CH1)  + CN2pin15(PB3=CH2)
 *   4. Publishes over micro-ROS (USART6, 2 Mbaud, D1=TX, D0=RX):
 *        /imu/data              sensor_msgs/Imu          100 Hz
 *        /wheel_ticks           std_msgs/Int32MultiArray  50 Hz
 *        /wheel_velocity        std_msgs/Float32MultiArray 50 Hz
 *
 * Future expansion planned on this board:
 *   - AI inference on IMU data (TensorFlow Lite for Microcontrollers)
 *   - Motor PWM control
 *   - Terrain risk pre-filtering before sending to ROS
 *
 * How to add to your CubeMX project:
 *   - Add app.c, icm20948.c, encoder.c, microros_transport.c to Src/
 *   - Add app.h, icm20948.h, encoder.h, microros_transport.h to Inc/
 *   - Link micro-ROS static library (libmicroros.a)
 *   - In main.c, after all MX_xxx_Init() calls, add:  App_Run();
 */

#include "app.h"
#include "icm20948.h"
#include "encoder.h"
#include "microros_transport.h"
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

/* ── HAL handles from CubeMX main.c ─────────────────────────────── */
extern I2C_HandleTypeDef  hi2c1;    /* SDA=PB9(SDA pin), SCL=PB8(SCL pin) */
extern TIM_HandleTypeDef  htim5;    /* LEFT  encoder: A0(PA0)+A1(PA1)      */
extern TIM_HandleTypeDef  htim2;    /* RIGHT encoder: D9(PA15)+CN2p15(PB3) */
extern UART_HandleTypeDef huart6;   /* USART6: D1(PC6=TX), D0(PC7=RX)      */

/* ── Timing ──────────────────────────────────────────────────────── */
#define IMU_PERIOD_MS       10     /* 100 Hz */
#define ENC_PERIOD_MS       10     /* 100 Hz */
#define IMU_PUB_PERIOD_MS   10     /* 100 Hz */
#define ENC_PUB_PERIOD_MS   20     /*  50 Hz */
#define CALIB_SAMPLES       200    /*   ~2 s  */
#define ALPHA_CF            0.98f

/* ── micro-ROS objects ───────────────────────────────────────────── */
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

/* Frame id string storage */
static char frame_id_buf[] = "imu_link";

/* ── Sensor state ────────────────────────────────────────────────── */
static ICM20948_Data  imu;
static ICM20948_Calib calib;

/* ── Covariance (computed once, reused every message) ───────────── */
/*
 * ICM-20948 datasheet noise density:
 *   Gyro  : 0.015 dps/√Hz  at 100 Hz → σ = 0.015 × √100 × π/180
 *         = 0.015 × 10 × 0.017453 = 0.002618 rad/s
 *   Accel : 230 µg/√Hz  at 100 Hz → σ = 230e-6 × √100 × 9.80665
 *         = 230e-6 × 10 × 9.807 = 0.02256 m/s²
 */
#define GY_COV   (0.002618f * 0.002618f)
#define AC_COV   (0.02256f  * 0.02256f)

/* ── Helper macros ───────────────────────────────────────────────── */
#define UROS_OK(fn)  do { if ((fn) != RCL_RET_OK) Error_Handler(); } while(0)

/* ── micro-ROS init ──────────────────────────────────────────────── */

static void uros_setup(void)
{
    /* Register UART transport */
    rmw_uros_set_custom_transport(
        true, NULL,
        transport_open, transport_close,
        transport_write, transport_read);

    alloc = rcutils_get_default_allocator();

    /* Block until micro-ROS agent is reachable */
    while (rmw_uros_ping_agent(200, 10) != RMW_RET_OK)
        HAL_Delay(200);

    UROS_OK(rclc_support_init(&support, 0, NULL, &alloc));
    UROS_OK(rclc_node_init_default(&node, "stm32f7_node", "terrain_bot", &support));

    /* Publishers */
    UROS_OK(rclc_publisher_init_best_effort(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"));

    UROS_OK(rclc_publisher_init_best_effort(
        &pub_ticks, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/wheel_ticks"));

    UROS_OK(rclc_publisher_init_best_effort(
        &pub_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_velocity"));

    UROS_OK(rclc_executor_init(&executor, &support.context, 1, &alloc));

    /* Wire multi-array messages to static buffers */
    msg_ticks.data.data     = tick_buf;
    msg_ticks.data.size     = ENC_NUM;
    msg_ticks.data.capacity = ENC_NUM;

    msg_vel.data.data       = vel_buf;
    msg_vel.data.size       = ENC_NUM;
    msg_vel.data.capacity   = ENC_NUM;

    /* IMU message frame_id */
    msg_imu.header.frame_id.data     = frame_id_buf;
    msg_imu.header.frame_id.size     = sizeof(frame_id_buf) - 1;
    msg_imu.header.frame_id.capacity = sizeof(frame_id_buf);

    /* Covariance — diagonal, filled once */
    memset(msg_imu.angular_velocity_covariance,    0, sizeof(double) * 9);
    memset(msg_imu.linear_acceleration_covariance, 0, sizeof(double) * 9);
    msg_imu.angular_velocity_covariance[0] = GY_COV;
    msg_imu.angular_velocity_covariance[4] = GY_COV;
    msg_imu.angular_velocity_covariance[8] = GY_COV;
    msg_imu.linear_acceleration_covariance[0] = AC_COV;
    msg_imu.linear_acceleration_covariance[4] = AC_COV;
    msg_imu.linear_acceleration_covariance[8] = AC_COV;
    msg_imu.orientation_covariance[0] = -1.0; /* orientation unknown — REP-145 */
}

/* ── Fill and publish IMU message ────────────────────────────────── */

static void publish_imu(void)
{
    uint64_t ns = (uint64_t)HAL_GetTick() * 1000000ULL;
    msg_imu.header.stamp.sec     = (int32_t)(ns / 1000000000ULL);
    msg_imu.header.stamp.nanosec = (uint32_t)(ns % 1000000000ULL);

    msg_imu.angular_velocity.x = imu.gx - calib.gx_bias;
    msg_imu.angular_velocity.y = imu.gy - calib.gy_bias;
    msg_imu.angular_velocity.z = imu.gz - calib.gz_bias;

    msg_imu.linear_acceleration.x = imu.ax;
    msg_imu.linear_acceleration.y = imu.ay;
    msg_imu.linear_acceleration.z = imu.az;

    rcl_publish(&pub_imu, &msg_imu, NULL);
}

/* ── Fill and publish encoder messages ───────────────────────────── */

static void publish_encoders(void)
{
    for (int i = 0; i < ENC_NUM; i++) {
        tick_buf[i] = Encoder_Ticks(i);
        vel_buf[i]  = Encoder_VelMmps(i);
    }
    rcl_publish(&pub_ticks, &msg_ticks, NULL);
    rcl_publish(&pub_vel,   &msg_vel,   NULL);
}

/* ── App entry point ─────────────────────────────────────────────── */

void App_Run(void)
{
    /* ── 1. Init IMU ── */
    if (ICM20948_Init(&hi2c1) != HAL_OK)
        Error_Handler();   /* check wiring — SDA=PB9, SCL=PB8, see PIN_CONNECTIONS.txt */

    /* ── 2. Init encoders ── */
    Encoder_Init(&htim5, &htim2);   /* LEFT=TIM5(A0+A1)  RIGHT=TIM2(D9+CN2p15) */

    /* ── 3. Init micro-ROS ── */
    uros_setup();

    /* ── 4. Gyro calibration — keep robot still for ~2 s ── */
    memset(&calib, 0, sizeof(calib));
    while (!calib.done) {
        if (ICM20948_Read(&hi2c1, &imu) == HAL_OK)
            ICM20948_Calibrate(&imu, &calib, CALIB_SAMPLES);
        HAL_Delay(IMU_PERIOD_MS);
    }

    /* Seed filter angles from accelerometer */
    imu.roll_rad  = atan2f(imu.ay, imu.az);
    imu.pitch_rad = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az));

    /* ── 5. Main loop ── */
    uint32_t t_imu     = HAL_GetTick();
    uint32_t t_enc     = HAL_GetTick();
    uint32_t t_pub_imu = HAL_GetTick();
    uint32_t t_pub_enc = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();

        /* Read IMU @ 100 Hz */
        if (now - t_imu >= IMU_PERIOD_MS) {
            float dt = (float)(now - t_imu) * 0.001f;
            t_imu = now;
            if (ICM20948_Read(&hi2c1, &imu) == HAL_OK)
                ICM20948_Filter(&imu, &calib, dt, ALPHA_CF);
        }

        /* Update encoders @ 100 Hz */
        if (now - t_enc >= ENC_PERIOD_MS) {
            t_enc = now;
            Encoder_Update();
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

        /* Spin micro-ROS (non-blocking) */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
    }
}
