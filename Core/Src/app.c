/**
 * app.c — STM32F746G-DISCO Terrain Bot  (4WD + ESP32 co-processor)
 *
 * Final hardware map:
 *   ESP32 encoders   : I2C1 PB8(SCL) PB9(SDA)       addr 0x30
 *   USART6 host link : PG14(TX) PG9(RX)             115200 baud
 *   LCD dashboard    : LTDC built-in 480×272
 *
 *   4× BTS7960 (one per motor, 2 PWM pins each):
 *     FL: RPWM=PF7(TIM11)      LPWM=PF6(TIM10)
 *     FR: RPWM=PF9(TIM14)      LPWM=PF8(TIM13)
 *     RL: RPWM=PH6(TIM12_CH1)  LPWM=PB15(TIM12_CH2)
 *     RR: RPWM=PA15(TIM2_CH1)  LPWM=PB4(TIM3_CH1)
 */

#include "app.h"
#include "icm20948.h"
#include "encoder.h"
#include "motor.h"
#include "madgwick.h"
#include "microros_transport.h"
#include "lcd_display.h"
#include "main.h"

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

/* ── HAL handles ─────────────────────────────────────────────────── */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim10;
extern TIM_HandleTypeDef  htim11;
extern TIM_HandleTypeDef  htim12;
extern TIM_HandleTypeDef  htim13;
extern TIM_HandleTypeDef  htim14;
extern UART_HandleTypeDef huart6;

/* ── Timing ──────────────────────────────────────────────────────── */
#define IMU_MS       10    /* 100 Hz */
#define ENC_MS       20    /*  50 Hz */
#define PID_MS       10    /* 100 Hz */
#define PUB_IMU_MS   10    /* 100 Hz */
#define PUB_ENC_MS   20    /*  50 Hz */
#define LCD_MS      100    /*  10 Hz */
#define CALIB_N     200

#define MADGWICK_BETA   0.033f

/* ── Safety ──────────────────────────────────────────────────────── */
#define S_W_PITCH   0.7f
#define S_W_ROLL    0.3f
#define S_SAFE      10.0f
#define S_CRIT      30.0f
#define S_W_INCL    0.7f
#define S_W_SHAKE   0.3f
#define S_LPF       0.90f
#define S_ESTOP_P   30.0f
#define S_ESTOP_R   40.0f
#define S_FLIP_RATE (30.0f * 0.017453293f)
#define S_FLIP_P    (15.0f * 0.017453293f)

/* ── Motor PI ────────────────────────────────────────────────────── */
#define KP          0.5f
#define KI          0.1f

/* cmd_vel watchdog */
#define WDG_MS      500

/* ── IMU covariance ──────────────────────────────────────────────── */
#define GY_COV  (0.00262f*0.00262f)
#define AC_COV  (0.02256f*0.02256f)
#define OR_COV  0.0001f

#define UROS_OK(fn) do{if((fn)!=RCL_RET_OK)Error_Handler();}while(0)

/* ── micro-ROS ───────────────────────────────────────────────────── */
static rcl_allocator_t  alloc;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  exec;
static rcl_publisher_t  pub_imu, pub_ticks, pub_vel;
static rcl_subscription_t sub_cmd;

static sensor_msgs__msg__Imu              msg_imu;
static std_msgs__msg__Int32MultiArray     msg_ticks;
static std_msgs__msg__Float32MultiArray   msg_vel;
static geometry_msgs__msg__Twist          msg_cmd;

static int32_t  tick_buf[MOTOR_NUM];
static float    vel_buf [MOTOR_NUM];
static char     fid[]  = "imu_link";

/* ── Sensor / filter state ───────────────────────────────────────── */
static ICM20948_Data  imu;
static ICM20948_Calib calib;
static Madgwick_t     ahrs;

/* ── Safety state ────────────────────────────────────────────────── */
static float g_risk = 0.0f;
static float g_scale = 1.0f;

/* ── Motor targets and PI state ──────────────────────────────────── */
static float    g_tgt[MOTOR_NUM]      = {0};
static float    g_intg[MOTOR_NUM]     = {0};
static uint32_t g_cmd_ms              = 0;
static uint8_t  g_enc_ok              = 0;
static uint8_t  g_host_ok             = 0;
static uint8_t  g_imu_ok              = 0;
static uint8_t  g_emergency_stop      = 0;

/* ── cmd_vel callback ────────────────────────────────────────────── */
static void cmd_cb(const void *msg_in)
{
    const geometry_msgs__msg__Twist *tw =
        (const geometry_msgs__msg__Twist *)msg_in;

    float vx = (float)tw->linear.x  * 1000.0f;
    float wz = (float)tw->angular.z;
    float hL = ENC_WHEELBASE_MM * 0.5f;

    float vl = vx - wz * hL;
    float vr = vx + wz * hL;

    /* Skid-steer: left pair same, right pair same */
    g_tgt[MOTOR_FL] = vl;
    g_tgt[MOTOR_RL] = vl;
    g_tgt[MOTOR_FR] = vr;
    g_tgt[MOTOR_RR] = vr;
    g_cmd_ms = HAL_GetTick();
    g_host_ok = 1u;
}

/* ── Safety layer ────────────────────────────────────────────────── */
static void update_safety(void)
{
    float pd = ahrs.pitch * (180.0f/3.14159265f);
    float rd = ahrs.roll  * (180.0f/3.14159265f);
    float pa = fabsf(pd), ra = fabsf(rd);
    float pr = fabsf(imu.gy - calib.gy_bias);

    if (pa >= S_ESTOP_P || ra >= S_ESTOP_R ||
        (pr > S_FLIP_RATE && ahrs.pitch > S_FLIP_P)) {
        g_risk = 1.0f; g_scale = 0.0f;
        g_emergency_stop = 1u;
        Motor_StopAll(); return;
    }
    g_emergency_stop = 0u;

    float tw  = S_W_PITCH*pa + S_W_ROLL*ra;
    float ri  = (tw - S_SAFE) / (S_CRIT - S_SAFE);
    if (ri < 0.0f) ri = 0.0f;
    if (ri > 1.0f) ri = 1.0f;

    float am  = sqrtf(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
    float gref= (calib.gravity > 9.0f) ? calib.gravity : 9.80665f;
    float rs  = fabsf(am - gref) / gref;
    if (rs > 1.0f) rs = 1.0f;

    float rr  = S_W_INCL*ri + S_W_SHAKE*rs;
    if (rr > 1.0f) rr = 1.0f;

    g_risk  = S_LPF*g_risk + (1.0f-S_LPF)*rr;
    {
        float s = 1.0f - g_risk;
        g_scale = s * s;
    }
}

/* ── Motor PI (per motor) ────────────────────────────────────────── */
static void motor_pid(float dt)
{
    int i;
    for (i = 0; i < MOTOR_NUM; i++) {
        float tgt  = g_tgt[i] * g_scale;
        float meas = Encoder_VelMmps((uint8_t)i);
        float err  = tgt - meas;

        g_intg[i] += err * dt;
        if (g_intg[i] >  MOTOR_ILIMIT) g_intg[i] =  MOTOR_ILIMIT;
        if (g_intg[i] < -MOTOR_ILIMIT) g_intg[i] = -MOTOR_ILIMIT;

        Motor_Set((Motor_ID)i, tgt + KP*err + KI*g_intg[i]);
    }
}

/* ── micro-ROS setup ─────────────────────────────────────────────── */
static void uros_setup(void)
{
    rmw_uros_set_custom_transport(true, NULL,
        transport_open, transport_close,
        transport_write, transport_read);
    alloc = rcutils_get_default_allocator();
    while (rmw_uros_ping_agent(200,10) != RMW_RET_OK) HAL_Delay(200);

    UROS_OK(rclc_support_init(&support, 0, NULL, &alloc));
    UROS_OK(rclc_node_init_default(&node,"stm32f7_node","terrain_bot",&support));

    UROS_OK(rclc_publisher_init_best_effort(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),"/imu/data"));
    UROS_OK(rclc_publisher_init_best_effort(&pub_ticks, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32MultiArray),"/wheel_ticks"));
    UROS_OK(rclc_publisher_init_best_effort(&pub_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/wheel_velocity"));
    UROS_OK(rclc_subscription_init_best_effort(&sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/cmd_vel"));

    UROS_OK(rclc_executor_init(&exec, &support.context, 1, &alloc));
    UROS_OK(rclc_executor_add_subscription(&exec,&sub_cmd,&msg_cmd,cmd_cb,ON_NEW_DATA));

    msg_ticks.data.data = tick_buf; msg_ticks.data.size = MOTOR_NUM;
    msg_ticks.data.capacity = MOTOR_NUM;
    msg_vel.data.data  = vel_buf;  msg_vel.data.size  = MOTOR_NUM;
    msg_vel.data.capacity = MOTOR_NUM;
    msg_imu.header.frame_id.data = fid;
    msg_imu.header.frame_id.size = 8;
    msg_imu.header.frame_id.capacity = 9;

    memset(msg_imu.orientation_covariance,0,sizeof(double)*9);
    memset(msg_imu.angular_velocity_covariance,0,sizeof(double)*9);
    memset(msg_imu.linear_acceleration_covariance,0,sizeof(double)*9);
    msg_imu.orientation_covariance[0]=msg_imu.orientation_covariance[4]=
    msg_imu.orientation_covariance[8]=OR_COV;
    msg_imu.angular_velocity_covariance[0]=msg_imu.angular_velocity_covariance[4]=
    msg_imu.angular_velocity_covariance[8]=GY_COV;
    msg_imu.linear_acceleration_covariance[0]=msg_imu.linear_acceleration_covariance[4]=
    msg_imu.linear_acceleration_covariance[8]=AC_COV;
}

static void pub_imu_fn(void)
{
    uint64_t ns=(uint64_t)HAL_GetTick()*1000000ULL;
    msg_imu.header.stamp.sec=(int32_t)(ns/1000000000ULL);
    msg_imu.header.stamp.nanosec=(uint32_t)(ns%1000000000ULL);
    msg_imu.orientation.w=ahrs.q0; msg_imu.orientation.x=ahrs.q1;
    msg_imu.orientation.y=ahrs.q2; msg_imu.orientation.z=ahrs.q3;
    msg_imu.angular_velocity.x=imu.gx-calib.gx_bias;
    msg_imu.angular_velocity.y=imu.gy-calib.gy_bias;
    msg_imu.angular_velocity.z=imu.gz-calib.gz_bias;
    msg_imu.linear_acceleration.x=imu.ax;
    msg_imu.linear_acceleration.y=imu.ay;
    msg_imu.linear_acceleration.z=imu.az;
    rcl_publish(&pub_imu,&msg_imu,NULL);
}

static void pub_enc_fn(void)
{
    int i;
    for (i=0;i<MOTOR_NUM;i++){
        tick_buf[i]=(int32_t)Encoder_Ticks((uint8_t)i);
        vel_buf[i]=Encoder_VelMmps((uint8_t)i);
    }
    rcl_publish(&pub_ticks,&msg_ticks,NULL);
    rcl_publish(&pub_vel,&msg_vel,NULL);
}

/* ── App entry ───────────────────────────────────────────────────── */
void App_Run(void)
{
    uint32_t t_imu;
    uint32_t t_enc;
    uint32_t t_pid;
    uint32_t t_pimu;
    uint32_t t_penc;
    uint32_t t_lcd;

    LCD_Display_Init();

    g_imu_ok = (ICM20948_Init(&hi2c1) == HAL_OK) ? 1u : 0u;
    if (!g_imu_ok) Error_Handler();

    Encoder_Init(&hi2c1);
    g_enc_ok = 1u;

    Motor_Init(&htim10, &htim11, &htim12, &htim13,
               &htim14, &htim2, &htim3);

    Madgwick_Init(&ahrs, MADGWICK_BETA);

    uros_setup();

    /* Gyro calibration ~2s */
    memset(&calib, 0, sizeof(calib));
    while (!calib.done) {
        LCD_Display_Update(&ahrs, &imu, 0u, 0.0f, 0u, g_imu_ok, g_enc_ok, 0u);
        if (ICM20948_Read(&hi2c1, &imu) == HAL_OK)
            ICM20948_Calibrate(&imu, &calib, CALIB_N);
        HAL_Delay(IMU_MS);
    }

    /* Seed Madgwick */
    {
        float ax=imu.ax, ay=imu.ay, az=imu.az;
        float rn=1.0f/sqrtf(ax*ax+ay*ay+az*az);
        ax*=rn; ay*=rn; az*=rn;
        {
            float r=atan2f(ay,az), p=atan2f(-ax,sqrtf(ay*ay+az*az));
            ahrs.q0=cosf(r/2)*cosf(p/2); ahrs.q1=sinf(r/2)*cosf(p/2);
            ahrs.q2=cosf(r/2)*sinf(p/2); ahrs.q3=-sinf(r/2)*sinf(p/2);
        }
        ahrs.initialised=true;
    }

    g_cmd_ms = HAL_GetTick();

    t_imu=HAL_GetTick();
    t_enc=HAL_GetTick();
    t_pid=HAL_GetTick();
    t_pimu=HAL_GetTick();
    t_penc=HAL_GetTick();
    t_lcd=HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();

        g_host_ok = ((now - g_cmd_ms) <= WDG_MS) ? 1u : 0u;

        /* IMU + Madgwick + safety @ 100Hz */
        if (now - t_imu >= IMU_MS) {
            float dt=(float)(now-t_imu)*0.001f; t_imu=now;
            if (ICM20948_Read(&hi2c1, &imu) == HAL_OK) {
                g_imu_ok = 1u;
                float gx=imu.gx-calib.gx_bias;
                float gy=imu.gy-calib.gy_bias;
                float gz=imu.gz-calib.gz_bias;
                if (imu.mx||imu.my||imu.mz)
                    Madgwick_UpdateMARG(&ahrs,gx,gy,gz,
                        imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz,dt);
                else
                    Madgwick_UpdateIMU(&ahrs,gx,gy,gz,
                        imu.ax,imu.ay,imu.az,dt);
                update_safety();
            } else {
                g_imu_ok = 0u;
            }
        }

        /* Read encoders from ESP32 @ 50Hz */
        if (now - t_enc >= ENC_MS) {
            t_enc = now;
            g_enc_ok = (Encoder_Update() == HAL_OK) ? 1u : 0u;
        }

        /* Motor PID @ 100Hz */
        if (now - t_pid >= PID_MS) {
            float dt=(float)(now-t_pid)*0.001f; t_pid=now;
            if (!g_host_ok) {
                memset(g_tgt, 0, sizeof(g_tgt));
                memset(g_intg,0, sizeof(g_intg));
                Motor_StopAll();
            } else {
                motor_pid(dt);
            }
        }

        /* Publish IMU @ 100Hz */
        if (now - t_pimu >= PUB_IMU_MS) {
            t_pimu=now; pub_imu_fn();
        }

        /* Publish encoders @ 50Hz */
        if (now - t_penc >= PUB_ENC_MS) {
            t_penc=now; pub_enc_fn();
        }

        /* LCD @ 10Hz */
        if (now - t_lcd >= LCD_MS) {
            t_lcd=now;
            LCD_Display_Update(&ahrs, &imu, 1u, g_risk,
                               g_emergency_stop,
                               g_imu_ok, g_enc_ok, g_host_ok);
        }

        rclc_executor_spin_some(&exec, RCL_MS_TO_NS(0));
    }
}
