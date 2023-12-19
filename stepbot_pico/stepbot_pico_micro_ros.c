#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/structs/watchdog.h"
#include "hardware/pwm.h"

const double WHEEL_RAD = 0.025;  /* ホイール半径[m] */
const double WHEEL_SEP = 0.100;  /* トレッド[m] */
const int INTR_HZ = 10;          /* タイマー割り込み周期 */
const int FULLSTEP = 200;        /* フルステップでの1回転あたりのパルス数 */
int PPR = 0;                     /* 1回転あたりのパルス数(Pulse Per Revolution) */

/* Pico -> TB67S249 right wheel */
const uint R_ENABLEn  = 7;
const uint R_DMODE0   = 6;
const uint R_DMODE1   = 5;
const uint R_DMODE2   = 4;
const uint R_RESETn   = 3;
const uint R_AGC      = 2;
const uint R_NUM_STEP = 1;
const uint R_NUM_DIR  = 0;

/* Pico -> TB67S249 left wheel */
const uint L_ENABLEn  = 15;
const uint L_DMODE0   = 14;
const uint L_DMODE1   = 13;
const uint L_DMODE2   = 12;
const uint L_RESETn   = 11;
const uint L_AGC      = 10;
const uint L_NUM_STEP = 9;
const uint L_NUM_DIR  = 8;

/* PWM関連 */
const int DUTY_CLK = 125e2;      /* PWM周波数 */
/* PicoのPWMスライス番号 */
uint sliceNumR;   /* Right */
uint sliceNumL;   /* Left  */

/* PicoのPWMチャンネル番号 */
uint chanR;       /* Right */
uint chanL;       /* Left  */

double cmdV;       /* 車体の速度[m/s]     */
double cmdW;       /* 車体の角速度[rad/s] */
double target_wR;  /* モータの目標回転角速度[rad/s] */
double target_wL;  /* モータの目標回転角速度[rad/s] */

/* Micro-ROS */
/*
 * 車体・車輪の状態をpublishするための変数
 * 0:車体X方向速度[m/s]
 * 1:車体Y方向速度[m/s]
 * 2:車体角速度[m/s]
 * 3:左車軸角速度[rad/s]
 * 4:右車軸角速度[rad/s]
 * 5:左角度[rad]
 * 6:右角度[rad]
 */
static float wheelState[7];
rcl_publisher_t pub_wheelstate;
rcl_publisher_t pub_debug;

geometry_msgs__msg__Twist cmd_vel;
std_msgs__msg__Float64 debug_msg;
std_msgs__msg__Float32MultiArray present_wheelState;

int64_t cmd_vel_last_ms; /* 最後にcmd_velを受信したepoch起点の時間 */

void init_pins(void)
{
  /* 初期化 */
  gpio_init(R_ENABLEn);
  gpio_init(R_DMODE0);
  gpio_init(R_DMODE1);
  gpio_init(R_DMODE2);
  gpio_init(R_RESETn);
  gpio_init(R_AGC);
  gpio_init(R_NUM_STEP);
  gpio_init(R_NUM_DIR);

  gpio_init(L_ENABLEn);
  gpio_init(L_DMODE0);
  gpio_init(L_DMODE1);
  gpio_init(L_DMODE2);
  gpio_init(L_RESETn);
  gpio_init(L_AGC);
  gpio_init(L_NUM_STEP);
  gpio_init(L_NUM_DIR);

  /* 方向指定 */
  gpio_set_dir(R_ENABLEn,  GPIO_OUT);
  gpio_set_dir(R_DMODE0,   GPIO_OUT);
  gpio_set_dir(R_DMODE1,   GPIO_OUT);
  gpio_set_dir(R_DMODE2,   GPIO_OUT);
  gpio_set_dir(R_RESETn,   GPIO_OUT);
  gpio_set_dir(R_AGC,      GPIO_OUT);
  gpio_set_dir(R_NUM_DIR,  GPIO_OUT);

  gpio_set_dir(L_ENABLEn,  GPIO_OUT);
  gpio_set_dir(L_DMODE0,   GPIO_OUT);
  gpio_set_dir(L_DMODE1,   GPIO_OUT);
  gpio_set_dir(L_DMODE2,   GPIO_OUT);
  gpio_set_dir(L_RESETn,   GPIO_OUT);
  gpio_set_dir(L_AGC,      GPIO_OUT);
  gpio_set_dir(L_NUM_DIR,  GPIO_OUT);

  /* ピンの初期値設定 */
  gpio_put(R_ENABLEn, 1);
  gpio_put(R_DMODE0, 0);
  gpio_put(R_DMODE1, 0);
  gpio_put(R_DMODE2, 0);
  gpio_put(R_RESETn, 1);
  gpio_put(R_AGC, 0);
  gpio_put(R_NUM_DIR, 0);

  gpio_put(L_ENABLEn, 1);
  gpio_put(L_DMODE0, 0);
  gpio_put(L_DMODE1, 0);
  gpio_put(L_DMODE2, 0);
  gpio_put(L_RESETn, 1);
  gpio_put(L_AGC, 0);
  gpio_put(L_NUM_DIR, 0);

  /*
   * PWM出力設定
   */
  /* PWMピン指定 */
  gpio_set_function(R_NUM_STEP, GPIO_FUNC_PWM);
  gpio_set_function(L_NUM_STEP, GPIO_FUNC_PWM);

  /* スライス番号取得 */
  sliceNumR = pwm_gpio_to_slice_num(R_NUM_STEP);
  sliceNumL = pwm_gpio_to_slice_num(L_NUM_STEP);

  /* ラップアラウンド値設定 */
  /* 0からDUTY_CLK-1までカウントアップし、0に戻る */
  pwm_set_wrap(sliceNumR, DUTY_CLK - 1);
  pwm_set_wrap(sliceNumL, DUTY_CLK - 1);

  /* チャネル番号取得 */
  chanR = pwm_gpio_to_channel(R_NUM_STEP);
  chanL = pwm_gpio_to_channel(L_NUM_STEP);

  /* PWMデューティを設定。起動時はデューティ0% にする */
  pwm_set_chan_level(sliceNumR, chanR, 0);
  pwm_set_chan_level(sliceNumL, chanL, 0);

  sleep_ms(10);
}

void enable_motors(bool flg)
{
  gpio_put(R_RESETn, flg);
  gpio_put(L_RESETn, flg);

  gpio_put(R_ENABLEn, !flg);
  gpio_put(L_ENABLEn, !flg);

  /* PWMのDuty比を0.5にする */
  pwm_set_chan_level(sliceNumL, chanL, (int)(DUTY_CLK / 2));
  pwm_set_chan_level(sliceNumR, chanR, (int)(DUTY_CLK / 2));

  sleep_ms(10);
}

void set_step_size(int mode)
{
  switch (mode) {
  case 1: /* フルステップ */
    gpio_put(R_DMODE0, 0);
    gpio_put(R_DMODE1, 0);
    gpio_put(R_DMODE2, 1);
 
    gpio_put(L_DMODE0, 0);
    gpio_put(L_DMODE1, 0);
    gpio_put(L_DMODE2, 1);

    PPR = FULLSTEP * 1;

    break;
  case 2: /* Non circular ハーフステップ */
    gpio_put(R_DMODE0, 0);
    gpio_put(R_DMODE1, 1);
    gpio_put(R_DMODE2, 0);
 
    gpio_put(L_DMODE0, 0);
    gpio_put(L_DMODE1, 1);
    gpio_put(L_DMODE2, 0);

    PPR = FULLSTEP * 2;

    break;
  case 3: /* 1/4ステップ */
    gpio_put(R_DMODE0, 0);
    gpio_put(R_DMODE1, 1);
    gpio_put(R_DMODE2, 1);
 
    gpio_put(L_DMODE0, 0);
    gpio_put(L_DMODE1, 1);
    gpio_put(L_DMODE2, 1);

    PPR = FULLSTEP * 4;

    break;
  case 4: /* Circular ハーフステップ */
    gpio_put(R_DMODE0, 1);
    gpio_put(R_DMODE1, 0);
    gpio_put(R_DMODE2, 0);
 
    gpio_put(L_DMODE0, 1);
    gpio_put(L_DMODE1, 0);
    gpio_put(L_DMODE2, 0);

    PPR = FULLSTEP * 2;

    break;
  case 5: /* 1/8ステップ */
    gpio_put(R_DMODE0, 1);
    gpio_put(R_DMODE1, 0);
    gpio_put(R_DMODE2, 1);
 
    gpio_put(L_DMODE0, 1);
    gpio_put(L_DMODE1, 0);
    gpio_put(L_DMODE2, 1);

    PPR = FULLSTEP * 8;

    break;
  case 6: /* 1/16ステップ */
    gpio_put(R_DMODE0, 1);
    gpio_put(R_DMODE1, 1);
    gpio_put(R_DMODE2, 0);
 
    gpio_put(L_DMODE0, 1);
    gpio_put(L_DMODE1, 1);
    gpio_put(L_DMODE2, 0);

    PPR = FULLSTEP * 16;

    break;
  case 7: /* 1/32ステップ */
    gpio_put(R_DMODE0, 1);
    gpio_put(R_DMODE1, 1);
    gpio_put(R_DMODE2, 1);
 
    gpio_put(L_DMODE0, 1);
    gpio_put(L_DMODE1, 1);
    gpio_put(L_DMODE2, 1);

    PPR = FULLSTEP * 32;

    break;
  default: /* スタンバイモード(出力無効) */
    gpio_put(R_DMODE0, 0);
    gpio_put(R_DMODE1, 0);
    gpio_put(R_DMODE2, 0);
 
    gpio_put(L_DMODE0, 0);
    gpio_put(L_DMODE1, 0);
    gpio_put(L_DMODE2, 0);

    PPR = FULLSTEP * 0;

    break;
  }
  sleep_ms(10);
}

void cmd_vel_Cb(const void * msgin)
{
  cmd_vel_last_ms = rmw_uros_epoch_millis();

  const geometry_msgs__msg__Twist * cmdVelMsg 
    = (const geometry_msgs__msg__Twist *) msgin;

  cmdV = cmdVelMsg->linear.x;  /* 車体の目標速度[m/s]     */
  cmdW = cmdVelMsg->angular.z; /* 車体の目標角速度[rad/s] */

  /* 速度・角速度司令から左右車輪の速度司令に変換する */
  target_wR = cmdV / WHEEL_RAD + WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;
  target_wL = cmdV / WHEEL_RAD - WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  float target_valR;            /* 目標の1秒あたりのパルス数 */
  float target_valL;            /* 目標の1秒あたりのパルス数 */

  int64_t cur_ms = rmw_uros_epoch_millis();

  /* cmd_velが500[ms]以上来なかったら左右車輪の速度指令値を0にする */
  if ((cur_ms - cmd_vel_last_ms) > 500) {
    target_wR = 0.0;
    target_wL = 0.0;
  }

  /* 速度指令値を1秒あたりのパルス数に変換 */
  target_valR = target_wR * PPR / M_PI / 2.0;    /* Right */
  target_valL = target_wL * PPR / M_PI / 2.0;    /* Left  */

  /* 速度指令値の正負で回転方向を決める */
  if (target_valR >= 0) {
    gpio_put(R_NUM_DIR, 0);
  } else {
    gpio_put(R_NUM_DIR, 1);
  }
  /*
   * 周波数f=sysclock/(DUTY_CLK * clkdiv)
   * sysclock = 125MHz = 125 * 10^6
   * 周波数は1秒あたりのステップ数に相当する
   * したがって、
   * clkdiv = sysclock / DUTY_CLK / f
   *        = 125 * 10^6 / (125 * 10^2) / target_val(R/L)
   */
  if (target_valR != 0) {
    float clkdivR = fabs(125e6 / DUTY_CLK / target_valR);
    pwm_set_clkdiv(sliceNumR, clkdivR);
    pwm_set_enabled(sliceNumR, true);
  } else {
    pwm_set_enabled(sliceNumR, false);
  }

  /* 速度指令値の正負で回転方向を決める */
  /* L側はCW/CCWがR側とは逆になる */
  if (target_valL >= 0) {
    gpio_put(L_NUM_DIR, 1);
  } else {
    gpio_put(L_NUM_DIR, 0);
  }
  if (target_valL != 0) {
    float clkdivL = fabs(125e6 / DUTY_CLK / target_valL);
    pwm_set_clkdiv(sliceNumL, clkdivL);
    pwm_set_enabled(sliceNumL, true);
  } else {
    pwm_set_enabled(sliceNumL, false);
  }

  present_wheelState.data.data[0] = cmdV; // 車体X方向速度[m/s]
  present_wheelState.data.data[1] = 0.0;  // 車体Y方向速度[m/s]
  present_wheelState.data.data[2] = cmdW; // 車体角速度[rad/s]
  present_wheelState.data.data[3] = target_wL; // 左車軸回転角速度[rad/s]
  present_wheelState.data.data[4] = target_wR; // 右車軸回転角速度[rad/s]
  present_wheelState.data.data[5] = 0.0; // 左車輪角度
  present_wheelState.data.data[6] = 0.0; // 右車輪角度
  present_wheelState.data.size = 7;

  rcl_ret_t ret;
  ret = rcl_publish(&pub_wheelstate, &present_wheelState, NULL);

  debug_msg.data = target_valR;
  ret = rcl_publish(&pub_debug, &debug_msg, NULL);
}

int main()
{
    /* pico's pins setting */ 
    init_pins();

    /* TB67S249 */
    set_step_size(6);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
    );

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    const int timeout_ms = 1000; 
    const uint8_t attempts = 2;

    // Agentにつながらなかったらリブート
    if (RCL_RET_OK != rmw_uros_ping_agent(timeout_ms, attempts)) {
        watchdog_enable(1, 1);
        while(1);
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico", "", &support);

    // Subscription object
    rcl_subscription_t subscriber;
  
    executor = rclc_executor_get_zero_initialized_executor();

    // 3つめの引数は通信オブジェクトの数(Timerとsubscriptionの総数)
    // このプログラムはTimer1つsubscriber1つで通信オブジェクトは2
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    // Initialize a reliable subscriber
    rclc_subscription_init_default(&subscriber, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                   "cmd_vel"
                                   );
    rclc_executor_add_subscription(&executor, &subscriber,
                                   &cmd_vel, &cmd_vel_Cb, ON_NEW_DATA);


    rclc_publisher_init_default(
        &pub_debug,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "pico_pub_debug");

    present_wheelState.data.capacity = 7;
    present_wheelState.data.data = wheelState;
    present_wheelState.data.size = 7;

    rclc_publisher_init_default(
        &pub_wheelstate,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_state");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000 / INTR_HZ),
        timer_callback);

    rclc_executor_add_timer(&executor, &timer);

    enable_motors(true); 

    while (true)
    {
        if (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        } else {
            // Agentにつながらなかったらspinせず終了する
            break;
        }
    }

    // Agentにつながらなかったらリブートする
    watchdog_enable(1, 1);
    while(1)
        ;

    return 0;
}
