/*
    清朝老工程义肢安装
*/
#ifndef FOREARM_H
#define FOREARM_H

//头文件引�?
#include "remote_control.h"
#include "struct_typedef.h"
#include "can.h"
#include "CAN_receive.h"
#include "gpio.h"
#include "tim.h"
#include "catch_auto.h"
#include "DMPower.h"


#define DAMIAO_ROLL  0x01   //达妙roll轴电机

#define DAMIAO_PITCH 0x03   //达妙pitch轴电机


#define DAMIAO_ROLL_KP 2
#define DAMIAO_PITCH_KP 2

/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯  底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 1

//大喵电机结构体 同下forearm.can，基本没用
//寒假回家小调一下，caotamad
typedef struct 
{
    int16_t current_give;
    int16_t current_target;//一拖四重焕新生,暂时还是没啥太大用处，mad
    int16_t speed;
    int8_t  state;
    
    //mit控制参数
    float pos; //位置
    float vel;//速度
    float KP;//位置比例系数 [0,500]
    float KD; //位置微分系数 [0,5]
    float torq;//转矩给定值

    //电机返回数据
    int p_int;
    int v_int;
    int t_int;
    int position;
    int velocity;
    int torque;

}damiao_can;

typedef struct 
{
    uint16_t pos_tmp;
    uint16_t vel_tmp;
    uint16_t kp_tmp;
    uint16_t kd_tmp;
    uint16_t tor_tmp;

}damiao_can_e;

//小臂 大结构体 出爪
typedef struct
{
    //遥控器指针
    const RC_ctrl_t *rc_data;
    const motor_measure_t *motor_measure[6];
    const reset_t *reset_key;
    //函数指针定义

    //电机电流、电机状态存储
    struct
    {
        //出爪
        int16_t stretch;
        int16_t stretch_target;//机械臂目标值
        fp32    stretch_lenth;//机械臂长度（限位）
        int16_t stretch_speed;//电机速度
        int8_t  stretch_state;//机械臂状态
        
        //小臂横移
        int16_t slid;
        int16_t slid_target;
        int16_t slid_speed;
        int8_t  slid_state;
        
        //小臂roll 转矿
        int16_t roll;
        int16_t roll_target;
        int16_t roll_speed;
        int8_t  roll_state;

        //翻爪状态
        int8_t  flip_state;

        //翻爪左
        int16_t flip_left;
        int16_t flip_left_target;
        int16_t flip_left_speed;
        int16_t flip_left_speed_target;

        //翻爪 右
        int16_t flip_right;
        int16_t flip_right_target;
        int16_t flip_right_speed;
        int16_t flip_right_speed_target;


    }can;
    //达妙电机
    damiao_can damiao_roll;
    damiao_can damiao_pitch;

    float damiao_roll_angle;
    float damiao_pitch_angle;

    float flip_angle;
    float flip_stay_angle;
    float flip_reset_angle;
    
    float stretch_lenth;//出爪长度
    int16_t reset_flag;
    int64_t reset_last_flag;

    //遥控器状态命�?
    #define left_switch_is_up           (forearm.rc_data->rc.s[1] == 1)
    #define left_switch_is_mid          (forearm.rc_data->rc.s[1] == 3)
    #define left_switch_is_down         (forearm.rc_data->rc.s[1] == 2)
    #define right_switch_is_up          (forearm.rc_data->rc.s[0] == 1)
    #define right_switch_is_mid         (forearm.rc_data->rc.s[0] == 3)
    #define right_switch_is_down        (forearm.rc_data->rc.s[0] == 2)

    #define left_rocker_up              (forearm.rc_data->rc.ch[3] > 0)
    #define left_rocker_down            (forearm.rc_data->rc.ch[3] < 0)
    #define left_rocker_mid             (forearm.rc_data->rc.ch[3] == 0)

    //出爪状态
    #define stretch_state_is_stop       (forearm.can.stretch_state == stop)
    #define stretch_state_is_out        (forearm.can.stretch_state == stretch_out)
    #define stretch_state_is_back       (forearm.can.stretch_state == stretch_back)

    //横移状态
    #define slid_state_is_stop       (forearm.can.slid_state == stop)
    #define slid_state_is_right      (forearm.can.slid_state == forward)
    #define slid_state_is_left       (forearm.can.slid_state == reverse)

    //小臂roll转矿状态
    #define roll_state_is_stop          (forearm.can.roll_state == stop)
    #define roll_state_is_forward       (forearm.can.roll_state == forward)
    #define roll_state_is_reverse       (forearm.can.roll_state == reverse)

    //翻爪状态
    #define flip_state_is_stop       (forearm.can.flip_state == stop)
    #define flip_state_is_up         (forearm.can.flip_state == forward)//暂定
    #define flip_state_is_down       (forearm.can.flip_state == reverse)

    //达妙状态,感觉有点傻逼的冗余了
    #define damiao_roll_state_is_stop       (forearm.damiao_roll.state == stop)
    #define damiao_roll_state_is_forward    (forearm.damiao_roll.state = forward)
    #define damiao_roll_state_is_reverse    (forearm.damiao_roll.state = reverse)
    

}forearm_ctrl_t;

//出爪位置限位
fp32 stretch_out_lenth = 1050.0f;
fp32 stretch_back_lenth = 70.0f;
fp32 flip_angle_up = 250.0f;
fp32 flip_angle_down = -5.0f;
//PID状态
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};


enum
{
    stop    =   0,
    forward,
    reverse,
    stretch_out,
    stretch_back,
    close,
    open,
    shut
}forearm_state;


typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_strt;

//一号电机PID   小臂roll
float FOREARM_ROLL_KP     =   10.0f;
float FOREARM_ROLL_KI     =   0.0f;
float FOREARM_ROLL_KD     =   0.0f;
float FOREARM_ROLL_MOUT   =   2000.0f;
float FOREARM_ROLL_MIOUT  =   1.0f;
//二号电机PID   小臂横移
float FOREARM_SLID_KP     =   10.0f;
float FOREARM_SLID_KI     =   0.0f;
float FOREARM_SLID_KD     =   0.0f;
float FOREARM_SLID_MOUT   =   2000.0f;
float FOREARM_SLID_MIOUT  =   1.0f;
//三号电机PID   出爪
float FOREARM_STRETCH_KP     =   10.0f;
float FOREARM_STRETCH_KI     =   0.0f;
float FOREARM_STRETCH_KD     =   0.0f;
float FOREARM_STRETCH_MOUT   =   2000.0f;
float FOREARM_STRETCH_MIOUT  =   1.0f;
//四号电机PID   翻爪左
float FOREARM_FLIP_LEFT_KP     =   20.0f;
float FOREARM_FLIP_LEFT_KI     =   0.0f;
float FOREARM_FLIP_LEFT_KD     =   200.0f;
float FOREARM_FLIP_LEFT_MOUT   =   16000.0f;
float FOREARM_FLIP_LEFT_MIOUT  =   1.0f;
//五号电机PID   翻爪右
float FOREARM_FLIP_RIGHT_KP     =   20.0f;
float FOREARM_FLIP_RIGHT_KI     =   0.0f;
float FOREARM_FLIP_RIGHT_KD     =   200.0f;
float FOREARM_FLIP_RIGHT_MOUT   =   16000.0f;
float FOREARM_FLIP_RIGHT_MIOUT  =   1.0f;
//翻爪角度环
float CATCH_ANGLE_KP     =   60.0f;
float CATCH_ANGLE_KI     =   0.0f;
float CATCH_ANGLE_KD     =   5.0f;
float CATCH_ANGLE_MOUT   =   400.0f;
float CATCH_ANGLE_MIOUT  =   1.0f;



void forearm_set_mode(void);
void forearm_control(void);
void forearm_can_send(void);
void forearm_flip_can_send(void);
fp32 forearm_PID_calc(pid_strt *pid, int16_t ref, int16_t set);
void forearm_PID_init(void);
void forearm_init(void);
void auto_ore(void);

#endif
