#include "forearm.h"
#include "CAN_receive.h"
#include "DMPower.h"

void forearm_task(void const *argument)
{
    forearm_init();             //初始化函数指针及参数
    while(1)
    {
        forearm_set_mode();     //更改遥控器控制模式
        forearm_control();      //更改电机控制模式
        forearm_can_send();     //CAN协议发送
        // forearm_flip_can_send();  //翻爪can协议
    }
}
forearm_ctrl_t forearm;
pid_strt forearm_PID[6];
int16_t servo_data;

// int8_t  auto_get;//取矿
// int8_t  auto_in;//收矿
// int8_t  auto_out;//出矿
// int8_t  auto_ec;//换矿


// void auto_ore(void)
// {
//     auto_get = 0;
//     auto_in = 0;
//     auto_out = 0;
//     auto_ec = 0;
// }


// 0为遥控器模式，1为键盘模式
int8_t forearm_keyboard = 1;
int8_t forearm_forearm_flag = 0;
int8_t forearm_forearm_last_flag = 0;

//更改遥控器控制模式
void forearm_rc_chassis_recieve(void)
{
    #if CHASSIS_REMOTE_OPEN
    RC_ctrl_t.rc_ctrl.rc.ch[0] = can_receive.chassis_receive.ch_0;
    RC_ctrl_t.rc_ctrl.rc.ch[2] = can_receive.chassis_receive.ch_2;
    RC_ctrl_t.rc_ctrl.rc.ch[3] = can_receive.chassis_receive.ch_3;
    RC_ctrl_t.rc_ctrl.key.v = can_receive.chassis_receive.v;
    RC_ctrl_t.rc_ctrl.rc.s[1] = can_receive.chassis_receive.s1;
}

void forearm_set_mode(void)
{
    forearm.damiao_roll.state =  stop;
    if(left_switch_is_down&&right_switch_is_up)
    {
        forearm_keyboard = 1;
    }else{
        forearm_keyboard = 0;
    }

    //出爪 X键
    if(forearm_keyboard == 0)
    {
        // 遥控器模式
        if (left_switch_is_up && right_switch_is_mid)
        {
            //起始赋值
            if(left_rocker_up)
            {
                forearm.can.stretch_state = stretch_out;
            }
            if(left_rocker_down)
            {
                forearm.can.stretch_state = stretch_back;
            }
            if(left_rocker_mid)
            {
                forearm.can.stretch_state = stop;
            }
        }
        else
        {
            forearm.can.stretch_state =   stop;
        }
    }else{
        // 键盘模式
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_X)
        {
            //起始赋值
            if(forearm.rc_data->mouse.y < 0)
            {
                forearm.can.stretch_state = stretch_out;
            }
            if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.stretch_state = stretch_back;
            }
            if(forearm.rc_data->mouse.y == 0)
            {
                forearm.can.stretch_state = stop;
            }
        }
        else
        {
            forearm.can.stretch_state =  stop;
        }
    }
    // 电控限位
    // if(forearm.reset_key->Reset_flag == 0)
    // {
    //     if(forearm.stretch_lenth < stretch_back_lenth && stretch_state_is_back)
    //     {
    //         forearm.can.stretch_state =   stop;
    //     }
    //     if(forearm.stretch_lenth > stretch_out_lenth && stretch_state_is_out)
    //     {
    //         forearm.can.stretch_state =   stop;
    //     }
    // }
    
    //roll轴达妙旋转
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                forearm.damiao_roll.state = forward; 
            }
            if (left_rocker_down)
            {
                forearm.damiao_roll.state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.damiao_roll.state =  stop;
            }
        }
        else {
            forearm.damiao_roll.state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_C)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.damiao_roll.state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.damiao_roll.state =  reverse;

            }else
            {
                forearm.damiao_roll.state =  stop;
            }
        }
        else
        {
            forearm.damiao_roll.state =   stop;
        }   
    }

    //pitch轴达妙旋转
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_up && right_switch_is_up)
        {
            if (left_rocker_up)
            {
                forearm.damiao_pitch.state = forward;
            }
            if (left_rocker_down)
            {
                forearm.damiao_pitch.state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.damiao_pitch.state =  stop;
            }
        }
        else {
            forearm.damiao_pitch.state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.damiao_pitch.state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.damiao_pitch.state =  reverse;

            }else
            {
                forearm.damiao_pitch.state =  stop;
            }
        }
        else
        {
            forearm.damiao_pitch.state =   stop;
        }   
    }

    //小臂2006横移
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_mid && right_switch_is_up)
        {
            if (left_rocker_up)
            {
                forearm.can.slid_state = forward;
            }
            if (left_rocker_down)
            {
                forearm.can.slid_state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.can.slid_state =  stop;
            }
        }
        else {
            forearm.can.slid_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_Q)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.slid_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.slid_state =  reverse;

            }else
            {
                forearm.can.slid_state =  stop;
            }
        }
        else
        {
            forearm.can.slid_state =   stop;
        }   
    }

    //小臂2006转矿
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_down && right_switch_is_mid)
        {
            if (left_rocker_up)
            {
                forearm.can.roll_state = forward;
            }
            if (left_rocker_down)
            {
                forearm.can.roll_state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.can.roll_state =  stop;
            }
        }
        else {
            forearm.can.roll_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_G)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.roll_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.roll_state =  reverse;

            }else
            {
                forearm.can.roll_state =  stop;
            }
        }
        else
        {
            forearm.can.roll_state =   stop;
        }   
    }

    //小臂3508翻爪
    if(forearm_keyboard == 0)
    {
        if (left_switch_is_mid && right_switch_is_down)
        {
            if (left_rocker_up)
            {
                forearm.can.flip_state = forward;
            }
            if (left_rocker_down)
            {
                forearm.can.flip_state =  reverse;
            }
            if(left_rocker_mid)
            {
                forearm.can.flip_state =  stop;
            }
        }
        else {
            forearm.can.flip_state =  stop;
        }
    }else{
        if (forearm.rc_data->key.v == KEY_PRESSED_OFFSET_B)
        {
            if (forearm.rc_data->mouse.y < 0)
            {
               forearm.can.flip_state =  forward;

            }else if(forearm.rc_data->mouse.y > 0)
            {
                forearm.can.flip_state =  reverse;

            }else
            {
                forearm.can.flip_state =  stop;
            }
        }
        else
        {
            forearm.can.flip_state =   stop;
        }   
    }



}

//更改电机控制模式
void forearm_control(void)
{ 
    //接收电机状态
    forearm.can.roll_speed = forearm.motor_measure[0]->speed_rpm;
    forearm.can.slid_speed = forearm.motor_measure[1]->speed_rpm;
    forearm.can.stretch_speed = forearm.motor_measure[2]->speed_rpm;
    forearm.can.flip_left_speed = forearm.motor_measure[3]->speed_rpm;
    forearm.can.flip_right_speed = forearm.motor_measure[4]->speed_rpm;


    // 出爪
    if (stretch_state_is_stop)
    {
        forearm.can.stretch_target   =   0;
    }

    if (stretch_state_is_out)
    {   
        forearm.can.stretch_target   =   160 * 19;
    }
    if (stretch_state_is_back)
    {
        forearm.can.stretch_target   =   -160 * 19;
    }
    forearm.can.stretch = (int16_t)forearm_PID_calc(&forearm_PID[2],forearm.can.stretch_speed,forearm.can.stretch_target);
    


    //小臂横移
    if (slid_state_is_stop)
    {
        forearm.can.slid_target   =   0;
    }

    if (slid_state_is_right)
    {   
        forearm.can.slid_target   =   160 * 19;
    }
    if (slid_state_is_left)
    {
        forearm.can.slid_target   =   -160 * 19;
    }
    forearm.can.slid = (int16_t)forearm_PID_calc(&forearm_PID[1],forearm.can.slid_speed,forearm.can.slid_target);
    


    //翻爪
    if (flip_state_is_stop)
    {
        forearm.can.flip_left_speed_target  =   1*(int16_t)forearm_PID_calc(&forearm_PID[5],(int16_t)forearm.flip_angle,(int16_t)forearm.flip_stay_angle);
        forearm.can.flip_right_speed_target =   -1*forearm.can.flip_left_speed_target;

    }
    if (flip_state_is_up)
    {
        forearm.can.flip_left_speed_target  =   100 * 19;
        forearm.can.flip_right_speed_target =   -100 * 19;
    }
    if (flip_state_is_down)
    {
        forearm.can.flip_left_speed_target  =   -100 * 19;
        forearm.can.flip_right_speed_target =   100 * 19;
    }
    forearm.can.flip_left  = (int16_t)forearm_PID_calc(&forearm_PID[3],forearm.can.flip_left_speed,forearm.can.flip_left_speed_target);
    forearm.can.flip_right = (int16_t)forearm_PID_calc(&forearm_PID[4],forearm.can.flip_right_speed,forearm.can.flip_right_speed_target);


    //小臂roll
    if (roll_state_is_stop)
    {
        // forearm.can.roll_target  =   1*(int16_t)forearm_PID_calc(&forearm_PID[5],(int16_t)forearm.flip_angle,(int16_t)forearm.flip_stay_angle);
        forearm.can.roll_target   =   0 ;

    }

    if (roll_state_is_forward)
    {   
        forearm.can.roll_target   =   60 * 15;
    }
    if (roll_state_is_reverse)
    {
        forearm.can.roll_target   =   -60 * 15;
    }
    forearm.can.roll = (int16_t)forearm_PID_calc(&forearm_PID[0],forearm.can.roll_speed,forearm.can.roll_target);
    


    //roll达妙 ID为1
    if (forearm.damiao_roll.state ==   stop)
    {

    }else if (forearm.damiao_roll.state ==  forward)
    {   
        forearm.damiao_roll_angle += 0.00005;
    }else if (forearm.damiao_roll.state ==  reverse)
    {
        forearm.damiao_roll_angle -= 0.00005;
    }
    if (forearm.damiao_roll_angle > 3.14 )
    {
        forearm.damiao_roll_angle = 3.14;

    }else if (forearm.damiao_roll_angle <-3.14)
    {
        forearm.damiao_roll_angle = -3.14;
    }
    MIT_CtrlMotor(&hcan1,MOTOR1,forearm.damiao_roll_angle,0,2,1,0);
    
    //pitch达妙 DAMIAO_PITCH
    if (forearm.damiao_pitch.state ==   stop)
    {


    }else if (forearm.damiao_pitch.state ==  forward)
    {   
        forearm.damiao_pitch_angle += 0.00005;

    }else if (forearm.damiao_pitch.state ==  reverse)
    {
        forearm.damiao_pitch_angle -= 0.00005;
    }
    if (forearm.damiao_pitch_angle > 2.50 ){
        forearm.damiao_pitch_angle = 2.50;
    } else if (forearm.damiao_pitch_angle <-2.50){
        forearm.damiao_pitch_angle = -2.50;
    }
    MIT_CtrlMotor(&hcan1,MOTOR2,forearm.damiao_pitch_angle,0,2,1,0);

}


// void catch_auto_control(void)


//CAN协议发送
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              forearm_can_send_data[8];
void forearm_can_send(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    forearm_can_send_data[0] = forearm.can.roll >> 8;
    forearm_can_send_data[1] = forearm.can.roll;
    forearm_can_send_data[2] = forearm.can.slid >> 8;
    forearm_can_send_data[3] = forearm.can.slid;
    forearm_can_send_data[4] = forearm.can.stretch >> 8;
    forearm_can_send_data[5] = forearm.can.stretch;
    forearm_can_send_data[6] = forearm.can.flip_left >> 8;
    forearm_can_send_data[7] = forearm.can.flip_right;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, forearm_can_send_data, &send_mail_box);
}

//翻爪CAN发送
// static CAN_TxHeaderTypeDef  flip_can_tx_message;
// static uint8_t              forearm_flip_can_send_data[8];
// void forearm_flip_can_send(void)
// {
//     uint32_t send_mail_box;
//     flip_can_tx_message.StdId = 0x1FF;
//     flip_can_tx_message.IDE = CAN_ID_STD;
//     flip_can_tx_message.RTR = CAN_RTR_DATA;
//     flip_can_tx_message.DLC = 0x08;
//     forearm_flip_can_send_data[0] = forearm.can.flip_right >> 8;
//     forearm_flip_can_send_data[1] = forearm.can.flip_right;
//     forearm_flip_can_send_data[2] = 0 >> 8;
//     forearm_flip_can_send_data[3] = 0; 
//     forearm_flip_can_send_data[4] = 0 >> 8;
//     forearm_flip_can_send_data[5] = 0;
//     forearm_flip_can_send_data[6] = 0 >> 8;
//     forearm_flip_can_send_data[7] = 0;

//     HAL_CAN_AddTxMessage(&hcan2, &flip_can_tx_message, forearm_flip_can_send_data, &send_mail_box);
// }


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

fp32 forearm_PID_calc(pid_strt *pid, int16_t ref, int16_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


void forearm_PID_init(void)
{

    forearm_PID[0].mode = PID_POSITION;
    forearm_PID[0].Kp = FOREARM_ROLL_KP;
    forearm_PID[0].Ki = FOREARM_ROLL_KI;
    forearm_PID[0].Kd = FOREARM_ROLL_KD;
    forearm_PID[0].max_out = FOREARM_ROLL_MOUT;
    forearm_PID[0].max_iout = FOREARM_ROLL_MIOUT;
    forearm_PID[0].Dbuf[0] = forearm_PID[0].Dbuf[1] = forearm_PID[0].Dbuf[2] = 0.0f;
    forearm_PID[0].error[0] = forearm_PID[0].error[1] = forearm_PID[0].error[2] = forearm_PID[0].Pout = forearm_PID[0].Iout = forearm_PID[0].Dout = forearm_PID[0].out = 0.0f;


    forearm_PID[1].mode = PID_POSITION;
    forearm_PID[1].Kp = FOREARM_SLID_KP;
    forearm_PID[1].Ki = FOREARM_SLID_KI;
    forearm_PID[1].Kd = FOREARM_SLID_KD;
    forearm_PID[1].max_out = FOREARM_SLID_MOUT;
    forearm_PID[1].max_iout = FOREARM_SLID_MIOUT;
    forearm_PID[1].Dbuf[0] = forearm_PID[1].Dbuf[1] = forearm_PID[1].Dbuf[2] = 0.0f;
    forearm_PID[1].error[0] = forearm_PID[1].error[1] = forearm_PID[1].error[2] = forearm_PID[1].Pout = forearm_PID[1].Iout = forearm_PID[1].Dout = forearm_PID[1].out = 0.0f;


    forearm_PID[2].mode = PID_POSITION;
    forearm_PID[2].Kp = FOREARM_STRETCH_KP;
    forearm_PID[2].Ki = FOREARM_STRETCH_KI;
    forearm_PID[2].Kd = FOREARM_STRETCH_KD;
    forearm_PID[2].max_out = FOREARM_STRETCH_MOUT;
    forearm_PID[2].max_iout = FOREARM_STRETCH_MIOUT;
    forearm_PID[2].Dbuf[0] = forearm_PID[2].Dbuf[1] = forearm_PID[2].Dbuf[2] = 0.0f;
    forearm_PID[2].error[0] = forearm_PID[2].error[1] = forearm_PID[2].error[2] = forearm_PID[2].Pout = forearm_PID[2].Iout = forearm_PID[2].Dout = forearm_PID[2].out = 0.0f;

    forearm_PID[3].mode = PID_POSITION;
    forearm_PID[3].Kp = FOREARM_FLIP_LEFT_KP;
    forearm_PID[3].Ki = FOREARM_STRETCH_KI;
    forearm_PID[3].Kd = FOREARM_FLIP_LEFT_KD;
    forearm_PID[3].max_out = FOREARM_FLIP_LEFT_MOUT;
    forearm_PID[3].max_iout = FOREARM_FLIP_LEFT_MIOUT;
    forearm_PID[3].Dbuf[0] = forearm_PID[3].Dbuf[1] = forearm_PID[3].Dbuf[2] = 0.0f;
    forearm_PID[3].error[0] = forearm_PID[3].error[1] = forearm_PID[3].error[2] = forearm_PID[3].Pout = forearm_PID[3].Iout = forearm_PID[3].Dout = forearm_PID[3].out = 0.0f;

    forearm_PID[4].mode = PID_POSITION;
    forearm_PID[4].Kp = FOREARM_FLIP_RIGHT_KP;
    forearm_PID[4].Ki = FOREARM_FLIP_RIGHT_KI;
    forearm_PID[4].Kd = FOREARM_FLIP_RIGHT_KD;
    forearm_PID[4].max_out = FOREARM_FLIP_RIGHT_MOUT;
    forearm_PID[4].max_iout = FOREARM_FLIP_RIGHT_MIOUT;
    forearm_PID[4].Dbuf[0] = forearm_PID[4].Dbuf[1] = forearm_PID[4].Dbuf[2] = 0.0f;
    forearm_PID[4].error[0] = forearm_PID[4].error[1] = forearm_PID[4].error[2] = forearm_PID[4].Pout = forearm_PID[4].Iout = forearm_PID[4].Dout = forearm_PID[4].out = 0.0f;

    forearm_PID[5].mode = PID_POSITION;
    forearm_PID[5].Kp = CATCH_ANGLE_KP;
    forearm_PID[5].Ki = CATCH_ANGLE_KI;
    forearm_PID[5].Kd = CATCH_ANGLE_KD;
    forearm_PID[5].max_out = CATCH_ANGLE_MOUT;
    forearm_PID[5].max_iout = CATCH_ANGLE_MIOUT;
    forearm_PID[5].Dbuf[0] = forearm_PID[5].Dbuf[1] = forearm_PID[5].Dbuf[2] = 0.0f;
    forearm_PID[5].error[0] = forearm_PID[5].error[1] = forearm_PID[5].error[2] = forearm_PID[5].Pout = forearm_PID[5].Iout = forearm_PID[5].Dout = forearm_PID[5].out = 0.0f;

}


//初始化函数指针及参数
void forearm_init(void)
{

    forearm.rc_data   =   get_remote_control_point();
    forearm.reset_key = get_reset_point();
    
    forearm_PID_init();

    for (uint8_t i = 0; i < 6; i++)
    {
        forearm.motor_measure[i] = get_motor_measure_point(i);
    }
    forearm.can.stretch_state =   stop;
    forearm.can.slid_state = stop;
    forearm.can.roll_state = stop;
    forearm.can.flip_state = stop;
    forearm.damiao_roll.state =  stop;
    forearm.damiao_pitch.state = stop;
    forearm.stretch_lenth = 0.0f;
    forearm.damiao_pitch_angle = 0.0f;
    forearm.damiao_roll_angle = 0.0f;


    servo_data = 1500;
    //达妙电机使能，必须


    Enable_CtrlMotor(&hcan1,MOTOR1,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    HAL_Delay(200);
    Enable_CtrlMotor(&hcan1,MOTOR2,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
    HAL_Delay(200);


}


const forearm_ctrl_t *get_forearm_control_point(void)
{
    return &forearm;
}
 