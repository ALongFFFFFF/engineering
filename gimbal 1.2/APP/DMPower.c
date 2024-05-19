#include "main.h"
#include "DMPower.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
Motor_t MOTOR1_td,MOTOR2_td,MOTOR3_t;
//达妙电机电源CTRL_C
/**
 * @brief  uintתfloat
 * @param  x_int   ���ֵ
 * @param  x_min   ��Сֵ
 * @param  bits    λ��
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  floatתuint
 * @param  x_int   ���ֵ
 * @param  x_min   ��Сֵ
 * @param  bits    λ��
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MITģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 * @param  _KP    λ�ñ���ϵ��
 * @param  _KD    λ��΢��ϵ��
 * @param  _torq  ת�ظ���ֵ
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
    static CAN_TxPacketTypeDef packet;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    
    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    packet.payload[0] = (pos_tmp >> 8);
    packet.payload[1] = pos_tmp;
    packet.payload[2] = (vel_tmp >> 4);
    packet.payload[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    packet.payload[4] = kp_tmp;
    packet.payload[5] = (kd_tmp >> 4);
    packet.payload[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    packet.payload[7] = tor_tmp;

    //�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

/**
 * @brief  λ���ٶ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _pos, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    packet.payload[0] = *pbuf;;
    packet.payload[1] = *(pbuf+1);
    packet.payload[2] = *(pbuf+2);
    packet.payload[3] = *(pbuf+3);
    packet.payload[4] = *vbuf;
    packet.payload[5] = *(vbuf+1);
    packet.payload[6] = *(vbuf+2);
    packet.payload[7] = *(vbuf+3);

    //�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}


/**
 * @brief  �ٶ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _vel   �ٶȸ���
 */
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
{
    static CAN_TxPacketTypeDef packet;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x04;

    packet.payload[0] = *vbuf;
    packet.payload[1] = *(vbuf+1);
    packet.payload[2] = *(vbuf+2);
    packet.payload[3] = *(vbuf+3);

    //�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

/**
 * @brief  �������
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID      
 * @param  data   ����֡
 */
 void Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7)
 {
    static CAN_TxPacketTypeDef packet;
    
    packet.hdr.StdId = ID;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;
    packet.payload[0] = (uint8_t)data0;
    packet.payload[1] = (uint8_t)data1;
    packet.payload[2] = (uint8_t)data2;
    packet.payload[3] = (uint8_t)data3;
    packet.payload[4] = (uint8_t)data4;
    packet.payload[5] = (uint8_t)data5;
    packet.payload[6] = (uint8_t)data6;
    packet.payload[7] = (uint8_t)data7;

    /*�ҵ��յķ������䣬�����ݷ��ͳ�ȥ*/
	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

/**
 * @brief  CAN��FIFO����
 * @param  canHandle  CAN�жϾ��
 */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
// {
// 	static CAN_RxPacketTypeDef packet;
//     // CAN���ݽ���
//     if (canHandle->Instance == hcan1.Instance)
//     {
//         if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		//��ý��յ�������ͷ������
//         {
//             if(packet.hdr.StdId == MOTOR1){        //����MOTOR1����
//               MOTOR1_td.p_int=(packet.payload[1]<<8)|packet.payload[2];
//               MOTOR1_td.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
//               MOTOR1_td.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
//               MOTOR1_td.position = uint_to_float(MOTOR1_t.p_int, P_MIN, P_MAX, 16); 
//               MOTOR1_td.velocity = uint_to_float(MOTOR1_t.v_int, V_MIN, V_MAX, 12);
//               MOTOR1_td.torque = uint_to_float(MOTOR1_t.t_int, T_MIN, T_MAX, 12); 
//             }//else if(packet.hdr.StdId == MOTOR2){  //����MOTOR2����
//             //   MOTOR2_t.p_int=(packet.payload[1]<<8)|packet.payload[2];
//             //   MOTOR2_t.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
//             //   MOTOR2_t.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
//             //   MOTOR2_t.position = uint_to_float(MOTOR2_t.p_int, P_MIN, P_MAX, 16); 
//             //   MOTOR2_t.velocity = uint_to_float(MOTOR2_t.v_int, V_MIN, V_MAX, 12); 
//             //   MOTOR2_t.torque = uint_to_float(MOTOR2_t.t_int, T_MIN, T_MAX, 12);
//             // }else if(packet.hdr.StdId == MOTOR3){   //����MOTOR3����
//             //   MOTOR3_t.p_int=(packet.payload[1]<<8)|packet.payload[2];
//             //   MOTOR3_t.v_int=(packet.payload[3]<<4)|(packet.payload[4]>>4);
//             //   MOTOR3_t.t_int=((packet.payload[4]&0xF)<<8)|packet.payload[5];
//             //   MOTOR3_t.position = uint_to_float(MOTOR3_t.p_int, P_MIN, P_MAX, 16); 
//             //   MOTOR3_t.velocity = uint_to_float(MOTOR3_t.v_int, V_MIN, V_MAX, 12); 
//             //   MOTOR3_t.torque = uint_to_float(MOTOR3_t.position, T_MIN, T_MAX, 12); 
// //             // }
//             HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);						// �ٴ�ʹ��FIFO0�����ж�
//         }
//     }
// }
