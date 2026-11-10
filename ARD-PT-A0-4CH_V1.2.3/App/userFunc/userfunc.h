#ifndef _USERFUNC_H_
#define _USERFUNC_H_

#include "stm32f1xx_hal.h"

#define YLdelay_count		8		//压料延时*10
#define JLdelay_count		10		//夹料延时*10

#define RSTlimitPulseCount 100000 //100000传感模式复位最大脉冲数。

extern uint16_t recvOffsetPulseCount;
extern uint16_t recvDistance, recvDistance1;
extern uint16_t recvBLDistance;
extern uint8_t recvOffsetdir;

extern uint16_t OLM_step;

extern uint16_t KP_step;  // 空跑步骤
extern uint16_t OLM_step;// 联机模式
extern uint16_t OLM_DOWN_step;// 下光纤
extern uint16_t OLM_UP_step;// 上光纤
extern uint16_t Init_step;
extern uint16_t IO_HC_step;
extern uint16_t IO_RST_step;
extern uint16_t AM_step;

extern uint8_t FLflag; //放料标志位
extern uint8_t USLflag; //上收料料标志位
extern uint8_t DSLflag; //下收料料标志位
extern uint8_t RST_flag;

//extern uint16_t Pick_Position_count;	//取标位
//extern uint16_t Stick_Position_count;	//贴标位
extern uint16_t Label_count;			//贴标位
extern uint16_t Label_Position;

void controlOutputSignal(void *pv);
void KongPaoMoShi(void *pv);

void BoliaoHCRST_Step_IO(void *pv);
void BoliaoHC_Step_IO(void *pv);
void BoliaoRST_Step_IO(void *pv);

void huiyuandian_UI(void *pv);
uint8_t GoHome_step(void *pv);
void PickLabel(void *pv);

void ReceivingLabel(void *pv);
void ReceivingLabel_Rst(void *pv);

void PickLabel(void *pv);
void PickLabel_Rst(void *pv);


void PickLabel_Rst(void *pv);
void StickLabel(void *pv);
void ChuShiHua(void *pv);

void GivenMatieralOnline(void *pv);
void onlineMode_Step_task(void *pv);
void onlineMode_Step_Down_task(void *pv);
void onlineMode_Step_Up_task(void *pv);
void onlineMode_Step_Fixed_task(void *pv);
void EXO_sigON(uint8_t no);
void EXO_sigOFF(uint8_t no);
void M0driveALM(void);
void M2driveALM(void);
uint8_t EmergencyStop(void);

void EnterStopModeUI(void *pv);
void StopModeUI(void *pv);
void StopModeIO(void *pv);

uint8_t GetIOLevel(uint8_t pin, uint8_t level, uint16_t time);
uint8_t GetEXIOLevel(uint8_t pin, uint8_t level, uint16_t time);

void Alarm(void *pv);
void Clear(void);


void UpslTask_handler(void *pv);
void DownslTask_handler(void *pv);
void slDownTask_handler(void *pv);
void slUpTask_handler(void *pv);
void flTask_handler(void *pv);

void Pick_Position(void *pv);
void Stick_Position(void *pv);

uint8_t SJ_Action(uint8_t Action);
uint8_t TB_Action(uint8_t Action);
uint8_t XZ_Action(uint8_t Action);
uint8_t PT_Action(uint8_t Action);
uint8_t ZK_Action(uint8_t Action);
uint8_t YLPY_Action(uint8_t Action);
uint8_t YL_Action(uint8_t Action);

void LetMotorExceedPulseLimitAlert(void);
void GivenMotorExceedPulseLimitAlert(void);
void ShouMotorExceedPulseLimitAlert(void);

void switchMode(uint8_t mode);
void ADJ_SL_step(uint8_t dir, uint8_t step);
void ADJ_BL_step(uint8_t dir, uint8_t step);


#endif
