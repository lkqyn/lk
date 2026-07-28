#ifndef _USERFUNC_H_
#define _USERFUNC_H_

#include "stm32f1xx_hal.h"

void huiyuandian_UI(void *pv);
void pozhenkong(void *pv);

void konghanshu(void *pv);

uint8_t CutLable(uint8_t Cut);
uint8_t GoHome_CutLable(void *pv);

void ShengChanMoShi(void *pv);
void TiaoShiMoShi(void *pv);
void huanliao_dongzuo(void *pv);
void KongPaoMoShi(void *pv);
void ChuShiHua(void *pv);
void controlOutputSignal(void *pv);
void autoMode_task(void *pv);
void autoMode_doubleSensor_task(void *pv);
void autoMode_Step_task(void *pv);
void onlineMode_task(void *pv);
void SL_Step_task(void *pv);
void OLM_Step_task(void *pv);
void RST_Step_task(void *pv);
void CutLable_Step_level(void *pv);
void CutLable_Step_RST(void *pv);
void GoHome(void *pv);
uint8_t GoHome_step(void *pv);

void onlineMode_doubleSensor_task(void *pv);
void GivenMatieralOnline(void *pv);
void GivenMaterialAuto(void *pv);

void EnterStopModeUI(void *pv);
void StopModeUI(void *pv);
void StopModeIO(void *pv);

void slTask_handler(void *pv);
void slDownTask_handler(void *pv);
void slUpTask_handler(void *pv);
void flTask_handler(void *pv);

void LetMotorExceedPulseLimitAlert(void);
void GivenMotorExceedPulseLimitAlert(void);
void SL_GivenMotorExceedPulseLimitAlert(void);
void ShouMotorExceedPulseLimitAlert(void);
void WLYJ_Alert(void);

uint8_t GetIOLevel(uint8_t pin, uint8_t level, uint16_t time);
uint8_t GetEXIOLevel(uint8_t pin, uint8_t level, uint16_t time);

void EXO_sigON(uint8_t no);
void EXO_sigOFF(uint8_t no);
uint8_t JL_Action(uint8_t Action);
uint8_t PTSS_Action(uint8_t Action);
uint8_t PTZK_Action(uint8_t Action);
uint8_t ZP_Action(uint8_t Action);
void Alarm(void *pv);
void Clear(void);
void Given_EN(uint8_t state);
void Reset_Given_Alarm(void);
void Shut_vacuum(void);
void Detect_vacuum(void);

void MdriveALM(void);
uint8_t EmergencyStop(void);

extern uint16_t OLM_RST_step;
extern uint16_t OLM_step;
extern uint16_t Init_step;

extern uint8_t FLflag,FLflag1; //放料标志位
extern uint8_t USLflag; //上收料料标志位
extern uint8_t DSLflag; //下收料料标志位
extern uint8_t RST_flag;
extern uint8_t replacenment_flag;//换料标志

#endif
