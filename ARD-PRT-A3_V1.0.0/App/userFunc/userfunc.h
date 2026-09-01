#ifndef _USERFUNC_H_
#define _USERFUNC_H_

#include "stm32f1xx_hal.h"

void huiyuandian_UI(void *pv);
void controlOutputSignal(void *pv);

void konghanshu(void *pv);

void KongPaoMoShi(void *pv);
void ChuShiHua(void *pv);
uint8_t GoHome_step(void *pv);
void Taking(void *pv);
void Labeling(void *pv);
void GotoLabelPosi(void *pv);
void GotoTakePosi(void *pv);

void EnterStopModeUI(void *pv);
void StopModeUI(void *pv);
void StopModeIO(void *pv);

void slUpTask_handler(void *pv);

void WLYJ_Alert(void);

uint8_t GetIOLevel(uint8_t pin, uint8_t level, uint16_t time);
uint8_t GetEXIOLevel(uint8_t pin, uint8_t level, uint16_t time);

void EXO_sigON(uint8_t no);
void EXO_sigOFF(uint8_t no);


uint8_t Vacuum_Action(uint8_t Action, uint8_t En_outtime);
void Blow_Action(uint8_t Action);
uint8_t Cylinder_Action(uint8_t Action, uint8_t En_outtime);

void switchMode(uint8_t mode);
void Alarm(void *pv);
void Clear(void);

extern uint8_t sendBuffer[34][8];

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
