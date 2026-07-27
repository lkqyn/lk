#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_

/*
 * Feeder-motor-only test mode:
 * - Initialization only enables and homes GIVEN_MOTOR.
 * - Cutter, unwind/rewind motors, cylinders and vacuum are bypassed.
 * - Feed buttons directly start a feeder move.
 * - Input 2 is treated as a running safety limit.
 * Comment CONF_GIVEN_MOTOR_TEST_ONLY to restore the normal machine flow.
 */
#define     CONF_GIVEN_MOTOR_TEST_ONLY

// 加速度 单位: Hz/(10ms)
#define     CONF_GM_ACC          300
#define     CONF_GM_DEC          1000

#define     CONF_LM_ACC          500
#define     CONF_LM_DEC          1000

#define     CONF_BM_ACC          100
#define     CONF_BM_DEC          500

#define     CONF_SM_ACC          500
#define     CONF_SM_DEC          1000

// 全步时, 运动1mm所需脉冲数, 需考虑减速机

#define     CONF_GM_MM_PULSE         10   	// 拉料电机 细分800，导程10MM，同步轮2：1
#define     CONF_LM_MM_PULSE         42     // 放料电机32		1:20
#define     CONF_BM_MM_PULSE         100     // 上收料电机11		1:5
#define     CONF_SM_MM_PULSE         42     // 下收料电机32		1:20

// 4个电机对应通道号
#define     CONF_GM_NO         0       ///< 送料电机
#define     CONF_LM_NO         1       ///< 放料电机
#define     CONF_BM_NO         2       ///< 上收料料电机
#define     CONF_SM_NO         3       ///< 下收料料电机

// 默认运行参数, 2位定点, 设定2000时, 实际值为20.00
#define     CONF_GM_SPEED       5000    ///< 送料速度
#define     CONF_GM_OFFSET      5000    ///< 送料偏移
#define     CONF_LM_SPEED       5000    ///< 放料速度
#define     CONF_LM_OFFSET      5000    ///< 放料偏移
#define     CONF_BM_SPEED       5000    ///< 剥料速度
#define     CONF_BM_OFFSET      5000    ///< 剥料偏移
#define     CONF_SM_SPEED       5000    ///< 收料速度
#define     CONF_SM_DISTANCE    5000    ///< 收料行程
#define     CONF_GOZERO_SPEED   3000     ///< 回原点速度
#define     CONF_ZERO_OFFSET    0       ///< 原点偏移
#define     CONF_GM_AMP         50     ///< 送料电机电流
#define     CONF_GM_MS          3       ///< 送料电机细分
#define     CONF_LM_AMP         150      ///< 放料电机电流
#define     CONF_LM_MS          2       ///< 放料电机细分
#define     CONF_BM_AMP         150      ///< 剥料电机电流
#define     CONF_BM_MS          2       ///< 剥料电机细分
#define     CONF_SM_AMP         150      ///< 收料电机电流
#define     CONF_SM_MS          2       ///< 收料电机细分
#define		CONF_HC_D2			0		///< 后退行程
#define		CONF_BS_POINT		5000		///< 变速点后退速度

#define     CONF_GM_OFFSET_SETSPEED       200     ///< 送料电机偏移set速度, 2位定点小数

#define     CONF_USER_PASSWORD              "000000"    ///< 默认用户密码
#define     CONF_VENDOR_PASSWORD            "654321"    ///< 默认供应商密码
#define     CONF_SUPER_PASSWORD             "729183"    ///< 超级密码

// 版本显示页显示内容, 修改时注意字符串与字符串长度需对应
#define     CONF_DEVICE_NAME                "自裁自切卷式供料器"
#define     CONF_DEVICE_NAME_LEN            18
#define     CONF_DEVICE_NO                  "ARD-LBC-A1-4CC"
#define     CONF_DEVICE_NO_LEN              14
#define     CONF_HARDWARE_VER               "AREED-4C-2000-C"
#define     CONF_HARDWARE_VER_LEN           15
#define     CONF_SOFTWARE_VER               "V1.0.1 202230208"
#define     CONF_SOFTWARE_VER_LEN           15

// 外部触发方式
// 删除该宏定义, 则为扫描方式, 否则为中断方式
//#define     CONF_EXTRIG_INT                 

// 输入信号页输入信号名称
#define     CONF_INPUT_00_NAME              "送料原点信号"
#define     CONF_INPUT_00_NAME_LEN          18

#define     CONF_INPUT_01_NAME              "送料到位信号"
#define     CONF_INPUT_01_NAME_LEN          18

#define     CONF_INPUT_001_NAME              "送料限位信号"
#define     CONF_INPUT_001_NAME_LEN          12

#define     CONF_INPUT_02_NAME              "压料气缸动点信号"
#define     CONF_INPUT_02_NAME_LEN          32

#define     CONF_INPUT_03_NAME              "切刀电机原点信号"
#define     CONF_INPUT_03_NAME_LEN          24

#define     CONF_INPUT_04_NAME              "内部急停信号"
#define     CONF_INPUT_04_NAME_LEN          22

#define     CONF_INPUT_004_NAME              "备用"
#define     CONF_INPUT_004_NAME_LEN          22


#define     CONF_INPUT_05_NAME              "平台真空信号"
#define     CONF_INPUT_05_NAME_LEN          27

#define     CONF_INPUT_06_NAME              "平台气缸原点信号"
#define     CONF_INPUT_06_NAME_LEN          18

#define     CONF_INPUT_07_NAME              "平台气缸动点信号"
#define     CONF_INPUT_07_NAME_LEN          16

#define     CONF_INPUT_08_NAME              "放料启动信号"
#define     CONF_INPUT_08_NAME_LEN          22

#define     CONF_INPUT_008_NAME              "备用"
#define     CONF_INPUT_008_NAME_LEN          22

#define     CONF_INPUT_09_NAME              "放料停止信号"
#define     CONF_INPUT_09_NAME_LEN          21

#define     CONF_INPUT_009_NAME              "备用"
#define     CONF_INPUT_009_NAME_LEN          21

#define     CONF_INPUT_10_NAME              "备用"
#define     CONF_INPUT_10_NAME_LEN          5

#define     CONF_INPUT_11_NAME              "伺服报警信号"
#define     CONF_INPUT_11_NAME_LEN          15
// 供料器对外输入信号名称
#define 	CONF_EXI0_NAME					"送料信号"
#define 	CONF_EXI0_NAME_LEN				11

#define 	EN_CONF_EXI0_NAME				"Feed signal"
#define 	EN_CONF_EXI0_NAME_LEN			11

#define 	CONF_EXI1_NAME					"关闭真空信号"
#define 	CONF_EXI1_NAME_LEN				12

#define 	EN_CONF_EXI1_NAME				"Cut signal"
#define 	EN_CONF_EXI1_NAME_LEN			16

#define 	CONF_EXI2_NAME					"初始化信号"
#define 	CONF_EXI2_NAME_LEN				21

#define 	EN_CONF_EXI2_NAME				"Initialization signal"
#define 	EN_CONF_EXI2_NAME_LEN			21


#define 	CONF_EXI3_NAME					"停止信号"
#define 	CONF_EXI3_NAME_LEN				12

#define 	EN_CONF_EXI3_NAME				"Stop signals"
#define 	EN_CONF_EXI3_NAME_LEN			12

#define 	CONF_EXI4_NAME					"备用"//
#define 	CONF_EXI4_NAME_LEN				5

#define 	EN_CONF_EXI4_NAME				"Spare"
#define 	EN_CONF_EXI4_NAME_LEN			5

#define 	CONF_EXI5_NAME					"备用"
#define 	CONF_EXI5_NAME_LEN				5

#define 	EN_CONF_EXI5_NAME				"Spare"
#define 	EN_CONF_EXI5_NAME_LEN			5

#define 	CONF_EXI6_NAME					"备用"
#define 	CONF_EXI6_NAME_LEN				5

#define 	EN_CONF_EXI6_NAME				"Spare"
#define 	EN_CONF_EXI6_NAME_LEN			5

#define 	CONF_EXI7_NAME					"备用"
#define 	CONF_EXI7_NAME_LEN				5

#define 	EN_CONF_EXI7_NAME				"Spare"
#define 	EN_CONF_EXI7_NAME_LEN			5

// 供料器内部控制信号名称显示
#define     CONF_INNER_CTRL_NAME_1          "压料气缸"
#define     CONF_INNER_CTRL_NAME_1_LEN      8

#define     CONF_INNER_CTRL_NAME_2          "平台气缸"
#define     CONF_INNER_CTRL_NAME_2_LEN      8

#define     CONF_INNER_CTRL_NAME_3          "平台真空电磁阀"
#define     CONF_INNER_CTRL_NAME_3_LEN      14

#define     CONF_INNER_CTRL_NAME_4          "整平气缸"
#define     CONF_INNER_CTRL_NAME_4_LEN      8

#define     CONF_INNER_CTRL_NAME_5          "伺服使能信号"
#define     CONF_INNER_CTRL_NAME_5_LEN      12

#define     CONF_INNER_CTRL_NAME_6          "伺服清除报警信号"
#define     CONF_INNER_CTRL_NAME_6_LEN      16

#define     CONF_INNER_CTRL_NAME_7          "备用"
#define     CONF_INNER_CTRL_NAME_7_LEN      5

#define     CONF_INNER_CTRL_NAME_8          "备用"
#define     CONF_INNER_CTRL_NAME_8_LEN      5

// 供料器对外部控制信号名称
#define     CONF_OUTER_CTRL_NAME_1          "初始化完成信号"
#define     CONF_OUTER_CTRL_NAME_1_LEN      14

#define     EN_CONF_OUTER_CTRL_NAME_1          " initialize OK"
#define     EN_CONF_OUTER_CTRL_NAME_1_LEN      14

#define     CONF_OUTER_CTRL_NAME_2          "异常报警信号"
#define     CONF_OUTER_CTRL_NAME_2_LEN      14

#define     EN_CONF_OUTER_CTRL_NAME_2          "Abnormal alarm"
#define     EN_CONF_OUTER_CTRL_NAME_2_LEN      14
#define     CONF_OUTER_CTRL_NAME_3          "真空关闭完成信号"
#define     CONF_OUTER_CTRL_NAME_3_LEN      16

#define     EN_CONF_OUTER_CTRL_NAME_3          "Cut completion"
#define     EN_CONF_OUTER_CTRL_NAME_3_LEN      14

#define     CONF_OUTER_CTRL_NAME_4          "  送料完成信号  "
#define     CONF_OUTER_CTRL_NAME_4_LEN      16

#define     EN_CONF_OUTER_CTRL_NAME_4          " Feed completion"
#define     EN_CONF_OUTER_CTRL_NAME_4_LEN      16

#define     CONF_OUTER_CTRL_NAME_5          "备用"
#define     CONF_OUTER_CTRL_NAME_5_LEN      5

#define     EN_CONF_OUTER_CTRL_NAME_5          "Spare"
#define     EN_CONF_OUTER_CTRL_NAME_5_LEN      5

#define     CONF_OUTER_CTRL_NAME_6          "备用"
#define     CONF_OUTER_CTRL_NAME_6_LEN      5

#define     EN_CONF_OUTER_CTRL_NAME_6          "Spare"
#define     EN_CONF_OUTER_CTRL_NAME_6_LEN      5

#define     CONF_OUTER_CTRL_NAME_7          "备用"
#define     CONF_OUTER_CTRL_NAME_7_LEN      5

#define     EN_CONF_OUTER_CTRL_NAME_7          "Spare"
#define     EN_CONF_OUTER_CTRL_NAME_7_LEN      5

#define     CONF_OUTER_CTRL_NAME_8          "备用"
#define     CONF_OUTER_CTRL_NAME_8_LEN      5

#define     EN_CONF_OUTER_CTRL_NAME_8          "Spare"
#define     EN_CONF_OUTER_CTRL_NAME_8_LEN      5

#define     USERNAME_DISP_NAME          "用户登录"
#define     USERNAME_DISP_NAME_LEN      8

#define     EN_USERNAME_DISP_NAME          " login  "
#define     EN_USERNAME_DISP_NAME_LEN      8

#define     LANGUAGE_DISP_NAME          " 中文"
#define     LANGUAGE_DISP_NAME_LEN      7

#define     EN_LANGUAGE_DISP_NAME          "English"
#define     EN_LANGUAGE_DISP_NAME_LEN      7


#define POP_UP_INFO_RES_E_SOTP() 		GUI_showMessage("等待解除急停！！！",18)
#define EN_POP_UP_INFO_RES_E_SOTP() 	GUI_showMessage("Please release the E-stop!!!",28)

#define POP_UP_INFO_AGING_RUN() 		GUI_showMessage("空跑模式运行中！",16)
#define EN_POP_UP_INFO_AGING_RUN() 		GUI_showMessage("Aging mode is running!",22)


#define MAIN_INFO_YL_CYL_OUT()			GUI_mainMessageDisp("压料气缸伸出.", 13)
#define EN_MAIN_INFO_YL_CYL_OUT()		GUI_mainMessageDisp("The pressing cylinder extends.", 30)

#define MAIN_INFO_YL_CYL_IN()			GUI_mainMessageDisp("压料气缸缩回.", 13)
#define EN_MAIN_INFO_YL_CYL_IN()		GUI_mainMessageDisp("The pressing cylinder retracts.", 31)

#define MAIN_INFO_JL_CYL_OUT()			GUI_mainMessageDisp("夹料气缸伸出.", 13)
#define EN_MAIN_INFO_JL_CYL_OUT()		GUI_mainMessageDisp("The clamp cylinder extends.", 27)

#define MAIN_INFO_JL_CYL_IN()			GUI_mainMessageDisp("夹料气缸缩回.", 13)
#define EN_MAIN_INFO_JL_CYL_IN()		GUI_mainMessageDisp("The Sensor cylinder retracts.", 29)

#define MAIN_INFO_USL_CYL_OUT()			GUI_mainMessageDisp("传感器气缸伸出.", 13)
#define EN_MAIN_INFO_USL_CYL_OUT()		GUI_mainMessageDisp("The Sensor cylinder extends.", 28)

#define MAIN_INFO_USL_CYL_IN()			GUI_mainMessageDisp("传感器气缸缩回.", 13)
#define EN_MAIN_INFO_USL_CYL_IN()		GUI_mainMessageDisp("The Sensor cylinder retracts.", 29)

#define MAIN_INFO_SW2DEBUG()			GUI_mainMessageDisp("切换到调试模式.", 15)
#define EN_MAIN_INFO_SW2DEBUG()			GUI_mainMessageDisp("Switch to debug mode.", 21)

#define MAIN_INFO_SW2ONLINE()			GUI_mainMessageDisp("切换到联机模式.", 15)
#define EN_MAIN_INFO_SW2ONLINE()		GUI_mainMessageDisp("Switch to online mode.", 22)

#define MAIN_INFO_SW2AGING()			GUI_mainMessageDisp("切换到空跑模式.", 15)
#define EN_MAIN_INFO_SW2AGING()			GUI_mainMessageDisp("Switch to aning mode.", 21)

#define MAIN_TIPS_INNI_FIRST()			GUI_mainMessageDisp("提示信息：请先初始动作.", 23)
#define EN_MAIN_TIPS_INNI_FIRST()		GUI_mainMessageDisp("Tips:Please initialize first.", 29)

#define MAIN_TIPS_HAVE_MATERIAL()			GUI_mainMessageDisp("提示信息：送料失败，检测到料！", 30)
#define EN_MAIN_TIPS_HAVE_MATERIAL()		GUI_mainMessageDisp("Tips:Feeding failed, label detected!", 36)

#define MAIN_TIPS_NO_MATERIAL() 			GUI_mainMessageDisp("提示信息：送料失败，未检测到料！", 32)
#define EN_MAIN_TIPS_NO_MATERIAL() 			GUI_mainMessageDisp("Tips:Feeding failed, label not detected!", 40)

#define MAIN_TIPS_WAIT_RST_E_STOP() 		GUI_mainMessageDisp("提示信息：等待解除急停！", 20)
#define EN_MAIN_TIPS_WAIT_RST_E_STOP() 		GUI_mainMessageDisp("Tips:Waiting for release E-stop!", 32)

#define MAIN_TIPS_RST_E_STOP() 				GUI_mainMessageDisp("提示信息：解除急停！", 20)
#define EN_MAIN_TIPS_RST_E_STOP() 			GUI_mainMessageDisp("Tips:Release E-stop!", 20)

#define MAIN_TIPS_EXI_E_STOP() 				GUI_mainMessageDisp("提示信息：外部紧急停止状态！", 28)
#define EN_MAIN_TIPS_EXI_E_STOP() 			GUI_mainMessageDisp("Tips:External E-stop state!", 27)

#define MAIN_TIPS_NO_CUT_FEED() 			GUI_mainMessageDisp("提示信息：未取走物料，禁止送料！", 32)
#define EN_MAIN_TIPS_CUT_FEED() 			GUI_mainMessageDisp("Tips: No material is cut, no feeding!", 37)

#define MAIN_TIPS_NO_FEED_CUT() 			GUI_mainMessageDisp("提示信息：未送料，禁止裁切！", 28)
#define EN_MAIN_TIPS_NO_FEED_CUT() 			GUI_mainMessageDisp("Tips: No feed, cutting is prohibited!", 37)

#define ERR_INFO_USL_CYL_OUT_TIMEOUT() 			GUI_mainMessageDisp("报警信息：传感器气缸伸出异常！", 30)
#define EN_ERR_INFO_USL_CYL_OUT_TIMEOUT() 		GUI_mainMessageDisp("Alarm info:abnormal extension of sensor cylinder!", 49)

#define ERR_INFO_USL_CYL_IN_TIMEOUT() 			GUI_mainMessageDisp("报警信息：传感器气缸缩回异常!", 29)
#define EN_ERR_INFO_USL_CYL_IN_TIMEOUT() 		GUI_mainMessageDisp("Alarm info:abnormal retracts of sensor cylinder!", 48)

#define ERR_INFO_GO_ORG_TIMEOUT() 				GUI_mainMessageDisp("报警信息：未检测到原点信号！", 28)
#define EN_ERR_INFO_GO_ORG_TIMEOUT() 			GUI_mainMessageDisp("Alarm info:No origin signal detected!", 37)

#define ERR_INFO_AVOID_ORG_TIMEOUT() 			GUI_mainMessageDisp("报警信息：避开原点失败！", 24)
#define EN_ERR_INFO_AVOID_ORG_TIMEOUT() 		GUI_mainMessageDisp("Alarm info:Failed to avoid origin!", 34)

#define ERR_INFO_OVRE_MATERIAL() 				GUI_mainMessageDisp("报警信息：料带用完，请补充新料！", 32)
#define EN_ERR_INFO_OVRE_MATERIAL() 			GUI_mainMessageDisp("Alarm info:The material over,please add new material!", 53)

#define ERR_INFO_NO_MATERIAL() 					GUI_mainMessageDisp("报警信息：送料失败，未检测到料！", 32)
#define EN_ERR_INFO_NO_MATERIAL() 				GUI_mainMessageDisp("Alarm info:Feeding failed, label not detected!", 46)


#define CANT_SWITCH_PAGE_MESSAGE()       	GUI_showMessage("先停止任务",19)
#define EN_CANT_SWITCH_PAGE_MESSAGE()       GUI_showMessage("Stop the task first",19)

#define CANT_SWITCH_PAGE_MESSAGE0()      	GUI_showMessage("请先初始化操作！",24)
#define EN_CANT_SWITCH_PAGE_MESSAGE0()      GUI_showMessage("Please initialize first!",24)

#define CANT_SWITCH_PAGE_MESSAGE1()     	GUI_showMessage("联机模式禁止操作！",32)
#define EN_CANT_SWITCH_PAGE_MESSAGE1()      GUI_showMessage("Online mode prohibits operation!",32)

#define CANT_SWITCH_PAGE_MESSAGE2()      	GUI_showMessage("请等待当前动作结束！",35)
#define EN_CANT_SWITCH_PAGE_MESSAGE2()      GUI_showMessage("wait for the current action to end!",35)

#define CANT_SWITCH_PAGE_MESSAGE3()      	GUI_showMessage("空跑模式禁止操作！",38)
#define EN_CANT_SWITCH_PAGE_MESSAGE3()      GUI_showMessage("Aging mode inhibits operation!",30)

#define MAXSPEED	20000

#define OVER_RANGE() GUI_showMessage("参数越界", 8)
#define EN_OVER_RANGE() GUI_showMessage("parameter out of bounds", 23)

//#define OVER_RANGE1() GUI_showMessage("复位速度需要大于送料速度，请重新设置！", 38)
//#define UNUSED() GUI_showMessage("此功能未开启，已被禁止使用！", 28)

#define OVER_RANGE_5_2000() 		GUI_showMessage("参数越界,设置范围5-2000", 28)
#define EN_OVER_RANGE_5_2000() 	GUI_showMessage("Parameter is invalid,setting range:5-2000", 41)

#define OVER_RANGE_10_2000() 		GUI_showMessage("参数越界,设置范围10-2000", 29)
#define EN_OVER_RANGE_10_2000() 	GUI_showMessage("Parameter is invalid,setting range:10-2000", 42)

#define OVER_RANGE_1_999() 			GUI_showMessage("参数越界,设置范围1-999", 27)
#define EN_OVER_RANGE_1_999() 		GUI_showMessage("Parameter is invalid,setting range:1-999", 40)

#define OVER_RANGE_5_500() 			GUI_showMessage("参数越界,设置范围5-500", 27)
#define EN_OVER_RANGE_5_500() 		GUI_showMessage("Parameter is invalid,setting range:5-500", 40)

#define OVER_RANGE_1_500() 			GUI_showMessage("参数越界,设置范围1-500", 27)
#define EN_OVER_RANGE_1_500() 		GUI_showMessage("Parameter is invalid,setting range:1-500", 40)

#define OVER_RANGE_1_200() 			GUI_showMessage("参数越界,设置范围1-200", 27)
#define EN_OVER_RANGE_1_200() 		GUI_showMessage("Parameter is invalid,setting range:1-200", 40)

#define OVER_RANGE_1_150() 			GUI_showMessage("参数越界,设置范围1-150", 27)
#define EN_OVER_RANGE_1_150() 		GUI_showMessage("Parameter is invalid,setting range:1-150", 40)

#define OVER_RANGE_1_100() 			GUI_showMessage("参数越界,设置范围1-100", 27)
#define EN_OVER_RANGE_1_100() 		GUI_showMessage("Parameter is invalid,setting range:1-100", 40)

#define OVER_RANGE_1_50() 			GUI_showMessage("参数越界,设置范围1-50", 27)
#define EN_OVER_RANGE_1_50() 		GUI_showMessage("Parameter is invalid,setting range:1-50", 39)

#define OVER_RANGE_1_4() 			GUI_showMessage("参数越界,设置范围1-4", 25)
#define EN_OVER_RANGE_1_4() 		GUI_showMessage("Parameter is invalid,setting range:1-4", 38)

#define OVER_RANGE_0_300()	 		GUI_showMessage("参数越界,设置范围0-300", 27)
#define EN_OVER_RANGE_0_300()	 	GUI_showMessage("Parameter is invalid,setting range:0-300", 40)

#define OVER_RANGE_0_500() 			GUI_showMessage("参数越界,设置范围0-500", 27)
#define EN_OVER_RANGE_0_500() 		GUI_showMessage("Parameter is invalid,setting range:0-500", 40)

#define OVER_RANGE_0_10() 			GUI_showMessage("参数越界,设置范围0-10", 26)
#define EN_OVER_RANGE_0_10() 		GUI_showMessage("Parameter is invalid,setting range:0-10", 39)

#define OVER_RANGE_01_25() 			GUI_showMessage("参数越界,设置范围0.1-2.5", 29)
#define EN_OVER_RANGE_01_25() 		GUI_showMessage("Parameter is invalid,setting range:0.1-2.5", 42)


// S型加减速
#define     CONF_S_TYPE_ACC    
#define     CONF_S_LEN                      100
#define     CONF_S_FACTOR_NO                0       // 选择加速曲线
#define     CONF_S_SPEED_MIN                600     // 最小速度
#define     CONF_S_TIMER_FREQ               200000  // 定时器时钟频率, 即分频后频率

#define     CONF_SOFTWARE_WAIT_INPUT_STABLE         // 软件防抖
#define     CONF_SOFTWARE_WAIT_INPUT_TIME      2    // 软件延时时间

#endif

