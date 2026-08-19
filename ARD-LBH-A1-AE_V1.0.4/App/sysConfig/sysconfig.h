#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_

#define     CONF_FEEDER_VER        	1   //0-A款
										//1-C款

#define     CONF_LET_VER           	1  	//0-第二通道为下收料
										//1-第二通道为放料

#define		CONF_BLRST_VER			0 	//0-断开剥料信号自动复位
										//1-收到送料信号再复位，再送料

#define     CONF_Fiber_VER			1 	//0-单光纤检测物料
										//1-双光纤检测物料

#define		CONF_COUNT_VER			1  	// 1 -换料结束后，送料计数清零

// 加速度 单位: Hz/(10ms) -------未使用
#define     CONF_GM_ACC          300
#define     CONF_GM_DEC          1000

#define     CONF_LM_ACC          500
#define     CONF_LM_DEC          1000

#define     CONF_BM_ACC          100   
#define     CONF_BM_DEC          500   

#define     CONF_SM_ACC          500
#define     CONF_SM_DEC          1000

// 全步时, 运动1mm所需脉冲数, 需考虑减速机 --标准后撤飞达
#define     CONF_GM_MM_PULSE         33      // 送料电机 1:18 同步轮 一圈转113mm   细分800
#define     CONF_LM_MM_PULSE         20     // 放料电机
#define     CONF_BM_MM_PULSE         20      // 剥料电机 丝杆1：10 小同步轮    细分800
#define     CONF_SM_MM_PULSE         20     // 收料电机

// 4个电机对应通道号
#define     CONF_GM_NO         0       ///< 送料电机
#define     CONF_LM_NO         1       ///< 放料电机
#define     CONF_BM_NO         2       ///< 剥料电机
#define     CONF_SM_NO         3       ///< 收料电机

// 默认运行参数, 2位定点, 设定2000时, 实际值为20.00
#define     CONF_GM_SPEED       5000    ///< 送料速度
#define     CONF_GM_OFFSET      500    	///< 送料限位
#define     CONF_LM_SPEED       5000    ///< 放料速度
#define     CONF_LM_OFFSET      500     ///< 放料偏移
#define     CONF_BM_SPEED       5000    ///< 剥料速度
#define     CONF_BM_OFFSET      2000    ///< 剥料偏移
#define     CONF_SM_SPEED       5000    ///< 收料速度
#define     CONF_SM_DISTANCE    500    ///< 收料行程
#define     CONF_GOZERO_SPEED   3000     ///< 回原点速度
#define     CONF_ZERO_OFFSET    0       ///< 原点偏移
#define     CONF_GM_AMP         40     ///< 送料电机电流
#define     CONF_GM_MS          3       ///< 送料电机细分
#define     CONF_LM_AMP         120      ///< 放料电机电流
#define     CONF_LM_MS          3       ///< 放料电机细分
#define     CONF_BM_AMP         32     ///< 剥料电机电流
#define     CONF_BM_MS          3       ///< 剥料电机细分
#define     CONF_SM_AMP         120      ///< 收料电机电流
#define     CONF_SM_MS          3       ///< 收料电机细分

#define		CONF_HC_D2			0		///< 第二次后撤距离
#define		CONF_BS_POINT		0		///< 变速位置//无法保存，有时会丢失
#define     CONF_RST_SPEED      5000    ///< 复位速度
#define     CONF_GM_DISTANCE	0		///< 送料补偿
#define		CONF_BS1_POINT		0		///< 变速位置

#define     CONF_GM_JOGSPEED    10    ///< 送料电机点动速度, 2位定点小数
#define     CONF_LM_JOGSPEED    10    ///< 放料电机点动速度, 2位定点小数
#define     CONF_BM_JOGSPEED    10    ///< 剥料电机点动速度, 2位定点小数
#define     CONF_SM_JOGSPEED    10    ///< 收料电机点动速度, 2位定点小数

#define     CONF_LOWGM_SPEED     	3000    ///< 送料变速速度
#define     CONF_LOWBO_DISTANCE		0		///< 慢速剥料距离
#define		CONF_LOWBO_SPEED		3000	///< 慢速剥料速度
#define		CONF_HL_DISTANCE		4000	///< 换料后退行程

#define     CONF_USER_PASSWORD              "000000"    ///< 默认用户密码
#define     CONF_VENDOR_PASSWORD            "654321"    ///< 默认供应商密码
#define     CONF_SUPER_PASSWORD             "729183"    ///< 超级密码

// 版本显示页显示内容, 修改时注意字符串与字符串长度需对应
#define     CONF_DEVICE_NAME                "后撤卷式供料器"
#define     CONF_DEVICE_NAME_LEN            23
#define     EN_CONF_DEVICE_NAME             "Retractable roll feeder"
#define     EN_CONF_DEVICE_NAME_LEN         23
#if CONF_LET_VER == 0
	#if CONF_Fiber_VER == 0
		#define     CONF_DEVICE_NO                  "ARD-LBH-A1-AE-CNEN"
	#else
		#define     CONF_DEVICE_NO                  "ARD-LBH-A11-AE-CNEN"
	#endif
#else
	#if CONF_Fiber_VER == 0
		#define     CONF_DEVICE_NO                  "ARD-LBH-A2-AE-CNEN"
	#else
		#define     CONF_DEVICE_NO                  "ARD-LBH-A21-AE-CNEN"
	#endif
#endif
#define     CONF_DEVICE_NO_LEN              18
#define     CONF_HARDWARE_VER               "ARD-22M-12I8O-AE-A"
#define     CONF_HARDWARE_VER_LEN           18

#if	CONF_COUNT_VER == 1
		#define     CONF_SOFTWARE_VER           "V1.0.4 20260319"
#else
		#define     CONF_SOFTWARE_VER           "V1.0.4 20260319"
#endif
#define     CONF_SOFTWARE_VER_LEN           15

// 外部触发方式
// 删除该宏定义, 则为扫描方式, 否则为中断方式
//#define     CONF_EXTRIG_INT                 

// 输入信号页输入信号名称
#define     CONF_INPUT_00_NAME              		"供料器原点信号"
#define     CONF_INPUT_00_NAME_LEN          		20

#define     EN_CONF_INPUT_00_NAME           		"Feeder origin signal"
#define     EN_CONF_INPUT_00_NAME_LEN       		20
#if CONF_Fiber_VER == 0
//标准上下光纤检测方式
#define     CONF_INPUT_01_NAME              		"下光纤检测信号"
#define     CONF_INPUT_01_NAME_LEN          		21

#define     EN_CONF_INPUT_01_NAME           		"Lower detection fiber"
#define     EN_CONF_INPUT_01_NAME_LEN       		21

#define     CONF_INPUT_02_NAME              		"上光纤检测信号"
#define     CONF_INPUT_02_NAME_LEN          		21

#define     EN_CONF_INPUT_02_NAME           		"Upper detection fiber"
#define     EN_CONF_INPUT_02_NAME_LEN       		21

//慢速检测+下光纤定位检测
#define     CONF_INPUT_001_NAME            		 	"标签定位信号"
#define     CONF_INPUT_001_NAME_LEN        		 	21

#define     EN_CONF_INPUT_001_NAME          		"Label location signal"
#define     EN_CONF_INPUT_001_NAME_LEN      		21

#define     CONF_INPUT_002_NAME             		"开始减速信号"
#define     CONF_INPUT_002_NAME_LEN         		21

#define     EN_CONF_INPUT_002_NAME          		"Start slowdown signal"
#define     EN_CONF_INPUT_002_NAME_LEN      		21

//上光纤定位+下光纤检测有无
#define     CONF_INPUT_0001_NAME            		"标签有无信号"//real-time tag signal
#define     CONF_INPUT_0001_NAME_LEN        		21

#define     EN_CONF_INPUT_0001_NAME         		"Real-time tag signal"//real-time tag signal
#define     EN_CONF_INPUT_0001_NAME_LEN     		21

#define     CONF_INPUT_0002_NAME            		"标签定位信号"
#define     CONF_INPUT_0002_NAME_LEN       	 		21

#define     EN_CONF_INPUT_0002_NAME         		"Label location signal"
#define     EN_CONF_INPUT_0002_NAME_LEN    	 		21

#else
//标准上下光纤检测方式
#define     CONF_INPUT_01_NAME              		"光纤检测信号1"
#define     CONF_INPUT_01_NAME_LEN          		14

#define     EN_CONF_INPUT_01_NAME           		"Fiber signal 1"
#define     EN_CONF_INPUT_01_NAME_LEN       		14

#define     CONF_INPUT_02_NAME              		"光纤检测信号2"
#define     CONF_INPUT_02_NAME_LEN          		14

#define     EN_CONF_INPUT_02_NAME           		"Fiber signal 2"
#define     EN_CONF_INPUT_02_NAME_LEN       		14
#endif

#define     CONF_INPUT_03_NAME              		"标签卷料检测信号"
#define     CONF_INPUT_03_NAME_LEN          		27

#define     EN_CONF_INPUT_03_NAME           		"Label roll detection signal"
#define     EN_CONF_INPUT_03_NAME_LEN       		27
#if CONF_LET_VER == 1

	#define     CONF_INPUT_04_NAME           		"收料启动信号"
	#define     CONF_INPUT_04_NAME_LEN         		18

	#define     EN_CONF_INPUT_04_NAME              	"film start signal "
	#define     EN_CONF_INPUT_04_NAME_LEN         	18

	#define     CONF_INPUT_004_NAME              	"备用"
	#define     CONF_INPUT_004_NAME_LEN          	18

	#define     EN_CONF_INPUT_004_NAME              "Spare"
	#define     EN_CONF_INPUT_004_NAME_LEN          18

	#define     CONF_INPUT_05_NAME              	"收料停止信号"
	#define     CONF_INPUT_05_NAME_LEN          	16

	#define     EN_CONF_INPUT_05_NAME              	"film stop signal"
	#define     EN_CONF_INPUT_05_NAME_LEN          	16

	#define     CONF_INPUT_005_NAME              	"备用"
	#define     CONF_INPUT_005_NAME_LEN          	16

	#define     EN_CONF_INPUT_005_NAME              "Spare"
	#define     EN_CONF_INPUT_005_NAME_LEN          16

	#define     CONF_INPUT_06_NAME              	"放料启动信号"
	#define     CONF_INPUT_06_NAME_LEN          	22

	#define     EN_CONF_INPUT_06_NAME              	"Unwinding start signal"
	#define     EN_CONF_INPUT_06_NAME_LEN          	22

	#define     CONF_INPUT_006_NAME              	"备用"
	#define     CONF_INPUT_006_NAME_LEN          	22

	#define     EN_CONF_INPUT_006_NAME              "Spare"
	#define     EN_CONF_INPUT_006_NAME_LEN          22

	#define     CONF_INPUT_07_NAME              	"放料停止信号"
	#define     CONF_INPUT_07_NAME_LEN          	21

	#define     EN_CONF_INPUT_07_NAME              	"Unwinding stop signal"
	#define     EN_CONF_INPUT_07_NAME_LEN          	21

	#define     CONF_INPUT_007_NAME              	"备用"
	#define     CONF_INPUT_007_NAME_LEN          	21

	#define     EN_CONF_INPUT_007_NAME              "Spare"
	#define     EN_CONF_INPUT_007_NAME_LEN          21

#else
	#define     CONF_INPUT_04_NAME              	"上收料启动信号"
	#define     CONF_INPUT_04_NAME_LEN          	23

	#define     EN_CONF_INPUT_04_NAME              	"Upper film start signal"
	#define     EN_CONF_INPUT_04_NAME_LEN          	23

	#define     CONF_INPUT_004_NAME              	"备用"
	#define     CONF_INPUT_004_NAME_LEN          	23

	#define     EN_CONF_INPUT_004_NAME              "Spare"
	#define     EN_CONF_INPUT_004_NAME_LEN          23

	#define     CONF_INPUT_05_NAME              	"上收料停止信号"
	#define     CONF_INPUT_05_NAME_LEN          	22

	#define     EN_CONF_INPUT_05_NAME              	"Upper film stop signal"
	#define     EN_CONF_INPUT_05_NAME_LEN          	22

	#define     CONF_INPUT_005_NAME              	"备用"
	#define     CONF_INPUT_005_NAME_LEN          	22

	#define     EN_CONF_INPUT_005_NAME              "Spare"
	#define     EN_CONF_INPUT_005_NAME_LEN          22

	#define     CONF_INPUT_06_NAME              	"下收料启动信号"
	#define     CONF_INPUT_06_NAME_LEN          	23

	#define     EN_CONF_INPUT_06_NAME             	"Lower film start signal"
	#define     EN_CONF_INPUT_06_NAME_LEN         	23

	#define     CONF_INPUT_006_NAME              	"备用"
	#define     CONF_INPUT_006_NAME_LEN          	23

	#define     EN_CONF_INPUT_006_NAME              "Spare"
	#define     EN_CONF_INPUT_006_NAME_LEN          23

	#define     CONF_INPUT_07_NAME              	"下收料停止信号"
	#define     CONF_INPUT_07_NAME_LEN          	22

	#define     EN_CONF_INPUT_07_NAME              	"Lower film stop signal"
	#define     EN_CONF_INPUT_07_NAME_LEN          	22

	#define     CONF_INPUT_007_NAME              	"备用"
	#define     CONF_INPUT_007_NAME_LEN          	22

	#define     EN_CONF_INPUT_007_NAME              "Spare"
	#define     EN_CONF_INPUT_007_NAME_LEN          22
#endif

#define     CONF_INPUT_08_NAME              		"传感器气缸原点信号"
#define     CONF_INPUT_08_NAME_LEN          		22

#define     EN_CONF_INPUT_08_NAME              		"Sensor cylinder origin"
#define     EN_CONF_INPUT_08_NAME_LEN          		22

#define     CONF_INPUT_008_NAME              		"备用"
#define     CONF_INPUT_008_NAME_LEN          		22

#define     EN_CONF_INPUT_008_NAME              	"Spare"
#define     EN_CONF_INPUT_008_NAME_LEN          	22

#define     CONF_INPUT_09_NAME              		"传感器气缸动点信号"
#define     CONF_INPUT_09_NAME_LEN          		22

#define     EN_CONF_INPUT_09_NAME              		"Sensor cylinder moving"
#define     EN_CONF_INPUT_09_NAME_LEN          		22

#define     CONF_INPUT_009_NAME              		"备用"
#define     CONF_INPUT_009_NAME_LEN         		22

#define     EN_CONF_INPUT_009_NAME             		"Spare"
#define     EN_CONF_INPUT_009_NAME_LEN          	22

#define     CONF_INPUT_10_NAME              		"剥刀复位检测信号"
#define     CONF_INPUT_10_NAME_LEN          		27

#define     EN_CONF_INPUT_10_NAME              		"Anti-pinch detection signal"
#define     EN_CONF_INPUT_10_NAME_LEN         	 	27

#define     CONF_INPUT_11_NAME              		"备用"
#define     CONF_INPUT_11_NAME_LEN          		22

#define     EN_CONF_INPUT_11_NAME              		"Spare"
#define     EN_CONF_INPUT_11_NAME_LEN          		22

#define     CONF_INPUT_011_NAME              		"内部急停信号"//常闭型
#define     CONF_INPUT_011_NAME_LEN          		22

#define     EN_CONF_INPUT_011_NAME              	"Internal E-stop signal"//常闭型
#define     EN_CONF_INPUT_011_NAME_LEN          	22

#if CONF_FEEDER_VER == 1
	// 供料器对外输入信号名称
	#define 	CONF_EXI0_NAME						"送料信号"
	#define 	CONF_EXI0_NAME_LEN					11

	#define 	EN_CONF_EXI0_NAME					"Feed signal"
	#define 	EN_CONF_EXI0_NAME_LEN				11

	#define 	CONF_EXI1_NAME						"剥料信号"
	#define 	CONF_EXI1_NAME_LEN					16

	#define 	EN_CONF_EXI1_NAME					"Stripping signal"
	#define 	EN_CONF_EXI1_NAME_LEN				16

	#define 	CONF_EXI2_NAME						"初始化信号"
	#define 	CONF_EXI2_NAME_LEN					21

	#define 	EN_CONF_EXI2_NAME					"Initialization signal"
	#define 	EN_CONF_EXI2_NAME_LEN				21

	#define 	CONF_EXI3_NAME						"停止信号 "
	#define 	CONF_EXI3_NAME_LEN					12

	#define 	EN_CONF_EXI3_NAME					"Stop signal"
	#define 	EN_CONF_EXI3_NAME_LEN				11
#else
	#define 	CONF_EXI0_NAME						"初始化信号"
	#define 	CONF_EXI0_NAME_LEN					21

	#define 	EN_CONF_EXI2_NAME					"Initialization signal"
	#define 	EN_CONF_EXI2_NAME_LEN				21


	#define 	CONF_EXI1_NAME						"送料信号"
	#define 	CONF_EXI1_NAME_LEN					15

	#define 	EN_CONF_EXI0_NAME					"Feed signal"
	#define 	EN_CONF_EXI0_NAME_LEN				11

	#define 	CONF_EXI2_NAME						"剥料信号"
	#define 	CONF_EXI2_NAME_LEN					16

	#define 	EN_CONF_EXI1_NAME					"Stripping signal"
	#define 	EN_CONF_EXI1_NAME_LEN				16

	#define 	CONF_EXI3_NAME						"停止信号"
	#define 	CONF_EXI3_NAME_LEN					12

	#define 	EN_CONF_EXI3_NAME					"Stop signals"
	#define 	EN_CONF_EXI3_NAME_LEN				12
#endif

#define 	CONF_EXI4_NAME							"备用"
#define 	CONF_EXI4_NAME_LEN						5

#define 	EN_CONF_EXI4_NAME						"Spare"
#define 	EN_CONF_EXI4_NAME_LEN					5

#define 	CONF_EXI5_NAME							"备用"
#define 	CONF_EXI5_NAME_LEN						5

#define 	EN_CONF_EXI5_NAME						"Spare"
#define 	EN_CONF_EXI5_NAME_LEN					5

#define 	CONF_EXI6_NAME							"备用"
#define 	CONF_EXI6_NAME_LEN						5

#define 	EN_CONF_EXI6_NAME						"Spare"
#define 	EN_CONF_EXI6_NAME_LEN					5

#define 	CONF_EXI7_NAME							"备用"
#define 	CONF_EXI7_NAME_LEN						5

#define 	EN_CONF_EXI7_NAME						"Spare"
#define 	EN_CONF_EXI7_NAME_LEN					5

// 供料器内部控制信号名称显示
#define     CONF_INNER_CTRL_NAME_1          		"压料气缸"
#define     CONF_INNER_CTRL_NAME_1_LEN      		14

#define     EN_CONF_INNER_CTRL_NAME_1         	 	"Press Cylinder"
#define     EN_CONF_INNER_CTRL_NAME_1_LEN     		14

#define     CONF_INNER_CTRL_NAME_2          		"夹料气缸"
#define     CONF_INNER_CTRL_NAME_2_LEN      		14

#define     EN_CONF_INNER_CTRL_NAME_2         	 	"Clamp Cylinder"
#define     EN_CONF_INNER_CTRL_NAME_2_LEN      		14

#define     CONF_INNER_CTRL_NAME_3          		"传感器气缸"
#define     CONF_INNER_CTRL_NAME_3_LEN      		15

#define     EN_CONF_INNER_CTRL_NAME_3          		"Sensor Cylinder"
#define     EN_CONF_INNER_CTRL_NAME_3_LEN      		15

#define     CONF_INNER_CTRL_NAME_03          		"备用"
#define     CONF_INNER_CTRL_NAME_03_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_03          	"Spare"
#define     EN_CONF_INNER_CTRL_NAME_03_LEN      	5

#define     CONF_INNER_CTRL_NAME_4          		"备用"
#define     CONF_INNER_CTRL_NAME_4_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_4          		"Spare"
#define     EN_CONF_INNER_CTRL_NAME_4_LEN      		5

#define     CONF_INNER_CTRL_NAME_5          		"备用"
#define     CONF_INNER_CTRL_NAME_5_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_5          		"Spare"
#define     EN_CONF_INNER_CTRL_NAME_5_LEN      		5

#define     CONF_INNER_CTRL_NAME_6          		"备用"
#define     CONF_INNER_CTRL_NAME_6_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_6          		"Spare"
#define     EN_CONF_INNER_CTRL_NAME_6_LEN      		5

#define     CONF_INNER_CTRL_NAME_7          		"备用"
#define     CONF_INNER_CTRL_NAME_7_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_7          		"Spare"
#define     EN_CONF_INNER_CTRL_NAME_7_LEN      		5

#define     CONF_INNER_CTRL_NAME_8          		"备用"
#define     CONF_INNER_CTRL_NAME_8_LEN      		5

#define     EN_CONF_INNER_CTRL_NAME_8          		"Spare"
#define     EN_CONF_INNER_CTRL_NAME_8_LEN      		5

#if CONF_Fiber_VER == 0
	// 供料器对外部控制信号名称
	#define     CONF_OUTER_CTRL_NAME_1          		"初始化完成信号"
	#define     CONF_OUTER_CTRL_NAME_1_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_1          		" Initialize OK"
	#define     EN_CONF_OUTER_CTRL_NAME_1_LEN      		14

	#define		EXI_FEEDER								EXIO_getInput(0)

	#define     CONF_OUTER_CTRL_NAME_2          		"异常报警信号"
	#define     CONF_OUTER_CTRL_NAME_2_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_2         		"Abnormal alarm"
	#define     EN_CONF_OUTER_CTRL_NAME_2_LEN      		14

	#define     CONF_OUTER_CTRL_NAME_3          		"剥料完成信号"
	#define     CONF_OUTER_CTRL_NAME_3_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_3          		"Strip complete"
	#define     EN_CONF_OUTER_CTRL_NAME_3_LEN      		14

	#define     CONF_OUTER_CTRL_NAME_4          		" 送料完成信号 "
	#define     CONF_OUTER_CTRL_NAME_4_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_4          		" Feed complete"
	#define     EN_CONF_OUTER_CTRL_NAME_4_LEN      		14
#else
	// 供料器对外部控制信号名称
	#define     CONF_OUTER_CTRL_NAME_1          		"初始化完成信号"
	#define     CONF_OUTER_CTRL_NAME_1_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_1          		" Initialize OK"
	#define     EN_CONF_OUTER_CTRL_NAME_1_LEN      		14

	#define		EXI_FEEDER								EXIO_getInput(0)

	#define     CONF_OUTER_CTRL_NAME_2          		" 平台有无物料2"
	#define     CONF_OUTER_CTRL_NAME_2_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_2         		"Feed complete2"
	#define     EN_CONF_OUTER_CTRL_NAME_2_LEN      		14

	#define     CONF_OUTER_CTRL_NAME_3          		"剥料完成信号"
	#define     CONF_OUTER_CTRL_NAME_3_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_3          		"Strip complete"
	#define     EN_CONF_OUTER_CTRL_NAME_3_LEN      		14

	#define     CONF_OUTER_CTRL_NAME_4          		" 平台有无物料1"
	#define     CONF_OUTER_CTRL_NAME_4_LEN      		14

	#define     EN_CONF_OUTER_CTRL_NAME_4          		"Feed complete1"
	#define     EN_CONF_OUTER_CTRL_NAME_4_LEN      		14
#endif
#define     CONF_OUTER_CTRL_NAME_5          		"备用"
#define     CONF_OUTER_CTRL_NAME_5_LEN      		5

#define     EN_CONF_OUTER_CTRL_NAME_5         	 	"Spare"
#define     EN_CONF_OUTER_CTRL_NAME_5_LEN      		5

#define     CONF_OUTER_CTRL_NAME_6          		"备用"
#define     CONF_OUTER_CTRL_NAME_6_LEN      		5

#define     EN_CONF_OUTER_CTRL_NAME_6          		"Spare"
#define     EN_CONF_OUTER_CTRL_NAME_6_LEN      		5

#define     CONF_OUTER_CTRL_NAME_7          		"备用"
#define     CONF_OUTER_CTRL_NAME_7_LEN      		5

#define     EN_CONF_OUTER_CTRL_NAME_7          		"Spare"
#define     EN_CONF_OUTER_CTRL_NAME_7_LEN      		5

#define     CONF_OUTER_CTRL_NAME_8          		"备用"
#define     CONF_OUTER_CTRL_NAME_8_LEN      		5

#define     EN_CONF_OUTER_CTRL_NAME_8          		"Spare"
#define     EN_CONF_OUTER_CTRL_NAME_8_LEN      		5

#define     USERNAME_DISP_NAME          			"用户登录"
#define     USERNAME_DISP_NAME_LEN      			10

#define     EN_USERNAME_DISP_NAME          			"User Login"
#define     EN_USERNAME_DISP_NAME_LEN      			10

#define     LANGUAGE_DISP_NAME         	 			"中文"
#define     LANGUAGE_DISP_NAME_LEN      			8

#define     EN_LANGUAGE_DISP_NAME          			"English"
#define     EN_LANGUAGE_DISP_NAME_LEN      			8


#define POP_UP_INFO_RES_E_SOTP() 			GUI_showMessage("等待解除急停！！！",18)
#define EN_POP_UP_INFO_RES_E_SOTP() 		GUI_showMessage("Please release the E-stop!!!",28)

#define POP_UP_INFO_AGING_RUN() 			GUI_showMessage("空跑模式运行中！",16)
#define EN_POP_UP_INFO_AGING_RUN() 			GUI_showMessage("Aging mode is running!",22)


#define MAIN_INFO_YL_CYL_OUT()				GUI_mainMessageDisp("压料气缸伸出.", 13)
#define EN_MAIN_INFO_YL_CYL_OUT()			GUI_mainMessageDisp("The pressing cylinder extends.", 30)

#define MAIN_INFO_YL_CYL_IN()				GUI_mainMessageDisp("压料气缸缩回.", 13)
#define EN_MAIN_INFO_YL_CYL_IN()			GUI_mainMessageDisp("The pressing cylinder retracts.", 31)

#define MAIN_INFO_JL_CYL_OUT()				GUI_mainMessageDisp("夹料气缸伸出.", 13)
#define EN_MAIN_INFO_JL_CYL_OUT()			GUI_mainMessageDisp("The clamp cylinder extends.", 27)

#define MAIN_INFO_JL_CYL_IN()				GUI_mainMessageDisp("夹料气缸缩回.", 13)
#define EN_MAIN_INFO_JL_CYL_IN()			GUI_mainMessageDisp("The Sensor cylinder retracts.", 29)

#define MAIN_INFO_USL_CYL_OUT()				GUI_mainMessageDisp("传感器气缸伸出.", 15)
#define EN_MAIN_INFO_USL_CYL_OUT()			GUI_mainMessageDisp("The Sensor cylinder extends.", 28)

#define MAIN_INFO_USL_CYL_IN()				GUI_mainMessageDisp("传感器气缸缩回.", 15)
#define EN_MAIN_INFO_USL_CYL_IN()			GUI_mainMessageDisp("The Sensor cylinder retracts.", 29)

#define MAIN_INFO_SW2DEBUG()				GUI_mainMessageDisp("切换到调试模式.", 15)
#define EN_MAIN_INFO_SW2DEBUG()				GUI_mainMessageDisp("Switch to debug mode.", 21)

#define MAIN_INFO_SW2ONLINE()				GUI_mainMessageDisp("切换到联机模式.", 15)
#define EN_MAIN_INFO_SW2ONLINE()			GUI_mainMessageDisp("Switch to online mode.", 22)

#define MAIN_INFO_SW2AGING()				GUI_mainMessageDisp("切换到空跑模式.", 15)
#define EN_MAIN_INFO_SW2AGING()				GUI_mainMessageDisp("Switch to aning mode.", 21)

#define MAIN_TIPS_INNI_FIRST()				GUI_mainMessageDisp("提示信息：请先初始动作.", 23)
#define EN_MAIN_TIPS_INNI_FIRST()			GUI_mainMessageDisp("Tips:Please initialize first.", 29)

#define MAIN_TIPS_HAVE_MATERIAL()			GUI_mainMessageDisp("提示信息：送料失败，检测到料！", 30)
#define EN_MAIN_TIPS_HAVE_MATERIAL()		GUI_mainMessageDisp("Tips:Feeding failed, label detected!", 36)

#define MAIN_TIPS_NO_MATERIAL() 			GUI_mainMessageDisp("提示信息：送料失败，未检测到料！", 32)
#define EN_MAIN_TIPS_NO_MATERIAL() 			GUI_mainMessageDisp("Tips:Feeding failed, label not detected!", 40)

#define MAIN_TIPS_WAIT_RST_E_STOP() 		GUI_mainMessageDispIsolate("提示信息：等待解除急停！", 24)
#define EN_MAIN_TIPS_WAIT_RST_E_STOP() 		GUI_mainMessageDispIsolate("Tips:Waiting for release E-stop!", 32)

#define MAIN_TIPS_RST_E_STOP() 				GUI_mainMessageDispIsolate("提示信息：解除急停！", 20)
#define EN_MAIN_TIPS_RST_E_STOP() 			GUI_mainMessageDispIsolate("Tips:Release E-stop!", 20)

#define MAIN_TIPS_EXI_E_STOP() 				GUI_mainMessageDisp("提示信息：等待解除外部急停！", 32)
#define EN_MAIN_TIPS_EXI_E_STOP() 			GUI_mainMessageDisp("Tips:External external E-stop state!", 36)

#define MAIN_TIPS_RST_EXI_E_STOP() 			GUI_mainMessageDispIsolate("提示信息：解除外部急停！", 24)
#define EN_MAIN_TIPS_RST_EXI_E_STOP() 		GUI_mainMessageDispIsolate("Tips:Release external E-stop!", 29)

#define ERR_INFO_USL_CYL_OUT_TIMEOUT() 		GUI_mainMessageDisp("报警信息：传感器气缸伸出异常！", 30)
#define EN_ERR_INFO_USL_CYL_OUT_TIMEOUT() 	GUI_mainMessageDisp("Alarm info:abnormal extension of sensor cylinder!", 49)

#define ERR_INFO_USL_CYL_IN_TIMEOUT() 		GUI_mainMessageDisp("报警信息：传感器气缸缩回异常!", 29)
#define EN_ERR_INFO_USL_CYL_IN_TIMEOUT() 	GUI_mainMessageDisp("Alarm info:abnormal retracts of sensor cylinder!", 48)

#define ERR_INFO_GO_ORG_TIMEOUT() 			GUI_mainMessageDisp("报警信息：未检测到原点信号！", 28)
#define EN_ERR_INFO_GO_ORG_TIMEOUT() 		GUI_mainMessageDisp("Alarm info:No origin signal detected!", 37)

#define ERR_INFO_AVOID_ORG_TIMEOUT() 		GUI_mainMessageDisp("报警信息：避开原点失败！", 24)
#define EN_ERR_INFO_AVOID_ORG_TIMEOUT() 	GUI_mainMessageDisp("Alarm info:Failed to avoid origin!", 34)

#define ERR_INFO_STEPBACK_TIMEOUT() 		GUI_mainMessageDisp("报警信息：剥料后撤失败！", 24)
#define EN_ERR_INFO_STEPBACK_TIMEOUT() 		GUI_mainMessageDisp("Alarm info:Failed to peel back!", 31)

#define ERR_INFO_OVRE_MATERIAL() 			GUI_mainMessageDisp("报警信息：料带用完，请补充新料！", 32)
#define EN_ERR_INFO_OVRE_MATERIAL() 		GUI_mainMessageDisp("Alarm info:The material over,please add new material!", 53)

#define ERR_INFO_NO_MATERIAL() 				GUI_mainMessageDisp("报警信息：送料失败，未检测到料！", 32)
#define EN_ERR_INFO_NO_MATERIAL() 			GUI_mainMessageDisp("Alarm info:Feeding failed, label not detected!", 46)

#define CANT_SWITCH_PAGE_MESSAGE()       	GUI_showMessage("先停止任务",10)
#define EN_CANT_SWITCH_PAGE_MESSAGE()       GUI_showMessage("Stop the task first",19)

#define CANT_SWITCH_PAGE_MESSAGE0()      	GUI_showMessage("请先初始化操作！",16)
#define EN_CANT_SWITCH_PAGE_MESSAGE0()      GUI_showMessage("Please initialize first!",24)

#define CANT_SWITCH_PAGE_MESSAGE1()     	GUI_showMessage("联机模式禁止操作！",32)
#define EN_CANT_SWITCH_PAGE_MESSAGE1()      GUI_showMessage("Online mode prohibits operation!",32)

#define CANT_SWITCH_PAGE_MESSAGE2()      	GUI_showMessage("请等待当前动作结束！",35)
#define EN_CANT_SWITCH_PAGE_MESSAGE2()      GUI_showMessage("wait for the current action to end!",35)

#define CANT_SWITCH_PAGE_MESSAGE3()      	GUI_showMessage("空跑模式禁止操作！",38)
#define EN_CANT_SWITCH_PAGE_MESSAGE3()      GUI_showMessage("Aging mode inhibits operation!",30)

#define CANT_SWITCH_PAGE_MESSAGE4()     	GUI_showMessage("传感器减速功能已开启，请关闭！",30)

#define CANT_SWITCH_PAGE_MESSAGE5() 		GUI_showMessage("当前感应器模式不可选此模式！",28)

#define MAXSPEED	20000

#define OVER_RANGE() 						GUI_showMessage("参数越界", 8)
#define EN_OVER_RANGE() 					GUI_showMessage("parameter out of bounds", 23)

//#define OVER_RANGE1() 					GUI_showMessage("复位速度需要大于送料速度，请重新设置！", 38)
//#define UNUSED() 							GUI_showMessage("此功能未开启，已被禁止使用！", 28)

#define OVER_RANGE_10_2000() 				GUI_showMessage("参数越界,设置范围10-2000", 24)
#define EN_OVER_RANGE_10_2000() 			GUI_showMessage("Parameter is invalid,setting range:10-2000", 42)

#define OVER_RANGE_2_2000() 				GUI_showMessage("参数越界,设置范围2-2000", 23)
#define EN_OVER_RANGE_2_2000() 				GUI_showMessage("Parameter is invalid,setting range:2-2000", 41)

#define OVER_RANGE_1_999() 					GUI_showMessage("参数越界,设置范围1-999", 22)
#define EN_OVER_RANGE_1_999() 				GUI_showMessage("Parameter is invalid,setting range:1-999", 40)

#define OVER_RANGE_5_500() 					GUI_showMessage("参数越界,设置范围5-500", 22)
#define EN_OVER_RANGE_5_500() 				GUI_showMessage("Parameter is invalid,setting range:5-500", 40)

#define OVER_RANGE_1_500() 					GUI_showMessage("参数越界,设置范围1-500", 22)
#define EN_OVER_RANGE_1_500() 				GUI_showMessage("Parameter is invalid,setting range:1-500", 40)

#define OVER_RANGE_1_200() 					GUI_showMessage("参数越界,设置范围1-200", 22)
#define EN_OVER_RANGE_1_200() 				GUI_showMessage("Parameter is invalid,setting range:1-200", 40)

#define OVER_RANGE_1_150() 					GUI_showMessage("参数越界,设置范围1-150", 22)
#define EN_OVER_RANGE_1_150() 				GUI_showMessage("Parameter is invalid,setting range:1-150", 40)

#define OVER_RANGE_1_100() 					GUI_showMessage("参数越界,设置范围1-100", 22)
#define EN_OVER_RANGE_1_100() 				GUI_showMessage("Parameter is invalid,setting range:1-100", 40)

#define OVER_RANGE_1_50() 					GUI_showMessage("参数越界,设置范围1-50", 21)
#define EN_OVER_RANGE_1_50() 				GUI_showMessage("Parameter is invalid,setting range:1-50", 39)

#define OVER_RANGE_1_4() 					GUI_showMessage("参数越界,设置范围1-4", 20)
#define EN_OVER_RANGE_1_4() 				GUI_showMessage("Parameter is invalid,setting range:1-4", 38)

#define OVER_RANGE_0_300()	 				GUI_showMessage("参数越界,设置范围0-300", 22)
#define EN_OVER_RANGE_0_300()			 	GUI_showMessage("Parameter is invalid,setting range:0-300", 40)

#define OVER_RANGE_0_500() 					GUI_showMessage("参数越界,设置范围0-500", 22)
#define EN_OVER_RANGE_0_500() 				GUI_showMessage("Parameter is invalid,setting range:0-500", 40)

#define OVER_RANGE_0_10() 					GUI_showMessage("参数越界,设置范围0-10", 21)
#define EN_OVER_RANGE_0_10() 				GUI_showMessage("Parameter is invalid,setting range:0-10", 39)

#define OVER_RANGE_01_25() 					GUI_showMessage("参数越界,设置范围0.1-2.5", 23)
#define EN_OVER_RANGE_01_25() 				GUI_showMessage("Parameter is invalid,setting range:0.1-2.5", 42)

// S型加减速
#define     CONF_S_TYPE_ACC    

#define 	CONF_S_100							0		///< 1减速固定100步,0减速步数1000

#if CONF_S_100 == 1
	#define     CONF_S_LEN                      100
	#define     CONF_S_FACTOR_NO                2       // 选择加速曲线

	#define 	CONF_S_OFFSET_GIVEN				1		///< 送料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_LET				1		///< 放料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_BO				1		///< 剥料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_SHOU				1		///< 收料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数

#else
	#define     CONF_S_LEN                      1000
	#define     CONF_S_1000_FACTOR_NO           2       // 选择加速曲线-1000步//经过测试，曲线2效果最佳
	//可修改数值    1、   2、  4、	5、  10
	//对应加减速步数1000、500、250、200、100
	#define 	CONF_S_OFFSET_GIVEN				5		///< 送料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_LET				5		///< 放料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_BO				5		///< 剥料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数
	#define 	CONF_S_OFFSET_SHOU				5		///< 收料电机，查询数值表的偏移、1000/CONF_S_OFFSET=减速步数

#endif

#define     CONF_S_SPEED_MIN                	600     // 最小速度
#define     CONF_S_TIMER_FREQ               	2000000  // 定时器时钟频率, 即分频后频率

#define 	CONF_S_OFFSET						2		///< 查询数值表的偏移、1000/CONF_S_OFFSET=减速步数

#define     CONF_SOFTWARE_WAIT_INPUT_STABLE                    // 软件防抖
#define     CONF_SOFTWARE_WAIT_INPUT_TIME       2          // 软件延时时间

#define 	CONF_SLAVE_ADDR						0x00

#endif

