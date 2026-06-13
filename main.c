/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
#include  "iic.h"
#include  "font.h"
#include  "oled.h"




/*************	功能说明	**************

高级PWM定时器 PWM5,PWM6,PWM7,PWM8 每个通道都可独立实现PWM输出.

4个通道PWM根据需要设置对应输出口，可通过示波器观察输出的信号.

PWM周期和占空比可以自定义设置，最高可达65535.

下载时, 选择时钟 24MHZ (用户可在"config.h"修改频率).

******************************************/

sbit AIN1=P4^5;
sbit AIN2=P2^7;
sbit BIN1=P2^5;
sbit BIN2=P2^6;
sbit zuo2=P0^3;
sbit zuo1=P0^4;
sbit zhong=P0^2;
sbit you1=P0^1;
sbit you2=P0^0;
sbit LED1=P4^1;
sbit LED2=P4^2;
sbit LED3=P4^4;
sbit TRIG = P0^6;
sbit ECHO = P1^4;
sbit KEY1 = P2^2;
sbit KEY2 = P2^3;
sbit KEY3 = P2^4;
sbit KEY4 = P1^3;
sbit KEY5 = P1^7;

volatile unsigned int time_us = 0;
bit measuring = 0;
u32 Flag = 0;
u32 n = 0;

PWMx_Duty PWMB_Duty;
u16 PWM_Period = 2000; // 2000
bit PWM5_Flag;
bit PWM6_Flag;


/*************	本地函数声明	**************/

void Motor_SetForward(void);
u8 LimitPercent(int16 speed);
void Motor_RunPercent(int16 left, int16 right);
u8 ReadLineMask(void);
bit ComputeLineError(u8 mask, int16 *error);
int16 LinePidUpdate(int16 error);
void ApplyLinePid(int16 error);
void LineLostSearch(void);
bit IsTStateMask(u8 mask);
void UpdateTState(u8 mask);
bit HandleSpecialLineState(u8 mask);
void UpdateTTimeAction(void);
void LineControlStep(void);


/*************  外部函数和变量声明 *****************/

#define T_STATE_TIME_TICKS 10000   // T状态延时计数阈值，按1ms循迹周期约10s
bit T_State_Last = 0;
bit T_Time_Enable = 0;
bit T_Time_Done = 0;
bit T_State_45 = 1;
u16 T_State_Count = 0;
u32 T_Time_Tick = 0;
bit Force_Turn_Enable = 0;
bit T_Left_Enable = 0;
bit T_you_Enable = 0;
u16 Force_Turn_Count = 0;
u16 dalay_Start_Count = 0;
u16 end_Count = 0;
u16 T_Left_Count = 0;
u16 U_Count = 0;
u16 Turn_you_Count = 0;
bit delay_Start = 0;  // 0:上电立即循迹；1:先按DELAY_START_TICKS停车等待
bit Toggle_Start = 0;

#define FORCE_TURN_TICKS  3000   // 45度/急弯强制转向保持时间，Timer2 100us tick，约300ms
#define DELAY_START_TICKS 5000  // 开机停车等待时间，Timer2 100us tick，约500ms
#define END_TICKS 18000  // 终点/停止判定计数阈值，Timer2 100us tick，当前保留未启用
#define T_LEFT_TICKS 8000 // T型左转保持时间，Timer2 100us tick，当前保留未启用
#define U_TICKS 12000  // 掉头动作保持时间，Timer2 100us tick，当前保留未启用
#define YOU_TICKS 1000  // T型右转保持时间，Timer2 100us tick，约100ms
#define TOGGLE_TICKS 1000  // 左右轮反转延迟

#define LINE_CTRL_DIVIDER      1  // Timer2每1次中断执行一次PID，100us*1=100us
#define LINE_BASE_SPEED        75 // PID正常循迹基础速度百分比
#define LINE_MAX_SPEED         85  // PWM输出限幅最大速度百分比
#define LINE_SEARCH_SPEED      75  // 全白丢线后的低速搜索速度百分比
#define LINE_LOST_HOLD_TICKS   800  // 丢线后先保持上次输出的时间，按100us循迹周期约8ms
#define LINE_PID_KP            80  // PID比例系数，放大后由LINE_PID_SCALE缩放
#define LINE_PID_KI            0  // PID积分系数，默认0避免低速抖动和积分饱和
#define LINE_PID_KD            0  // PID微分系数，用于抑制转向过冲
#define LINE_PID_SCALE         100  // PID定点缩放系数，输出=(KP*P+KI*I+KD*D)/100
#define LINE_PID_I_LIMIT       200  // PID积分项限幅，防止长时间偏差导致积分过大
#define LINE_STEER_SIGN        1  // 转向方向符号，若实车左右修正反了改为-1

#define LINE_MASK_LOST         0x00  // 00000：全白/丢线，没有传感器检测到黑线
#define LINE_MASK_CROSS        0x1f  // 11111：十字路口，五路传感器全部检测到黑线
#define LINE_MASK_T_RIGHT      0x11  // 10001：T型右转触发模式，左右最外侧检测到黑线
#define LINE_MASK_45_RIGHT     0x07  // 00111：右侧45度/急弯触发模式

int16 Line_Last_Error = 0;
int16 Line_Pid_Last_Error = 0;
int16 Line_Pid_Integral = 0;
u16 Line_Lost_Count = 0;
u16 Toggle_Count = 0;
u8 Line_Last_Left = LINE_BASE_SPEED;
u8 Line_Last_Right = LINE_BASE_SPEED;
/************************ IO口配置 ****************************/
/*
 * 作用：预留GPIO集中配置入口；当前IO模式主要在main()中直接配置。
 * 参数：无。
 * 返回：无。
 */
void	GPIO_config(void)
{

	
}

/************************ 定时器配置 ****************************/
/*
 * 作用：配置Timer0为1ms周期中断，用于通用毫秒节拍标志。
 * 参数：无。
 * 返回：无。
 */
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					//8位预分频器(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/*
 * 作用：配置Timer1为10us周期中断，给超声波测距计时使用。
 * 参数：无。
 * 返回：无。
 */
void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL)); // 10us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer1, &TIM_InitStructure);                          // 初始化定时器1
    NVIC_Timer1_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

/*
 * 作用：配置Timer2为100us周期中断，作为循迹控制主循环节拍。
 * 参数：无。
 * 返回：无。
 */
void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000L)); // 100us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer2, &TIM_InitStructure);                          // 初始化定时器2
    NVIC_Timer2_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

/***************  超声波函数 *****************/
/*
 * 作用：Timer1中断服务函数；超声波ECHO测量期间累加10us计数。
 * 参数：无；由TMR1_VECTOR硬件中断入口调用。
 * 返回：无。
 */
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//进中断时已经清除标志
{
	if (measuring) time_us++;
}

/*
 * 作用：触发HC-SR04超声波模块并计算障碍物距离。
 * 参数：无。
 * 返回：距离值，单位mm；超时或无有效回波时返回0。
 */
unsigned int Ultrasonic_GetDistance(void) 
{
    unsigned int Distance_mm = 0;
    TRIG = 0;
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 1;
    _nop_(); _nop_(); _nop_(); _nop_();    // 等价大约10us
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    // 等待 ECHO 高电平开始
    while (!ECHO);
    
    // 开始计时
    time_us = 0;
    measuring = 1;

    // 等待 ECHO 拉低（结束）
    while (ECHO);

    measuring = 0;
		if(time_us/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
		{
			Distance_mm=(time_us*346)/100;						//计算距离，25°C空气中的音速为346m/s   因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
		}
    return Distance_mm;
}

/***************  PWM初始化函数 *****************/
/*
 * 作用：初始化PWMB的PWM5/PWM6通道，并映射到右/左电机PWM输出引脚。
 * 参数：无。
 * 返回：无。
 */
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
//	PWMB_Duty.PWM5_Duty = 128;
//	PWMB_Duty.PWM6_Duty = 256;


	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB


	PWMx_InitStructure.PWM_Period   = PWM_Period; //2000							//周期时间,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM通用寄存器,  PWMA,PWMB


		PWM6_USE_P21();
		PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

/*
 * 作用：设置左电机PWM占空比缓存值，实际更新由PWM_Run统一提交。
 * 参数：pwm为左电机速度百分比，建议范围0-100。
 * 返回：无。
 */
void PWM_Left(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //注意PWM_Period此时为100倍数 2000
	
}

/*
 * 作用：设置右电机PWM占空比缓存值，实际更新由PWM_Run统一提交。
 * 参数：pwm为右电机速度百分比，建议范围0-100。
 * 返回：无。
 */
void PWM_Right(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
	
}


/*
 * 作用：同时设置左右电机PWM并写入PWMB寄存器。
 * 参数：pwm1为左电机速度百分比；pwm2为右电机速度百分比，建议范围0-100。
 * 返回：无。
 */
void PWM_Run(int pwm1,int pwm2)  //左右PWM
{
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

/*
 * 作用：电机PWM测试函数，按固定速度组合依次运行左右电机。
 * 参数：无。
 * 返回：无。
 */
void test01(void)
{
		PWM_Run(100,80);
		delay_ms(1000);
		PWM_Run(80,100);
		delay_ms(1000);
		PWM_Run(0,100);
		delay_ms(1000);
		PWM_Run(100,0);
		delay_ms(1000);
}

/*
 * 作用：设置H桥方向脚，使左右电机进入前进方向。
 * 参数：无。
 * 返回：无。
 */
void Motor_SetForward(void)
{
	AIN1 = 0;
	AIN2 = 1;
	BIN1 = 1;
	BIN2 = 0;
}

/*
 * 作用：将速度百分比限制在电机允许输出范围内。
 * 参数：speed为待限幅速度百分比，可为负数或超过最大值。
 * 返回：限幅后的速度百分比，范围0到LINE_MAX_SPEED。
 */
u8 LimitPercent(int16 speed)
{
	if(speed < 0)
	{
		return 0;
	}
	if(speed > LINE_MAX_SPEED)
	{
		return LINE_MAX_SPEED;
	}
	return (u8)speed;
}

/*
 * 作用：设置电机前进方向，限幅左右速度，并输出到PWM。
 * 参数：left为左电机速度百分比；right为右电机速度百分比，正常范围0-100，实际限幅到LINE_MAX_SPEED。
 * 返回：无。
 */
void Motor_RunPercent(int16 left, int16 right)
{
    if(left < 0)
    {
        AIN1 = 1;
        AIN2 = 0;
        BIN1 = 1;
        BIN2 = 0;
        Toggle_Start = 1;
        Toggle_Count = TOGGLE_TICKS;
    }
    else if(right < 0)
    {
        AIN1 = 0;
        AIN2 = 1;
        BIN1 = 0;
        BIN2 = 1;
        Toggle_Start = 1;
        Toggle_Count = TOGGLE_TICKS;
    }
    else
    {
        AIN1 = 0;
        AIN2 = 1;
        BIN1 = 1;
        BIN2 = 0;
        Toggle_Start = 0;
    }

    Line_Last_Left = left < 0 ? 75 : LimitPercent(left);
    Line_Last_Right = right < 0 ? 75 : LimitPercent(right);
    PWM_Run(Line_Last_Left, Line_Last_Right);
}

/*
 * 作用：读取5路循迹传感器并组合成位图。
 * 参数：无。
 * 返回：5位mask，bit4..bit0分别为zuo2、zuo1、zhong、you1、you2；1表示黑线，0表示白底。
 */
u8 ReadLineMask(void)
{
	u8 mask = 0;

	if(zuo2)  mask |= 0x10;
	if(zuo1)  mask |= 0x08;
	if(zhong) mask |= 0x04;
	if(you1)  mask |= 0x02;
	if(you2)  mask |= 0x01;

	return mask;
}

/*
 * 作用：根据传感器位图计算循迹位置误差。
 * 参数：mask为5路传感器位图；error为输出参数，负数表示线偏左，正数表示线偏右。
 * 返回：1表示成功计算误差；0表示无黑线，无法计算误差。
 */
bit ComputeLineError(u8 mask, int16 *error)
{
	int16 sum = 0;
	u8 count = 0;

	if(mask & 0x10)
	{
		sum -= 95;
		count++;
	}
	if(mask & 0x08)
	{
		sum -= 30;
		count++;
	}
	if(mask & 0x04)
	{
		count++;
	}
	if(mask & 0x02)
	{
		sum += 30;
		count++;
	}
	if(mask & 0x01)
	{
		sum += 110;
		count++;
	}

	if(count == 0)
	{
		return 0;
	}

	*error = sum / count;
	return 1;
}

/*
 * 作用：根据当前循迹误差计算整数PID转向输出。
 * 参数：error为当前位置误差，负数偏左，正数偏右。
 * 返回：PID计算出的转向修正量，正负方向再由LINE_STEER_SIGN决定。
 */
int16 LinePidUpdate(int16 error)
{
	int32 output;
	int16 derivative;

	Line_Pid_Integral += error;
	if(Line_Pid_Integral > LINE_PID_I_LIMIT)
	{
		Line_Pid_Integral = LINE_PID_I_LIMIT;
	}
	else if(Line_Pid_Integral < -LINE_PID_I_LIMIT)
	{
		Line_Pid_Integral = -LINE_PID_I_LIMIT;
	}

	derivative = error - Line_Pid_Last_Error;
	Line_Pid_Last_Error = error;

	output = (int32)LINE_PID_KP * error;
	output += (int32)LINE_PID_KI * Line_Pid_Integral;
	output += (int32)LINE_PID_KD * derivative;
	output /= LINE_PID_SCALE;

	return (int16)output;
}

/*
 * 作用：把循迹误差转换为左右电机差速并输出。
 * 参数：error为当前位置误差，负数偏左，正数偏右。
 * 返回：无。
 */
void ApplyLinePid(int16 error)
{
	int16 steer;
	int16 left;
	int16 right;

	steer = LinePidUpdate(error) * LINE_STEER_SIGN; // sign可以不用改，默认为1
	left = LINE_BASE_SPEED + steer;
	right = LINE_BASE_SPEED - steer;

	Motor_RunPercent(left, right);
}

/*
 * 作用：处理全白丢线状态，先短暂保持上次输出，再按最后误差方向低速搜索。
 * 参数：无；使用Line_Lost_Count和Line_Last_Error等全局状态。
 * 返回：无。
 */
void LineLostSearch(void)
{
	Line_Pid_Integral = 0;

	if(Line_Lost_Count < LINE_LOST_HOLD_TICKS)
	{
		Motor_RunPercent(Line_Last_Left, Line_Last_Right);
		return;
	}

//	if(Line_Last_Error < 0)
//	{
//		Motor_RunPercent(0, LINE_SEARCH_SPEED);
//	}
//	else if(Line_Last_Error > 0)
//	{
//		Motor_RunPercent(LINE_SEARCH_SPEED, 0);
//	}
	else
	{
//		AIN1=0;
//		AIN2=1;
//		BIN1=0;
//		BIN2=1;
		Motor_RunPercent(LINE_SEARCH_SPEED, -LINE_SEARCH_SPEED);
	}
}

/*
 * 作用：判断当前传感器位图是否属于需要计数的路口/特殊状态。
 * 参数：mask为5路传感器位图，1表示黑线。
 * 返回：1表示属于T/十字/急弯状态；0表示普通循迹状态。
 */
bit IsTStateMask(u8 mask)
{
	return (mask == LINE_MASK_CROSS) ||
			 (mask == LINE_MASK_T_RIGHT) ||
			 (mask == 0x1c) ||
			 (mask == 0x07);
}

/*
 * 作用：检测是否刚进入特殊路口状态，并更新路口计数和延时触发标志。
 * 参数：mask为5路传感器位图，1表示黑线。
 * 返回：无。
 */
void UpdateTState(u8 mask)
{
	bit T_State_Now;

	T_State_Now = IsTStateMask(mask);

	if(T_State_Now && !T_State_Last)
	{
		T_State_Count++;
		T_Time_Tick = 0;
		T_Time_Enable = 1;
		T_Time_Done = 0;
	}

	if(!T_State_Now)
	{
		T_State_Last = 0;
	}
	else
	{
		T_State_Last = 1;
	}
}

/*
 * 作用：优先处理十字、T型右转和45度急弯等特殊循迹状态。
 * 参数：mask为5路传感器位图，1表示黑线。
 * 返回：1表示已处理特殊状态；0表示应继续走普通PID循迹。
 */
bit HandleSpecialLineState(u8 mask)
{
	if(mask == LINE_MASK_CROSS)
	{
		Motor_RunPercent(LINE_BASE_SPEED, LINE_BASE_SPEED);
		return 1;
	}

	if(mask == LINE_MASK_T_RIGHT)
	{
		T_you_Enable = 1;
		Turn_you_Count = YOU_TICKS;
		Motor_RunPercent(LINE_BASE_SPEED, LINE_BASE_SPEED);
		return 1;
	}

	if(mask == LINE_MASK_45_RIGHT)
	{
		Force_Turn_Enable = 1;
		Force_Turn_Count = FORCE_TURN_TICKS;
		Motor_RunPercent(85, 0);
		return 1;
	}

	return 0;
}

/*
 * 作用：处理进入T状态后一段时间触发的一次性动作。
 * 参数：无；使用T_Time_Enable、T_Time_Tick等全局状态。
 * 返回：无。
 */
void UpdateTTimeAction(void)
{
	if(T_Time_Enable && !T_Time_Done)
	{
		T_Time_Tick++;

		if(T_Time_Tick >= T_STATE_TIME_TICKS)
		{
			T_Time_Done = 1;
			T_Time_Enable = 0;
			Motor_RunPercent(70, 70);
		}
	}
}

/*
 * 作用：执行一次完整循迹控制步骤，包括读传感器、特殊状态处理、丢线处理和PID输出。
 * 参数：无。
 * 返回：无。
 */
void LineControlStep(void)
{
	u8 mask;
	int16 error;

	mask = ReadLineMask();
	UpdateTState(mask);

	if(HandleSpecialLineState(mask))
	{
		Line_Lost_Count = 0;
		Line_Pid_Integral = 0;
		return;
	}

	if(mask == LINE_MASK_LOST)
	{
		Line_Lost_Count++;
		LineLostSearch();
		return;
	}

	Line_Lost_Count = 0;
	if(ComputeLineError(mask, &error))
	{
		Line_Last_Error = error;
		ApplyLinePid(error);
	}
}

/*
 * 作用：Timer2中断服务函数；以100us节拍调度强制转向、T型转向和循迹控制。
 * 参数：无；由TMR2_VECTOR硬件中断入口调用。
 * 返回：无。
 */
void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR
{
	static u16 delay_Start_Count = DELAY_START_TICKS;
	static u8 line_ctrl_tick = 0;

	if(Toggle_Start)
	{
		if(Toggle_Count > 0)
		{
			Toggle_Count--;
		}
		else
		{
			Toggle_Start = 0;
		}
		return;
	}
	if(delay_Start)
	{
		PWM_Run(0, 0);
		if(delay_Start_Count > 0)
		{
			delay_Start_Count--;
		}
		else
		{
			delay_Start = 0;
		}
		return;
	}

	if(Force_Turn_Enable)
	{
		Motor_RunPercent(85, 0);
		if(Force_Turn_Count > 0)
		{
			Force_Turn_Count--;
		}
		else
		{
			Force_Turn_Enable = 0;
		}
		return;
	}

	if(T_you_Enable)
	{
		Motor_RunPercent(85, 45);
		if(Turn_you_Count > 0)
		{
			Turn_you_Count--;
		}
		else
		{
			T_you_Enable = 0;
		}
		return;
	}

	if(T_Left_Enable)
	{
		Motor_RunPercent(0, 85);
		if(T_Left_Count > 0)
		{
			T_Left_Count--;
		}
		else
		{
			T_Left_Enable = 0;
		}
		return;
	}

	UpdateTTimeAction();

	line_ctrl_tick++;
	if(line_ctrl_tick >= LINE_CTRL_DIVIDER)
	{
		line_ctrl_tick = 0;
		LineControlStep();
	}

	Flag += 1;
}

/******************** main **************************/
/*
 * 作用：系统入口函数，初始化时钟访问、IO、定时器、PWM和OLED，然后开启中断进入空循环。
 * 参数：无。
 * 返回：无。
 */
void main(void)
{
//	unsigned char i;
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度

	P2M1=0x00;
	P2M0=0xFF;
	P2M0 &= ~(1 << 2);  // M0=0		p2.2
	P2M0 &= ~(1 << 3);  // M0=0		p2.3
	P2M0 &= ~(1 << 4);  // M0=0		p2.4	
	P0M1=0x00;
	P0M0=0x00;
	P1M1=0x00;
	P1M0=0x00;
	P4M1=0x00;
	P4M0=0xFF;	
	P1M1 |= (1 << 4);   // M1=1
	P1M0 &= ~(1 << 4);  // M0=0	
	
	Timer_config();
	Timer1_config();
	Timer2_config();
	PWM_config();
	oled_Init();
	
	EA = 1;
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
	while (1)
	{
//			unsigned int d = Ultrasonic_GetDistance();
//			oled_clear();
//			oled_ShowNum(56,2,d,5,16);
//			oled_ShowNum(10,2,1,2,16);		
//			delay_ms(300);	

//		if(KEY4 == 0)
//		{
//			LED1=0;
//			delay_ms(1000);
//			LED1=1;
//			delay_ms(1000);
//		}
//		if(KEY5 == 0)
//		{
//			LED2=0;
//			delay_ms(1000);
//			LED2=1;
//			delay_ms(1000);
//		}
//		if(KEY3 == 0)
//		{
//			LED3=0;
//			delay_ms(1000);
//			LED3=1;
//			delay_ms(1000);
//		}
//		for(i=10;i>0;i--)
//		{
//			oled_clear();
//			oled_ShowNum(10,2,i,2,16);		
//			delay_ms(1000);
//		}



//		if(T0_1ms)
//		{
//			
//			PWMB_Duty.PWM5_Duty = 2047;
//			PWMB_Duty.PWM6_Duty = 1;
//			T0_1ms = 0;
			
//			if(!PWM5_Flag)
//			{
//				PWMB_Duty.PWM5_Duty++;
//				if(PWMB_Duty.PWM5_Duty >= 2047) PWM5_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM5_Duty--;
//				if(PWMB_Duty.PWM5_Duty <= 0) PWM5_Flag = 0;
//			}
//			if(!PWM6_Flag)
//			{
//				PWMB_Duty.PWM6_Duty++;
//				if(PWMB_Duty.PWM6_Duty >= 2047) PWM6_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM6_Duty--;
//				if(PWMB_Duty.PWM6_Duty <= 0) PWM6_Flag = 0;
//			}
			
			
		}
	}




