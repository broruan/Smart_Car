#ifndef _OLED_H
#define _OLED_H

#include "iic.h"
#include "STC32G_Delay.h"
#include "font.h"



void oled_IIC_SendCommand(unsigned char IIC_Command);//写命令
void oled_IIC_SendData(unsigned char IIC_Data);//写数据
void oled_Init(void);//OLED初始化
void oled_Display_ON(void);//开显示
void oled_Display_OFF(void);//关显示
void oled_Allfill(unsigned char fill_Data);//填充全部区域
void oled_clear(void);//清除显示
void oled_SetPos(unsigned char x,unsigned char y);//设置起始点坐标

unsigned int oled_Pow(unsigned char m,unsigned char n);//计算m^n
void oled_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len,unsigned char size);//显示数字
void oled_ShowChar(unsigned char x,unsigned char y,unsigned char ch,unsigned char size);//显示字符
void oled_ShowStr(unsigned char x,unsigned char y,const unsigned char *p,unsigned char size);//显示字符串
void oled_ShowCN(unsigned char x,unsigned char y,unsigned char bit_temp);//显示汉字
void oled_ShowBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//显示图片

#endif
