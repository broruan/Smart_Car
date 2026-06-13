#include "oled.h"

/*这个是oled发送指令的函数*/
void oled_IIC_SendCommand(unsigned char IIC_Command)
{
	iic_Start();//IIC协议开始信号
	iic_SendData(0x78);//发送设备地址
	iic_Wait_Ack();//等待应答，这里应答可以不用处理，一般会给一个应答信号
	iic_SendData(0x00);//设置D/C为0，为指令模式
	iic_Wait_Ack();//等待应答
	iic_SendData(IIC_Command);//发送指令
	iic_Wait_Ack();//等待应答
	iic_Stop();//IIC协议结束信号
}

/*这个是oled发送数据的函数*/
void oled_IIC_SendData(unsigned char IIC_Data)
{
	iic_Start();//IIC协议开始信号
	iic_SendData(0x78);//发送设备地址
	iic_Wait_Ack();//等待应答，这里应答可以不用处理，一般会给一个应答信号
	iic_SendData(0x40);//设置D/C为1，为数据模式
	iic_Wait_Ack();//等待应答
	iic_SendData(IIC_Data);//发送数据
	iic_Wait_Ack();//等待应答
	iic_Stop();//IIC协议结束信号
}

/*这个是oled清屏函数*/
void oled_clear(void)
{
	unsigned char i,j;//定义2个变量等下来实现循环
	for(i = 0;i < 8;i++)//这里是由于SSD1306分了8页：0-7
	{
		oled_IIC_SendCommand((unsigned char)(0xb0 + i));//这里是按页分配总共8页：0xb0-0xb7
		oled_IIC_SendCommand(0x00);//设置低4位地址，这里重新配置会被置为0000
		oled_IIC_SendCommand(0x10);//设置高4位地址，这里重新配置会被置为0000
		for(j = 0;j < 128;j++)//每一行的长度是128bit
		{
			oled_IIC_SendData(0x00);//这里时每次都写0x00
		}
	}
}

/*这个是oled初始化函数*/
void oled_Init(void)
{
	iic_Init();//IIC协议初始化
	delay_ms(200);//这里是上电后延迟，等待oled复位。
	oled_IIC_SendCommand(0xAE);//.关闭显示
	
	oled_IIC_SendCommand(0xD5);//.设置时钟分频因子，震荡频率
	oled_IIC_SendCommand(0x80);//.低4位设置分频因子，高4位设置震荡频率
	
	oled_IIC_SendCommand(0xA8);//.设置驱动路数
	oled_IIC_SendCommand(0x3F);//.默认0x3F（1/64）
	
	oled_IIC_SendCommand(0xD3);//.设置显示偏移
	oled_IIC_SendCommand(0x00);//.默认为0
	
	oled_IIC_SendCommand(0x00);//.设置显示位置-列低地址
	oled_IIC_SendCommand(0x10);//.设置显示位置-列高地址
	oled_IIC_SendCommand(0x40);//.设置显示开始行[5:0]，行数
	
	oled_IIC_SendCommand(0x8D);//.电荷泵设置
	oled_IIC_SendCommand(0x14);//.bit2：开启/关闭
	
	oled_IIC_SendCommand(0x20);//设置内存寻址模式
	oled_IIC_SendCommand(0x02);//设置成为页面寻址模式
	
	oled_IIC_SendCommand(0xA1);//.段重定义设置，bit0：0，0->1;1:0->127;这里是设置成1，是将127映射到SEG0
	
	oled_IIC_SendCommand(0xC8);//.设置COM扫描方向，bit3：0：普通模式；1：重定义模式（COM[N-1]->COM0,N:驱动路数）;
	oled_IIC_SendCommand(0xDA);//.设置COM引脚的硬件配置
	oled_IIC_SendCommand(0x12);//.A[5:4]，这里只配置了第5位，启动了COM的左右映射，顺序COM引脚配置
	
	oled_IIC_SendCommand(0x81);//.设置对比度
	oled_IIC_SendCommand(0xEF);//.这里设置成EF，默认是7F，数据越大对比度越大
	
	oled_IIC_SendCommand(0xD9);//.设置预充电周期
	oled_IIC_SendCommand(0xF1);//.第一阶段为1个时钟周期，第二阶段为15个时间周期
	
	oled_IIC_SendCommand(0xDB);//.设置VCOMH电压倍率
	oled_IIC_SendCommand(0x30);//.[6:4]000：0.65*vcc；001：0.77*vcc；011：0.83*vcc；
	
	oled_IIC_SendCommand(0xA4);//恢复到RAM内容显示
	oled_IIC_SendCommand(0xA6);//.设置显示方式；bit0：1反向显示；0正常显示；
	oled_IIC_SendCommand(0xAF);//.开启显示
	oled_clear();
}

/*这个是oled开显示函数*/
void oled_Display_ON(void)
{
	oled_IIC_SendCommand(0x8D);//电荷泵设置指令
	oled_IIC_SendCommand(0x14);//打开电荷泵
	oled_IIC_SendCommand(0xAF);//打开显示
}

/*这个是oled关显示函数*/
void oled_Display_OFF(void)
{
	oled_IIC_SendCommand(0x8D);//电荷泵设置指令
	oled_IIC_SendCommand(0x10);//关闭电荷泵
	oled_IIC_SendCommand(0xAE);//关闭显示
}


/*这个是oled全屏填充函数*/
void oled_Allfill(unsigned char fill_Data)
{
	unsigned char i,j;//定义2个变量等下来实现循环
	for(i = 0;i < 8;i++)//这里是由于SSD1306分了8页：0-7
	{
		oled_IIC_SendCommand((unsigned char)(0xb0 + i));//这里是按页分配总共8页：0xb0-0xb7
		oled_IIC_SendCommand(0x00);//设置低4位地址，这里重新配置会被置为0000
		oled_IIC_SendCommand(0x10);//设置高4位地址，这里重新配置会被置为0000
		for(j = 0;j < 128;j++)//每一行的长度是128bit
		{
			oled_IIC_SendData(fill_Data);//这里时每次都写fill_Data这个值
		}
	}
}

/*这个是oled设置光标函数*/
void oled_SetPos(unsigned char x,unsigned char y)
{
	oled_IIC_SendCommand((unsigned char)(0XB0+y));//这里是确定在哪个页面开始的光标
	oled_IIC_SendCommand((x&0xf0)>>4|0x10);//这里是设置高4位的列
	oled_IIC_SendCommand((x&0x0f));//这里是设置低4位的列
}


/*计算m^n的函数*/
unsigned int oled_Pow(unsigned char m,unsigned char n)
{
	unsigned int	result = 1;//设置变量为当n为0时的数据1
	if(n!=0){while(n--)result *= m;}//这里时判断n是否为0，如果是直接输出1即可，如果不是再运算
	return result;
}

/*这个是oled显示数字函数*/
void oled_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len,unsigned char size)
{
	unsigned char t,temp;
	
	if(num<0)//这里是判断输入的数是负数不是
	{
		num *= (-1);//是负数就转为
		oled_ShowChar((unsigned char)(x-(size/2)),y,'-',16);//在x位置的前一个size位置上显示负号
	}
	for(t=0;t<len;t++)//根据数字长度循环
	{
		temp = (num/oled_Pow(10,(unsigned char)(len - t - 1)))%10;//取出进制位的数字
		oled_ShowChar((unsigned char)(x + (size / 2) * t),y,(unsigned char)(temp+'0'),size);//显示每一个位的数字
	}
	
}

/*这个是oled显示一个字符函数*/
void oled_ShowChar(unsigned char x,unsigned char y,unsigned char ch,unsigned char size)
{
	unsigned char c=0,i=0;
	c = ch - ' ';//获取字符的偏移量
	if(x+size/2>127){x = 0; y = y+2;}//这里是判断x位置是否已经超过显示宽度，如果超过就到2页后
	
	if(size == 8)//判断是8*6的字符
	{
		oled_SetPos(x,y);//设置开始位置为x，y（第一页）
		for(i=0;i<6;i++)//循环6个列数据，因为只需要一页，所以写完一页即可
			oled_IIC_SendData(F6X8[c][i]);
	}
	if(size == 16)//判断是16*8的字符
	{
		oled_SetPos(x,y);//设置开始位置为x，y（第一页）
		for(i=0;i<8;i++)//循环8个列数据
			oled_IIC_SendData(F8X16[c*16+i]);//这里的C是偏移量，然后c*16是为了在数组里找到我们要的字符，i是数组第0-7的数据
		oled_SetPos(x,(unsigned char)(y+1));	//因为这里的16*8的字符需要2页，设置第二页为开始位置
		for(i=0;i<8;i++)//这里是第二页的列数据循环
			oled_IIC_SendData(F8X16[c*16+i+8]);//这里出现了+8是因为我们数组的第8-15个数据
	}
}

/*这个是oled显示字符串函数*/
void oled_ShowStr(unsigned char x,unsigned char y,const unsigned char *p,unsigned char size)
{
	while((*p <= '~') && (*p >= ' '))//判这个是判断是否是ASCLL表里的字符，如果小于或者大于就不打印
	{
		if(x>(128-(size/2)))//这里是判断是否超过每行的宽度128bit
		{
			x = 0;//如果超过就把x置为第一位
			y += size;//然后把y加到下一行
		}
		if(y > (64-size))//这里是判断是否超过整个屏宽度64bit
		{
			y = x = 0;//这里是重新回到0行0列的位置
			oled_clear();//然后清屏
		}
		oled_ShowChar(x,y,*p,size);//这里就是运用到上面显示一个字符的函数（因为这个字符可以看成由多个字符组成的）
		x += size/2;//这里是每次写一个字符后要把x（列）往右偏移字符相关的宽度
		p++;//字符串指针增加，也就是到下一个字符
	}
}

/*
	@brief			显示中文
	@param	x：起始列；一个字体占16列
					y：起始页；一个字体占两页
	@retval			无
 */
void oled_ShowCN(unsigned char x,unsigned char y,unsigned char bit_temp)
{
	unsigned char i;
	oled_SetPos(x,y);//设置开始位置x，y（第一页）
	for(i = 0;i < 16;i++)//循环16次发送数据，因为这里一个汉字是占用2页，长度宽度都是16
	{
		oled_IIC_SendData(Hzk[2*bit_temp][i]);
		/*
		写入第一页数，由于这里是二维数组然后bit_temp是定位我们自定义中文字库里
		的位置相应的汉字，2*bit_temp就是一个汉字的size
		*/
	}
	oled_SetPos(x,(unsigned char)(y+1));//设置第二页位置
	for(i = 0;i < 16;i++)//然后再写入第二页数据
	{
		oled_IIC_SendData(Hzk[2*bit_temp+1][i]);//这里出现了+1这就是跳到下一页的数组数据
	}
}

/*这个是oled显示图片函数*/
void oled_ShowBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	int j=0;
	unsigned char x,y;
	if(y1%8 == 0)y=y1/8;//判断是否刚好一页
	else y=y1/8+1;//不是的话就再加一页
	
	for(y = y0;y < y1;y++)//循环页数
	{
		oled_SetPos(x0,y);//设置开始位置x0，y
		for(x = x0;x < x1; x++)//循环列数
		{
			oled_IIC_SendData(BMP[j++]);//发送图片数据
		}
	}
	
}
