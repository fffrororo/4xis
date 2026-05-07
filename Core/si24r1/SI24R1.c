#include "SI24R1.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x07,0x0E,0x01};  

/**
 * @brief SPI读写函数
 * @param byte 要通过SPI发送的数据字节
 * @return 返回从SPI接收的数据字节
 */
static uint8_t SPI_RW(uint8_t byte)	
{
	uint8_t SPI_RX_Data = 0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &SPI_RX_Data, 1, 1000);
	return(SPI_RX_Data);
}

/**
 * @brief 写入SI24R1寄存器的函数
 * @param reg 要写入的寄存器地址,格式：SI24R1_WRITE_REG + reg
 * @param value 要写入到寄存器的数据值
 * @return 返回操作状态
 */
uint8_t SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW;              
	status = SPI_RW(reg);				
	SPI_RW(value);
	CS_HIGH; 
	
	return(status);
}

/**
 * @brief 向SI24R1芯片写入数据多字节
 * @param reg 要写入的寄存器地址,格式：SI24R1_WRITE_REG + reg
 * @param pBuf 写数据首地址
 * @param bytes 要写入的字节数
 * @return 返回SPI通信的状态值
 */
uint8_t SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

	CS_LOW;                                  			
	status = SPI_RW(reg);                          
	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)     
		SPI_RW(*pBuf++);
	CS_HIGH;                                 	

	return(status);       
}							  					   

/**
 * @brief 读取SI24R1芯片寄存器的值
 * 
 * @param reg 要读取的寄存器地址,格式：SI24R1_READ_REG + reg
 * @return uint8_t 寄存器的值
 */
uint8_t SI24R1_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	CS_LOW;    
	SPI_RW(reg);			
	value = SPI_RW(0);
	CS_HIGH;              

	return(value);
}

/**
 * @brief 读取SI24R1芯片寄存器数据多字节
 * @param reg 要读取的寄存器地址
 * @param pBuf 读取数据首地址
 * @param bytes 要读取的数据字节数
 * @return uint8_t 操作状态，返回SPI通信的状态值
 */
uint8_t SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

	CS_LOW;                                        
	status = SPI_RW(reg);                           
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr] = SPI_RW(0);     //读取数据，低字节在前              
	CS_HIGH;                                        

	return(status);    
}
/**
 * @brief 配置SI24R1芯片进入接收模式
 * 
 * 该函数设置SI24R1芯片为接收模式，配置接收地址、使能接收通道、
 * 设置射频通道和数据包长度等参数。
 * 
 * @param 无参数
 * @return 无返回值
 */
void SI24R1_RX_Mode(void)
{
	// 拉低CE引脚以进入配置模式
	CE_LOW;
	// 配置接收地址到数据管道0
	SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);	
	// 使能自动应答功能到数据管道0
	SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);               						
	// 使能接收数据管道0
	SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);           						
	// 设置射频通信频道
	SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, SI24R1_CHANNEL);                 						
	// 设置接收数据管道0的有效载荷宽度
	SI24R1_Write_Reg(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  						
	// 配置射频设置（传输速率和功率）（2Mbps，4dBm）
	SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x0e);            						
	// 配置芯片基本参数（上电、CRC使能等）
	SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0f);              						
	// 清除状态寄存器标志位
	SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, 0xff);  												
	// 拉高CE引脚以启动接收模式
	CE_HIGH;                                            									
}						

/**
 * @brief 配置SI24R1芯片为发送模式
 * 
 * 此函数配置SI24R1无线模块进入发送模式，设置发射地址、接收地址、
 * 自动应答、重发机制、射频通道和功率等参数。
 * 
 * @note 此函数无输入参数，无返回值
 */
void SI24R1_TX_Mode(void)
{
	CE_LOW;  // 拉低CE引脚，使芯片进入待机模式以便进行配置
	// 设置发射地址到TX_ADDR寄存器
	SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     
	// 配置接收地址通道0，使其与发射地址相同，用于自动应答功能
	SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  

	// 使能通道0的自动应答功能
	SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);        
	// 使能接收数据通道0
	SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);    
	// 设置自动重发延迟时间为250us+86us，自动重发10次
	SI24R1_Write_Reg(SI24R1_WRITE_REG + SETUP_RETR, 0x0a);   
	// 选择射频频道0x40
	SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, SI24R1_CHANNEL);          
	// 设置数据传输速率为2Mbps，输出功率为+4dBm
	SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x0e);     
	// 配置寄存器：启用CRC校验，16位CRC校验，上电
	SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0e);       
	CE_HIGH;  // 拉高CE引脚，激活芯片开始工作
}

/**
 * @brief 接收SI24R1无线模块的数据包
 * @param rxbuf 指向存储接收到的数据的缓冲区指针
 * @return uint8_t 返回接收状态，0表示成功接收到数据，1表示未接收到任何数据
 */
uint8_t SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = SI24R1_Read_Reg(STATUS);  			                 //读取状态寄存器的值    	  
	SI24R1_Write_Reg(SI24R1_WRITE_REG+STATUS,state);               //清除RX_DS中断标志

	if(state & RX_DR)								                           //接收到数据
	{
		SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);     //读取数据
		SI24R1_Write_Reg(FLUSH_RX,0xff);					              //清空RX FIFO的寄存器
		return 0; 
	}	   
	return 1;                                                   //未接收到任何数据
}

/**
 * @brief 发送数据包到SI24R1无线模块
 * 
 * 该函数将指定的数据包发送到SI24R1无线模块，并等待发送完成或出现错误。
 * 函数会配置CE引脚状态，将数据写入TX FIFO，然后等待发送完成。
 * 
 * @param txbuf 指向要发送的数据缓冲区的指针
 * @return uint8_t 返回发送状态，0表示发送成功，1表示发送失败或达到最大重试次数
 */
uint8_t SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;														//拉低CE引脚，使SI24R1进入待机模式
  	SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	    //写数据到TX FIFO,32字节
 	CE_HIGH;													//拉高CE引脚，使能发送	   
																					  
	state = SI24R1_Read_Reg(STATUS);  											  //读取状态寄存器的值	   
	while (((state & TX_DS) == 0) && ((state & MAX_RT) == 0))   //等待发送完成或达到最大重试次数
	{
		state = SI24R1_Read_Reg(STATUS); 
	}
	
	SI24R1_Write_Reg(SI24R1_WRITE_REG+STATUS, state); 								//清除TX_DS和MAX_RT中断标志
	if(state&MAX_RT)																			    //达到最大重试次数
	{
		SI24R1_Write_Reg(FLUSH_TX,0xff);										    //清空TX FIFO的寄存器 
		return 1; 
	}
	if(state&TX_DS)																			      //发送完成
	{
		return 0;
	}
	return 1;																						  //发送失败
}

/**********移植printf*************/
// 发送字符串
void SI24R1_SendString(char *str)
{
    uint8_t buf[TX_PLOAD_WIDTH] = {0};

    strncpy((char*)buf, str, TX_PLOAD_WIDTH - 1);
	buf[TX_PLOAD_WIDTH - 1] = '\0';

    SI24R1_TxPacket(buf);
}
// 1. 重定向 fputc：无线发送单个字符
int fputc(int ch, FILE *f)
{
    uint8_t buf[TX_PLOAD_WIDTH] = {0};

    buf[0] = ch;

    SI24R1_TxPacket(buf);

    return ch;
}

// 2. 无线版 printf：用法和 printf / Serial_Printf 完全一样
void SI24R1_Printf(char *format, ...)
{
	SI24R1_TX_Mode();
	osDelay(1);

    char string[TX_PLOAD_WIDTH] = {0};

    va_list arg;

    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);

    SI24R1_TxPacket((uint8_t *)string);

	SI24R1_RX_Mode();
}

