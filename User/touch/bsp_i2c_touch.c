/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ���ݴ�������ר��iic����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./touch/bsp_i2c_touch.h"
#include "./touch/gt9xx.h"
#include "bsp_debug_usart.h"


/* STM32 I2C ����ģʽ */
#define I2C_Speed              400000

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
#define I2C_OWN_ADDRESS7      0x0A



static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}


/**
  * @brief  ʹ�ܴ������ж�
  * @param  ��
  * @retval ��
  */
void I2C_GTP_IRQEnable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;  
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  
  /*���� INT Ϊ�������� */   
  GPIO_InitStructure.GPIO_Pin = GTP_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);  
  
  /* ���� EXTI �ж�Դ ��INT ���� */
  SYSCFG_EXTILineConfig(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);

  /* ѡ�� EXTI �ж�Դ */
  EXTI_InitStructure.EXTI_Line = GTP_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);  
  
  /* NVIC_PriorityGroupConfig 已在 BSP_Init() 中统一设置为 Group_4，此处不再重复调用 */

	/*ʹ���ж�*/
  NVIC_InitStructure.NVIC_IRQChannel = GTP_INT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/**
  * @brief  �رմ������ж�
  * @param  ��
  * @retval ��
  */
void I2C_GTP_IRQDisable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /*���� INT Ϊ�������� */   
  GPIO_InitStructure.GPIO_Pin = GTP_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);

  /* ���� EXTI �ж�Դ ��INT ���� */
  SYSCFG_EXTILineConfig(GTP_INT_EXTI_PORTSOURCE, GTP_INT_EXTI_PINSOURCE);

  /* ѡ�� EXTI �ж�Դ */
  EXTI_InitStructure.EXTI_Line = GTP_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* NVIC_PriorityGroupConfig 已在 BSP_Init() 中统一设置为 Group_4，此处不再重复调用 */

  /* �ر��ж� */
  NVIC_InitStructure.NVIC_IRQChannel = GTP_INT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/**
  * @brief  ������ I/O����
  * @param  ��
  * @retval ��
  */
static void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
   
  /*ʹ��I2Cʱ�� */
  GTP_I2C_CLK_INIT(GTP_I2C_CLK, ENABLE);
  
  /*ʹ�ܴ�����ʹ�õ����ŵ�ʱ��*/
  RCC_AHB1PeriphClockCmd(GTP_I2C_SCL_GPIO_CLK|
                       	 GTP_I2C_SDA_GPIO_CLK|
	                       GTP_RST_GPIO_CLK|GTP_INT_GPIO_CLK, 
	                       ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#if !(SOFT_IIC)   //ʹ��Ӳ��IIC     
    /* ����I2C_SCLԴ*/
    GPIO_PinAFConfig(GTP_I2C_SCL_GPIO_PORT, 
                     GTP_I2C_SCL_SOURCE, GTP_I2C_SCL_AF);
    /* ����I2C_SDA Դ*/
    GPIO_PinAFConfig(GTP_I2C_SDA_GPIO_PORT, 
                     GTP_I2C_SDA_SOURCE, GTP_I2C_SDA_AF);  
    
    /*����SCL���� */   
    GPIO_InitStructure.GPIO_Pin = GTP_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GTP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*����SDA���� */
    GPIO_InitStructure.GPIO_Pin = GTP_I2C_SDA_PIN;
    GPIO_Init(GTP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

#else  //ʹ������IIC
    /*����SCL���� */   
    GPIO_InitStructure.GPIO_Pin = GTP_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GTP_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*����SDA���� */
    GPIO_InitStructure.GPIO_Pin = GTP_I2C_SDA_PIN;
    GPIO_Init(GTP_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
#endif
 
  /*����RST���ţ������������ */   
  GPIO_InitStructure.GPIO_Pin = GTP_RST_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GTP_RST_GPIO_PORT, &GPIO_InitStructure);
  
  /*���� INT���ţ�������������������ʼ�� */   
  GPIO_InitStructure.GPIO_Pin = GTP_INT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//����Ϊ�����������ʼ��
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);
}


/**
  * @brief  ��GT91xxоƬ���и�λ
  * @param  ��
  * @retval ��
  */
void I2C_ResetChip(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

  /*���� INT���ţ�������������������ʼ�� */   
	  GPIO_InitStructure.GPIO_Pin = GTP_INT_GPIO_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;       //����Ϊ�����������ʼ��
	  GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);

	  /*��ʼ��GT9157,rstΪ�ߵ�ƽ��intΪ�͵�ƽ����gt9157���豸��ַ������Ϊ0xBA*/

	  /*��λΪ�͵�ƽ��Ϊ��ʼ����׼��*/
	  GPIO_ResetBits (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN);
	  Delay(0x0FFFFF);

	  /*����һ��ʱ�䣬���г�ʼ��*/
	  GPIO_SetBits (GTP_RST_GPIO_PORT,GTP_RST_GPIO_PIN);
	  Delay(0x0FFFFF);

	  /*��INT��������Ϊ��������ģʽ���Ա���մ����ж��ź�*/
	  GPIO_InitStructure.GPIO_Pin = GTP_INT_GPIO_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_Init(GTP_INT_GPIO_PORT, &GPIO_InitStructure);
}

#if !(SOFT_IIC) //Ӳ��IICģʽ 
/**
  * @brief  I2C ����ģʽ����
  * @param  ��
  * @retval ��
  */
static void I2C_Mode_Config(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C ģʽ���� */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;	
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		                    
  I2C_InitStructure.I2C_OwnAddress1 =I2C_OWN_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;	
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	/* I2C��Ѱַģʽ */
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;	                          /* ͨ������ */
  I2C_Init(GTP_I2C, &I2C_InitStructure);	                                  /* I2C1 ��ʼ�� */
  I2C_Cmd(GTP_I2C, ENABLE);  	                                              /* ʹ�� I2C1 */

  I2C_AcknowledgeConfig(GTP_I2C, ENABLE);

}
#endif

/**
  * @brief  I2C ����(GT91xx)��ʼ��
  * @param  ��
  * @retval ��
  */
void I2C_Touch_Init(void)
{
  I2C_GPIO_Config(); 
 
#if !(SOFT_IIC) //Ӳ��IICģʽ 
    I2C_Mode_Config();
#endif
  
  I2C_ResetChip();

  I2C_GTP_IRQEnable();
}


#if !(SOFT_IIC)   //ʹ��Ӳ��IIC     


__IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT;   
static uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);

/**
  * @brief   ʹ��IIC��ȡ����
  * @param   
  * 	@arg ClientAddr:���豸��ַ
  *		@arg pBuffer:����ɴӻ���ȡ�����ݵĻ�����ָ��
  *		@arg NumByteToRead:��ȡ�����ݳ���
  * @retval  ��
  */
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead)
{  
    I2CTimeout = I2CT_LONG_TIMEOUT;

    while(I2C_GetFlagStatus(GTP_I2C, I2C_FLAG_BUSY))   
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(0);
    }

		/* Send STRAT condition  */
		I2C_GenerateSTART(GTP_I2C, ENABLE);
  
     I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(1);
   }
  /* Send GT91xx address for read */
  I2C_Send7bitAddress(GTP_I2C, ClientAddr, I2C_Direction_Receiver);
  
     I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(2);
   }
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(GTP_I2C, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(GTP_I2C, ENABLE);
    }


		I2CTimeout = I2CT_LONG_TIMEOUT;
		while(I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)==0)  
		{
			if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(3);
		} 	
		{
		  /* Read a byte from the device */
      *pBuffer = I2C_ReceiveData(GTP_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;
		}			

		
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(GTP_I2C, ENABLE);
  
  return 0;
}


/**
  * @brief   ʹ��IICд������
  * @param   
  * 	@arg ClientAddr:���豸��ַ
  *		@arg pBuffer:������ָ��
  *     @arg NumByteToWrite:д���ֽ���
  * @retval  ��
  */
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite)
{
  
  I2CTimeout = I2CT_LONG_TIMEOUT;

  while(I2C_GetFlagStatus(GTP_I2C, I2C_FLAG_BUSY))  
   {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(4);
  } 
  
  /* Send START condition */
  I2C_GenerateSTART(GTP_I2C, ENABLE);
  
  
  I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(5);
  } 
  
  /* Send GT91xx address for write */
  I2C_Send7bitAddress(GTP_I2C, ClientAddr, I2C_Direction_Transmitter);
  
  I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(6);
  } 
  /* While there is data to be written */
  while(NumByteToWrite--)  
  {
    /* Send the current byte */
    I2C_SendData(GTP_I2C, *pBuffer); 

    /* Point to the next byte to be written */
    pBuffer++; 
  
    I2CTimeout = I2CT_FLAG_TIMEOUT;

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(GTP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(7);
    } 
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(GTP_I2C, ENABLE);
  
  return 0;  

}


/**
  * @brief  IIC�ȴ���ʱ���ñ��������������Ϣ
  * @param  None.
  * @retval ����0xff����ʾIIC��ȡ����ʧ��
  */
static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  GTP_ERROR("I2C �ȴ���ʱ!errorCode = %d",errorCode);
  
  return 0xFF;
}

#else //ʹ������IIC

/*
*********************************************************************************************************
*	�� �� ��: i2c_Delay
*	����˵��: I2C����λ�ӳ٣����400KHz
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
    ����������CPU��Ƶ180MHz ��MDK���뻷����1���Ż�
      
		ѭ������Ϊ50ʱ��SCLƵ�� = 333KHz 
		ѭ������Ϊ30ʱ��SCLƵ�� = 533KHz��  
	 	ѭ������Ϊ20ʱ��SCLƵ�� = 727KHz�� 
  */
	for (i = 0; i < 10*5; i++);
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C���������ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C����ֹͣ�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_SendByte
*	����˵��: CPU��I2C�����豸����8bit����
*	��    �Σ�_ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_ReadByte
*	����˵��: CPU��I2C�����豸��ȡ8bit����
*	��    �Σ���
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_WaitAck
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    �Σ���
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Ack
*	����˵��: CPU����һ��ACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}



#define I2C_DIR_WR	0		/* д����bit */
#define I2C_DIR_RD	1		/* ������bit */

/**
  * @brief   ʹ��IIC��ȡ����
  * @param   
  * 	@arg ClientAddr:���豸��ַ
  *		@arg pBuffer:����ɴӻ���ȡ�����ݵĻ�����ָ��
  *		@arg NumByteToRead:��ȡ�����ݳ���
  * @retval  ��
  */
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead)
{
	
	/* ��1��������I2C���������ź� */
	i2c_Start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	i2c_SendByte(ClientAddr | I2C_DIR_RD);	/* �˴��Ƕ�ָ�� */
	
	/* ��3�����ȴ�ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* ������Ӧ�� */
	}

	while(NumByteToRead) 
  {
   if(NumByteToRead == 1)
    {
			i2c_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
      
      /* ����I2C����ֹͣ�ź� */
      i2c_Stop();
    }
    
   *pBuffer = i2c_ReadByte();
    
    /* ��ָ������ */
    pBuffer++; 
      
    /*�������Լ� */
    NumByteToRead--;
    
    i2c_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */  
  }

	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 0;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 1;
}

/**
  * @brief   ʹ��IICд������
  * @param   
  * 	@arg ClientAddr:���豸��ַ
  *		@arg pBuffer:������ָ��
  *     @arg NumByteToWrite:д���ֽ���
  * @retval  ��
  */
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite)
{
	uint16_t m;	

  /*����0������ֹͣ�źţ������ڲ�д������*/
  i2c_Stop();
  
  /* ͨ���������Ӧ��ķ�ʽ���ж��ڲ�д�����Ƿ����, һ��С�� 10ms 			
    CLKƵ��Ϊ200KHzʱ����ѯ����Ϊ30������
  */
  for (m = 0; m < 1000; m++)
  {				
    /* ��1��������I2C���������ź� */
    i2c_Start();
    
    /* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
    i2c_SendByte(ClientAddr | I2C_DIR_WR);	/* �˴���дָ�� */
    
    /* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
    if (i2c_WaitAck() == 0)
    {
      break;
    }
  }
  if (m  == 1000)
  {
    goto cmd_fail;	/* EEPROM����д��ʱ */
  }	
	
  while(NumByteToWrite--)
  {
  /* ��4������ʼд������ */
  i2c_SendByte(*pBuffer);

  /* ��5�������ACK */
  if (i2c_WaitAck() != 0)
  {
    goto cmd_fail;	/* ������Ӧ�� */
  }
  
      pBuffer++;	/* ��ַ��1 */		
  }
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	i2c_Stop();
	return 0;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 1;
}

#endif


/*********************************************END OF FILE**********************/
