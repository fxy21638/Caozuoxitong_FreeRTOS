/* ========================================================================== */
/*  stm32f4xx_it.c — 中断服务函数                                              */
/*                                                                            */
/*  SysTick_Handler        : FreeRTOS 系统时钟                                 */
/*  DEBUG_USART_IRQHandler : USART1 调试串口 (仅清除错误标志)                    */
/*  USART2_IRQHandler      : RS485 总线 (RXNE + TC + ORE)                     */
/* ========================================================================== */

#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp_debug_usart.h"
#include "./rs485/bsp_rs485.h"
#include "semphr.h"
#include "bsp_led.h"

extern void xPortSysTickHandler(void);

/* ========================================================================== */
/*  SysTick — FreeRTOS 系统时钟                                                */
/* ========================================================================== */
void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif
    xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif
}

/* main.c 中定义的 RS485 句柄，ISR 需要引用 */
extern TaskHandle_t  RS485RxTask_Handle;
extern TimerHandle_t FrameTimeout_Handle;

/* ========================================================================== */
/*  USART1 调试串口 — 仅清除错误，不处理接收                                     */
/* ========================================================================== */
void DEBUG_USART_IRQHandler(void)
{
  uint32_t sr;

  /* 清除 ORE 溢出标志 (防止 RXNE 锁死) */
  if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) != RESET)
  {
    sr = DEBUG_USART->SR;
    sr = DEBUG_USART->DR;
    (void)sr;
  }

  /* 丢弃接收到的字节 (调试串口仅用于 printf 输出) */
  if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
  {
    (void)USART_ReceiveData(DEBUG_USART);
  }
}

/* ========================================================================== */
/*  USART2 RS485 总线中断                                                       */
/*                                                                            */
/*  RXNE: 读取数据 → 写入环形缓冲 → 通知 RS485RxTask → 重置帧超时定时器          */
/*  TC  : 传输完成 (仅清除中断挂起位, DE 由 RS485_FinishTx 处理)                */
/*  ORE : 溢出错误 (读取 DR 清除锁死)                                           */
/* ========================================================================== */
void USART2_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

  /* RXNE: 数据接收就绪 */
  if (USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)
  {
    RS485_RxISR();    /* 读 DR → 清 ORE → 写入环形缓冲 */

#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
      /* 通知 RS485RxTask 有新字节 */
      xTaskNotifyFromISR(RS485RxTask_Handle, 0x01,
                         eSetBits, &pxHigherPriorityTaskWoken);

      /* 重置 100ms 帧超时定时器 */
      if (FrameTimeout_Handle != NULL) {
        xTimerResetFromISR(FrameTimeout_Handle, &pxHigherPriorityTaskWoken);
      }
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif
  }

  /* TC: 传输完成 (ISR 仅清除挂起位，DE 切换由任务上下文 RS485_FinishTx 负责) */
  if (USART_GetITStatus(RS485_USART, USART_IT_TC) != RESET)
  {
    USART_ClearITPendingBit(RS485_USART, USART_IT_TC);
  }

  /* ORE: 溢出错误 (读取 DR 防止 RXNE 锁死) */
  if (USART_GetFlagStatus(RS485_USART, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(RS485_USART);
  }

  /* 如果唤醒了更高优先级任务，触发上下文切换 */
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/* ========================================================================== */
/*  EXTI15_10 — GT9xx 触摸屏中断 (PD13, 下降沿)                                */
/*  极轻量 ISR: 仅 give 信号量，I2C 读取在任务上下文完成                        */
/* ========================================================================== */
extern QueueHandle_t xTouchSemaphore;

void EXTI15_10_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
    xSemaphoreGiveFromISR(xTouchSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
