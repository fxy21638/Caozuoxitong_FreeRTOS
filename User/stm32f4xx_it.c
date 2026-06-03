/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines
  ******************************************************************************
  */

#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp_debug_usart.h"
#include "bsp_led.h"

extern void xPortSysTickHandler(void);

/* SysTick ISR */
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

/* External references to main.c globals */
extern TaskHandle_t UartRxTask_Handle;
extern TimerHandle_t RxTimeoutTimer_Handle;
extern volatile uint8_t g_rx_ring[64];
extern volatile uint8_t g_rx_wr;

/**
  * @brief  USART1 RXNE interrupt handler.
  *         LED1 toggles on each byte (diagnostic).
  *         Writes byte into ring buffer, then wakes Task1 via task notification.
  *         Resets 200ms timeout timer on each byte.
  *         Clears USART error flags (ORE/FE/NE) to prevent RXNE lockup.
  */
void DEBUG_USART_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  uint32_t sr;

  /* Check each USART error flag individually */
  if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) != RESET)
  {
    /* Overrun error: read SR then DR to clear */
    sr = DEBUG_USART->SR;
    sr = DEBUG_USART->DR;
    (void)sr;
  }

  if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
  {
    uint8_t byte = USART_ReceiveData(DEBUG_USART);

    /* Diagnostic: toggle LED1 (red) on each received byte */
    LED1_TOGGLE;

    /* Write byte into ring buffer.
     * g_rx_wr is a free-running 8-bit counter; the task
     * consumes bytes by advancing g_rx_rd to match g_rx_wr. */
    g_rx_ring[g_rx_wr & 0x3F] = byte;
    g_rx_wr++;

    /* Only call FreeRTOS APIs after the scheduler has started.
     * Before that, bytes are discarded (ring buffer cleared in main). */
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
      xTaskNotifyFromISR(UartRxTask_Handle,
                         0x01,  /* NOTIFY_BIT_BYTES */
                         eSetBits,
                         &pxHigherPriorityTaskWoken);

      /* Reset 200ms timeout timer */
      if (RxTimeoutTimer_Handle != NULL) {
        xTimerResetFromISR(RxTimeoutTimer_Handle, &pxHigherPriorityTaskWoken);
      }
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif

    /* Request context switch if a higher priority task was woken */
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
  }
}
