#ifndef __ISR_H__
#define __ISR_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* interrupt routine service priority list  */
#define SBUS_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)
#define SYS_TIMER_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2)
#define UART4_TX_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3)

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#endif
