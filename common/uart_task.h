/******************************************************************************
 * File Name: uart_task.h
 *
 * Description: This file is public interface of uart_task.c source file
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 *******************************************************************************/

/* Include guard */
#ifndef UART_TASK_H
#define UART_TASK_H

/* Header file includes */
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"

/* (true) enables uart task and (false) disables it. Note that enabling
 * thread safe uart adds another task to the project and increases code
 * size. Retarget IO must be initialized before enabling the UART task */
#define UART_TASK_ENABLE   (true)

/* (true) enables eprintf(), wprintf(), iprintf() used for debug in this
 * application. (false) disables the three functions */
#define UART_DEBUG_ENABLE  (false)

/** Declaration of empty or default value macros if the thread safe
 *  uart is not enabled for efficient code generation. */
#if UART_TASK_ENABLE
/* Thread safe uart contents */

/* Debug message type */
typedef enum
{
	none = 0,
	info = 1,
	warning = 2,
	error = 3
} uart_message_type_t;

extern TaskHandle_t uart_task_handle;

/* Internal function prototypes */
void Task_Printf(uart_message_type_t messageType, char* stringPtr, ...);
void Task_UARTInit(void);

/* tprintf - thread safe generic printf that can be used from rtos tasks */
#define tprintf(...)		Task_Printf(none, __VA_ARGS__)
#define stdio_task_init() 	Task_UARTInit()

#if UART_DEBUG_ENABLE

/* eprintf - thread safe Error printf that can be used from rtos tasks */
#define eprintf(...)		Task_Printf(error, __VA_ARGS__)
/* wprintf - thread safe Warning printf that can be used from rtos tasks */
#define wprintf(...) 		Task_Printf(warning, __VA_ARGS__)
/* iprintf - thread safe Info printf that can be used from rtos tasks */
#define iprintf(...)		Task_Printf(info, __VA_ARGS__)

#else
#define eprintf(...)
#define wprintf(...)
#define iprintf(...)

#endif /* UART_DEBUG_ENABLE */

#else
#define tprintf(...)
#define eprintf(...)
#define wprintf(...)
#define iprintf(...)
#define stdio_task_init()

#endif /* UART_TASK_ENABLE */
#endif /* UART_TASK_H */
/* [] END OF FILE */
