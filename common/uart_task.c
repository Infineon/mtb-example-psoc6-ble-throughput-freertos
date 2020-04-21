/******************************************************************************
 * File Name: uart_task.c
 *
 * Description: This file contains the task that is used for thread-safe UART
 *              based debug.
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
/* Header file includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart_task.h"

/* This files used only if thread safe uart is enabled */
#if UART_TASK_ENABLE

/* Queue length of message queue used for uart task */
#define UART_QUEUE_SIZE 		(16u)

/* Maximum allowed length of a message */
#define UART_MESSAGE_MAX_LEN	(100u)

/* Queue handle for uart message Queue */
QueueHandle_t uartMessageQ;

/* Structure for storing debug message data */
typedef struct
{
	const char* stringPtr;
	uart_message_type_t messageType;
} uart_data_t;

TaskHandle_t uart_task_handle;

/*******************************************************************************
 * Function Name: void Task_UART(void* pvParameters)
 ********************************************************************************
 * Summary:
 *  Task that prints messages using retarget IO.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void Task_UART(void* pvParameters)
{
	uart_data_t messageData;

	/* Variable used to store the return values of RTOS APIs */
	BaseType_t rtosApiResult;

	/* Remove warning for unused parameter */
	(void) pvParameters;

	/* Repeatedly running part of the task */
	for(;;)
	{
		rtosApiResult = xQueueReceive(uartMessageQ, &messageData, portMAX_DELAY);

		if(rtosApiResult == pdPASS)
		{
			switch(messageData.messageType)
			{
				case none:
				{
					printf("%s", (char *)messageData.stringPtr);
					break;
				}
				case info:
				{
					printf("[Info]    %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				case warning:
				{
					printf("[Warning] %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				case error:
				{
					printf("[Error]   %s\r\n", (char *)messageData.stringPtr);
					break;
				}
				default:
				{
					break;
				}
			}

			/* free the message buffer allocated by the message sender */
			vPortFree((char *)messageData.stringPtr);
		}
		/** Task has timed out and received no commands during an interval of
		 * portMAXDELAY ticks */
		else
		{
			printf("UART TAsk: portMAX_DELAY exceeded\r\n");
		}
	}
}

/*******************************************************************************
 * Function Name: void Task_Printf(uart_message_type_t messageType,
 * 					char* stringPtr, ...)
 ********************************************************************************
 *
 * Description: This function sends messages to the debug Queue.
 *
 *******************************************************************************/
void Task_Printf(uart_message_type_t messageType, char* stringPtr, ...)
{
	uart_data_t messageData;
	char* messageBuffer;
	va_list args;

	/* Allocate the message buffer */
	messageBuffer = (char *) pvPortMalloc(UART_MESSAGE_MAX_LEN);

	if(messageBuffer != NULL)
	{
		va_start(args, stringPtr);

		vsnprintf((messageBuffer), (UART_MESSAGE_MAX_LEN), stringPtr, args);

		va_end(args);

		messageData.messageType = messageType;
		messageData.stringPtr = messageBuffer;

		/* The receiver task is responsible to free the memory from here on */

		if(uxQueueSpacesAvailable(uartMessageQ) != 0)
		{
			if(pdPASS != xQueueSend(uartMessageQ, &messageData, 0u))
			{
				/* Failed to send the message into the queue */
				vPortFree(messageBuffer);
			}
		}
		else
		{
			printf("UART Queue is full!! Message cannot be printed!!");
		}
	}
}

/*******************************************************************************
 * Function Name: void Task_UARTInit(void)
 ********************************************************************************
 *
 * Description: Initializes the underlying Task and Queue used for uart
 *
 * Parameters: None
 *
 * Return: None
 *
 *******************************************************************************/
void Task_UARTInit(void)
{
	uartMessageQ = xQueueCreate(UART_QUEUE_SIZE, sizeof(uart_data_t));
	xTaskCreate(Task_UART, "UART Task", (configMINIMAL_STACK_SIZE * 2),
			NULL, (configMAX_PRIORITIES - 4), NULL);
};

#endif

/* [] END OF FILE */
