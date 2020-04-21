/******************************************************************************
* File Name: led_task.c
*
* Description: This file contains FreeRTOS task that controls LED status.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "led_task.h"
#include "uart_task.h"
#include "cyhal.h"
#include "cybsp.h"

/*******************************************************************************
* Function Name: void task_LED(void *pvParameters)
********************************************************************************
*
* Summary: FreeRTOS task which controls the status of user LED1 and user LED2
*            User LED1 ON : BLE device connected with peer device
*            User LED1 OFF: BLE device disconnected with peer/ not connected to
*                           any peer device
*            User LED2 ON : GATT data being transmitted
*            User LED2 OFF: No GATT data transfer
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/

void task_LED(void *pvParameters)
{
    /* Variable to store led status received from queue */
    led_status_t ledStatus;

    /* Variable used to store return values of RTOS APIs */
    BaseType_t rtosApiResult;

    /* Remove warning for unused variable */
    (void)pvParameters;

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over led_cmdQ */
        rtosApiResult = xQueueReceive(led_cmdQ, &ledStatus, portMAX_DELAY);

        /* Command has been received from led_cmdQ */
        if(rtosApiResult == pdTRUE)
        {
            /* Turn user LED1 ON or OFF based on BLE connection status */
            if(ledStatus.conn_led == CYBSP_LED_STATE_ON)
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,
                                  CYBSP_LED_STATE_ON);
            }
            else
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1,
                                  CYBSP_LED_STATE_OFF);
            }

            /* Turn user LED2 ON or OFF based on GATT data traffic */
            if(ledStatus.data_led == CYBSP_LED_STATE_ON)
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2,
                                  CYBSP_LED_STATE_ON);
            }
            else
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2,
                                  CYBSP_LED_STATE_OFF);
            }
        }
    }
}
/* [] END OF FILE */
