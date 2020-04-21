/******************************************************************************
* File Name: main.c
*
* Description: This code example demonstrates the implementation of a BLE
*              GATT(Client) Throughput Measurement using PSoC 6 MCU with
*              Bluetooth Low Energy (BLE) Connectivity.
*
*              The code example uses the Cypress PSoC 6 Bluetooth Low Energy
*              Middleware. See https://cypresssemiconductorco.github.io/bless/ble_api_reference_manual/html/index.html
*              for more details on middleware.
*
*              This code example uses FreeRTOS. Visit the FreeRTOS website
*              (https://www.freertos.org) for documentation and API references
*              of FreeRTOS.
*
* Related Document: See README.md
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

/*******************************************************************************
 * Header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cybsp_types.h"
#include "cy_retarget_io.h"
#include "ble_task.h"
#include "led_task.h"
#include "uart_task.h"
#ifdef EINK_DISPLAY_SHIELD_PRESENT
#include "display_task.h"
#endif

/*******************************************************************************
* Macros
*******************************************************************************/
/* Priority of user tasks in this application. Valid range of task priority is
 * 0 to ( configMAX_PRIORITIES - 1 ), where configMAX_PRIORITIES is defined
 * within FreeRTOSConfig.h.
 */
#define BLE_TASK_PRIORITY       (configMAX_PRIORITIES - 1)
#define LED_TASK_PRIORITY       (configMAX_PRIORITIES - 2)
#define DISPLAY_TASK_PRIORITY   (configMAX_PRIORITIES - 3)
#define GPIO_INTERRUPT_PRIORITY (7u)

/* Stack size used for task */
#define BLE_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 4)
#define LED_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define DISPLAY_STACK_SIZE      (configMINIMAL_STACK_SIZE * 4)

/* Queue lengths of message queues used in this application */
#define BLE_CMD_Q_LEN           (10u)
#define LED_CMD_Q_LEN           (5u)

/* Queue lengths of message queues used in this application */
QueueHandle_t ble_cmdQ, led_cmdQ;
TaskHandle_t  ble_task_handle, led_task_handle, display_task_handle;
TimerHandle_t timer_handle;

/* Global variable */
tx_rx_mode mode_flag = GATT_NOTIF_STOC;

/***************************************************************************************************
* Function Name: static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
****************************************************************************************************
* Summary: GPIO interrupt handler
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
* Return:
*  None
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    /* Check for first button press - Device should start scanning on first
     * button press */
    if(scan_flag == false)
    {
        scan_flag = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(ble_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /* Process button press only after complete BLE connection is established */
    if(button_flag == true)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        ble_command_type_t ble_command = BLE_PROCESS_EVENTS;

        /* After connection, upon each button press:
         * change GATT data transfer mode in the following order:
         * GATT notifications -> GATT writes -> both GATT notification and
         * GATT write -> rollback to GATT Notifications
         */
        mode_flag = (mode_flag == GATT_WRITE_AND_NOTIF)? GATT_NOTIF_STOC : mode_flag + 1;
        switch(mode_flag)
        {
            case GATT_NOTIF_STOC:
            {
                ble_command = BLE_ENABLE_NOTIFICATION;
                gatt_write_flag = false;
                break;
            }
            case GATT_WRITE_CTOS:
            {
                ble_command = BLE_DISABLE_NOTIFICATION;
                gatt_write_flag = true;
                break;
            }
            case GATT_WRITE_AND_NOTIF:
            {
                ble_command = BLE_ENABLE_NOTIFICATION;
                gatt_write_flag = true;
                break;
            }
        }

        /* Notify ble task immediately to process BLE commands */
        xQueueSendFromISR(ble_cmdQ, &ble_command, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM4 CPU. This function creates required queues,
*  sets up user tasks and then starts the RTOS scheduler.
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    /* Variable to store return value from RTOS APIs */
    BaseType_t rtos_api_result = pdPASS;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                        CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LEDs */
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                     CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT,
                     CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the user button */
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_BTN1, CYHAL_GPIO_DIR_INPUT,
                     CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback((cyhal_gpio_t)CYBSP_USER_BTN1,
                                 gpio_interrupt_handler, NULL);
    cyhal_gpio_enable_event((cyhal_gpio_t)CYBSP_USER_BTN1, CYHAL_GPIO_IRQ_FALL,
                             GPIO_INTERRUPT_PRIORITY, true);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("~~~~~~ PSoC 6 MCU BLE GATT Throughput Measurement : CLIENT ROLE ~~~~~~ \r\n");

    ble_cmdQ = xQueueCreate(BLE_CMD_Q_LEN, sizeof(ble_command_type_t));
    led_cmdQ = xQueueCreate(LED_CMD_Q_LEN, sizeof(led_status_t));

    timer_handle = xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdTRUE,
                               NULL, rtos_timer_cb);

    /* Create the user tasks. See the respective Task definition for more
        * details of these tasks */

    rtos_api_result |= xTaskCreate(task_BLE, "BLE Task", BLE_TASK_STACK_SIZE,
                                   NULL, BLE_TASK_PRIORITY, &ble_task_handle);
    rtos_api_result |= xTaskCreate(task_LED, "LED Task", LED_TASK_STACK_SIZE,
                                   NULL, LED_TASK_PRIORITY, &led_task_handle);
#ifdef EINK_DISPLAY_SHIELD_PRESENT
    rtos_api_result |= xTaskCreate(task_Display, "Display Task",
                                   DISPLAY_STACK_SIZE, NULL,
                                   DISPLAY_TASK_PRIORITY, &display_task_handle);
#endif

    stdio_task_init();

    if (pdPASS  == rtos_api_result)
    {
        /* Start the RTOS scheduler. This function should never return */
        vTaskStartScheduler();

        /* Program should never reach here! */
        printf("[Error]   : FreeRTOS scheduler failed to start \r\n");
    }
    else
    {
        printf("[Error]   : FreeRTOS failed to create task \r\n");
        /* Halt the CPU if failed to create task or failed to start scheduler */
        CY_ASSERT(0u);
    }

    for(;;)
    {

    }
}
/* END OF FILE */
