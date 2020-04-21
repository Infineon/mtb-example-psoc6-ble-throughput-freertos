/******************************************************************************
* File Name: display_task.c
*
* Description: This file contains FreeRTOS task that updates E-ink display with
*              BLE GATT Throughput values.
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

#ifdef EINK_DISPLAY_SHIELD_PRESENT
/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "display_task.h"
#include "screen_contents.h"
#include "cy_eink_library.h"
#include "uart_task.h"
#include "ble_task.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"

#define EINK_INIT_TEMPERATURE_CELSIUS   (20u)

/* Frame buffers used for combining text and images into one frame */
cy_eink_frame_t frame_buffer_current[CY_EINK_FRAME_SIZE];
cy_eink_frame_t frame_buffer_prev[CY_EINK_FRAME_SIZE]={0};

/* Pointer to store address of the structure holding throughput values */
throughput_val_t* gatt_throughput = NULL;
/*******************************************************************************
* Function Name: void task_Display(void *pvParameters)
********************************************************************************
*
* Summary: FreeRTOS task which updates E-ink Display with BLE GATT Throughput
           Values
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void task_Display (void *pvParameters)
{
    /* Arrays to store TX and RX Throughput values in string format */
    char throughput_string_tx[6];
    char throughput_string_rx[6];

    /* Coordinates to display the image of Cypress logo */
    uint8_t   image_coordinates[] = {0u, 33u, 0u, 175u};

    /* Coordinates to display Throughput data */
    uint8_t   text_origin[] = {0x04u, 0x09u};

    /* Flag that indicates if the E-INK display has been detected */
    bool display_detected = false;

    /* Remove warning for unused parameter */
    (void)pvParameters ;

    /* Initialize the PSoC6 gpios connected to E-Ink display*/
    CY_EINK_InitDriver();

    /* Initialize the E-INK display hardware with the temperature value */
    Cy_EINK_Start(EINK_INIT_TEMPERATURE_CELSIUS);

    /* Power on the display and check if the operation was successful */
    display_detected = Cy_EINK_Power(CY_EINK_ON);

    /* Do this only if the E-INK display has been detected */
    if(display_detected)
    {
        iprintf("E-INK display power on");

        /* Clear the display */
        Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND, CY_EINK_POWER_MANUAL);

        /* Load the display buffer with Cypress logo and project heading */
        Cy_EINK_ImageToFrameBuffer(frame_buffer_prev,
                                  (cy_eink_frame_t *)logo,
                                  image_coordinates);
        Cy_EINK_TextToFrameBuffer(frame_buffer_prev, (char *)Heading_common,
                                    CY_EINK_FONT_8X12BLACK, text_origin);

        /* Display the initial message*/
        Cy_EINK_ShowFrame(CY_EINK_WHITE_FRAME, frame_buffer_prev,
                            CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);

        /* Repeatedly running part of the task */
        for(;;)
        {
            /* Block till notification from timer(2 seconds) callback*/
            ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

            /* If device BLE connection is lost display the message */
            if(device_disconnect_flag)
            {
                uint8_t   text_origin[] = {0x04u, 0x09u};
                /* Load the buffer with disconnection message and
                    * display it*/
                Cy_EINK_TextToFrameBuffer(frame_buffer_current,
                                         (char *)Disconnect_msg,
                                         CY_EINK_FONT_8X12BLACK,
                                         text_origin);

                Cy_EINK_ShowFrame(frame_buffer_prev,
                                 frame_buffer_current,
                                 CY_EINK_FULL_2STAGE,
                                 CY_EINK_POWER_AUTO);
            }
            else
            {
                /* Coordinates for Throughput message and values */
                uint8_t   text_origin[]    = {0x04u, 0x09u};
                uint8_t   throughput_origin_tx[] = {0x14u, 0x0Bu};
                uint8_t   throughput_origin_rx[] = {0x14u, 0x0Cu};

                /* Get the current throughput value from ble task */
                gatt_throughput = get_throughput();

                /* Convert throughput value to string for display */
                sprintf(throughput_string_tx," %lu ",
                        gatt_throughput->tx);

                sprintf(throughput_string_rx," %lu ",
                        gatt_throughput->rx);

                /* Load the values for the frame with new throughput values
                    * and display */
                Cy_EINK_ImageToFrameBuffer(frame_buffer_current,
                                            (cy_eink_frame_t *)logo,
                                            image_coordinates);
                Cy_EINK_TextToFrameBuffer(frame_buffer_current,
                                            (char *)Heading,
                                            CY_EINK_FONT_8X12BLACK,
                                            text_origin);
                Cy_EINK_TextToFrameBuffer(frame_buffer_current,
                                            (char *)throughput_string_tx,
                                            CY_EINK_FONT_8X12BLACK,
                                            throughput_origin_tx);
                Cy_EINK_TextToFrameBuffer(frame_buffer_current,
                                            (char *)throughput_string_rx,
                                            CY_EINK_FONT_8X12BLACK,
                                            throughput_origin_rx);
                Cy_EINK_ShowFrame(frame_buffer_prev, frame_buffer_current,
                                    CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);

                /* Copy the frame to use for next update */
                memcpy(frame_buffer_prev, frame_buffer_current,
                        sizeof(frame_buffer_current));
            }
        }
    }
    else
    {
        eprintf("E-ink Display Power ON failure");
        /* Suspend the task for power ON failure*/
        vTaskSuspend(NULL);
    }
}

#endif /* EINK_DISPLAY_SHIELD_PRESENT */
/* [] END OF FILE */
