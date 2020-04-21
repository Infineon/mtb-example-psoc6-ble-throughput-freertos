/******************************************************************************
* File Name: ble_task.c
*
* Description: This file contains the task that initializes BLE and
*              handles different BLE events.
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
#include "ble_task.h"
#include "led_task.h"
#include "uart_task.h"
#include "cycfg_ble.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_pdl.h"

#ifdef EINK_DISPLAY_SHIELD_PRESENT
#include "display_task.h"
#endif

/*************************************************************************************************************************************
 * Macros
 ************************************************************************************************************************************/
#define CUSTOM_SERVICE_NOTIF_CCCD_ATT_HANDLE      CY_BLE_THROUGHPUT_MEASUREMENT_NOTIFY_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE
#define CUSTOM_SERVICE_GATT_WRITE_CHAR_HANDLE     CY_BLE_THROUGHPUT_MEASUREMENT_WRITEME_CHAR_HANDLE
#define CUSTOM_SERVICE_NOTIF_ATT_HANDLE           CY_BLE_THROUGHPUT_MEASUREMENT_NOTIFY_CHAR_HANDLE
#define BLE_INTERRUPT_PRIORITY                    (1u)
#define BLE_PROCESS_EVENTS                        (1u)
#define CONN_INTERVAL_MULTIPLIER                  (1.25f)
#define MIN_CONN_INTERVAL                         (54)
#define MAX_CONN_INTERVAL                         (60)
#define SUPERVISION_TIMEOUT                       (400)

/******************************************************************************
 * Global Variables
 *****************************************************************************/
/* Variables to hold GATT notification byte count or GATT write byte count */
static uint32_t gatt_write_rx_bytes;
static uint32_t notif_tx_bytes;

/* Variable to store latest connection interval for the BLE connection */
static float conn_interval;

/* Flags */
/* To indicate if GATT Client has subscribed for notifications or not */
static bool notify_flag;
/* To indicate if BLE stack is busy or free */
static bool stack_free = true;
/* To indicate display task that the device has disconnected */
bool device_disconnect_flag = false;

/* Variable to store TX and RX throughput values */
throughput_val_t server_throughput = {0u, 0u};

/* Variable to store connection parameters after GAP connection */
static cy_stc_ble_gap_connected_param_t conn_param;

/* Connection handle to identify the connected peer device */
static cy_stc_ble_conn_handle_t conn_handle;

/* Pointer to store a single attribute information present in the GATT DB of the server */
static cy_stc_ble_gatt_write_param_t *write_req_param;

/* Variable to store information about custom data to be sent as notification to GATT Client */
static cy_stc_ble_gatt_handle_value_pair_t custom_data;

/* Variable to store MTU size for active BLE connection */
static uint16_t att_mtu_size = CY_BLE_GATT_MTU;

/* Variable to store user LED status */
static led_status_t led_status = {CYBSP_LED_STATE_OFF, CYBSP_LED_STATE_OFF};

#ifdef EINK_DISPLAY_SHIELD_PRESENT
/* Variable used to refresh the E-ink display every 5 seconds */
static uint8_t count_five_sec;
#endif

/******************************************************************************
 * Function Prototypes
******************************************************************************/
static void ble_init(void);
static void bless_interrupt_handler(void);
static void ble_controller_interrupt_handler(void);
static void ble_stack_event_handler(uint32_t event, void *eventParam);
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param);
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param);

/*******************************************************************************
* Function Name: void task_BLE(void *pvParameters)
********************************************************************************
*
* Summary: FreeRTOS task which handles the BLE activity of the application
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void task_BLE(void *pvParameters)
{
    /* Remove compiler warning for unused variable */
    (void)pvParameters;

    /* Variable to store return value from FreeRTOS APIs */
    BaseType_t rtos_api_result;

    /* Variable to keep track of the BLE API result */
    static cy_en_ble_api_result_t ble_api_result;

    /* Variable to store BLE command received from BLE queue */
    uint8_t bleCmd = BLE_PROCESS_EVENTS;

    /* Initialize BLE and process any stack events */
    ble_init();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a BLE command has been received over ble_cmdQ */
        rtos_api_result = xQueueReceive(ble_cmdQ, &bleCmd, portMAX_DELAY);

        /* Command has been received from ble_cmdQ */
        if(rtos_api_result == pdTRUE)
        {
            /* Process the ble stack events */
            if(bleCmd == BLE_PROCESS_EVENTS)
            {
                Cy_BLE_ProcessEvents();
            }
        }

        /* notify_flag is used to indicate if notifications are enabled by
         * GATT client */
        if(notify_flag)
        {
            if(stack_free)
            {
                /* Send notification */
                ble_api_result = Cy_BLE_GATTS_SendNotification(&conn_handle, &custom_data);

                if(ble_api_result == CY_BLE_SUCCESS)
                {
                    /* Increment notification byte count only if notification
                     * was sent successfully */
                    notif_tx_bytes += custom_data.value.len;

                    /* Switch ON LED2 to show data TX */
                    led_status.data_led = CYBSP_LED_STATE_ON;
                    xQueueSend(led_cmdQ, &led_status, (TickType_t)0);
                }
                else
                {
                    /* Sending notifications failed. Switch OFF LED2 to show there is no data TX */
                    led_status.data_led = CYBSP_LED_STATE_OFF;
                    xQueueSend(led_cmdQ, &led_status, (TickType_t)0);
                }
            }
            else
            {
                    /* Stack is busy. Switch OFF LED2 to show data TX stopped */
                    led_status.data_led = CYBSP_LED_STATE_OFF;
                    xQueueSend(led_cmdQ, &led_status, (TickType_t)0);
            }
        }
    }
}

/*******************************************************************************
* Function Name: static void ble_init(void)
********************************************************************************
* Summary:
*  This function initializes the BLE Host and Controller, configures BLE
*  interrupt, and registers Application Host callbacks.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void ble_init(void)
{
    cy_en_ble_api_result_t ble_api_result = CY_BLE_SUCCESS;

    /* BLESS interrupt configuration structure */
    const cy_stc_sysint_t bless_isr_config =
    {
       /* The BLESS interrupt */
      .intrSrc = bless_interrupt_IRQn,

       /* The interrupt priority number */
      .intrPriority = BLE_INTERRUPT_PRIORITY
    };

    /* Store the pointer to blessIsrCfg in the BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

    /* Register the generic callback functions */
    Cy_BLE_RegisterEventCallback(ble_stack_event_handler);

    /* Register the application Host callback */
    Cy_BLE_RegisterAppHostCallback(ble_controller_interrupt_handler);

    /* Initialize the BLE */
    ble_api_result = Cy_BLE_Init(&cy_ble_config);
    if(ble_api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration, notify error
         * and halt CPU in debug mode
         */
        eprintf("Cy_BLE_Init API, errorcode: 0x%X", ble_api_result);
        vTaskSuspend(NULL);
    }

    /* Enable BLE */
    ble_api_result = Cy_BLE_Enable();
    if(ble_api_result != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration, notify error
         * and halt CPU in debug mode
         */
        eprintf("Cy_BLE_Enable API errorcode: 0x%X ", ble_api_result);
        vTaskSuspend(NULL);
    }
    /* Process BLE events after enabling BLE */
    Cy_BLE_ProcessEvents();
}

/*******************************************************************************
* Function Name: void ble_stack_event_handler (uint32_t event, void *eventParam)
********************************************************************************
* Summary: Call back event function to handle various events from the BLE stack.
*
* Parameters:
*  uint32_t event        :    Event from BLE stack
*  void eventParam       :    Pointer to the value of event specific parameters
*
* Return:
*  None
*
*******************************************************************************/
static void ble_stack_event_handler (uint32_t event, void *eventParam)
{
    /* Variable used to store the return values of BLE APIs */
    cy_en_ble_api_result_t ble_api_result;

    /* Take an action based on the current event */
    switch(event)
    {
        /***********************************************************************
        *                       General Events                                 *
        ***********************************************************************/
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_STACK_ON");
            /* Start Advertisement and enter discoverable mode */
            ble_api_result = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            if(ble_api_result == CY_BLE_SUCCESS)
            {
                iprintf("BLE Advertisement started successfully");
            }
            else
            {
                eprintf("BLE Advertisement API, errorcode = 0x%X", ble_api_result);
            }
            break;
        }

        /* This event indicates BLE stack status. This event is used to handle
         * data throttling which may occur due to continuous notification being
         * sent */
        case CY_BLE_EVT_STACK_BUSY_STATUS:
        {
            /* Variable to store status of the stack */
            cy_stc_ble_l2cap_state_info_t stack_status;
            stack_status = *( cy_stc_ble_l2cap_state_info_t*)eventParam;

            if(stack_status.flowState == CY_BLE_STACK_STATE_BUSY)
            {
                /* If stack is busy, stop notifications */
                stack_free = false;
            }
            else
            {
                /* If stack is free, start notifications */
                stack_free = true;
            }
            break;
        }

        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
        {
            iprintf("BLE Stack Event - Event timeout");
            break;
        }

        /**********************************************************************
        *                       GAP Events                                    *
        **********************************************************************/

        /* This event indicates that the peripheral device has started/stopped
         * advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            cy_en_ble_adv_state_t adv_state;
            iprintf("BLE Stack Event : CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP");
            adv_state = Cy_BLE_GetAdvertisementState();
            if(adv_state == CY_BLE_ADV_STATE_ADVERTISING)
            {
                tprintf("Advertising.....\r\n");
            }
            break;
        }

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_CONNECTED");
            conn_param = *((cy_stc_ble_gap_connected_param_t*)eventParam);

            /* Variable to store values to update PHY to 2M */
            cy_stc_ble_set_phy_info_t phy_param;
            phy_param.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
            phy_param.bdHandle = conn_handle.bdHandle;
            phy_param.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
            phy_param.txPhyMask = CY_BLE_PHY_MASK_LE_2M;

            /* Reset the connection status flag upon reconnection */
            device_disconnect_flag = false;

            /* Send user LED1 status to the LED queue */
            led_status.conn_led = CYBSP_LED_STATE_ON;
            xQueueSend(led_cmdQ, &led_status, (TickType_t)0);

            /* Function call to set PHY to 2M */
            ble_api_result = Cy_BLE_SetPhy(&phy_param);
            if(ble_api_result == CY_BLE_SUCCESS)
            {
                iprintf("Set PHY to 2M successfull");
                tprintf("Request sent to switch PHY to 2M\r\n");
            }
            else
            {
                eprintf("Set PHY to 2M API failure, errorcode = 0x%X", ble_api_result);
            }
            conn_interval = (conn_param.connIntv) * CONN_INTERVAL_MULTIPLIER;
            tprintf("Connection Interval is: %f ms\r\n", conn_interval);
            break;
        }

        /* This event is generated when the device is disconnected from remote
         * device or fails to establish connection */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_DISCONNECTED");
            tprintf("Disconnected from peer!!!\r\n");

            /* Stop the FreeRTOS timer upon device disconnection */
            xTimerStop(timer_handle, (TickType_t)0);

            /* Send user LED1 status to the LED queue */
            led_status.conn_led = CYBSP_LED_STATE_OFF;
            led_status.data_led = CYBSP_LED_STATE_OFF;
            xQueueSend(led_cmdQ, &led_status, (TickType_t)0);

            /* Set the device disconnected flag */
            device_disconnect_flag = true;

            /* Reset the notify flag and stack_free flag after disconnection */
            notify_flag = false;
            stack_free = true;
            /* Clear the byte count */
            gatt_write_rx_bytes = 0u;
            notif_tx_bytes = 0u;
            /* Start undirected advertisement again */
            ble_api_result = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            if(ble_api_result == CY_BLE_SUCCESS)
            {
                iprintf("BLE Advertisement API successfull");
            }
            else
            {
                eprintf("BLE Advertisement API failure, errorcode = 0x%X", ble_api_result);
            }

#ifdef EINK_DISPLAY_SHIELD_PRESENT
            /* Notify display task immediately to update E-ink display with
             * appropriate message */
            count_five_sec = 0u;
            xTaskNotifyGive(display_task_handle);
#endif
            break;
        }

        /* This event is generated when PHY is updated during an active connection */
        case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_PHY_UPDATE_COMPLETE");

            cy_stc_ble_events_param_generic_t *genericParam;
            cy_stc_ble_phy_param_t *currentPHY;
            genericParam = (cy_stc_ble_events_param_generic_t*)eventParam;

            /* GenericParam has to be cast to cy_stc_ble_phy_param_t to get TX
             * and RX PHY */
            currentPHY = (cy_stc_ble_phy_param_t*)(genericParam->eventParams);

            /* Print the RX PHY selected on UART terminal */
            switch(currentPHY->rxPhyMask)
            {
                case CY_BLE_PHY_MASK_LE_1M:
                tprintf("Selected Rx PHY: 1M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_2M:
                tprintf("Selected Rx PHY: 2M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_CODED:
                tprintf("Selected Rx PHY: LE Coded\r\n");
                break;
            }

            /* Print the TX PHY selected on UART terminal */
            switch(currentPHY->txPhyMask)
            {
                case CY_BLE_PHY_MASK_LE_1M:
                tprintf("Selected Tx PHY: 1M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_2M:
                tprintf("Selected Tx PHY: 2M\r\n");
                break;

                case CY_BLE_PHY_MASK_LE_CODED:
                tprintf("Selected Tx PHY: LE Coded\r\n");
                break;
            }

            /* Variable to hold data for new connection parameters request */
            cy_stc_ble_gap_conn_update_param_info_t conn_params = {MIN_CONN_INTERVAL,
                                                                   MAX_CONN_INTERVAL,
                                                                   0u,
                                                                   SUPERVISION_TIMEOUT,
                                                                   conn_handle.bdHandle,
                                                                   0u};
            /* Check if connection parameters decided by Client is in the desired range */
            if((conn_param.connIntv <= MIN_CONN_INTERVAL) || (conn_param.connIntv >= MAX_CONN_INTERVAL))
            {
                /* Request for new connection parameters */
                ble_api_result = Cy_BLE_L2CAP_LeConnectionParamUpdateRequest(&conn_params);
                if(ble_api_result == CY_BLE_SUCCESS)
                {
                    iprintf("L2CAP Connection Parameter update request sent successfully");
                }
                else
                {
                    eprintf("FAILURE : L2CAP Connection Parameter update request failed, errorcode = 0x%X", ble_api_result);
                }
            }
            break;
        }

        /* This event is generated when connection parameter update is
         * requested. If the request is accepted by the Central, this event is
         * generated on both the devices. If the request is rejected, this event
         * is not generated */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE");
            cy_stc_ble_gap_conn_param_updated_in_controller_t *new_conn_params;
            new_conn_params = (cy_stc_ble_gap_conn_param_updated_in_controller_t*)eventParam;

            /* Store the new connection interval value */
            conn_interval = (new_conn_params->connIntv) * CONN_INTERVAL_MULTIPLIER;
            tprintf("Updated Connection Interval: %f ms\r\n", conn_interval);
            break;
        }

        /***********************************************************************
        *                          GATT Events                                 *
        ***********************************************************************/
        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            conn_handle = *((cy_stc_ble_conn_handle_t*)eventParam);
            iprintf("BLE Stack Event : CY_BLE_EVT_GATT_CONNECT_IND");
            tprintf("GATT connected\r\n");

            /* Start 1 second timer to calculate throughput */
            xTimerStart(timer_handle, (TickType_t)0);
            break;
        }

        /* This event indicates that the GATT is disconnected.*/
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GATT_DISCONNECT_IND");
            break;
        }

        /* This event is generated when a 'write request' is received from GATT
         * Client device */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GATTS_WRITE_REQ");
            write_req_param = (cy_stc_ble_gatt_write_param_t*)eventParam;

            /* Clear both tx and rx data bytes count */
            gatt_write_rx_bytes = 0u;
            notif_tx_bytes = 0u;

            /* Send response to GATT Client device */
            Cy_BLE_GATTS_WriteRsp(conn_handle);

            /* Update the attribute with new values received from the client
             * device i.e., In this case, enable or disable notification  */
            update_gatt_db_notif(write_req_param);
#ifdef EINK_DISPLAY_SHIELD_PRESENT
            /* Notify the display task */
            xTaskNotifyGive(display_task_handle);
#endif
            break;
        }

        /* This event is generated when a 'write command' is received from GATT
         * client device */
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
        {
            write_req_param = (cy_stc_ble_gatt_write_param_t*)eventParam;

            /* Write the value received from GATT Client device into GATT Server
             * database */
            update_gatt_db_write(write_req_param);
            break;
        }

         /* This event is triggered when a 'read request' is received from GATT
          * client device */
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ");
            break;
        }

         /* This event is triggered when 'GATT MTU Exchange Request' is received
          * from GATT client device
          */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            iprintf("BLE Stack Event : CY_BLE_EVT_GATTS_XCNHG_MTU_REQ");
            cy_stc_ble_gatt_xchg_mtu_param_t mtuParam;
            mtuParam = *(cy_stc_ble_gatt_xchg_mtu_param_t*)eventParam;
            tprintf("Client MTU Size: %d \r\n", mtuParam.mtu);
            if(mtuParam.mtu > CY_BLE_GATT_MTU)
            {
                att_mtu_size = CY_BLE_GATT_MTU;
            }
            else
            {
                att_mtu_size = mtuParam.mtu;
            }
            break;
        }

        /***********************************************************************
        *                          L2CAP Events                                 *
        ***********************************************************************/
        /* This event is generated when there is response from host device for
         * connection parameter update request */
        case CY_BLE_EVT_L2CAP_CONN_PARAM_UPDATE_RSP:
        {
            cy_stc_ble_l2cap_conn_update_rsp_param_t conn_param_rsp;
            conn_param_rsp = *((cy_stc_ble_l2cap_conn_update_rsp_param_t*)eventParam);

            /* Print the response on UART terminal*/
            if(conn_param_rsp.result == 0u)
            {
                tprintf("Connection Interval update request accepted\r\n");
            }
            else
            {
                tprintf("Connection Interval update request rejected\r\n");
            }
            break;
        }

        /***********************************************************************
        *                           Other Events                               *
        ***********************************************************************/
        default:
        {
            iprintf("Other BLE event: 0x%X", (uint32_t)event);
            break;
        }
    }
}

/*******************************************************************************
* Function Name: static void bless_interrupt_handler(void)
********************************************************************************
* Summary:
*  Wrapper function for BLESS interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void bless_interrupt_handler(void)
{
    /* Process interrupt events generated by the BLE sub-system */
    Cy_BLE_BlessIsrHandler();
}

/*******************************************************************************
* Function Name: static void ble_controller_interrupt_handler(void)
********************************************************************************
* Summary:
*  Call back event function to handle interrupts from BLE Controller
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void ble_controller_interrupt_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Send command to process BLE events  */
    uint8_t bleCommand = BLE_PROCESS_EVENTS;
    xQueueSendFromISR(ble_cmdQ, &bleCommand, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**********************************************************************************
* Function Name: void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
***********************************************************************************
* Summary: This function updates the CCCD attribute of GATT Notify
*          characteristic present in the custom service
*
* Parameters:
*  cy_stc_ble_gatt_write_param_t *write_param: parameter which holds information
*                                    about attribute handle and attribute value
*
* Return:
*  None
**********************************************************************************/
static void update_gatt_db_notif(cy_stc_ble_gatt_write_param_t *write_param)
{
    cy_en_ble_gatt_err_code_t gattErrorCode;

    iprintf("Info: Attribute handle: 0x%X", write_param->handleValPair.attrHandle);
    iprintf("Info: Attribute Value: 0x%X", write_param->handleValPair.value.val[0]);

    /* Stop the FreeRTOS timer */
    xTimerStop(timer_handle, (TickType_t)0);
    notif_tx_bytes = 0u;
    gatt_write_rx_bytes = 0u;

    /* Write into the identified attribute in GATT database */
    gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle, &(write_param->handleValPair));
    if(gattErrorCode == CY_BLE_GATT_ERR_NONE)
    {
        /* If the attribute is CCCD for notification characteristic */
        if(write_param->handleValPair.attrHandle == CUSTOM_SERVICE_NOTIF_CCCD_ATT_HANDLE)
        {
            /* If notifications are enabled */
            if(write_param->handleValPair.value.val[0])
            {
                /* Custom data value for BLE notification. */
                uint8_t custom_notif_data[CY_BLE_GATT_MTU] = {0};
                tprintf("Notifications Enabled\r\n");
                notify_flag = true;
                /* Set the custom notification data using the MTU size */
                /* Packet length = (ATT_MTU_SIZE - ATT_OPCODE(1 byte) - ATT_HEADER(2 bytes))
                 *               = (ATT_MTU_SIZE - 3)*/
                custom_data.value.len = (att_mtu_size - 3u);
                custom_data.value.val = custom_notif_data;
                custom_data.attrHandle = CUSTOM_SERVICE_NOTIF_ATT_HANDLE;
            }
            /* If notifications are disabled */
            else
            {
                tprintf("Notifications Disabled\r\n");
                notify_flag = false;
                /* Switch OFF LED2 to show notifications stopped */
                led_status.data_led = CYBSP_LED_STATE_OFF;
                xQueueSend(led_cmdQ, &led_status, (TickType_t)0);
            }
            /* Start 1 second timer to calculate throughput */
            xTimerStart(timer_handle, (TickType_t)0);
        }
    }
    /* If the operation is not successful */
    else
    {
        eprintf("GATT Write API, errorcode = 0x%X", gattErrorCode);
    }
}

/**********************************************************************************
* Function Name: void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
***********************************************************************************
* Summary: This function updates the value attribute of GATT Write
*          characteristic present in the custom service
*
* Parameters:
*  cy_stc_ble_gatt_write_param_t *write_param: Parameter which holds information
*                              about attribute handle and value.
* Return:
*  None
**********************************************************************************/
static void update_gatt_db_write(cy_stc_ble_gatt_write_param_t *write_param)
{
    cy_en_ble_gatt_err_code_t gattErrorCode;

    /* Write the attribute value into GATT database of server */
    gattErrorCode = Cy_BLE_GATTS_WriteAttributeValuePeer(&conn_handle, &(write_param->handleValPair));

    /* Check for successful write operation */
    if(gattErrorCode == CY_BLE_GATT_ERR_NONE)
    {
        /* If data is written into value attribute of GATT write characteristic */
        if(write_param->handleValPair.attrHandle == CUSTOM_SERVICE_GATT_WRITE_CHAR_HANDLE)
        {
            /* Copy the data to a variable for further processing in application
             * layer if required */
            memcpy(custom_data.value.val, write_param->handleValPair.value.val, write_param->handleValPair.value.len);
            /* Increment count of gatt write bytes received */
            gatt_write_rx_bytes += write_param->handleValPair.value.len;
        }
    }
    else
    {
        eprintf("GATT Write API, errorcode = 0x%X ", gattErrorCode);
    }
}

/*******************************************************************************
* Function Name: void rtos_timer_cb(TimerHandle_t timer_handle)
********************************************************************************
* Summary: This is a freeRTOS Software timer callback function. It calculates
*          the throughput and displays the value over UART terminal.
*
* Parameters:
*  TimerHandle_t timer_handle: parameter defined during timer creation (unused)
*
* Return:
*  None
*******************************************************************************/
void rtos_timer_cb(TimerHandle_t timer_handle)
{
    /* Avoid warning for unused parameter */
    (void)timer_handle;
    server_throughput.rx = 0u;
    server_throughput.tx = 0u;
    /* Calculate TX and RX throughput
     *
     * throughput(kbps) = (number of bytes sent/received in 1 second) * 8(bits)
     *                    -----------------------------------------------------
     *                                   1 second * 2^10 (kilo)
     */

    if(notif_tx_bytes != 0u)
    {
    server_throughput.tx = (notif_tx_bytes) >> 7u;
    notif_tx_bytes = 0u;
    tprintf("GATT NOTIFICATION: Server Throughput Tx= %lu kbps\r\n", server_throughput.tx);
    }

    if(gatt_write_rx_bytes != 0u)
    {
    server_throughput.rx = (gatt_write_rx_bytes) >> 7u;
    gatt_write_rx_bytes = 0u;
    tprintf("GATT WRITE:        Server Throughput Rx= %lu kbps\r\n", server_throughput.rx);
    }

#ifdef EINK_DISPLAY_SHIELD_PRESENT
    count_five_sec++;
    if(count_five_sec == 5u)
    {
        /* Notify the display task */
        xTaskNotifyGive(display_task_handle);
        count_five_sec = 0u;
    }
#endif
}

/*******************************************************************************
* Function Name: throughput_val_t* get_throughput(void)
********************************************************************************
* Summary: This function returns the pointer to the tx and rx throughput value.
*
* Parameters:
*  None
*
* Return:
*  throughput_val_t* gatt_throughput: BLE gatt throughput calculated in ble task.
*******************************************************************************/
throughput_val_t* get_throughput(void)
{
    return(&server_throughput);
}
/* [] END OF FILE */
