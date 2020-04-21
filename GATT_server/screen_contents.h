/******************************************************************************
* File Name: screen_contents.h
*
* Description: This file is public interface of screen_contents.c source file
*
* Related Document: Readme.md
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
******************************************************************************/

/******************************************************************************
 * Include guard
 *****************************************************************************/
#ifdef EINK_DISPLAY_SHIELD_PRESENT
#ifndef EINK_FILES_SCREEN_CONTENTS_H_
#define EINK_FILES_SCREEN_CONTENTS_H_

/* Header file includes */
#include "cy_eink_library.h"

/* Variables from screen_contents.c that store images and text in flash */
extern cy_eink_image_t const logo[CY_EINK_IMAGE_SIZE];
extern char const Heading[];
extern char const Heading_common[];
extern char const Disconnect_msg[];

#endif /* EINK_FILES_SCREEN_CONTENTS_H_ */
#endif /* EINK_DISPLAY_SHIELD_PRESENT */
/* [] END OF FILE */
