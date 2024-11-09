/******************************************************************************
* File Name:   MotorCtrlHWConfig.h
*
* Description: Motor control hardware configuration header file.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "stdio.h"
#include "General.h"

#ifndef MOTOR_CTRL_HW_CONFIG
#define MOTOR_CTRL_HW_CONFIG

/* Temperature sensor configurations */
#define ACTIVE_TEMP_SENSOR  false        // Active IC (e.g. MCP9700T-E/TT) vs Passive NTC (e.g. NCP18WF104J03RB)
extern  TEMP_SENS_LUT_t     Temp_Sens_LUT;

/* ADC configurations*/
#define ADC_VREF_GAIN        (1.0f)                  // [V/V], voltage-reference buffer gain (e.g. scaling 5.0V down to 3.3V)
#define ADC_CS_OPAMP_GAIN    (12.0f)                 // [V/V]
#define ADC_CS_SHUNT_RES     (10.0E-3f)                // [Ohm], cs shunt-resistor value, default
#define ADC_CS_SETTLE_RATIO  (0.5f)                    // [], settling ratio used for single-shunt current sampling
#define ADC_SCALE_VUVW      ((5.6f)/(56.0f+5.6f))   // [V/V] = [Ohm/Ohm]
#define ADC_SCALE_VDC       ((5.6f)/(56.0f+5.6f))   // [V/V] = [Ohm/Ohm]
enum
{
    // ADC sequence 0 results
    ADC_ISAMPA = 0, ADC_VU = 1, ADC_VBUS = 2,
    // ADC sequence 1 results:
    ADC_ISAMPB = 3, ADC_VV = 4, ADC_VPOT = 5,
    // ADC sequence 2 results:
    ADC_ISAMPC = 6, ADC_VW = 7, ADC_TEMP = 8,
    // Totals
    ADC_SEQ_MAX = 3, ADC_SAMP_PER_SEQ_MAX = 3, ADC_MAX = 9
};
extern void* ADC_Result_Regs[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];
extern uint8_t DMA_Result_Indices[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];
extern cy_stc_dma_descriptor_t* DMA_Descriptors[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];
extern const cy_stc_dma_descriptor_config_t* DMA_Descriptor_Configs[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];

// 3 simultaneous sampling ADCs
void MCU_RoutingConfigMUXA();  // Routing ADCs, ADC0::[ISAMPA,VU,VBUS] & ADC1::[ISAMPB,VV,VPOT] & ADC2::[ISAMPC,VW,TEMP], {ISAMPA,ISAMPB,ISAMPC}={IU,IV,IW}
void MCU_RoutingConfigMUXB();  // Routing ADCs, ADC0::[ISAMPA,VU,VBUS] & ADC1::[ISAMPB,VV,VPOT] & ADC2::[ISAMPC,VW,TEMP], {ISAMPA,ISAMPB,ISAMPC}={I_DC_LINK,I_DC_LINK,-}

// Enable/disable timer reloads
void MCU_EnableTimerReload();
void MCU_DisableTimerReload();

/* PWM configurations*/
#define PWM_INVERSION       true
#define PWM_TRIG_ADVANCE    4U        // [ticks]

/* Miscellaneous BSP definitions */
#define KIT_ID               (0x0008UL)    // For GUI's recognition of HW

#endif
