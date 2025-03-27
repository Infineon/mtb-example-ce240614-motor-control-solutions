/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates the implementation of PMSM sensorless
* field-oriented control (FOC) using the Infineon's MCUs.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "HardwareIface.h"
#include "cybsp.h"
#include "Controller.h"

/*******************************************************************************
* Global variable
********************************************************************************/
/* XMC7x - GCC_ARM: EEPROM storage */
#if defined(APP_KIT_XMC7200_DC_V1)
uint8_t Em_Eeprom_Storage[srss_0_eeprom_0_PHYSICAL_SIZE] __attribute__ ((section(".cy_em_eeprom")));
#endif

/*******************************************************************************
* Function prototype
********************************************************************************/
#if defined(APP_KIT_PSC3M5_2GO)
void Motor_Control_POT_Control(void);
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    
    #if defined(APP_KIT_XMC7200_DC_V1)// Disabled the D-CACHE for XMC7200 device.
    SCB_DisableDCache();
    #endif
    result = cybsp_init();                 /* Initialize the device and board peripherals */
    CY_ASSERT(result == CY_RSLT_SUCCESS);  /* Board init failed. Stop program execution   */

    // Initialize controller
    HW_IFACE_ConnectFcnPointers();         /* must be called before STATE_MACHINE_Init()  */
    STATE_MACHINE_Init();

    // Enable global interrupts
    __enable_irq();

    (void) (result);
    for (;;)
    {
        #if defined(APP_KIT_PSC3M5_2GO)
        Motor_Control_POT_Control();
        #endif
    }
}

#if defined(APP_KIT_PSC3M5_2GO)
/*******************************************************************************
* Function Name: Motor_Control_POT_Control
********************************************************************************
* Summary: This function controls the pot and enable the gate drive.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Motor_Control_POT_Control(void)
{
    static bool flag;
    static float t_min_configured;
    static bool state_check=false;
   /*Disable the driver when pot value is less than or equal to  5% */
    if(params.sys.cmd.source == Internal)
    {
          if((sensor_iface.pot.raw >= 0.05f)) /* flag is added to allow GUI control of driver ON/OFF*/
        {
              if(flag == true)
              {
                flag = false;
                vars.en = true;
              }

        }
          else if(sensor_iface.pot.raw <= 0.025f )
        {
              flag = true;
              vars.en = false;

        }

    }
    else
    {
        flag = false;
    }
    /*Enable or disable Gate driver*/
    Cy_GPIO_Write(EN_IPM_PORT, EN_IPM_NUM, vars.en);
    /* Reduce the minimum time for current measurement in profile mode*/
    if((Prof_Rot_Lock <= sm.current) && (sm.current <= Prof_Lq)&&(params.ctrl.mode == Profiler_Mode))
    {
       if (state_check == false)
       {
          MCU_EnterCriticalSection();
          t_min_configured = params.sys.analog.shunt.hyb_mod.adc_t_min;
          params.sys.analog.shunt.hyb_mod.adc_t_min =  (t_min_configured <=3.0f)? 0.0f:t_min_configured-3.0f;
          params.sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * params.sys.analog.shunt.hyb_mod.adc_t_min * params.sys.samp.fpwm; // [%]
          MCU_ExitCriticalSection();
          state_check =true;
       }
    }
    else
    {
      if(state_check ==true)
      {
        MCU_EnterCriticalSection();
        params.sys.analog.shunt.hyb_mod.adc_t_min =t_min_configured;
        params.sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * params.sys.analog.shunt.hyb_mod.adc_t_min * params.sys.samp.fpwm; // [%]
        MCU_ExitCriticalSection();
      }
      state_check =false;
    }
}
#endif
