/******************************************************************************
* File Name:   HWConfigPsocC3.c
*
* Description: Motor control hardware configuration file for PSOC Control C3.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <Configuration/HWConfigPsocC3.h>

#if defined(COMPONENT_CAT1B)

TEMP_SENS_LUT_t   Temp_Sens_LUT   =
{
    .step = 1.0f / (TEMP_SENS_LUT_WIDTH + 1.0f),    // [%], normalized voltage wrt Vcc
    .step_inv = (TEMP_SENS_LUT_WIDTH + 1.0f),       // [1/%], inverse normalized voltage
    .val = {109.5f, 85.4f, 71.7f, 62.0f, 54.3f, 47.7f, 41.9f, 36.5f, 31.4f, 26.3f, 21.2f, 16.0f, 10.2f, 3.7f, -4.3f, -16.1f} // [degree C]
};

#define ADC_RESULT_ADDR(channel)    ((void*)CY_HPPASS_SAR_CHAN_RSLT_PTR(channel))

static void* const ADC_Result_Regs_MUXA[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{ADC_RESULT_ADDR(0), ADC_RESULT_ADDR(2), ADC_RESULT_ADDR(8), ADC_RESULT_ADDR(10), ADC_RESULT_ADDR(12)},
         {ADC_RESULT_ADDR(1), ADC_RESULT_ADDR(4), ADC_RESULT_ADDR(9), ADC_RESULT_ADDR(11), ADC_RESULT_ADDR(20)}};

static const uint8_t DMA_Result_Indices_MUXA[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{ADC_ISAMPA, ADC_ISAMPC, ADC_VU, ADC_VW,     ADC_VPOT},
         {ADC_ISAMPB, ADC_VBUS,   ADC_VV, ADC_ISAMPD, ADC_TEMP}};

static void* const ADC_Result_Regs_MUXB[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{ADC_RESULT_ADDR(3), ADC_RESULT_ADDR(4), ADC_RESULT_ADDR(9),  ADC_RESULT_ADDR(11), ADC_RESULT_ADDR(12)},
         {ADC_RESULT_ADDR(3), ADC_RESULT_ADDR(8), ADC_RESULT_ADDR(10), ADC_RESULT_ADDR(11), ADC_RESULT_ADDR(20)}};

static const uint8_t DMA_Result_Indices_MUXB[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{ADC_ISAMPA, ADC_VBUS, ADC_VV, ADC_ISAMPC, ADC_VPOT},
         {ADC_ISAMPB, ADC_VU,   ADC_VW, ADC_ISAMPD, ADC_TEMP}};

void* ADC_Result_Regs[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];
uint8_t DMA_Result_Indices[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX];

cy_stc_dma_descriptor_t* DMA_Descriptors[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{&DMA_ADC_0_Descriptor_0, &DMA_ADC_0_Descriptor_1, &DMA_ADC_0_Descriptor_2, &DMA_ADC_0_Descriptor_3, &DMA_ADC_0_Descriptor_4},
         {&DMA_ADC_1_Descriptor_0, &DMA_ADC_1_Descriptor_1, &DMA_ADC_1_Descriptor_2, &DMA_ADC_1_Descriptor_3, &DMA_ADC_1_Descriptor_4}};

const cy_stc_dma_descriptor_config_t* DMA_Descriptor_Configs[ADC_SEQ_MAX][ADC_SAMP_PER_SEQ_MAX] = \
        {{&DMA_ADC_0_Descriptor_0_config, &DMA_ADC_0_Descriptor_1_config, &DMA_ADC_0_Descriptor_2_config, &DMA_ADC_0_Descriptor_3_config, &DMA_ADC_0_Descriptor_4_config},
         {&DMA_ADC_1_Descriptor_0_config, &DMA_ADC_1_Descriptor_1_config, &DMA_ADC_1_Descriptor_2_config, &DMA_ADC_1_Descriptor_3_config, &DMA_ADC_1_Descriptor_4_config}};

void MCU_RoutingConfigMUXA()
{
    const cy_stc_hppass_sar_grp_t ADC_SEQ0_Config =
    {
        .dirSampMsk = 0x505U,
        .muxSampMsk = 0x1U,
        .muxChanIdx = {0U,0U,0U,0U},
        .trig = CY_HPPASS_SAR_TRIG_0,
        .sampTime = CY_HPPASS_SAR_SAMP_TIME_0,
        .priority = true,
        .continuous = false,
    };
    const cy_stc_hppass_sar_grp_t ADC_SEQ1_Config =
    {
        .dirSampMsk = 0xA12U,
        .muxSampMsk = 0x4U,
        .muxChanIdx = {0U,0U,0U,0U},
        .trig = CY_HPPASS_SAR_TRIG_1,
        .sampTime = CY_HPPASS_SAR_SAMP_TIME_0,
        .priority = false,
        .continuous = false,
    };
    Cy_HPPASS_SAR_GroupConfig(0U, &ADC_SEQ0_Config);
    Cy_HPPASS_SAR_GroupConfig(1U, &ADC_SEQ1_Config);

    Cy_HPPASS_AC_Start(0U, 1000U);

    for (uint8_t seq_idx=0U; seq_idx<ADC_SEQ_MAX; ++seq_idx)
    {
        for (uint8_t samp_idx=0U; samp_idx<ADC_SAMP_PER_SEQ_MAX; ++samp_idx)
        {
            ADC_Result_Regs[seq_idx][samp_idx] = (void*)ADC_Result_Regs_MUXA[seq_idx][samp_idx];
            DMA_Result_Indices[seq_idx][samp_idx] = (uint8_t)DMA_Result_Indices_MUXA[seq_idx][samp_idx];
        }
    }
}

void MCU_RoutingConfigMUXB()
{
    const cy_stc_hppass_sar_grp_t ADC_SEQ0_Config =
    {
        .dirSampMsk = 0xA18U,    // MUXA: 0x505U
        .muxSampMsk = 0x1U,
        .muxChanIdx = {0U,0U,0U,0U},
        .trig = CY_HPPASS_SAR_TRIG_0,
        .sampTime = CY_HPPASS_SAR_SAMP_TIME_0,
        .priority = true,
        .continuous = false,
    };
    const cy_stc_hppass_sar_grp_t ADC_SEQ1_Config =
    {
        .dirSampMsk = 0xD08U,    // MUXA: 0xA12U
        .muxSampMsk = 0x4U,
        .muxChanIdx = {0U,0U,0U,0U},
        .trig = CY_HPPASS_SAR_TRIG_1,
        .sampTime = CY_HPPASS_SAR_SAMP_TIME_0,
        .priority = false,
        .continuous = false,
    };
    Cy_HPPASS_SAR_GroupConfig(0U, &ADC_SEQ0_Config);
    Cy_HPPASS_SAR_GroupConfig(1U, &ADC_SEQ1_Config);

    Cy_HPPASS_AC_Start(0,1000U);

    for (uint8_t seq_idx=0U; seq_idx<ADC_SEQ_MAX; ++seq_idx)
    {
        for (uint8_t samp_idx=0U; samp_idx<ADC_SAMP_PER_SEQ_MAX; ++samp_idx)
        {
            ADC_Result_Regs[seq_idx][samp_idx] = (void*)ADC_Result_Regs_MUXB[seq_idx][samp_idx];
            DMA_Result_Indices[seq_idx][samp_idx] = (uint8_t)DMA_Result_Indices_MUXB[seq_idx][samp_idx];
        }
    }
}

void MCU_DisableTimerReload()
{
    TCPWM_GRP_CNT_TR_OUT_SEL(SYNC_ISR1_HW, TCPWM_GRP_CNT_GET_GRP(SYNC_ISR1_NUM), SYNC_ISR1_NUM) =
      (_VAL2FLD(TCPWM_GRP_CNT_V2_TR_OUT_SEL_OUT0, CY_TCPWM_CNT_TRIGGER_ON_DISABLED) |
       _VAL2FLD(TCPWM_GRP_CNT_V2_TR_OUT_SEL_OUT1, CY_TCPWM_CNT_TRIGGER_ON_DISABLED));
}

void MCU_EnableTimerReload()
{
    TCPWM_GRP_CNT_TR_OUT_SEL(SYNC_ISR1_HW, TCPWM_GRP_CNT_GET_GRP(SYNC_ISR1_NUM), SYNC_ISR1_NUM) =
      (_VAL2FLD(TCPWM_GRP_CNT_V2_TR_OUT_SEL_OUT0, CY_TCPWM_CNT_TRIGGER_ON_DISABLED) |
       _VAL2FLD(TCPWM_GRP_CNT_V2_TR_OUT_SEL_OUT1, CY_TCPWM_CNT_TRIGGER_ON_OVERFLOW));
}

#endif
