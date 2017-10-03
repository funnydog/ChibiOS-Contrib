/*
 * Copyright (C) 2014-2016 Fabio Utzig, http://fabioutzig.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _K64_H_
#define _K64_H_

/*
 * ==============================================================
 * ---------- Interrupt Number Definition -----------------------
 * ==============================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ****************/
  InitialSP_IRQn                = -15,
  InitialPC_IRQn                = -15,
  NonMaskableInt_IRQn           = -14,
  HardFault_IRQn                = -13,
  MemoryManagement_IRQn         = -12,
  BusFault_IRQn                 = -11,
  UsageFault_IRQn               = -10,
  SVCall_IRQn                   = -5,
  DebugMonitor_IRQn             = -4,
  PendSV_IRQn                   = -2,
  SysTick_IRQn                  = -1,

/******  K20x Specific Interrupt Numbers ***********************/
  DMA0_IRQn                     = 0,  // Vector40
  DMA1_IRQn                     = 1,  // Vector44
  DMA2_IRQn                     = 2,  // Vector48
  DMA3_IRQn                     = 3,  // Vector4C
  DMA4_IRQn                     = 4,  // Vector50
  DMA5_IRQn                     = 5,  // Vector54
  DMA6_IRQn                     = 6,  // Vector58
  DMA7_IRQn                     = 7,  // Vector5C
  DMA8_IRQn                     = 8,  // Vector60
  DMA9_IRQn                     = 9,  // Vector64
  DMA10_IRQn                    = 10, // Vector68
  DMA11_IRQn                    = 11, // Vector6C
  DMA12_IRQn                    = 12, // Vector70
  DMA13_IRQn                    = 13, // Vector74
  DMA14_IRQn                    = 14, // Vector78
  DMA15_IRQn                    = 15, // Vector7C
  DMAError_IRQn                 = 16, // Vector80
  MCM_IRQn                      = 17, // Vector84
  FlashMemComplete_IRQn         = 18, // Vector88
  FlashMemReadCollision_IRQn    = 19, // Vector8C
  LowVoltageWarning_IRQn        = 20, // Vector90
  LLWU_IRQn                     = 21, // Vector94
  WDOG_IRQn                     = 22, // Vector98
  RNG_IRQn                      = 23, // Vector9C
  I2C0_IRQn                     = 24, // VectorA0
  I2C1_IRQn                     = 25, // VectorA4
  SPI0_IRQn                     = 26, // VectorA8
  SPI1_IRQn                     = 27, // VectorAC
  I2S0Tx_IRQn                   = 28, // VectorB0
  I2S0Rx_IRQn                   = 29, // VectorB4
  // reserved                   = 30, // VectorB8
  UART0Status_IRQn              = 31, // VectorBC
  UART0Error_IRQn               = 32, // VectorC0
  UART1Status_IRQn              = 33, // VectorC4
  UART1Error_IRQn               = 34, // VectorC8
  UART2Status_IRQn              = 35, // VectorCC
  UART2Error_IRQn               = 36, // VectorD0
  UART3Status_IRQn              = 37, // VectorD4
  UART3Error_IRQn               = 38, // VectorD8
  ADC0_IRQn                     = 39, // VectorDC
  CMP0_IRQn                     = 40, // VectorE0
  CMP1_IRQn                     = 41, // VectorE4
  FTM0_IRQn                     = 42, // VectorE8
  FTM1_IRQn                     = 43, // VectorEC
  FTM2_IRQn                     = 44, // VectorF0
  CMT_IRQn                      = 45, // VectorF4
  RTCAlarm_IRQn                 = 46, // VectorF8
  RTCSeconds_IRQn               = 47, // VectorFC
  PITChannel0_IRQn              = 48, // Vector100
  PITChannel1_IRQn              = 49, // Vector104
  PITChannel2_IRQn              = 50, // Vector108
  PITChannel3_IRQn              = 51, // Vector10C
  PDB_IRQn                      = 52, // Vector110
  USB_OTG_IRQn                  = 53, // Vector114
  USBChargerDetect_IRQn         = 54, // Vector118
  // reserved                   = 55, // Vector11C
  DAC0_IRQn                     = 56, // Vector120
  MCG_IRQn                      = 57, // Vector124
  LPTMR0_IRQn                   = 58, // Vector128
  PINA_IRQn                     = 59, // Vector12C
  PINB_IRQn                     = 60, // Vector130
  PINC_IRQn                     = 61, // Vector134
  PIND_IRQn                     = 62, // Vector138
  PINE_IRQn                     = 63, // Vector13C
  SoftInitInt_IRQn              = 64, // Vector140
  SPI2_IRQn                     = 65, // Vector144
  UART4Status_IRQn              = 66, // Vector148
  UART4Error_IRQn               = 67, // Vector14C
  UART5Status_IRQn              = 68, // Vector150
  UART5Error_IRQn               = 69, // Vector154
  CMP2_IRQn                     = 70, // Vector158
  FTM3_IRQn                     = 71, // Vector15C
  DAC1_IRQn                     = 72, // Vector160
  ADC1_IRQn                     = 73, // Vector164
  I2C2_IRQn                     = 74, // Vector168
  CANMessage_IRQn               = 75, // Vector16C
  CANBusOff                     = 76, // Vector170
  CANError                      = 77, // Vector174
  CANTxWarning                  = 78, // Vector178
  CANRxWarning                  = 79, // Vector17C
  CANWakeUp                     = 80, // Vector180
  SDHC_IRQn                     = 81, // Vector184
  ENET_1588_Timer_IRQn          = 82, // Vector188
  ENET_Transmit_IRQn            = 83, // Vector18C
  ENET_Receive_IRQn             = 84, // Vector190
  ENET_Error_IRQn               = 85, // Vector194
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/**
 * @brief K64 Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
#define __FPU_PRESENT             1
#define __MPU_PRESENT             0
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    0

#include "core_cm4.h"            /* Cortex-M4 processor and core peripherals */

/****************************************************************/
/*                                                              */
/*           Analog-to-Digital Converter (ADC)                  */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t SC1A;
  __IO uint32_t SC1B;
  __IO uint32_t CFG1;
  __IO uint32_t CFG2;
  __I  uint32_t RA;
  __I  uint32_t RB;
  __IO uint32_t CV1;
  __IO uint32_t CV2;
  __IO uint32_t SC2;
  __IO uint32_t SC3;
  __IO uint32_t OFS;
  __IO uint32_t PG;
  __IO uint32_t MG;
  __IO uint32_t CLPD;
  __IO uint32_t CLPS;
  __IO uint32_t CLP4;
  __IO uint32_t CLP3;
  __IO uint32_t CLP2;
  __IO uint32_t CLP1;
  __IO uint32_t CLP0;
       uint32_t RESERVED0;
  __IO uint32_t CLMD;
  __IO uint32_t CLMS;
  __IO uint32_t CLM4;
  __IO uint32_t CLM3;
  __IO uint32_t CLM2;
  __IO uint32_t CLM1;
  __IO uint32_t CLM0;
} ADC_TypeDef;

#define ADCx_SC1n_COCO              ((uint32_t)0x00000080)
#define ADCx_SC1n_AIEN              ((uint32_t)0x00000040)
#define ADCx_SC1n_DIFF              ((uint32_t)0x00000020)
#define ADCx_SC1n_ADCH_SHIFT        0
#define ADCx_SC1n_ADCH_MASK         ((uint32_t)0x0000001F)
#define ADCx_SC1n_ADCH(x)           ((uint32_t)(((uint32_t)(x)<<ADCx_SC1n_ADCH_SHIFT)&ADCx_SC1n_ADCH_MASK))

#define ADCx_CFG1_ADLPC             ((uint32_t)0x00000080)
#define ADCx_CFG1_ADIV_SHIFT        5
#define ADCx_CFG1_ADIV_MASK         ((uint32_t)0x00000060)
#define ADCx_CFG1_ADIV(x)           ((uint32_t)(((uint32_t)(x)<<ADCx_CFG1_ADIV_SHIFT)&ADCx_CFG1_ADIV_MASK))
#define ADCx_CFG1_ADLSMP            ((uint32_t)0x00000010)
#define ADCx_CFG1_MODE_SHIFT        2
#define ADCx_CFG1_MODE_MASK         ((uint32_t)0x0000000C)
#define ADCx_CFG1_MODE(x)           ((uint32_t)(((uint32_t)(x)<<ADCx_CFG1_MODE_SHIFT)&ADCx_CFG1_MODE_MASK))
#define ADCx_CFG1_ADICLK_SHIFT      0
#define ADCx_CFG1_ADICLK_MASK       ((uint32_t)0x00000003)
#define ADCx_CFG1_ADICLK(x)         ((uint32_t)(((uint32_t)(x)<<ADCx_CFG1_ADICLK_SHIFT)&ADCx_CFG1_ADICLK_MASK))

#define ADCx_CFG2_MUXSEL            ((uint32_t)0x00000010)
#define ADCx_CFG2_ADACKEN           ((uint32_t)0x00000008)
#define ADCx_CFG2_ADHSC             ((uint32_t)0x00000004)
#define ADCx_CFG2_ADLSTS_SHIFT      0
#define ADCx_CFG2_ADLSTS_MASK       ((uint32_t)0x00000003)
#define ADCx_CFG2_ADLSTS(x)         ((uint32_t)(((uint32_t)(x)<<ADCx_CFG2_ADLSTS_SHIFT)&ADCx_CFG2_ADLSTS_MASK))

#define ADCx_SC2_ADACT              ((uint32_t)0x00000080)
#define ADCx_SC2_ADTRG              ((uint32_t)0x00000040)
#define ADCx_SC2_ACFE               ((uint32_t)0x00000020)
#define ADCx_SC2_ACFGT              ((uint32_t)0x00000010)
#define ADCx_SC2_ACREN              ((uint32_t)0x00000008)
#define ADCx_SC2_DMAEN              ((uint32_t)0x00000004)
#define ADCx_SC2_REFSEL_SHIFT       0
#define ADCx_SC2_REFSEL_MASK        ((uint32_t)0x00000003)
#define ADCx_SC2_REFSEL(x)          ((uint32_t)(((uint32_t)(x)<<ADCx_SC2_REFSEL_SHIFT)&ADCx_SC2_REFSEL_MASK))

#define ADCx_SC3_CAL                ((uint32_t)0x00000080)
#define ADCx_SC3_CALF               ((uint32_t)0x00000040)
#define ADCx_SC3_ADCO               ((uint32_t)0x00000008)
#define ADCx_SC3_AVGE               ((uint32_t)0x00000004)
#define ADCx_SC3_AVGS_SHIFT         0
#define ADCx_SC3_AVGS_MASK          ((uint32_t)0x00000003)
#define ADCx_SC3_AVGS(x)            ((uint32_t)(((uint32_t)(x)<<ADCx_SC3_AVGS_SHIFT)&ADCx_SC3_AVGS_MASK))

/****************************************************************/
/*                                                              */
/*                       FlexCAN (CAN)                          */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t MCR;
  __IO uint32_t CTRL1;
  __IO uint32_t TIMER;
       uint32_t RESERVED0;
  __IO uint32_t RXMGMASK;
  __IO uint32_t RX14MASK;
  __IO uint32_t RX15MASK;
  __IO uint32_t ECR;
  __IO uint32_t ESR1;
       uint32_t RESERVED1;
  __IO uint32_t IMASK1;
       uint32_t RESERVED2;
  __IO uint32_t IFLAG1;
  __IO uint32_t CTRL2;
  __I  uint32_t ESR2;
       uint32_t RESERVED3[2];
  __I  uint32_t CRCR;
  __IO uint32_t RXFGMASK;
  __I  uint32_t RXFIR;
       uint32_t RESERVED4[12];
  struct {
    __IO uint32_t CS;
    __IO uint32_t ID;
    __IO uint32_t WORD0;
    __IO uint32_t WORD1;
  } MB[16];
       uint32_t RESERVED5[448];
  __IO uint32_t RXIMR[16];
} CAN_TypeDef;

#define CANx_MCR_MDIS               ((uint32_t)0x80000000)
#define CANx_MCR_FRZ                ((uint32_t)0x40000000)
#define CANx_MCR_RFEN               ((uint32_t)0x20000000)
#define CANx_MCR_HALT               ((uint32_t)0x10000000)
#define CANx_MCR_NOTRDY             ((uint32_t)0x08000000)
#define CANx_MCR_WAKMSK             ((uint32_t)0x04000000)
#define CANx_MCR_SOFTRST            ((uint32_t)0x02000000)
#define CANx_MCR_FRZACK             ((uint32_t)0x01000000)
#define CANx_MCR_SUPV               ((uint32_t)0x00800000)
#define CANx_MCR_SLFWAK             ((uint32_t)0x00400000)
#define CANx_MCR_WRNEN              ((uint32_t)0x00200000)
#define CANx_MCR_LPMACK             ((uint32_t)0x00100000)
#define CANx_MCR_WAKSRC             ((uint32_t)0x00080000)
#define CANx_MCR_SRXDIS             ((uint32_t)0x00020000)
#define CANx_MCR_IRMQ               ((uint32_t)0x00010000)
#define CANx_MCR_LPRIOEN            ((uint32_t)0x00002000)
#define CANx_MCR_AEN                ((uint32_t)0x00001000)
#define CANx_MCR_IDAM_SHIFT         8
#define CANx_MCR_IDAM_MASK          ((uint32_t)0x0x000300)
#define CANx_MCR_IDAM(x)            ((uint32_t)(((uint32_t)(x)<<CANx_MCR_IDAM_SHIFT)&CANx_MCR_IDAM_MASK))
#define CANx_MCR_MAXMB_SHIFT        0
#define CANx_MCR_MAXMB_MASK         ((uint32_t)0x0000007F)
#define CANx_MCR_MAXMB(x)           ((uint32_t)(((uint32_t)(x)<<CANx_MCR_MAXMB_SHIFT)&CANx_MCR_MAXMB_MASK))

#define CANx_CTRL1_PRESDIV_SHIFT    24
#define CANx_CTRL1_PRESDIV_MASK     ((uint32_t)0xFF000000)
#define CANx_CTRL1_PRESDIV(x)       ((uint32_t)(((uint32_t)(x)<<CANx_CTRL1_PRESDIV_SHIFT)&CANx_CTRL1_PRESDIV_MASK))
#define CANx_CTRL1_RJW_SHIFT        22
#define CANx_CTRL1_RJW_MASK         ((uint32_t)0x00C00000)
#define CANx_CTRL1_RJW(x)           ((uint32_t)(((uint32_t)(x)<<CANx_CTRL1_RJW_SHIFT)&CANx_CTRL1_RJW_MASK))
#define CANx_CTRL1_PSEG1_SHIFT      19
#define CANx_CTRL1_PSEG1_MASK       ((uint32_t)0x00380000)
#define CANx_CTRL1_PSEG1(x)         ((uint32_t)(((uint32_t)(x)<<CANx_CTRL1_PSEG1_SHIFT)&CANx_CTRL1_PSEG1_MASK))
#define CANx_CTRL1_PSEG2_SHIFT      16
#define CANx_CTRL1_PSEG2_MASK       ((uint32_t)0x00070000)
#define CANx_CTRL1_PSEG2(x)         ((uint32_t)(((uint32_t)(x)<<CANx_CTRL1_PSEG2_SHIFT))&CANx_CTRL1_PSEG2_MASK))
#define CANx_CTRL1_BOFFMSK          ((uint32_t)0x00008000)
#define CANx_CTRL1_ERRMSK           ((uint32_t)0x00004000)
#define CANx_CTRL1_CLKSRC           ((uint32_t)0x00002000)
#define CANx_CTRL1_LPB              ((uint32_t)0x00001000)
#define CANx_CTRL1_TWRNMSK          ((uint32_t)0x00000800)
#define CANx_CTRL1_RWRNMSK          ((uint32_t)0x00000400)
#define CANx_CTRL1_SMP              ((uint32_t)0x00000080)
#define CANx_CTRL1_BOFFREC          ((uint32_t)0x00000040)
#define CANx_CTRL1_TSYN             ((uint32_t)0x00000020)
#define CANx_CTRL1_LBUF             ((uint32_t)0x00000010)
#define CANx_CTRL1_LOM              ((uint32_t)0x00000008)
#define CANx_CTRL1_PROPSEG(x)       ((uint32_t)(((uint32_t)(x)<<CANx_CTRL1_PROPSEG_SHIFT)&CANx_CTRL1_PROPSEG_MASK))
#define CANx_CTRL1_PROPSEG_SHIFT    0
#define CANx_CTRL1_PROPSEG_MASK     ((uint32_t)0x00000007)

#define CANx_TIMER_TIMER_SHIFT      0
#define CANx_TIMER_TIMER_MASK       ((uint32_t)0x0000FFFF)
#define CANx_TIMER_TIMER(x)         ((uint32_t)(((uint32_t)(x)<<CANx_TIMER_TIMER_SHIFT)&CANx_TIMER_TIMER_MASK))

#define CANx_ECR_RXERRCNT_SHIFT     8
#define CANx_ECR_RXERRCNT_MASK      ((uint32_t)0x0000FF00)
#define CANx_ECR_RXERRCNT(x)        ((uint32_t)(((uint32_t)(x)<<CANx_ECR_RXERRCNT_SHIFT)&CANx_ECR_RXERRCNT_MASK))
#define CANx_ECR_TXERRCNT_SHIFT     0
#define CANx_ECR_TXERRCNT_MASK      ((uint32_t)0x000000FF)
#define CANx_ECR_TXERRCNT(x)        ((uint32_t)(((uint32_t)(x)<<CANx_ECR_TXERRCNT_SHIFT)&CANx_ECR_TXERRCNT_MASK))

#define CANx_ESR1_SYNCH             ((uint32_t)0x00040000)
#define CANx_ESR1_TWRNINT           ((uint32_t)0x00020000)
#define CANx_ESR1_RWRNINT           ((uint32_t)0x00010000)
#define CANx_ESR1_BIT1ERR           ((uint32_t)0x00008000)
#define CANx_ESR1_BIT0ERR           ((uint32_t)0x00004000)
#define CANx_ESR1_ACKERR            ((uint32_t)0x00002000)
#define CANx_ESR1_CRCERR            ((uint32_t)0x00001000)
#define CANx_ESR1_FRMERR            ((uint32_t)0x00000800)
#define CANx_ESR1_STFERR            ((uint32_t)0x00000400)
#define CANx_ESR1_TXWRN             ((uint32_t)0x00000200)
#define CANx_ESR1_RXWRN             ((uint32_t)0x00000100)
#define CANx_ESR1_IDLE              ((uint32_t)0x00000080)
#define CANx_ESR1_TX                ((uint32_t)0x00000040)
#define CANx_ESR1_FLTCONF_SHIFT     4
#define CANx_ESR1_FLTCONF_MASK      ((uint32_t)0x00000030)
#define CANx_ESR1_FLTCONF(x)        ((uint32_t)(((uint32_t)(x)<<CANx_ESR1_FLTCONF_SHIFT)&CANx_ESR1_FLTCONF_MASK))
#define CANx_ESR1_RX                ((uint32_t)0x00000008)
#define CANx_ESR1_BOFFINT           ((uint32_t)0x00000004)
#define CANx_ESR1_ERRINT            ((uint32_t)0x00000002)
#define CANx_ESR1_WAKINT            ((uint32_t)0x00000001)

#define CANx_IFLAG1_BUF31TO8I_SHIFT 8
#define CANx_IFLAG1_BUF31TO8I_MASK  ((uint32_t)0xFFFFFF00)
#define CANx_IFLAG1_BUF31TO8I(x)    ((uint32_t)(((uint32_t)(x)<<CANx_IFLAG1_BUF31TO8I_SHIFT)&CANx_IFLAG1_BUF31TO8I_MASK))
#define CANx_IFLAG1_BUF7I           ((uint32_t)0x00000080)
#define CANx_IFLAG1_BUF6I           ((uint32_t)0x00000040)
#define CANx_IFLAG1_BUF5I           ((uint32_t)0x00000020)
#define CANx_IFLAG1_BUF4TO1I_SHIFT  1
#define CANx_IFLAG1_BUF4TO1I_MASK   ((uint32_t)0x0000001E)
#define CANx_IFLAG1_BUF4TO1I(x)     ((uint32_t)(((uint32_t)(x)<<CANx_IFLAG1_BUF4TO1I_SHIFT)&CANx_IFLAG1_BUF4TO1I_MASK))
#define CANx_IFLAG1_BUF0I           ((uint32_t)0x00000001)

#define CANx_CTRL2_WRMFRZ           ((uint32_t)0x10000000)
#define CANx_CTRL2_RFFN_SHIFT       24
#define CANx_CTRL2_RFFN_MASK        ((uint32_t)0x0F000000)
#define CANx_CTRL2_RFFN(x)          ((uint32_t)(((uint32_t)(x)<<CANx_CTRL2_RFFN_SHIFT)&CANx_CTRL2_RFFN_MASK))
#define CANx_CTRL2_TASD_SHIFT       19
#define CANx_CTRL2_TASD_MASK        ((uint32_t)0x00F80000)
#define CANx_CTRL2_TASD(x)          ((uint32_t)(((uint32_t)(x)<<CANx_CTRL2_TASD_SHIFT)&CANx_CTRL2_TASD_MASK))
#define CANx_CTRL2_MRP              ((uint32_t)0x00040000)
#define CANx_CTRL2_RRS              ((uint32_t)0x00020000)
#define CANx_CTRL2_EACEN            ((uint32_t)0x00010000)

#define CANx_ESR2_LPTM_SHIFT        16
#define CANx_ESR2_LPTM_MASK         ((uint32_t)0x007F0000)
#define CANx_ESR2_LPTM(x)           ((uint32_t)(((uint32_t)(x)<<CANx_ESR2_LPTM_SHIFT)&CANx_ESR2_LPTM_MASK))
#define CANx_ESR2_VPS               ((uint32_t)0x00004000)
#define CANx_ESR2_IMB               ((uint32_t)0x00002000)

#define CANx_CRCR_MBCRC_SHIFT       16
#define CANx_CRCR_MBCRC_MASK        ((uint32_t)0x007F0000)
#define CANx_CRCR_MBCRC(x)          ((uint32_t)(((uint32_t)(x)<<CANx_CRCR_MBCRC_SHIFT)&CANx_CRCR_MBCRC_MASK))
#define CANx_CRCR_TXCRC_SHIFT       0
#define CANx_CRCR_TXCRC_MASK        ((uint32_t)0x00007FFF)
#define CANx_CRCR_TXCRC(x)          ((uint32_t)(((uint32_t)(x)<<CANx_CRCR_TXCRC_SHIFT)&CANx_CRCR_TXCRC_MASK))

#define CANx_RXFIR_IDHIT_MASK       ((uint32_t)0x000001FF)
#define CANx_RXFIR_IDHIT(x)         ((uint32_t)(uint32_t)(x)&CANx_RXFIR_IDHIT_MASK))

#define CANx_CS_CODE_SHIFT          24
#define CANx_CS_CODE_MASK           ((uint32_t)0x0F000000)
#define CANx_CS_CODE(x)             ((uint32_t)(((uint32_t)(x)<<CANx_CS_CODE_SHIFT)&CANx_CS_CODE_MASK))
#define CANx_CS_SRR                 ((uint32_t)0x00400000)
#define CANx_CS_IDE                 ((uint32_t)0x00200000)
#define CANx_CS_RTR                 ((uint32_t)0x00100000)
#define CANx_CS_DLC_SHIFT           16
#define CANx_CS_DLC_MASK            ((uint32_t)0x000F0000)
#define CANx_CS_DLC(x)              ((uint32_t)(((uint32_t)(x)<<CANx_CS_DLC_SHIFT)&CANx_CS_DLC_MASK))
#define CANx_CS_TIME_STAMP_SHIFT    0
#define CANx_CS_TIME_STAMP_MASK     ((uint32_t)0x0000FFFF)
#define CANx_CS_TIME_STAMP(x)       ((uint32_t)(((uint32_t)(x)<<CANx_CS_TIME_STAMP_SHIFT)&CANx_CS_TIME_STAMP_MASK))

#define CANx_ID_PRIO_SHIFT          29
#define CANx_ID_PRIO_MASK           ((uint32_t)0xE0000000)
#define CANx_ID_PRIO(x)             ((uint32_t)(((uint32_t)(x)<<CANx_ID_PRIO_SHIFT)&CANx_ID_PRIO_MASK))
#define CANx_ID_STD_SHIFT           18
#define CANx_ID_STD_MASK            ((uint32_t)0x1FFC0000)
#define CANx_ID_STD(x)              ((uint32_t)(((uint32_t)(x)<<CANx_ID_STD_SHIFT)&CANx_ID_STD_MASK))
#define CANx_ID_EXT_SHIFT           0
#define CANx_ID_EXT_MASK            ((uint32_t)0x0003FFFF)
#define CANx_ID_EXT(x)              ((uint32_t)(((uint32_t)(x)<<CANx_ID_EXT_SHIFT)&CANx_ID_EXT_MASK))

#define CANx_WORD0_DATA_BYTE_0_SHIFT 24
#define CANx_WORD0_DATA_BYTE_0_MASK  ((uint32_t)0xFF000000)
#define CANx_WORD0_DATA_BYTE_0(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD0_DATA_BYTE_0_SHIFT)&CANx_WORD0_DATA_BYTE_0_MASK))
#define CANx_WORD0_DATA_BYTE_1_SHIFT 16
#define CANx_WORD0_DATA_BYTE_1_MASK  ((uint32_t)0x00FF0000)
#define CANx_WORD0_DATA_BYTE_1(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD0_DATA_BYTE_1_SHIFT)&CANx_WORD0_DATA_BYTE_1_MASK))
#define CANx_WORD0_DATA_BYTE_2_SHIFT 8
#define CANx_WORD0_DATA_BYTE_2_MASK  ((uint32_t)0x0000FF00)
#define CANx_WORD0_DATA_BYTE_2(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD0_DATA_BYTE_2_SHIFT)&CANx_WORD0_DATA_BYTE_2_MASK))
#define CANx_WORD0_DATA_BYTE_3_SHIFT 0
#define CANx_WORD0_DATA_BYTE_3_MASK  ((uint32_t)0x000000FF)
#define CANx_WORD0_DATA_BYTE_3(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD0_DATA_BYTE_3_SHIFT)&CANx_WORD0_DATA_BYTE_3_MASK))

#define CANx_WORD1_DATA_BYTE_4_SHIFT 24
#define CANx_WORD1_DATA_BYTE_4_MASK  ((uint32_t)0xFF000000)
#define CANx_WORD1_DATA_BYTE_4(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD1_DATA_BYTE_4_SHIFT)&CANx_WORD1_DATA_BYTE_4_MASK))
#define CANx_WORD1_DATA_BYTE_5_SHIFT 16
#define CANx_WORD1_DATA_BYTE_5_MASK  ((uint32_t)0x00FF0000)
#define CANx_WORD1_DATA_BYTE_5(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD1_DATA_BYTE_5_SHIFT)&CANx_WORD1_DATA_BYTE_5_MASK))
#define CANx_WORD1_DATA_BYTE_6_SHIFT 8
#define CANx_WORD1_DATA_BYTE_6_MASK  ((uint32_t)0x0000FF00)
#define CANx_WORD1_DATA_BYTE_6(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD1_DATA_BYTE_6_SHIFT)&CANx_WORD1_DATA_BYTE_6_MASK))
#define CANx_WORD1_DATA_BYTE_7_SHIFT 0
#define CANx_WORD1_DATA_BYTE_7_MASK  ((uint32_t)0x000000FF)
#define CANx_WORD1_DATA_BYTE_7(x)    ((uint32_t)(((uint32_t)(x)<<CANx_WORD1_DATA_BYTE_7_SHIFT)&CANx_WORD1_DATA_BYTE_7_MASK))

/****************************************************************/
/*                                                              */
/*           Cryptographic Acceleration Unit (CAU)              */
/*                                                              */
/****************************************************************/
typedef struct
{
  __O  uint32_t DIRECT[16];
       uint32_t RESERVED0[512];
  __O  uint32_t LDR_CASR;
  __O  uint32_t LDR_CAA;
  __O  uint32_t LDR_CA[9];
       uint32_t RESERVED1[5];
  __I  uint32_t STR_CASR;
  __I  uint32_t STR_CAA;
  __I  uint32_t STR_CA[9];
       uint32_t RESERVED2[5];
  __O  uint32_t ADR_CASR;
  __O  uint32_t ADR_CAA;
  __O  uint32_t ADR_CA[9];
       uint32_t RESERVED3[5];
  __O  uint32_t RADR_CASR;
  __O  uint32_t RADR_CAA;
  __O  uint32_t RADR_CA[9];
       uint32_t RESERVED4[21];
  __O  uint32_t XOR_CASR;
  __O  uint32_t XOR_CAA;
  __O  uint32_t XOR_CA[9];
       uint32_t RESERVED5[5];
  __O  uint32_t ROTL_CASR;
  __O  uint32_t ROTL_CAA;
  __O  uint32_t ROTL_CA[9];
       uint32_t RESERVED6[69];
  __O  uint32_t AESC_CASR;
  __O  uint32_t AESC_CAA;
  __O  uint32_t AESC_CA[9];
       uint32_t RESERVED7[5];
  __O  uint32_t AESIC_CASR;
  __O  uint32_t AESIC_CAA;
  __O  uint32_t AESIC_CA[9];
} CAU_TypeDef;

/****************************************************************/
/*                                                              */
/*           Low-Leakage Wakeup Unit (LLWU)                     */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t  PE1;
  __IO uint8_t  PE2;
  __IO uint8_t  PE3;
  __IO uint8_t  PE4;
  __IO uint8_t  ME;
  __IO uint8_t  F1;
  __IO uint8_t  F2;
  __I  uint8_t  F3;
  __IO uint8_t  FILT1;
  __IO uint8_t  FILT2;
  __IO uint8_t  RST;
} LLWU_TypeDef;

#define LLWU_PE1_WUPE3_SHIFT        6
#define LLWU_PE1_WUPE3_MASK         ((uint8_t)0xC0)
#define LLWU_PE1_WUPE3(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE1_WUPE3_SHIFT) & LLWU_PE1_WUPE3_MASK))
#define LLWU_PE1_WUPE2_SHIFT        4
#define LLWU_PE1_WUPE2_MASK         ((uint8_t)0x30)
#define LLWU_PE1_WUPE2(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE1_WUPE2_SHIFT) & LLWU_PE1_WUPE2_MASK))
#define LLWU_PE1_WUPE1_SHIFT        2
#define LLWU_PE1_WUPE1_MASK         ((uint8_t)0x0C)
#define LLWU_PE1_WUPE1(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE1_WUPE1_SHIFT) & LLWU_PE1_WUPE1_MASK))
#define LLWU_PE1_WUPE0_SHIFT        0
#define LLWU_PE1_WUPE0_MASK         ((uint8_t)0x03)
#define LLWU_PE1_WUPE0(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE1_WUPE0_SHIFT) & LLWU_PE1_WUPE0_MASK))

#define LLWU_PE2_WUPE3_SHIFT        6
#define LLWU_PE2_WUPE3_MASK         ((uint8_t)0xC0)
#define LLWU_PE2_WUPE3(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE2_WUPE3_SHIFT) & LLWU_PE2_WUPE3_MASK))
#define LLWU_PE2_WUPE2_SHIFT        4
#define LLWU_PE2_WUPE2_MASK         ((uint8_t)0x30)
#define LLWU_PE2_WUPE2(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE2_WUPE2_SHIFT) & LLWU_PE2_WUPE2_MASK))
#define LLWU_PE2_WUPE1_SHIFT        2
#define LLWU_PE2_WUPE1_MASK         ((uint8_t)0x0C)
#define LLWU_PE2_WUPE1(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE2_WUPE1_SHIFT) & LLWU_PE2_WUPE1_MASK))
#define LLWU_PE2_WUPE0_SHIFT        0
#define LLWU_PE2_WUPE0_MASK         ((uint8_t)0x03)
#define LLWU_PE2_WUPE0(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE2_WUPE0_SHIFT) & LLWU_PE2_WUPE0_MASK))

#define LLWU_PE3_WUPE3_SHIFT        6
#define LLWU_PE3_WUPE3_MASK         ((uint8_t)0xC0)
#define LLWU_PE3_WUPE3(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE3_WUPE3_SHIFT) & LLWU_PE3_WUPE3_MASK))
#define LLWU_PE3_WUPE2_SHIFT        4
#define LLWU_PE3_WUPE2_MASK         ((uint8_t)0x30)
#define LLWU_PE3_WUPE2(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE3_WUPE2_SHIFT) & LLWU_PE3_WUPE2_MASK))
#define LLWU_PE3_WUPE1_SHIFT        2
#define LLWU_PE3_WUPE1_MASK         ((uint8_t)0x0C)
#define LLWU_PE3_WUPE1(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE3_WUPE1_SHIFT) & LLWU_PE3_WUPE1_MASK))
#define LLWU_PE3_WUPE0_SHIFT        0
#define LLWU_PE3_WUPE0_MASK         ((uint8_t)0x03)
#define LLWU_PE3_WUPE0(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE3_WUPE0_SHIFT) & LLWU_PE3_WUPE0_MASK))

#define LLWU_PE4_WUPE3_SHIFT        6
#define LLWU_PE4_WUPE3_MASK         ((uint8_t)0xC0)
#define LLWU_PE4_WUPE3(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE4_WUPE3_SHIFT) & LLWU_PE4_WUPE3_MASK))
#define LLWU_PE4_WUPE2_SHIFT        4
#define LLWU_PE4_WUPE2_MASK         ((uint8_t)0x30)
#define LLWU_PE4_WUPE2(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE4_WUPE2_SHIFT) & LLWU_PE4_WUPE2_MASK))
#define LLWU_PE4_WUPE1_SHIFT        2
#define LLWU_PE4_WUPE1_MASK         ((uint8_t)0x0C)
#define LLWU_PE4_WUPE1(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE4_WUPE1_SHIFT) & LLWU_PE4_WUPE1_MASK))
#define LLWU_PE4_WUPE0_SHIFT        0
#define LLWU_PE4_WUPE0_MASK         ((uint8_t)0x03)
#define LLWU_PE4_WUPE0(x)           ((uint8_t)(((uint8_t)(x)<<LLWU_PE4_WUPE0_SHIFT) & LLWU_PE4_WUPE0_MASK))

#define LLWU_ME_WUME7               ((uint8_t)0x80)
#define LLWU_ME_WUME6               ((uint8_t)0x40)
#define LLWU_ME_WUME5               ((uint8_t)0x20)
#define LLWU_ME_WUME4               ((uint8_t)0x10)
#define LLWU_ME_WUME3               ((uint8_t)0x08)
#define LLWU_ME_WUME2               ((uint8_t)0x04)
#define LLWU_ME_WUME1               ((uint8_t)0x02)
#define LLWU_ME_WUME0               ((uint8_t)0x01)

#define LLWU_F1_WUF7                ((uint8_t)0x80)
#define LLWU_F1_WUF6                ((uint8_t)0x40)
#define LLWU_F1_WUF5                ((uint8_t)0x20)
#define LLWU_F1_WUF4                ((uint8_t)0x10)
#define LLWU_F1_WUF3                ((uint8_t)0x08)
#define LLWU_F1_WUF2                ((uint8_t)0x04)
#define LLWU_F1_WUF1                ((uint8_t)0x02)
#define LLWU_F1_WUF0                ((uint8_t)0x01)

#define LLWU_F2_WUF7                ((uint8_t)0x80)
#define LLWU_F2_WUF6                ((uint8_t)0x40)
#define LLWU_F2_WUF5                ((uint8_t)0x20)
#define LLWU_F2_WUF4                ((uint8_t)0x10)
#define LLWU_F2_WUF3                ((uint8_t)0x08)
#define LLWU_F2_WUF2                ((uint8_t)0x04)
#define LLWU_F2_WUF1                ((uint8_t)0x02)
#define LLWU_F2_WUF0                ((uint8_t)0x01)

#define LLWU_F3_WUF7                ((uint8_t)0x80)
#define LLWU_F3_WUF6                ((uint8_t)0x40)
#define LLWU_F3_WUF5                ((uint8_t)0x20)
#define LLWU_F3_WUF4                ((uint8_t)0x10)
#define LLWU_F3_WUF3                ((uint8_t)0x08)
#define LLWU_F3_WUF2                ((uint8_t)0x04)
#define LLWU_F3_WUF1                ((uint8_t)0x02)
#define LLWU_F3_WUF0                ((uint8_t)0x01)

#define LLWU_FILT1_FILTF            ((uint8_t)0x80)
#define LLWU_FILT1_FILTE_SHIFT      5
#define LLWU_FILT1_FILTE_MASK       ((uint8_t)0x60)
#define LLWU_FILT1_FILTE(x)         ((uint8_t)(((uint8_t)(x)<<LLWU_FILT1_FILTE_SHIFT) & LLWU_FILT1_FILTE_MASK))
#define LLWU_FILT1_FILTSEL_SHIFT    0
#define LLWU_FILT1_FILTSEL_MASK     ((uint8_t)0x0F)
#define LLWU_FILT1_FILTSEL(x)       ((uint8_t)(((uint8_t)(x)<<LLWU_FILT1_FILTSEL_SHIFT) & LLWU_FILT1_FILTSEL_MASK))

#define LLWU_FILT2_FILTF            ((uint8_t)0x80)
#define LLWU_FILT2_FILTE_SHIFT      5
#define LLWU_FILT2_FILTE_MASK       ((uint8_t)0x60)
#define LLWU_FILT2_FILTE(x)         ((uint8_t)(((uint8_t)(x)<<LLWU_FILT2_FILTE_SHIFT) & LLWU_FILT2_FILTE_MASK))
#define LLWU_FILT2_FILTSEL_SHIFT    0
#define LLWU_FILT2_FILTSEL_MASK     ((uint8_t)0x0F)
#define LLWU_FILT2_FILTSEL(x)       ((uint8_t)(((uint8_t)(x)<<LLWU_FILT2_FILTSEL_SHIFT) & LLWU_FILT2_FILTSEL_MASK))

#define LLWU_RST_RSTFILT            ((uint8_t)0x01)
#define LLWU_RST_LLRSTE             ((uint8_t)0x02)

/****************************************************************/
/*                                                              */
/*             Port Control and interrupts (PORT)               */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t PCR[32];
  __O  uint32_t GPCLR;
  __O  uint32_t GPCHR;
       uint32_t RESERVED0[6];
  __IO uint32_t ISFR;
       uint32_t RESERVED1[7];
  __IO uint32_t DFER;
  __IO uint32_t DFCR;
  __IO uint32_t DFWR;
} PORT_TypeDef;

#define PORTx_PCRn_ISF               ((uint32_t)0x01000000)
#define PORTx_PCRn_IRQC_SHIFT        16
#define PORTx_PCRn_IRQC_MASK         ((uint32_t)((uint32_t)0xF << PORTx_PCRn_IRQC_SHIFT))
#define PORTx_PCRn_IRQC(x)           ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_IRQC_SHIFT) & PORTx_PCRn_IRQC_MASK))
#define PORTx_PCRn_LK                ((uint32_t)0x00008000)
#define PORTx_PCRn_MUX_SHIFT         8
#define PORTx_PCRn_MUX_MASK          ((uint32_t)((uint32_t)0x7 << PORTx_PCRn_MUX_SHIFT))   /*!< Pin Mux Control (mask) */
#define PORTx_PCRn_MUX(x)            ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_MUX_SHIFT) & PORTx_PCRn_MUX_MASK))  /*!< Pin Mux Control */
#define PORTx_PCRn_DSE               ((uint32_t)0x00000040)
#define PORTx_PCRn_ODE               ((uint32_t)0x00000020)
#define PORTx_PCRn_PFE               ((uint32_t)0x00000010)
#define PORTx_PCRn_SRE               ((uint32_t)0x00000004)
#define PORTx_PCRn_PE                ((uint32_t)0x00000002)
#define PORTx_PCRn_PS                ((uint32_t)0x00000001)

/****************************************************************/
/*                                                              */
/*             Multipurpose Clock Generator (MCG)               */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t  C1;
  __IO uint8_t  C2;
  __IO uint8_t  C3;
  __IO uint8_t  C4;
  __IO uint8_t  C5;
  __IO uint8_t  C6;
  __I  uint8_t  S;
       uint8_t  RESERVED0[1];
  __IO uint8_t  SC;
       uint8_t  RESERVED1[1];
  __IO uint8_t  ATCVH;
  __IO uint8_t  ATCVL;
  __IO uint8_t  C7;
  __IO uint8_t  C8;
} MCG_TypeDef;

#define MCG_C1_CLKS_SHIFT           6
#define MCG_C1_CLKS_MASK            ((uint8_t)0xC0)
#define MCG_C1_CLKS(x)              ((uint8_t)(((uint8_t)(x) << MCG_C1_CLKS_SHIFT) & MCG_C1_CLKS_MASK))
#define MCG_C1_CLKS_FLLPLL          MCG_C1_CLKS(0)
#define MCG_C1_CLKS_IRCLK           MCG_C1_CLKS(1)
#define MCG_C1_CLKS_ERCLK           MCG_C1_CLKS(2)
#define MCG_C1_FRDIV_SHIFT          3
#define MCG_C1_FRDIV_MASK           ((uint8_t)0x38)
#define MCG_C1_FRDIV(x)             ((uint8_t)(((uint8_t)(x) << MCG_C1_FRDIV_SHIFT) & MCG_C1_FRDIV_MASK))
#define MCG_C1_IREFS                ((uint8_t)0x04)
#define MCG_C1_IRCLKEN              ((uint8_t)0x02)
#define MCG_C1_IREFSTEN             ((uint8_t)0x01)

#define MCG_C2_LOCRE0               ((uint8_t)0x80)
#define MCG_C2_RANGE0_SHIFT         4
#define MCG_C2_RANGE0_MASK          ((uint8_t)0x30)
#define MCG_C2_RANGE0(x)            ((uint8_t)(((uint8_t)(x) << MCG_C2_RANGE0_SHIFT) & MCG_C2_RANGE0_MASK))
#define MCG_C2_HGO0                 ((uint8_t)0x08)
#define MCG_C2_EREFS0               ((uint8_t)0x04)
#define MCG_C2_LP                   ((uint8_t)0x02)
#define MCG_C2_IRCS                 ((uint8_t)0x01)

#define MCG_C4_DMX32                ((uint8_t)0x80)
#define MCG_C4_DRST_DRS_SHIFT       5
#define MCG_C4_DRST_DRS_MASK        ((uint8_t)0x60)
#define MCG_C4_DRST_DRS(x)          ((uint8_t)(((uint8_t)(x) << MCG_C4_DRST_DRS_SHIFT) & MCG_C4_DRST_DRS_MASK))
#define MCG_C4_FCTRIM_SHIFT         1
#define MCG_C4_FCTRIM_MASK          ((uint8_t)0x1E)
#define MCG_C4_FCTRIM(x)            ((uint8_t)(((uint8_t)(x) << MCG_C4_FCTRIM_SHIFT) & MCG_C4_FCTRIM_MASK))
#define MCG_C4_SCFTRIM              ((uint8_t)0x01)

#define MCG_C5_PLLCLKEN0            ((uint8_t)0x40)
#define MCG_C5_PLLSTEN0             ((uint8_t)0x20)
#define MCG_C5_PRDIV0_MASK          ((uint8_t)0x1F)
#define MCG_C5_PRDIV0(x)            ((uint8_t)((uint8_t)(x) & MCG_C5_PRDIV0_MASK))

#define MCG_C6_LOLIE0               ((uint8_t)0x80)
#define MCG_C6_PLLS                 ((uint8_t)0x40)
#define MCG_C6_CME0                 ((uint8_t)0x20)
#define MCG_C6_VDIV0_MASK           ((uint8_t)0x1F)
#define MCG_C6_VDIV0(x)             ((uint8_t)((uint8_t)(x) & MCG_C6_VDIV0_MASK))

#define MCG_S_LOLS                  ((uint8_t)0x80)
#define MCG_S_LOCK0                 ((uint8_t)0x40)
#define MCG_S_PLLST                 ((uint8_t)0x20)
#define MCG_S_IREFST                ((uint8_t)0x10)
#define MCG_S_CLKST_SHIFT           2
#define MCG_S_CLKST_MASK            ((uint8_t)0x0C)
#define MCG_S_CLKST(x)              ((uint8_t)(((uint8_t)(x) << MCG_S_CLKST_SHIFT) & MCG_S_CLKST_MASK))
#define MCG_S_CLKST_FLL             MCG_S_CLKST(0)
#define MCG_S_CLKST_IRCLK           MCG_S_CLKST(1)
#define MCG_S_CLKST_ERCLK           MCG_S_CLKST(2)
#define MCG_S_CLKST_PLL             MCG_S_CLKST(3)
#define MCG_S_OSCINIT0              ((uint8_t)0x02)
#define MCG_S_IRCST                 ((uint8_t)0x01)

#define MCG_SC_ATME                 ((uint8_t)0x80)
#define MCG_SC_ATMS                 ((uint8_t)0x40)
#define MCG_SC_ATMF                 ((uint8_t)0x20)
#define MCG_SC_FLTPRSRV             ((uint8_t)0x10)
#define MCG_SC_FCRDIV_SHIFT         1
#define MCG_SC_FCRDIV_MASK          ((uint8_t)0x0E)
#define MCG_SC_FCRDIV(x)            ((uint8_t)(((uint8_t)(x) << MCG_SC_FCRDIV_SHIFT) & MCG_SC_FCRDIV_MASK))
#define MCG_SC_FCRDIV_DIV1          MCG_SC_FCRDIV(0)
#define MCG_SC_FCRDIV_DIV2          MCG_SC_FCRDIV(1)
#define MCG_SC_FCRDIV_DIV4          MCG_SC_FCRDIV(2)
#define MCG_SC_FCRDIV_DIV8          MCG_SC_FCRDIV(3)
#define MCG_SC_FCRDIV_DIV16         MCG_SC_FCRDIV(4)
#define MCG_SC_FCRDIV_DIV32         MCG_SC_FCRDIV(5)
#define MCG_SC_FCRDIV_DIV64         MCG_SC_FCRDIV(6)
#define MCG_SC_FCRDIV_DIV128        MCG_SC_FCRDIV(7)
#define MCG_SC_LOCS0                ((uint8_t)0x01)

#define MCG_C7_OSCSEL               ((uint8_t)0x01)

#define MCG_C8_LOCRE1               ((uint8_t)0x80)
#define MCG_C8_LOLRE                ((uint8_t)0x40)
#define MCG_C8_CME1                 ((uint8_t)0x20)
#define MCG_C8_LOCS1                ((uint8_t)0x01)

typedef struct
{
  __IO uint32_t CESR;
       uint8_t  RESERVED0[12];
  struct {
    __I  uint32_t EAR;
    __I  uint32_t EDR;
  } SP[5];
       uint8_t  RESERVED1[968];
  __IO uint32_t WORD[12][4];
       uint8_t  RESERVED2[832];
  __IO uint32_t RGDAAC[12];
} MPU_TypeDef;

#define MPU_CESR_SPERR_SHIFT        27
#define MPU_CESR_SPERR_MASK         ((uint32_t)0xF8000000)
#define MPU_CESR_SPERR(x)           ((uint32_t)(((uint32_t)(x)<<MPU_CESR_SPERR_SHIFT)&MPU_CESR_SPERR_MASK))
#define MPU_CESR_HRL_SHIFT          16
#define MPU_CESR_HRL_MASK           ((uint32_t)0x000F0000)
#define MPU_CESR_HRL(x)             ((uint32_t)(((uint32_t)(x)<<MPU_CESR_HRL_SHIFT)&MPU_CESR_HRL_MASK))
#define MPU_CESR_NSP_SHIFT          12
#define MPU_CESR_NSP_MASK           ((uint32_t)0x0000F000)
#define MPU_CESR_NSP(x)             ((uint32_t)(((uint32_t)(x)<<MPU_CESR_NSP_SHIFT)&MPU_CESR_NSP_MASK))
#define MPU_CESR_NRGD_SHIFT         8
#define MPU_CESR_NRGD_MASK          ((uint32_t)0x00000F00)
#define MPU_CESR_NRGD(x)            ((uint32_t)(((uint32_t)(x)<<MPU_CESR_NRGD_SHIFT)&MPU_CESR_NRGD_MASK))
#define MPU_CESR_VLD                ((uint32_t)0x00000001)

#define MPU_EDRn_EACD_SHIFT         16
#define MPU_EDRn_EACD_MASK          ((uint32_t)0xFFFF0000)
#define MPU_EDRn_EACD(x)            ((uint32_t)(((uint32_t)(x)<<MPU_EDRn_EACD_SHIFT)&MPU_EDRn_EACD_MASK))
#define MPU_EDRn_EPID_SHIFT         8
#define MPU_EDRn_EPID_MASK          ((uint32_t)0x0000FF00)
#define MPU_EDRn_EPID(x)            ((uint32_t)(((uint32_t)(x)<<MPU_EDRn_EPID_SHIFT)&MPU_EDRn_EPID_MASK))
#define MPU_EDRn_EMN_SHIFT          4
#define MPU_EDRn_EMN_MASK           ((uint32_t)0x000000F0)
#define MPU_EDRn_EMN(x)             ((uint32_t)(((uint32_t)(x)<<MPU_EDRn_EMN_SHIFT)&MPU_EDRn_EMN_MASK))
#define MPU_EDRn_EATTR_SHIFT        1
#define MPU_EDRn_EATTR_MASK         ((uint32_t)0x0000000E)
#define MPU_EDRn_EATTR(x)           ((uint32_t)(((uint32_t)(x)<<MPU_EDRn_EATTR_SHIFT)&MPU_EDRn_EATTR_MASK))
#define MPU_EDRn_ERW                ((uint32_t)0x00000001)

#define MPU_RGDAACn_M7RE            ((uint32_t)0x80000000)
#define MPU_RGDAACn_M7WE            ((uint32_t)0x40000000)
#define MPU_RGDAACn_M6RE            ((uint32_t)0x20000000)
#define MPU_RGDAACn_M6WE            ((uint32_t)0x10000000)
#define MPU_RGDAACn_M5RE            ((uint32_t)0x08000000)
#define MPU_RGDAACn_M5WE            ((uint32_t)0x04000000)
#define MPU_RGDAACn_M4RE            ((uint32_t)0x02000000)
#define MPU_RGDAACn_M4WE            ((uint32_t)0x01000000)
#define MPU_RGDAACn_M3PE            ((uint32_t)0x00800000)
#define MPU_RGDAACn_M3SM_SHIFT      21
#define MPU_RGDAACn_M3SM_MASK       ((uint32_t)0x00600000)
#define MPU_RGDAACn_M3SM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M3SM_SHIFT)&MPU_RGDAACn_M3SM_MASK))
#define MPU_RGDAACn_M3UM_SHIFT      18
#define MPU_RGDAACn_M3UM_MASK       ((uint32_t)0x001C0000)
#define MPU_RGDAACn_M3UM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M3UM_SHIFT)&MPU_RGDAACn_M3UM_MASK))
#define MPU_RGDAACn_M2PE            ((uint32_t)0x00020000)
#define MPU_RGDAACn_M2SM_SHIFT      15
#define MPU_RGDAACn_M2SM_MASK       ((uint32_t)0x00018000)
#define MPU_RGDAACn_M2SM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M2SM_SHIFT)&MPU_RGDAACn_M2SM_MASK))
#define MPU_RGDAACn_M2UM            ((uint32_t)0x00007000)
#define MPU_RGDAACn_M1PE            ((uint32_t)0x00000800)
#define MPU_RGDAACn_M1SM_SHIFT      9
#define MPU_RGDAACn_M1SM_MASK       ((uint32_t)0x00000600)
#define MPU_RGDAACn_M1SM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M1SM_SHIFT)&MPU_RGDAACn_M1SM_MASK))
#define MPU_RGDAACn_M1UM_SHIFT      6
#define MPU_RGDAACn_M1UM_MASK       ((uint32_t)0x000001C0)
#define MPU_RGDAACn_M1UM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M1UM_SHIFT)&MPU_RGDAACn_M1UM_MASK))
#define MPU_RGDAACn_M0PE_SHIFT      5
#define MPU_RGDAACn_M0PE_MASK       ((uint32_t)0x00000020)
#define MPU_RGDAACn_M0SM_SHIFT      3
#define MPU_RGDAACn_M0SM_MASK       ((uint32_t)0x00000018)
#define MPU_RGDAACn_M0SM(x)         ((uint32_t)(((uint32_t)(x)<<MPU_RGDAACn_M0SM_SHIFT)&MPU_RGDAACn_M0SM_MASK))
#define MPU_RGDAACn_M0UM            ((uint32_t)0x00000007)

/****************************************************************/
/*                                                              */
/*                      Oscillator (OSC)                        */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t  CR;
} OSC_TypeDef;

#define OSC_CR_ERCLKEN              ((uint8_t)0x80)
#define OSC_CR_EREFSTEN             ((uint8_t)0x20)
#define OSC_CR_SC2P                 ((uint8_t)0x08)
#define OSC_CR_SC4P                 ((uint8_t)0x04)
#define OSC_CR_SC8P                 ((uint8_t)0x02)
#define OSC_CR_SC16P                ((uint8_t)0x01)

/****************************************************************/
/*                                                              */
/*                          DMA TCD                             */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t SADDR;
  __IO uint16_t SOFF;
  __IO uint16_t ATTR;
  union {
    __IO uint32_t NBYTES_MLNO;
    __IO uint32_t NBYTES_MLOFFNO;
    __IO uint32_t NBYTES_MLOFFYES;
  };
  __IO uint32_t SLAST;
  __IO uint32_t DADDR;
  __IO uint16_t DOFF;
  union {
    __IO uint16_t CITER_ELINKNO;
    __IO uint16_t CITER_ELINKYES;
  };
  __IO uint32_t DLASTSGA;
  __IO uint16_t CSR;
  union {
    __IO uint16_t BITER_ELINKNO;
    __IO uint16_t BITER_ELINKYES;
  };
} DMA_TCD_TypeDef;

#define DMA_ATTR_SMOD_SHIFT          11
#define DMA_ATTR_SMOD_MASK           ((uint16_t)0xF800)
#define DMA_ATTR_SMOD(x)             ((uint16_t)(((uint16_t)(x) << DMA_ATTR_SMOD_SHIFT) & DMA_ATTR_SMOD_MASK))
#define DMA_ATTR_SSIZE_SHIFT         8
#define DMA_ATTR_SSIZE_MASK          ((uint16_t)0x0700)
#define DMA_ATTR_SSIZE(x)            ((uint16_t)(((uint16_t)(x) << DMA_ATTR_SSIZE_SHIFT) & DMA_ATTR_SSIZE_MASK))
#define DMA_ATTR_DMOD_SHIFT          3
#define DMA_ATTR_DMOD_MASK           ((uint16_t)0x00F8)
#define DMA_ATTR_DMOD(x)             ((uint16_t)(((uint16_t)(x) << DMA_ATTR_DMOD_SHIFT) & DMA_ATTR_DMOD_MASK))
#define DMA_ATTR_DSIZE_SHIFT         0
#define DMA_ATTR_DSIZE_MASK          ((uint16_t)0x0007)
#define DMA_ATTR_DSIZE(x)            ((uint16_t)(((uint16_t)(x) << DMA_ATTR_DSIZE_SHIFT) & DMA_ATTR_DSIZE_MASK))

#define DMA_CSR_BWC_SHIFT            14
#define DMA_CSR_BWC_MASK             ((uint16_t)0xC000)
#define DMA_CSR_BWC(x)               ((uint16_t)(((uint16_t)(x) << DMA_CSR_BWC_SHIFT) & DMA_CSR_BWC_MASK))
#define DMA_CSR_MAJORLINKCH_SHIFT    8
#define DMA_CSR_MAJORLINKCH_MASK     ((uint16_t)0x0F00)
#define DMA_CSR_MAJORLINKCH(x)       ((uint16_t)(((uint16_t)(x) << DMA_CSR_MAJORLINKCH_SHIFT) & DMA_CSR_MAJORLINKCH_MASK))
#define DMA_CSR_DONE_MASK            ((uint16_t)0x0080)
#define DMA_CSR_ACTIVE_MASK          ((uint16_t)0x0040)
#define DMA_CSR_MAJORELINK_MASK      ((uint16_t)0x0020)
#define DMA_CSR_ESG_MASK             ((uint16_t)0x0010)
#define DMA_CSR_DREQ_MASK            ((uint16_t)0x0008)
#define DMA_CSR_INTHALF_MASK         ((uint16_t)0x0004)
#define DMA_CSR_INTMAJOR_MASK        ((uint16_t)0x0002)
#define DMA_CSR_START_MASK           ((uint16_t)0x0001)

/****************************************************************/
/*                                                              */
/*           Direct Memory Access Controller (eDMA)             */
/*                                                              */
/****************************************************************/
typedef struct {
  __IO uint32_t CR;
  __IO uint32_t ES;
       uint32_t RESERVED0;
  __IO uint32_t ERQ;
       uint32_t RESERVED1;
  __IO uint32_t EEI;
  __IO uint8_t CEEI;
  __IO uint8_t SEEI;
  __IO uint8_t CERQ;
  __IO uint8_t SERQ;
  __IO uint8_t CDNE;
  __IO uint8_t SSRT;
  __IO uint8_t CERR;
  __IO uint8_t CINT;
       uint32_t RESERVED2;
  __IO uint32_t INT;
       uint32_t RESERVED3;
  __IO uint32_t ERR;
       uint32_t RESERVED4[4];
  __IO uint32_t HRS;
       uint8_t RESERVED5[200];
  __IO uint8_t DCHPRI3;
  __IO uint8_t DCHPRI2;
  __IO uint8_t DCHPRI1;
  __IO uint8_t DCHPRI0;
  __IO uint8_t DCHPRI7;
  __IO uint8_t DCHPRI6;
  __IO uint8_t DCHPRI5;
  __IO uint8_t DCHPRI4;
  __IO uint8_t DCHPRI11;
  __IO uint8_t DCHPRI10;
  __IO uint8_t DCHPRI9;
  __IO uint8_t DCHPRI8;
  __IO uint8_t DCHPRI15;
  __IO uint8_t DCHPRI14;
  __IO uint8_t DCHPRI13;
  __IO uint8_t DCHPRI12;
       uint8_t RESERVED6[3824];
  DMA_TCD_TypeDef TCD[16];
} DMA_TypeDef;


/****************************************************************/
/*                                                              */
/*         Direct Memory Access Multiplexer (DMAMUX)            */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t  CHCFG[16];
} DMAMUX_TypeDef;

#define DMAMUX_CHCFGn_ENBL           ((uint8_t)0x80)
#define DMAMUX_CHCFGn_TRIG           ((uint8_t)0x40)
#define DMAMUX_CHCFGn_SOURCE_SHIFT   0
#define DMAMUX_CHCFGn_SOURCE_MASK    ((uint8_t)0x3F)
#define DMAMUX_CHCFGn_SOURCE(x)      ((uint8_t)(((uint8_t)(x) << DMAMUX_CHCFGn_SOURCE_SHIFT) & DMAMUX_CHCFGn_SOURCE_MASK))

/****************************************************************/
/*                                                              */
/*               Periodic Interrupt Timer (PIT)                 */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t MCR;
       uint8_t  RESERVED0[252];
  struct PIT_CHANNEL {
    __IO uint32_t LDVAL;
    __IO uint32_t CVAL;
    __IO uint32_t TCTRL;
    __IO uint32_t TFLG;
  } CHANNEL[4];
} PIT_TypeDef;

#define PIT_MCR_MDIS                ((uint32_t)0x00000002)
#define PIT_MCR_FRZ                 ((uint32_t)0x00000001)

#define PIT_TCTRLn_CHN              ((uint32_t)0x00000004)
#define PIT_TCTRLn_TIE              ((uint32_t)0x00000002)
#define PIT_TCTRLn_TEN              ((uint32_t)0x00000001)

#define PIT_TFLGn_TIF               ((uint32_t)0x00000001)

/****************************************************************/
/*                                                              */
/*                 Flash Memory Module (FTFE)                   */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t FSTAT;
  __IO uint8_t FCNFG;
  __I  uint8_t FSEC;
  __I  uint8_t FOPT;
  __IO uint8_t FCCOB3;
  __IO uint8_t FCCOB2;
  __IO uint8_t FCCOB1;
  __IO uint8_t FCCOB0;
  __IO uint8_t FCCOB7;
  __IO uint8_t FCCOB6;
  __IO uint8_t FCCOB5;
  __IO uint8_t FCCOB4;
  __IO uint8_t FCCOBB;
  __IO uint8_t FCCOBA;
  __IO uint8_t FCCOB9;
  __IO uint8_t FCCOB8;
  __IO uint8_t FPROT3;
  __IO uint8_t FPROT2;
  __IO uint8_t FPROT1;
  __IO uint8_t FPROT0;
       uint8_t RESERVED_0[2];
  __IO uint8_t FEPROT;
  __IO uint8_t FDPROT;
} FTFE_TypeDef;

#define FTFE_FSTAT_CCIF              ((uint8_t)0x80)
#define FTFE_FSTAT_RDCOLERR          ((uint8_t)0x40)
#define FTFE_FSTAT_ACCERR            ((uint8_t)0x20)
#define FTFE_FSTAT_FPVIOL            ((uint8_t)0x10)
#define FTFE_FSTAT_MGSTAT0           ((uint8_t)0x01)

#define FTFE_FCNFG_CCIE              ((uint8_t)0x80)
#define FTFE_FCNFG_RDCOLLIE          ((uint8_t)0x40)
#define FTFE_FCNFG_ERSAREQ           ((uint8_t)0x20)
#define FTFE_FCNFG_ERSSUSP           ((uint8_t)0x10)
#define FTFE_FCNFG_SWAP              ((uint8_t)0x08)
#define FTFE_FCNFG_PFLSH             ((uint8_t)0x04)
#define FTFE_FCNFG_RAMRDY            ((uint8_t)0x02)
#define FTFE_FCNFG_EEERDY            ((uint8_t)0x01)

#define FTFE_FSEC_KEYEN_SHIFT        6
#define FTFE_FSEC_KEYEN_MASK         ((uint8_t)0xC0)
#define FTFE_FSEC_KEYEN(x)           ((uint8_t)(((uint8_t)(x)<<FTFE_FSEC_KEYEN_SHIFT)&FTFE_FSEC_KEYEN_MASK))
#define FTFE_FSEC_MEEN_SHIFT         4
#define FTFE_FSEC_MEEN_MASK          ((uint8_t)0x30)
#define FTFE_FSEC_MEEN(x)            ((uint8_t)(((uint8_t)(x)<<FTFE_FSEC_MEEN_SHIFT)&FTFE_FSEC_MEEN_MASK))
#define FTFE_FSEC_FSLACC_SHIFT       2
#define FTFE_FSEC_FSLACC_MASK        ((uint8_t)0x0C)
#define FTFE_FSEC_FSLACC(x)          ((uint8_t)(((uint8_t)(x)<<FTFE_FSEC_FSLACC_SHIFT)&FTFE_FSEC_FSLACC_MASK))
#define FTFE_FSEC_SEC_SHIFT          0
#define FTFE_FSEC_SEC_MASK           ((uin8t_t)0x03)
#define FTFE_FSEC_SEC(x)             ((uint8_t)(((uint8_t)(x)<<FTFE_FSEC_SEC_SHIFT)&FTFE_FSEC_SEC_MASK))

/****************************************************************/
/*                                                              */
/*                   FlexTimer Module (FTM)                     */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t SC;             /**< Status and Control                     */
  __IO uint32_t CNT;            /**< Counter                                */
  __IO uint32_t MOD;            /**< Modulo                                 */
   struct FTM_Channel {
     __IO uint32_t CnSC;        /**< Channel Status and Control             */
     __IO uint32_t CnV;         /**< Channel Value                          */
   } CHANNEL[8];
  __IO uint32_t CNTIN;          /**< Counter Initial Value                  */
  __IO uint32_t STATUS;         /**< Capture and Compare Status             */
  __IO uint32_t MODE;           /**< Features Mode Selection                */
  __IO uint32_t SYNC;           /**< Synchronization                        */
  __IO uint32_t OUTINIT;        /**< Initial State for Channels Output      */
  __IO uint32_t OUTMASK;        /**< Output Mask                            */
  __IO uint32_t COMBINE;        /**< Function for Linked Channels           */
  __IO uint32_t DEADTIME;       /**< Deadtime Insertion Control             */
  __IO uint32_t EXTTRIG;        /**< FTM External Trigger                   */
  __IO uint32_t POL;            /**< Channels Polarity                      */
  __IO uint32_t FMS;            /**< Fault Mode Status                      */
  __IO uint32_t FILTER;         /**< Input Capture Filter Control           */
  __IO uint32_t FLTCTRL;        /**< Fault Control                          */
  __IO uint32_t QDCTRL;         /**< Quadrature Decode Control and Status   */
  __IO uint32_t CONF;           /**< Configuration                          */
  __IO uint32_t FTLPOL;         /**< FTM Fault Input Polarity               */
  __IO uint32_t SYNCONF;        /**< Synchronization Configuration          */
  __IO uint32_t INVCTRL;        /**< FTM Inverting Control                  */
  __IO uint32_t SWOCTRL;        /**< FTM Software Output Control            */
  __IO uint32_t PWMLOAD;        /**< FTM PWM Load                           */
} FTM_TypeDef;

#define FTM_SC_TOF                   ((uint32_t)0x00000080)
#define FTM_SC_TOIE                  ((uint32_t)0x00000040)
#define FTM_SC_CPWMS                 ((uint32_t)0x00000020)
#define FTM_SC_CLKS_SHIFT            3
#define FTM_SC_CLKS_MASK             ((uint32_t)0x00000018)
#define FTM_SC_CLKS(x)               ((uint32_t)(((uint32_t)(x)<<FTM_SC_CLKS_SHIFT)&FTM_SC_CLKS_MASK))
#define FTM_SC_PS_SHIFT              0
#define FTM_SC_PS_MASK               ((uint32_t)0x00000007)
#define FTM_SC_PS(x)                 ((uint32_t)(((uint32_t)(x)<<FTM_SC_PS_SHIFT)&FTM_SC_PS_MASK))

#define FTM_CnSC_CHF                 ((uint32_t)0x00000080)
#define FTM_CnSC_CHIE                ((uint32_t)0x00000040)
#define FTM_CnSC_MSB                 ((uint32_t)0x00000020)
#define FTM_CnSC_MSA                 ((uint32_t)0x00000010)
#define FTM_CnSC_ELSB                ((uint32_t)0x00000008)
#define FTM_CnSC_ELSA                ((uint32_t)0x00000004)
#define FTM_CnSC_DMA                 ((uint32_t)0x00000001)

#define FTM_STATUS_CH7F              ((uint32_t)0x00000080)
#define FTM_STATUS_CH6F              ((uint32_t)0x00000040)
#define FTM_STATUS_CH5F              ((uint32_t)0x00000020)
#define FTM_STATUS_CH4F              ((uint32_t)0x00000010)
#define FTM_STATUS_CH3F              ((uint32_t)0x00000008)
#define FTM_STATUS_CH2F              ((uint32_t)0x00000004)
#define FTM_STATUS_CH1F              ((uint32_t)0x00000002)
#define FTM_STATUS_CH0F              ((uint32_t)0x00000001)

#define FTM_MODE_FAULTIE             ((uint32_t)0x00000080)
#define FTM_MODE_FAULTM_SHIFT        5
#define FTM_MODE_FAULTM_MASK         ((uint32_t)0x00000060)
#define FTM_MODE_FAULTM(x)           ((uint32_t)(((uint32_t)(x)<<FTM_MODE_FAULTM_SHIFT)&FTM_MODE_FAULTM_MASK))
#define FTM_MODE_CAPTEST             ((uint32_t)0x00000010)
#define FTM_MODE_PWMSYNC             ((uint32_t)0x00000008)
#define FTM_MODE_WPDIS               ((uint32_t)0x00000004)
#define FTM_MODE_INIT                ((uint32_t)0x00000002)
#define FTM_MODE_FTMEN               ((uint32_t)0x00000001)
#define FTM_MODE_PWMSYNC_MASK        FTM_MODE_PWMSYNC
#define FTM_MODE_FTMEN_MASK          FTM_MODE_FTMEN

#define FTM_SYNC_SWSYNC_MASK         ((uint32_t)0x00000080)
#define FTM_SYNC_TRIG2               ((uint32_t)0x00000040)
#define FTM_SYNC_TRIG1               ((uint32_t)0x00000020)
#define FTM_SYNC_TRIG0               ((uint32_t)0x00000010)
#define FTM_SYNC_SYNCHOM             ((uint32_t)0x00000008)
#define FTM_SYNC_REINIT              ((uint32_t)0x00000004)
#define FTM_SYNC_CNTMAX              ((uint32_t)0x00000002)
#define FTM_SYNC_CNTMIN              ((uint32_t)0x00000001)
#define FTM_SYNC_CNTMAX_MASK         FTM_SYNC_CNTMAX
#define FTM_SYNC_CNTMIN_MASK         FTM_SYNC_CNTMIN

#define FTM_OUTINIT_CH7OI            ((uint32_t)0x00000080)
#define FTM_OUTINIT_CH6OI            ((uint32_t)0x00000040)
#define FTM_OUTINIT_CH5OI            ((uint32_t)0x00000020)
#define FTM_OUTINIT_CH4OI            ((uint32_t)0x00000010)
#define FTM_OUTINIT_CH3OI            ((uint32_t)0x00000008)
#define FTM_OUTINIT_CH2OI            ((uint32_t)0x00000004)
#define FTM_OUTINIT_CH1OI            ((uint32_t)0x00000002)
#define FTM_OUTINIT_CH0OI            ((uint32_t)0x00000001)

#define FTM_OUTMASK_CH7OM            ((uint32_t)0x00000080)
#define FTM_OUTMASK_CH6OM            ((uint32_t)0x00000040)
#define FTM_OUTMASK_CH5OM            ((uint32_t)0x00000020)
#define FTM_OUTMASK_CH4OM            ((uint32_t)0x00000010)
#define FTM_OUTMASK_CH3OM            ((uint32_t)0x00000008)
#define FTM_OUTMASK_CH2OM            ((uint32_t)0x00000004)
#define FTM_OUTMASK_CH1OM            ((uint32_t)0x00000002)
#define FTM_OUTMASK_CH0OM            ((uint32_t)0x00000001)

#define FTM_COMBINE_FAULTEN3         ((uint32_t)0x40000000)
#define FTM_COMBINE_SYNCEN3          ((uint32_t)0x20000000)
#define FTM_COMBINE_DTEN3            ((uint32_t)0x10000000)
#define FTM_COMBINE_DECAP3           ((uint32_t)0x08000000)
#define FTM_COMBINE_DECAPEN3         ((uint32_t)0x04000000)
#define FTM_COMBINE_COMP3            ((uint32_t)0x02000000)
#define FTM_COMBINE_COMBINE3         ((uint32_t)0x01000000)
#define FTM_COMBINE_FAULTEN2         ((uint32_t)0x00400000)
#define FTM_COMBINE_SYNCEN2          ((uint32_t)0x00200000)
#define FTM_COMBINE_DTEN2            ((uint32_t)0x00100000)
#define FTM_COMBINE_DECAP2           ((uint32_t)0x00080000)
#define FTM_COMBINE_DECAPEN2         ((uint32_t)0x00040000)
#define FTM_COMBINE_COMP2            ((uint32_t)0x00020000)
#define FTM_COMBINE_COMBINE2         ((uint32_t)0x00010000)
#define FTM_COMBINE_FAULTEN1         ((uint32_t)0x00004000)
#define FTM_COMBINE_SYNCEN1          ((uint32_t)0x00002000)
#define FTM_COMBINE_DTEN1            ((uint32_t)0x00001000)
#define FTM_COMBINE_DECAP1           ((uint32_t)0x00000800)
#define FTM_COMBINE_DECAPEN1         ((uint32_t)0x00000400)
#define FTM_COMBINE_COMP1            ((uint32_t)0x00000200)
#define FTM_COMBINE_COMBINE1         ((uint32_t)0x00000100)
#define FTM_COMBINE_FAULTEN0         ((uint32_t)0x00000040)
#define FTM_COMBINE_SYNCEN0          ((uint32_t)0x00000020)
#define FTM_COMBINE_DTEN0            ((uint32_t)0x00000010)
#define FTM_COMBINE_DECAP0           ((uint32_t)0x00000008)
#define FTM_COMBINE_DECAPEN0         ((uint32_t)0x00000004)
#define FTM_COMBINE_COMP0            ((uint32_t)0x00000002)
#define FTM_COMBINE_COMBINE0         ((uint32_t)0x00000001)
#define FTM_COMBINE_SYNCEN3_MASK     FTM_COMBINE_SYNCEN3
#define FTM_COMBINE_SYNCEN2_MASK     FTM_COMBINE_SYNCEN2
#define FTM_COMBINE_SYNCEN1_MASK     FTM_COMBINE_SYNCEN1
#define FTM_COMBINE_SYNCEN0_MASK     FTM_COMBINE_SYNCEN3

#define FTM_DEADTIME_DTPS_SHIFT      6
#define FTM_DEADTIME_DTPS_MASK       ((uint32_t)0x000000C0)
#define FTM_DEADTIME_DTPS(x)         ((uint32_t)(((uint32_t)(x)<<FTM_DEADTIME_DTPS_SHIFT)&FTM_DEADTIME_DTPS_MASK))
#define FTM_DEADTIME_DTVAL_SHIFT     0
#define FTM_DEADTIME_DTVAL_MASK      ((uint32_t)0x0000003F)
#define FTM_DEADTIME_DTVAL(x)        ((uint32_t)(((uint32_t)(x)<<FTM_DEADTIME_DTVAL_SHIFT)&FTM_DEADTIME_DTVAL_MASK))

#define FTM_EXTTRIG_TRIGF            ((uint32_t)0x00000080)
#define FTM_EXTTRIG_INITTRIGEN       ((uint32_t)0x00000040)
#define FTM_EXTTRIG_CH1TRIG          ((uint32_t)0x00000020)
#define FTM_EXTTRIG_CH0TRIG          ((uint32_t)0x00000010)
#define FTM_EXTTRIG_CH5TRIG          ((uint32_t)0x00000008)
#define FTM_EXTTRIG_CH4TRIG          ((uint32_t)0x00000004)
#define FTM_EXTTRIG_CH3TRIG          ((uint32_t)0x00000002)
#define FTM_EXTTRIG_CH2TRIG          ((uint32_t)0x00000001)

#define FTM_POL_POL7                 ((uint32_t)0x00000080)
#define FTM_POL_POL6                 ((uint32_t)0x00000040)
#define FTM_POL_POL5                 ((uint32_t)0x00000020)
#define FTM_POL_POL4                 ((uint32_t)0x00000010)
#define FTM_POL_POL3                 ((uint32_t)0x00000008)
#define FTM_POL_POL2                 ((uint32_t)0x00000004)
#define FTM_POL_POL1                 ((uint32_t)0x00000002)
#define FTM_POL_POL0                 ((uint32_t)0x00000001)

#define FTM_FMS_FAULTF               ((uint32_t)0x00000080)
#define FTM_FMS_WPEN                 ((uint32_t)0x00000040)
#define FTM_FMS_FAULTIN              ((uint32_t)0x00000020)
#define FTM_FMS_FAULTF3              ((uint32_t)0x00000008)
#define FTM_FMS_FAULTF2              ((uint32_t)0x00000004)
#define FTM_FMS_FAULTF1              ((uint32_t)0x00000002)
#define FTM_FMS_FAULTF0              ((uint32_t)0x00000001)

#define FTM_FILTER_CH3FVAL_SHIFT     12
#define FTM_FILTER_CH3FVAL_MASK      ((uint32_t)0x0000F000)
#define FTM_FILTER_CH3FVAL(x)        ((uint32_t)(((uint32_t)(x)<<FTM_FILTER_CH3FVAL_SHIFT)&FTM_FILTER_CH3FVAL_MASK))
#define FTM_FILTER_CH2FVAL_SHIFT     8
#define FTM_FILTER_CH2FVAL_MASK      ((uint32_t)0x00000F00)
#define FTM_FILTER_CH2FVAL(x)        ((uint32_t)(((uint32_t)(x)<<FTM_FILTER_CH2FVAL_SHIFT)&FTM_FILTER_CH2FVAL_MASK))
#define FTM_FILTER_CH1FVAL_SHIFT     4
#define FTM_FILTER_CH1FVAL_MASK      ((uint32_t)0x000000F0)
#define FTM_FILTER_CH1FVAL(x)        ((uint32_t)(((uint32_t)(x)<<FTM_FILTER_CH1FVAL_SHIFT)&FTM_FILTER_CH1FVAL_MASK))
#define FTM_FILTER_CH0FVAL_SHIFT     0
#define FTM_FILTER_CH0FVAL_MASK      ((uint32_t)0x0000000F)
#define FTM_FILTER_CH0FVAL(x)        ((uint32_t)(((uint32_t)(x)<<FTM_FILTER_CH0FVAL_SHIFT)&FTM_FILTER_CH0FVAL_MASK))

#define FTM_FLTCTRL_FFVAL_SHIFT      8
#define FTM_FLTCTRL_FFVAL_MASK       ((uint32_t)0x00000F00)
#define FTM_FLTCTRL_FFVAL(x)         ((uint32_t)(((uint32_t)(x)<<FTM_FLTCTRL_FFVAL_SHIFT)&FTM_FLTCTRL_FFVAL_MASK))
#define FTM_FLTCTRL_FFLTR3EN         ((uint32_t)0x00000080)
#define FTM_FLTCTRL_FFLTR2EN         ((uint32_t)0x00000040)
#define FTM_FLTCTRL_FFLTR1EN         ((uint32_t)0x00000020)
#define FTM_FLTCTRL_FFLTR0EN         ((uint32_t)0x00000010)
#define FTM_FLTCTRL_FAULT3EN         ((uint32_t)0x00000008)
#define FTM_FLTCTRL_FAULT2EN         ((uint32_t)0x00000004)
#define FTM_FLTCTRL_FAULT1EN         ((uint32_t)0x00000002)
#define FTM_FLTCTRL_FAULT0EN         ((uint32_t)0x00000001)

#define FTM_QDCTRL_PHAFLTREN         ((uint32_t)0x00000080)
#define FTM_QDCTRL_PHBFLTREN         ((uint32_t)0x00000040)
#define FTM_QDCTRL_PHAPOL            ((uint32_t)0x00000020)
#define FTM_QDCTRL_PHBPOL            ((uint32_t)0x00000010)
#define FTM_QDCTRL_QUADMODE          ((uint32_t)0x00000008)
#define FTM_QDCTRL_QUADIR            ((uint32_t)0x00000004)
#define FTM_QDCTRL_TOFDIR            ((uint32_t)0x00000002)
#define FTM_QDCTRL_QUADEN            ((uint32_t)0x00000001)

#define FTM_CONF_GTBEOUT             ((uint32_t)0x00000400)
#define FTM_CONF_GTBEEN              ((uint32_t)0x00000200)
#define FTM_CONF_BDMMODE_SHIFT       6
#define FTM_CONF_BDMMODE_MASK        ((uint32_t)0x000000C0)
#define FTM_CONF_BDMMODE(x)          ((uint32_t)(((uint32_t)(x)<<FTM_CONF_BDMMODE_SHIFT)&FTM_CONF_BDMMODE_MASK))
#define FTM_CONF_NUMTOF_SHIFT        0
#define FTM_CONF_NUMTOF_MASK         ((uint32_t)0x0000001F)
#define FTM_CONF_NUMTOF(x)           ((uint32_t)(((uint32_t)(x)<<FTM_CONF_NUMTOF_SHIFT)&FTM_CONF_NUMTOF_MASK))

#define FTM_FLTPOL_FLT3POL           ((uint32_t)0x00000008)
#define FTM_FLTPOL_FLT2POL           ((uint32_t)0x00000004)
#define FTM_FLTPOL_FLT1POL           ((uint32_t)0x00000002)
#define FTM_FLTPOL_FLT0POL           ((uint32_t)0x00000001)

#define FTM_SYNCONF_HWSOC            ((uint32_t)0x00100000)
#define FTM_SYNCONF_HWINVC           ((uint32_t)0x00080000)
#define FTM_SYNCONF_HWOM             ((uint32_t)0x00040000)
#define FTM_SYNCONF_HWWRBUF          ((uint32_t)0x00020000)
#define FTM_SYNCONF_HWRSTCNT         ((uint32_t)0x00010000)
#define FTM_SYNCONF_SWSOC            ((uint32_t)0x00001000)
#define FTM_SYNCONF_SWINVC           ((uint32_t)0x00000800)
#define FTM_SYNCONF_SWOM             ((uint32_t)0x00000400)
#define FTM_SYNCONF_SWWRBUF          ((uint32_t)0x00000200)
#define FTM_SYNCONF_SWRSTCNT         ((uint32_t)0x00000100)
#define FTM_SYNCONF_SYNCMODE_MASK    ((uint32_t)0x00000080)
#define FTM_SYNCONF_SWOC             ((uint32_t)0x00000020)
#define FTM_SYNCONF_INVC             ((uint32_t)0x00000010)
#define FTM_SYNCONF_CNTINC           ((uint32_t)0x00000004)
#define FTM_SYNCONF_HWTRIGMODE       ((uint32_t)0x00000001)

#define FTM_INVCTRL_INV3EN           ((uint32_t)0x00000008)
#define FTM_INVCTRL_INV2EN           ((uint32_t)0x00000004)
#define FTM_INVCTRL_INV1EN           ((uint32_t)0x00000002)
#define FTM_INVCTRL_INV0EN           ((uint32_t)0x00000001)

#define FTM_SWOCTRL_CH7OCV           ((uint32_t)0x00008000)
#define FTM_SWOCTRL_CH6OCV           ((uint32_t)0x00004000)
#define FTM_SWOCTRL_CH5OCV           ((uint32_t)0x00002000)
#define FTM_SWOCTRL_CH4OCV           ((uint32_t)0x00001000)
#define FTM_SWOCTRL_CH3OCV           ((uint32_t)0x00000800)
#define FTM_SWOCTRL_CH2OCV           ((uint32_t)0x00000400)
#define FTM_SWOCTRL_CH1OCV           ((uint32_t)0x00000200)
#define FTM_SWOCTRL_CH0OCV           ((uint32_t)0x00000100)
#define FTM_SWOCTRL_CH7OC            ((uint32_t)0x00000080)
#define FTM_SWOCTRL_CH6OC            ((uint32_t)0x00000040)
#define FTM_SWOCTRL_CH5OC            ((uint32_t)0x00000020)
#define FTM_SWOCTRL_CH4OC            ((uint32_t)0x00000010)
#define FTM_SWOCTRL_CH3OC            ((uint32_t)0x00000008)
#define FTM_SWOCTRL_CH2OC            ((uint32_t)0x00000004)
#define FTM_SWOCTRL_CH1OC            ((uint32_t)0x00000002)
#define FTM_SWOCTRL_CH0OC            ((uint32_t)0x00000001)

#define FTM_PWMLOAD_LDOK             ((uint32_t)0x00000200)
#define FTM_PWMLOAD_CH7SEL           ((uint32_t)0x00000080)
#define FTM_PWMLOAD_CH6SEL           ((uint32_t)0x00000040)
#define FTM_PWMLOAD_CH5SEL           ((uint32_t)0x00000020)
#define FTM_PWMLOAD_CH4SEL           ((uint32_t)0x00000010)
#define FTM_PWMLOAD_CH3SEL           ((uint32_t)0x00000008)
#define FTM_PWMLOAD_CH2SEL           ((uint32_t)0x00000004)
#define FTM_PWMLOAD_CH1SEL           ((uint32_t)0x00000002)
#define FTM_PWMLOAD_CH0SEL           ((uint32_t)0x00000001)
#define FTM_PWMLOAD_LDOK_MASK        FTM_PWMLOAD_LDOK

/****************************************************************/
/*                                                              */
/*                   Low-Power Timer (LPTMR)                    */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t CSR;            /**< Control Status Register */
  __IO uint32_t PSR;            /**< Prescale Register */
  __IO uint32_t CMR;            /**< Compare Register */
  __I  uint32_t CNR;            /**< Counter Register */
} LPTMR_TypeDef;

/*********  Bits definition for LPTMR_CSR register  *************/
#define LPTMR_CSR_TCF                ((uint32_t)0x00000080)
#define LPTMR_CSR_TIE                ((uint32_t)0x00000040)
#define LPTMR_CSR_TPS_SHIFT          4
#define LPTMR_CSR_TPS_MASK           ((uint32_t)0x00000030)
#define LPTMR_CSR_TPS(x)             ((uint32_t)(((uint32_t)(x)<<LPTMR_CSR_TPS_SHIFT)&LPTMR_CSR_TPS_MASK))
#define LPTMR_CSR_TPP                ((uint32_t)0x00000008)
#define LPTMR_CSR_TFC                ((uint32_t)0x00000004)
#define LPTMR_CSR_TMS                ((uint32_t)0x00000002)
#define LPTMR_CSR_TEN                ((uint32_t)0x00000001)

/*********  Bits definition for LPTMR_PSR register  *************/
#define LPTMR_PSR_PRESCALE_SHIFT     3
#define LPTMR_PSR_PRESCALE_MASK      ((uint32_t)0x00000078)
#define LPTMR_PSR_PRESCALE(x)        ((uint32_t)(((uint32_t)(x)<<LPTMR_PSR_PRESCALE_SHIFT)&LPTMR_PSR_PRESCALE_MASK))
#define LPTMR_PSR_PBYP               ((uint32_t)0x00000004)
#define LPTMR_PSR_PCS_SHIFT          0
#define LPTMR_PSR_PCS_MASK           ((uint32_t)0x00000003)
#define LPTMR_PSR_PCS(x)             ((uint32_t)(((uint32_t)(x)<<LPTMR_PSR_PCS_SHIFT)&LPTMR_PSR_PCS_MASK))

/****************************************************************/
/*                                                              */
/*             General-Purpose Input/Output (GPIO)              */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t PDOR;           /**< Port Data Output Register */
  __IO uint32_t PSOR;           /**< Port Set Output Register */
  __IO uint32_t PCOR;           /**< Port Clear Output Register */
  __IO uint32_t PTOR;           /**< Port Toggle Output Register */
  __IO uint32_t PDIR;           /**< Port Data Input Register */
  __IO uint32_t PDDR;           /**< Port Data Direction Register */
} GPIO_TypeDef;

/****************************************************************/
/*                                                              */
/*             Serial Peripheral Interface (SPI)                */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t MCR;            /**< DSPI Module Configuration Register, offset: 0x0 */
       uint8_t  RESERVED0[1];
  __IO uint32_t TCR;            /**< DSPI Transfer Count Register, offset: 0x8 */
  union {                       /* offset: 0xC */
    __IO uint32_t CTAR[2];        /**< DSPI Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];  /**< DSPI Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint32_t RESERVED1[6];
  __IO uint32_t SR;             /**< DSPI Status Register, offset: 0x2C */
  __IO uint32_t RSER;           /**< DSPI DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                       /* offset: 0x34 */
    __IO uint32_t PUSHR;        /**< DSPI PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;  /**< DSPI PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;           /**< DSPI POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR[4];        /**< DSPI Transmit FIFO Registers, offset: 0x3C */
       uint32_t RESERVED2[12];
  __I  uint32_t RXFR[4];        /**< DSPI Receive FIFO Registers, offset: 0x7C */
} SPI_TypeDef;

#define SPIx_MCR_MSTR                ((uint32_t)0x80000000)
#define SPIx_MCR_CONT_SCKE           ((uint32_t)0x40000000)
#define SPIx_MCR_DCONF_SHIFT         28
#define SPIx_MCR_DCONF_MASK          ((uint32_t)0x30000000)
#define SPIx_MCR_DCONF(x)            ((uint32_t)(((uint32_t)(x) << SPIx_MCR_DCONF_SHIFT) & SPIx_MCR_DCONF_MASK))
#define SPIx_MCR_FRZ                 ((uint32_t)0x08000000)
#define SPIx_MCR_MTFE                ((uint32_t)0x04000000)
#define SPIx_MCR_PCSSE               ((uint32_t)0x02000000)
#define SPIx_MCR_ROOE                ((uint32_t)0x01000000)
#define SPIx_MCR_PCSIS_SHIFT         16
#define SPIx_MCR_PCSIS_MASK          ((uint32_t)0x003F0000)
#define SPIx_MCR_PCSIS(x)            ((uint32_t)(((uint32_t)(x) << SPIx_MCR_PCSIS_SHIFT) & SPIx_MCR_PCSIS_MASK))
#define SPIx_MCR_DOZE                ((uint32_t)0x00008000)
#define SPIx_MCR_MDIS                ((uint32_t)0x00004000)
#define SPIx_MCR_DIS_TXF             ((uint32_t)0x00002000)
#define SPIx_MCR_DIS_RXF             ((uint32_t)0x00001000)
#define SPIx_MCR_CLR_TXF             ((uint32_t)0x00000800)
#define SPIx_MCR_CLR_RXF             ((uint32_t)0x00000400)
#define SPIx_MCR_SMPL_PT             ((uint32_t)0x00000300)
#define SPIx_MCR_HALT                ((uint32_t)0x00000001)

#define SPIx_TCR_SPIx_TCNT_SHIFT      26
#define SPIx_TCR_SPIx_TCNT_MASK       ((uint32_t)0xFFFF0000)
#define SPIx_TCR_SPIx_TCNT(x)         ((uint32_t)(((uint32_t)(x) << SPIx_TCR_SPIx_TCNT_SHIFT) & SPIx_TCR_SPIx_TCNT_MASK))

#define SPIx_CTARn_DBR               ((uint32_t)0x80000000)
#define SPIx_CTARn_FMSZ_SHIFT        27
#define SPIx_CTARn_FMSZ_MASK         ((uint32_t)0x78000000)
#define SPIx_CTARn_FMSZ(x)           ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_FMSZ_SHIFT) & SPIx_CTARn_FMSZ_MASK))
#define SPIx_CTARn_CPOL              ((uint32_t)0x04000000)
#define SPIx_CTARn_CPHA              ((uint32_t)0x02000000)
#define SPIx_CTARn_LSBFE             ((uint32_t)0x01000000)
#define SPIx_CTARn_PCSSCK_SHIFT      22
#define SPIx_CTARn_PCSSCK_MASK       ((uint32_t)0x00C00000)
#define SPIx_CTARn_PCSSCK(x)         ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_PCSSCK_SHIFT) & SPIx_CTARn_PCSSCK_MASK))
#define SPIx_CTARn_PASC_SHIFT        20
#define SPIx_CTARn_PASC_MASK         ((uint32_t)0x00300000)
#define SPIx_CTARn_PASC(x)           ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_PASC_SHIFT) & SPIx_CTARn_PASC_MASK))
#define SPIx_CTARn_PDT_SHIFT         18
#define SPIx_CTARn_PDT_MASK          ((uint32_t)0x000C0000)
#define SPIx_CTARn_PDT(x)            ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_PDT_SHIFT) & SPIx_CTARn_PDT_MASK))
#define SPIx_CTARn_PBR_SHIFT         16
#define SPIx_CTARn_PBR_MASK          ((uint32_t)0x00030000)
#define SPIx_CTARn_PBR(x)            ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_PBR_SHIFT) & SPIx_CTARn_PBR_MASK))
#define SPIx_CTARn_CSSCK_SHIFT       12
#define SPIx_CTARn_CSSCK_MASK        ((uint32_t)0x0000F000)
#define SPIx_CTARn_CSSCK(x)          ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_CSSCK_SHIFT) & SPIx_CTARn_CSSCK_MASK))
#define SPIx_CTARn_ASC_SHIFT         8
#define SPIx_CTARn_ASC_MASK          ((uint32_t)0x00000F00)
#define SPIx_CTARn_ASC(x)            ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_ASC_SHIFT) & SPIx_CTARn_ASC_MASK))
#define SPIx_CTARn_DT_SHIFT          4
#define SPIx_CTARn_DT_MASK           ((uint32_t)0x000000F0)
#define SPIx_CTARn_DT(x)             ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_DT_SHIFT) & SPIx_CTARn_DT_MASK))
#define SPIx_CTARn_BR_SHIFT          0
#define SPIx_CTARn_BR_MASK           ((uint32_t)0x0000000F)
#define SPIx_CTARn_BR(x)             ((uint32_t)(((uint32_t)(x) << SPIx_CTARn_BR_SHIFT) & SPIx_CTARn_BR_MASK))

#define SPIx_SR_TCF                  ((uint32_t)0x80000000)
#define SPIx_SR_TXRXS                ((uint32_t)0x40000000)
#define SPIx_SR_EOQF                 ((uint32_t)0x10000000)
#define SPIx_SR_TFUF                 ((uint32_t)0x08000000)
#define SPIx_SR_TFFF                 ((uint32_t)0x02000000)
#define SPIx_SR_RFOF                 ((uint32_t)0x00080000)
#define SPIx_SR_RFDF                 ((uint32_t)0x00020000)
#define SPIx_SR_TXCTR_SHIFT          12
#define SPIx_SR_TXCTR_MASK           ((uint32_t)0x0000F000)
#define SPIx_SR_TXCTR(x)             ((uint32_t)(((uint32_t)(x) << SPIx_SR_TXCTR_SHIFT) & SPIx_SR_TXCTR_MASK))
#define SPIx_SR_TXNXTPTR_SHIFT       8
#define SPIx_SR_TXNXTPTR_MASK        ((uint32_t)0x00000F00)
#define SPIx_SR_TXNXTPTR(x)          ((uint32_t)(((uint32_t)(x) << SPIx_SR_TXNXTPTR_SHIFT) & SPIx_SR_TXNXTPTR_MASK))
#define SPIx_SR_RXCTR_SHIFT          4
#define SPIx_SR_RXCTR_MASK           ((uint32_t)0x000000F0)
#define SPIx_SR_RXCTR(x)             ((uint32_t)(((uint32_t)(x) << SPIx_SR_RXCTR_SHIFT) & SPIx_SR_RXCTR_MASK))
#define SPIx_SR_POPNXTPTR_SHIFT      0
#define SPIx_SR_POPNXTPTR_MASK       ((uint32_t)0x0000000F)
#define SPIx_SR_POPNXTPTR(x)         ((uint32_t)(((uint32_t)(x) << SPIx_SR_POPNXTPTR_SHIFT) & SPIx_SR_POPNXTPTR_MASK))

#define SPIx_RSER_RFDF_DIRS          ((uint32_t)0x00010000)
#define SPIx_RSER_RFDF_RE            ((uint32_t)0x00020000)
#define SPIx_RSER_RFOF_RE            ((uint32_t)0x00080000)
#define SPIx_RSER_TFFF_DIRS          ((uint32_t)0x01000000)
#define SPIx_RSER_TFFF_RE            ((uint32_t)0x02000000)
#define SPIx_RSER_TFUF_RE            ((uint32_t)0x08000000)
#define SPIx_RSER_EOQF_RE            ((uint32_t)0x10000000)
#define SPIx_RSER_TCF_RE             ((uint32_t)0x80000000)

#define SPIx_PUSHR_CONT              ((uint32_t)0x80000000)
#define SPIx_PUSHR_CTAS_SHIFT        28
#define SPIx_PUSHR_CTAS_MASK         ((uint32_t)0x70000000)
#define SPIx_PUSHR_CTAS(x)           ((uint32_t)(((uint32_t)(x) << SPIx_PUSHR_CTAS_SHIFT) & SPIx_PUSHR_CTAS_MASK))
#define SPIx_PUSHR_EOQ               ((uint32_t)0x08000000)
#define SPIx_PUSHR_CTCNT             ((uint32_t)0x04000000)
#define SPIx_PUSHR_PCS_MASK          ((uint32_t)0x003F0000)
#define SPIx_PUSHR_PCS_SHIFT         16
#define SPIx_PUSHR_PCS(x)            ((uint32_t)(((uint32_t)(x) << SPIx_PUSHR_PCS_SHIFT) && SPIx_PUSHR_PCS_MASK))
#define SPIx_PUSHR_TXDATA_SHIFT      0
#define SPIx_PUSHR_TXDATA_MASK       ((uint32_t)0x0000FFFF)
#define SPIx_PUSHR_TXDATA(x)         ((uint32_t)(((uint32_t)(x) << SPIx_PUSHR_TXDATA_SHIFT) & SPIx_PUSHR_TXDATA_MASK))

typedef struct
{
  __IO uint8_t  A1;
  __IO uint8_t  F;
  __IO uint8_t  C1;
  __IO uint8_t  S;
  __IO uint8_t  D;
  __IO uint8_t  C2;
  __IO uint8_t  FLT;
  __IO uint8_t  RA;
  __IO uint8_t  SMB;
  __IO uint8_t  A2;
  __IO uint8_t  SLTH;
  __IO uint8_t  SLTL;
} I2C_TypeDef;

#define I2Cx_C1_DMAEN                ((uint8_t)0x01)
#define I2Cx_C1_WUEN                 ((uint8_t)0x02)
#define I2Cx_C1_RSTA                 ((uint8_t)0x04)
#define I2Cx_C1_TXAK                 ((uint8_t)0x08)
#define I2Cx_C1_TX                   ((uint8_t)0x10)
#define I2Cx_C1_MST                  ((uint8_t)0x20)
#define I2Cx_C1_IICIE                ((uint8_t)0x40)
#define I2Cx_C1_IICEN                ((uint8_t)0x80)

#define I2Cx_S_TCF                   ((uint8_t)0x80)
#define I2Cx_S_IAAS                  ((uint8_t)0x40)
#define I2Cx_S_BUSY                  ((uint8_t)0x20)
#define I2Cx_S_ARBL                  ((uint8_t)0x10)
#define I2Cx_S_RAM                   ((uint8_t)0x08)
#define I2Cx_S_SRW                   ((uint8_t)0x04)
#define I2Cx_S_IICIF                 ((uint8_t)0x02)
#define I2Cx_S_RXAK                  ((uint8_t)0x01)

typedef struct
{
  __IO uint8_t BDH;
  __IO uint8_t BDL;
  __IO uint8_t C1;
  __IO uint8_t C2;
  __I  uint8_t S1;
  __IO uint8_t S2;
  __IO uint8_t C3;
  __IO uint8_t D;
  __IO uint8_t MA1;
  __IO uint8_t MA2;
  __IO uint8_t C4;
  __IO uint8_t C5;
  __I  uint8_t ED;
  __IO uint8_t MODEM;
  __IO uint8_t IR;
       uint8_t RESERVED0[1];
  __IO uint8_t PFIFO;
  __IO uint8_t CFIFO;
  __IO uint8_t SFIFO;
  __IO uint8_t TWFIFO;
  __I  uint8_t TCFIFO;
  __IO uint8_t RWFIFO;
  __I  uint8_t RCFIFO;
       uint8_t RESERVED1[1];
  __IO uint8_t C7816;
  __IO uint8_t IE7816;
  __IO uint8_t IS7816;
  union {
    __IO uint8_t WP7816T0;
    __IO uint8_t WP7816T1;
  };
  __IO uint8_t WN7816;
  __IO uint8_t WF7816;
  __IO uint8_t ET7816;
  __IO uint8_t TL7816;
       uint8_t RESERVED2[2];
  __IO uint8_t C6;
  __IO uint8_t PCTH;
  __IO uint8_t PCTL;
  __IO uint8_t B1T;
  __IO uint8_t SDTH;
  __IO uint8_t SDTL;
  __IO uint8_t PRE;
  __IO uint8_t TPL;
  __IO uint8_t IE;
  __IO uint8_t WB;
  __IO uint8_t S3;
  __IO uint8_t S4;
  __I  uint8_t RPL;
  __I  uint8_t RPREL;
  __IO uint8_t CPW;
  __IO uint8_t RIDT;
  __IO uint8_t TIDT;
} UART_TypeDef;

/****************************************************************/
/*                                                              */
/*     Universal Asynchronous Receiver/Transmitter (UART)       */
/*                                                              */
/****************************************************************/
/*********  Bits definition for UARTx_BDH register  *************/
#define UARTx_BDH_LBKDIE             ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Enable */
#define UARTx_BDH_RXEDGIE            ((uint8_t)0x40)    /*!< RxD Input Active Edge Interrupt Enable */
#define UARTx_BDH_SBR_MASK           ((uint8_t)0x1F)
#define UARTx_BDH_SBR(x)             ((uint8_t)((uint8_t)(x) & UARTx_BDH_SBR_MASK))  /*!< Baud Rate Modulo Divisor */

/*********  Bits definition for UARTx_BDL register  *************/
#define UARTx_BDL_SBR_SHIFT          0                  /*!< Baud Rate Modulo Divisor */
#define UARTx_BDL_SBR_MASK           ((uint8_t)((uint8_t)0xFF << UARTx_BDL_SBR_SHIFT))
#define UARTx_BDL_SBR(x)             ((uint8_t)(((uint8_t)(x) << UARTx_BDL_SBR_SHIFT) & UARTx_BDL_SBR_MASK))

/*********  Bits definition for UARTx_C1 register  **************/
#define UARTx_C1_LOOPS               ((uint8_t)0x80)    /*!< Loop Mode Select */
#define UARTx_C1_DOZEEN              ((uint8_t)0x40)    /*!< Doze Enable */
#define UARTx_C1_UARTSWAI            ((uint8_t)0x40)    /*!< UART Stops in Wait Mode */
#define UARTx_C1_RSRC                ((uint8_t)0x20)    /*!< Receiver Source Select */
#define UARTx_C1_M                   ((uint8_t)0x10)    /*!< 9-Bit or 8-Bit Mode Select */
#define UARTx_C1_WAKE                ((uint8_t)0x08)    /*!< Receiver Wakeup Method Select */
#define UARTx_C1_ILT                 ((uint8_t)0x04)    /*!< Idle Line Type Select */
#define UARTx_C1_PE                  ((uint8_t)0x02)    /*!< Parity Enable */
#define UARTx_C1_PT                  ((uint8_t)0x01)    /*!< Parity Type */

/*********  Bits definition for UARTx_C2 register  **************/
#define UARTx_C2_TIE                 ((uint8_t)0x80)    /*!< Transmit Interrupt Enable for TDRE */
#define UARTx_C2_TCIE                ((uint8_t)0x40)    /*!< Transmission Complete Interrupt Enable for TC */
#define UARTx_C2_RIE                 ((uint8_t)0x20)    /*!< Receiver Interrupt Enable for RDRF */
#define UARTx_C2_ILIE                ((uint8_t)0x10)    /*!< Idle Line Interrupt Enable for IDLE */
#define UARTx_C2_TE                  ((uint8_t)0x08)    /*!< Transmitter Enable */
#define UARTx_C2_RE                  ((uint8_t)0x04)    /*!< Receiver Enable */
#define UARTx_C2_RWU                 ((uint8_t)0x02)    /*!< Receiver Wakeup Control */
#define UARTx_C2_SBK                 ((uint8_t)0x01)    /*!< Send Break */

/*********  Bits definition for UARTx_S1 register  **************/
#define UARTx_S1_TDRE                ((uint8_t)0x80)    /*!< Transmit Data Register Empty Flag */
#define UARTx_S1_TC                  ((uint8_t)0x40)    /*!< Transmission Complete Flag */
#define UARTx_S1_RDRF                ((uint8_t)0x20)    /*!< Receiver Data Register Full Flag */
#define UARTx_S1_IDLE                ((uint8_t)0x10)    /*!< Idle Line Flag */
#define UARTx_S1_OR                  ((uint8_t)0x08)    /*!< Receiver Overrun Flag */
#define UARTx_S1_NF                  ((uint8_t)0x04)    /*!< Noise Flag */
#define UARTx_S1_FE                  ((uint8_t)0x02)    /*!< Framing Error Flag */
#define UARTx_S1_PF                  ((uint8_t)0x01)    /*!< Parity Error Flag */

/*********  Bits definition for UARTx_S2 register  **************/
#define UARTx_S2_LBKDIF              ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Flag */
#define UARTx_S2_RXEDGIF             ((uint8_t)0x40)    /*!< UART_RX Pin Active Edge Interrupt Flag */
#define UARTx_S2_MSBF                ((uint8_t)0x20)    /*!< MSB First */
#define UARTx_S2_RXINV               ((uint8_t)0x10)    /*!< Receive Data Inversion */
#define UARTx_S2_RWUID               ((uint8_t)0x08)    /*!< Receive Wake Up Idle Detect */
#define UARTx_S2_BRK13               ((uint8_t)0x04)    /*!< Break Character Generation Length */
#define UARTx_S2_LBKDE               ((uint8_t)0x02)    /*!< LIN Break Detect Enable */
#define UARTx_S2_RAF                 ((uint8_t)0x01)    /*!< Receiver Active Flag */

/*********  Bits definition for UARTx_C3 register  **************/
#define UARTx_C3_R8                  ((uint8_t)0x80)    /*!< Ninth Data Bit for Receiver */
#define UARTx_C3_T8                  ((uint8_t)0x40)    /*!< Ninth Data Bit for Transmitter */
#define UARTx_C3_TXDIR               ((uint8_t)0x20)    /*!< UART_TX Pin Direction in Single-Wire Mode */
#define UARTx_C3_TXINV               ((uint8_t)0x10)    /*!< Transmit Data Inversion */
#define UARTx_C3_ORIE                ((uint8_t)0x08)    /*!< Overrun Interrupt Enable */
#define UARTx_C3_NEIE                ((uint8_t)0x04)    /*!< Noise Error Interrupt Enable */
#define UARTx_C3_FEIE                ((uint8_t)0x02)    /*!< Framing Error Interrupt Enable */
#define UARTx_C3_PEIE                ((uint8_t)0x01)    /*!< Parity Error Interrupt Enable */

/*********  Bits definition for UARTx_D register  ***************/
#define UARTx_D_R7T7                 ((uint8_t)0x80)    /*!< Read receive data buffer 7 or write transmit data buffer 7 */
#define UARTx_D_R6T6                 ((uint8_t)0x40)    /*!< Read receive data buffer 6 or write transmit data buffer 6 */
#define UARTx_D_R5T5                 ((uint8_t)0x20)    /*!< Read receive data buffer 5 or write transmit data buffer 5 */
#define UARTx_D_R4T4                 ((uint8_t)0x10)    /*!< Read receive data buffer 4 or write transmit data buffer 4 */
#define UARTx_D_R3T3                 ((uint8_t)0x08)    /*!< Read receive data buffer 3 or write transmit data buffer 3 */
#define UARTx_D_R2T2                 ((uint8_t)0x04)    /*!< Read receive data buffer 2 or write transmit data buffer 2 */
#define UARTx_D_R1T1                 ((uint8_t)0x02)    /*!< Read receive data buffer 1 or write transmit data buffer 1 */
#define UARTx_D_R0T0                 ((uint8_t)0x01)    /*!< Read receive data buffer 0 or write transmit data buffer 0 */

/*********  Bits definition for UARTx_MA1 register  *************/
#define UARTx_MA1_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_MA2 register  *************/
#define UARTx_MA2_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_C4 register  **************/
#define UARTx_C4_MAEN1               ((uint8_t)0x80)    /*!< Match Address Mode Enable 1 */
#define UARTx_C4_MAEN2               ((uint8_t)0x40)    /*!< Match Address Mode Enable 2 */
#define UARTx_C4_M10                 ((uint8_t)0x20)    /*!< 10-bit Mode Select */
#define UARTx_C4_BRFA_MASK           ((uint8_t)0x1F)
#define UARTx_C4_BRFA(x)             ((uint8_t)((uint8_t)(x) & UARTx_C4_BRFA_MASK))  /*!< Baud Rate Fine Adjust */

/*********  Bits definition for UARTx_C5 register  **************/
#define UARTx_C5_TDMAE               ((uint8_t)0x80)    /*!< Transmitter DMA Enable */
#define UARTx_C5_TCDMAE              ((uint8_t)0x40)
#define UARTx_C5_RDMAE               ((uint8_t)0x20)    /*!< Receiver Full DMA Enable */
#define UARTx_C5_ILDMAE              ((uint8_t)0x10)
#define UARTx_C5_LBKDDMAE            ((uint8_t)0x08)

/*********  Bits definition for UARTx_ED register  **************/
#define UARTx_ED_NOISY               ((uint8_t)0x80)
#define UARTx_ED_PARITYE             ((uint8_t)0x40)

/*******  Bits definition for UARTx_MODEM register  *************/
#define UARTx_MODEM_RXRTSE           ((uint8_t)0x08)
#define UARTx_MODEM_TXRTSPOL         ((uint8_t)0x04)
#define UARTx_MODEM_TXRTSE           ((uint8_t)0x02)
#define UARTx_MODEM_TXCTSE           ((uint8_t)0x01)

/********  Bits definition for UARTx_IR register  **************/
#define UARTx_IR_IREN                ((uint8_t)0x04)
#define UARTx_IR_TNP_MASK            ((uint8_t)0x03)
#define UARTx_IR_TNP(x)              ((uint8_t)((uint8_t)(x)&UART_IR_RNP_MASK))

/*******  Bits definition for UARTx_PFIFO register  ************/
#define UARTx_PFIFO_TXFE             ((uint8_t)0x80)    /*!< Transmit FIFO Enable */
#define UARTx_PFIFO_TXFIFOSIZE_SHIFT 4
#define UARTx_PFIFO_TXFIFOSIZE_MASK  ((uint8_t)0x70)
#define UARTx_PFIFO_TXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_TXFIFOSIZE_SHIFT) & UARTx_PFIFO_TXFIFOSIZE_MASK))  /*!< Transmit FIFO Buffer depth */
#define UARTx_PFIFO_RXFE             ((uint8_t)0x08)    /*!< Receive FIFOh */
#define UARTx_PFIFO_RXFIFOSIZE_SHIFT 0
#define UARTx_PFIFO_RXFIFOSIZE_MASK  ((uint8_t)0x07)
#define UARTx_PFIFO_RXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_RXFIFOSIZE_SHIFT) & UARTx_PFIFO_RXFIFOSIZE_MASK))  /*!< Receive FIFO Buffer depth */

/*******  Bits definition for UARTx_CFIFO register  ************/
#define UARTx_CFIFO_TXFLUSH          ((uint8_t)0x80)    /*!< Transmit FIFO/Buffer Flush */
#define UARTx_CFIFO_RXFLUSH          ((uint8_t)0x40)    /*!< Receive FIFO/Buffer Flush */
#define UARTx_CFIFO_RXOFE            ((uint8_t)0x04)    /*!< Receive FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_TXOFE            ((uint8_t)0x02)    /*!< Transmit FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_RXUFE            ((uint8_t)0x01)    /*!< Receive FIFO Underflow Interrupt Enable */

/*******  Bits definition for UARTx_SFIFO register  ************/
#define UARTx_SFIFO_TXEMPT           ((uint8_t)0x80)
#define UARTx_SFIFO_RXEMPT           ((uint8_t)0x40)
#define UARTx_SFIFO_RXOF             ((uint8_t)0x04)
#define UARTx_SFIFO_TXOF             ((uint8_t)0x02)
#define UARTx_SFIFO_RXUF             ((uint8_t)0x01)

/*******  Bits definition for UARTx_C7816 register  ************/
#define UARTx_C7816_ONACK            ((uint8_t)0x10)
#define UARTx_C7816_ANACK            ((uint8_t)0x08)
#define UARTx_C7816_INIT             ((uint8_t)0x04)
#define UARTx_C7816_TTYPE            ((uint8_t)0x02)
#define UARTx_C7816_ISO_7816E        ((uint8_t)0x01)

/******** Bits definition for UARTx_IE76816 register  **********/
#define UARTx_IE7816_WTE             ((uint8_t)0x80)
#define UARTx_IE7816_CWTE            ((uint8_t)0x40)
#define UARTx_IE7816_BWTE            ((uint8_t)0x20)
#define UARTx_IE7816_INITDE          ((uint8_t)0x10)
#define UARTx_IE7816_GTVE            ((uint8_t)0x04)
#define UARTx_IE7816_TXTE            ((uint8_t)0x02)
#define UARTx_IE7816_RXTE            ((uint8_t)0x01)

/******** Bits definition for UARTx_IS76816 register  **********/
#define UARTx_IS7816_WT              ((uint8_t)0x80)
#define UARTx_IS7816_CWT             ((uint8_t)0x40)
#define UARTx_IS7816_BWT             ((uint8_t)0x20)
#define UARTx_IS7816_INITD           ((uint8_t)0x10)
#define UARTx_IS7816_GTV             ((uint8_t)0x04)
#define UARTx_IS7816_TXT             ((uint8_t)0x02)
#define UARTx_IS7816_RXT             ((uint8_t)0x01)

/******** Bits definition for UARTx_ET76816 register  **********/
#define UARTx_ET7816_TXTHRESHOLD_SHIFT 4
#define UARTx_ET7816_TXTHRESHOLD_MASK  0xF0
#define UARTx_ET7816_TXTHRESHOLD(x)    ((uint8_t)(((uint8_t)(x) << UARTx_ET7816_TXTHRESHOLD_SHIFT) & UARTx_TXTHRESHOLD_MASK))
#define UARTx_ET7816_RXTHRESHOLD_SHIFT 0
#define UARTx_ET7816_RXTHRESHOLD_MASK  0x0F
#define UARTx_ET7816_RXTHRESHOLD(x)    ((uint8_t)(((uint8_t)(x) << UARTx_ET7816_RXTHRESHOLD_SHIFT) & UARTx_RXTHRESHOLD_MASK))

/****************************************************************/
/*                                                              */
/*             Power Management Controller (PMC)                */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint8_t LVDSC1;
  __IO uint8_t LVDSC2;
  __IO uint8_t REGSC;
} PMC_TypeDef;

#define PMC_LVDSC1_LVDF             ((uint8_t)0x80)
#define PMC_LVDSC1_LVDACK           ((uint8_t)0x40)
#define PMC_LVDSC1_LVDIE            ((uint8_t)0x20)
#define PMC_LVDSC1_LVDRE            ((uint8_t)0x10)
#define PMC_LVDSC1_LVDV_MASK        ((uint8_t)0x03)
#define PMC_LVDSC1_LVDV(x)          ((uint8_t)((uint8_t)(x)&PMC_LVDSC1_LVDV_MASK))

#define PMC_LVDSC2_LVWF             ((uint8_t)0x80)
#define PMC_LVDSC2_LVWACK           ((uint8_t)0x40)
#define PMC_LVDSC2_LVWIE            ((uint8_t)0x20)
#define PMC_LVDSC2_LVDV_MASK        ((uint8_t)0x03)
#define PMC_LVDSC2_LVDV(x)          ((uint8_t)((uint8_t)(x)&PMC_LVDSC2_LVDV_MASK))

#define PMC_REGSC_BGEN              ((uint8_t)0x10)
#define PMC_REGSC_ACKISO            ((uint8_t)0x08)
#define PMC_REGSC_REGONS            ((uint8_t)0x04)
#define PMC_REGSC_BGBE              ((uint8_t)0x01)

/****************************************************************/
/*                                                              */
/*                        Watchdog (WDOG)                       */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint16_t STCTRLH;
  __IO uint16_t STCTRLL;
  __IO uint16_t TOVALH;
  __IO uint16_t TOVALL;
  __IO uint16_t WINH;
  __IO uint16_t WINL;
  __IO uint16_t REFRESH;
  __IO uint16_t UNLOCK;
  __IO uint16_t TMROUTH;
  __IO uint16_t TMROUTL;
  __IO uint16_t RSTCNT;
  __IO uint16_t PRESC;
} WDOG_TypeDef;

#define WDOG_STCTRLH_DISTESTWDOG    ((uint16_t)0x4000)
#define WDOG_STCTRLH_BYTESEL_1_0    ((uint16_t)0x3000)
#define WDOG_STCTRLH_TESTSEL        ((uint16_t)0x0800)
#define WDOG_STCTRLH_TESTWDOG       ((uint16_t)0x0400)
#define WDOG_STCTRLH_WAITEN         ((uint16_t)0x0080)
#define WDOG_STCTRLH_STOPEN         ((uint16_t)0x0040)
#define WDOG_STCTRLH_DBGEN          ((uint16_t)0x0020)
#define WDOG_STCTRLH_ALLOWUPDATE    ((uint16_t)0x0010)
#define WDOG_STCTRLH_WINEN          ((uint16_t)0x0008)
#define WDOG_STCTRLH_IRQRSTEN       ((uint16_t)0x0004)
#define WDOG_STCTRLH_CLKSRC         ((uint16_t)0x0002)
#define WDOG_STCTRLH_WDOGEN         ((uint16_t)0x0001)

#define WDOG_STCTRLL_INTFLG         ((uint16_t)0x8000)

#define WDOG_PRESC_PRESCVAL         ((uint16_t)0x0700)

/****************************************************************/
/*                                                              */
/*          Universal Serial Bus On-The-Go (USB_OTG)            */
/*                                                              */
/****************************************************************/
typedef struct
{
  __I  uint8_t PERID;
       uint8_t RESERVED0[3];
  __I  uint8_t IDCOMP;
       uint8_t RESERVED1[3];
  __I  uint8_t REV;
       uint8_t RESERVED2[3];
  __I  uint8_t ADDINFO;
       uint8_t RESERVED3[3];
  __IO uint8_t OTGISTAT;
       uint8_t RESERVED4[3];
  __IO uint8_t OTGICR;
       uint8_t RESERVED5[3];
  __IO uint8_t OTGSTAT;
       uint8_t RESERVED6[3];
  __IO uint8_t OTGCTL;
       uint8_t RESERVED7[99];
  __IO uint8_t ISTAT;
       uint8_t RESERVED8[3];
  __IO uint8_t INTEN;
       uint8_t RESERVED9[3];
  __IO uint8_t ERRSTAT;
       uint8_t RESERVED10[3];
  __IO uint8_t ERREN;
       uint8_t RESERVED11[3];
  __I  uint8_t STAT;
       uint8_t RESERVED12[3];
  __IO uint8_t CTL;
       uint8_t RESERVED13[3];
  __IO uint8_t ADDR;
       uint8_t RESERVED14[3];
  __IO uint8_t BDTPAGE1;
       uint8_t RESERVED15[3];
  __IO uint8_t FRMNUML;
       uint8_t RESERVED16[3];
  __IO uint8_t FRMNUMH;
       uint8_t RESERVED17[3];
  __IO uint8_t TOKEN;
       uint8_t RESERVED18[3];
  __IO uint8_t SOFTHLD;
       uint8_t RESERVED19[3];
  __IO uint8_t BDTPAGE2;
       uint8_t RESERVED20[3];
  __IO uint8_t BDTPAGE3;
       uint8_t RESERVED21[11];
  struct {
    __IO uint8_t V;
         uint8_t RESERVED0[3];
  } ENDPT[16];
  __IO uint8_t USBCTRL;
       uint8_t RESERVED22[3];
  __I  uint8_t OBSERVE;
       uint8_t RESERVED23[3];
  __IO uint8_t CONTROL;
       uint8_t RESERVED24[3];
  __IO uint8_t USBTRC0;
       uint8_t RESERVED25[7];
  __IO uint8_t USBFRMADJUST;
       uint8_t RESERVED26[43];
  __IO uint8_t CLK_RECOVER_CTRL;
       uint8_t RESERVED27[3];
  __IO uint8_t CLK_RECOVER_IRC_EN;
       uint8_t RESERVED28[23];
  __IO uint8_t CLK_RECOVER_INT_STATUS;
} USBOTG_TypeDef;

#define USBx_ISTAT_STALL            ((uint8_t)0x80)
#define USBx_ISTAT_ATTACH           ((uint8_t)0x40)
#define USBx_ISTAT_RESUME           ((uint8_t)0x20)
#define USBx_ISTAT_SLEEP            ((uint8_t)0x10)
#define USBx_ISTAT_TOKDNE           ((uint8_t)0x08)
#define USBx_ISTAT_SOFTOK           ((uint8_t)0x04)
#define USBx_ISTAT_ERROR            ((uint8_t)0x02)
#define USBx_ISTAT_USBRST           ((uint8_t)0x01)

#define USBx_INTEN_STALLEN          ((uint8_t)0x80)
#define USBx_INTEN_ATTACHEN         ((uint8_t)0x40)
#define USBx_INTEN_RESUMEEN         ((uint8_t)0x20)
#define USBx_INTEN_SLEEPEN          ((uint8_t)0x10)
#define USBx_INTEN_TOKDNEEN         ((uint8_t)0x08)
#define USBx_INTEN_SOFTOKEN         ((uint8_t)0x04)
#define USBx_INTEN_ERROREN          ((uint8_t)0x02)
#define USBx_INTEN_USBRSTEN         ((uint8_t)0x01)

#define USBx_ENDPTn_HOSTWOHUB       ((uint8_t)0x80)
#define USBx_ENDPTn_RETRYDIS        ((uint8_t)0x40)
#define USBx_ENDPTn_EPCTLDIS        ((uint8_t)0x10)
#define USBx_ENDPTn_EPRXEN          ((uint8_t)0x08)
#define USBx_ENDPTn_EPTXEN          ((uint8_t)0x04)
#define USBx_ENDPTn_EPSTALL         ((uint8_t)0x02)
#define USBx_ENDPTn_EPHSHK          ((uint8_t)0x01)

#define USBx_STAT_ENDP_SHIFT        4
#define USBx_STAT_ENDP_MASK         ((uint8_t)0xF0)
#define USBx_STAT_ENDP(x)           ((uint8_t)(((uint8_t)(x) << USBx_STAT_ENDP_SHIFT) & USBx_STAT_ENDP_MASK))
#define USBx_STAT_TX_SHIFT          3
#define USBx_STAT_TX_MASK           ((uint8_t)0x08)
#define USBx_STAT_ODD_SHIFT         2
#define USBx_STAT_ODD_MASK          ((uint8_t)0x04)

#define USBx_CTL_JSTATE             ((uint8_t)0x80)
#define USBx_CTL_SE0                ((uint8_t)0x40)
#define USBx_CTL_TXSUSPENDTOKENBUSY ((uint8_t)0x20)
#define USBx_CTL_RESET              ((uint8_t)0x10)
#define USBx_CTL_HOSTMODEEN         ((uint8_t)0x08)
#define USBx_CTL_RESUME             ((uint8_t)0x04)
#define USBx_CTL_ODDRST             ((uint8_t)0x02)
#define USBx_CTL_USBENSOFEN         ((uint8_t)0x01)

#define USBx_USBTRC0_USBRESET       ((uint8_t)0x80)
#define USBx_USBTRC0_USBRESMEN      ((uint8_t)0x20)
#define USBx_USBTRC0_USB_CLK_RECOVERY_INT ((uint8_t)0x04)
#define USBx_USBTRC0_SYNC_DET       ((uint8_t)0x02)
#define USBx_USBTRC0_USB_RESUME_INT ((uint8_t)0x01)

#define USBx_CONTROL_DPPULLUPNONOTG ((uint8_t)0x10)

#define USBx_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN    ((uint8_t)0x20)
#define USBx_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN ((uint8_t)0x40)
#define USBx_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN      ((uint8_t)0x80)

#define USBx_CLK_RECOVER_IRC_EN_REG_EN              ((uint8_t)0x01)
#define USBx_CLK_RECOVER_IRC_EN_IRC_EN              ((uint8_t)0x02)

#define USBx_CLK_RECOVER_INT_STATUS_OVF_ERROR       ((uint8_t)0x10)

/****************************************************************/
/*                                                              */
/*              Secured Digital Host Controller (SDHC)          */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t DSADDR;
  __IO uint32_t BLKATTR;
  __IO uint32_t CMDARG;
  __IO uint32_t XFERTYP;
  __I  uint32_t CMDRSP[4];
  __IO uint32_t DATPORT;
  __I  uint32_t PRSSTAT;
  __IO uint32_t PROCTL;
  __IO uint32_t SYSCTL;
  __IO uint32_t IRQSTAT;
  __IO uint32_t IRQSTATEN;
  __IO uint32_t IRQSIGEN;
  __I  uint32_t AC12ERR;
  __I  uint32_t HTCAPBLT;
  __IO uint32_t WML;
       uint32_t RESERVED0[2];
  __O  uint32_t FEVT;
  __I  uint32_t ADMAES;
  __IO uint32_t ADSADDR;
       uint32_t RESERVED1[25];
  __IO uint32_t VENDOR;
  __IO uint32_t MMCBOOT;
       uint32_t RESERVED2[13];
  __I  uint32_t HOSTVER;
} SDHC_TypeDef;

#define SDHC_DSADDR_DSADDR_MASK     ((uint32_t)0xFFFFFFFC)
#define SDHC_DSADDR_DSADDR_SHIFT    2
#define SDHC_DSADDR_DSADDR(x)       ((uint32_t)(((uint32_t)(x)<<SDHC_DSADDR_DSADDR_SHIFT)&SDHC_DSADDR_DSADDR_MASK))

#define SDHC_BLKATTR_BLKCNT_SHIFT   16
#define SDHC_BLKATTR_BLKCNT_MASK    ((uint32_t)0xFFFF0000)
#define SDHC_BLKATTR_BLKCNT(x)      ((uint32_t)(((uint32_t)(x)<<SDHC_BLKATTR_BLKCNT_SHIFT)&SDHC_BLKATTR_BLKCNT_MASK))
#define SDHC_BLKATTR_BLKSIZE_SHIFT  0
#define SDHC_BLKATTR_BLKSIZE_MASK   ((uint32_t)0x00001FFF)
#define SDHC_BLKATTR_BLKSIZE(x)     ((uint32_t)(((uint32_t)(x)<<SDHC_BLKATTR_BLKSIZE_SHIFT)&SDHC_BLKATTR_BLKSIZE_MASK))

#define SDHC_XFERTYP_CMDINX_SHIFT   24
#define SDHC_XFERTYP_CMDINX_MASK    ((uint32_t)0x3F000000)
#define SDHC_XFERTYP_CMDINX(x)      ((uint32_t)(((uint32_t)(x)<<SDHC_XFERTYP_CMDINX_SHIFT)&SDHC_XFERTYP_CMDINX_MASK))
#define SDHC_XFERTYP_CMDTYP_SHIFT   22
#define SDHC_XFERTYP_CMDTYP_MASK    ((uint32_t)0x00C00000)
#define SDHC_XFERTYP_CMDTYP(x)      ((uint32_t)(((uint32_t)(x)<<SDHC_XFERTYP_CMDTYP_SHIFT)&SDHC_XFERTYP_CMDTYP_MASK))
#define SDHC_XFERTYP_DPSEL          ((uint32_t)0x00200000)
#define SDHC_XFERTYP_CICEN          ((uint32_t)0x00100000)
#define SDHC_XFERTYP_CCCEN          ((uint32_t)0x00080000)
#define SDHC_XFERTYP_RSPTYP_SHIFT   16
#define SDHC_XFERTYP_RSPTYP_MASK    ((uint32_t)0x00030000)
#define SDHC_XFERTYP_RSPTYP(x)      ((uint32_t)(((uint32_t)(x)<<SDHC_XFERTYP_RSPTYP_SHIFT)&SDHC_XFERTYP_RSPTYP_MASK))
#define SDHC_XFERTYP_MSBSEL         ((uint32_t)0x00000020)
#define SDHC_XFERTYP_DTDSEL         ((uint32_t)0x00000010)
#define SDHC_XFERTYP_AC12EN         ((uint32_t)0x00000004)
#define SDHC_XFERTYP_BCEN           ((uint32_t)0x00000002)
#define SDHC_XFERTYP_DMAEN          ((uint32_t)0x00000001)

#define SDHC_PRSSTAT_DLSL_SHIFT     24
#define SDHC_PRSSTAT_DLSL_MASK      ((uint32_t)0xFF000000)
#define SDHC_PRSSTAT_DLSL(x)        ((uint32_t)(((uint32_t)(x)<<SDHC_PRSSTAT_DLSL_SHIFT)&SDHC_PRSSTAT_DLSL_MASK))
#define SDHC_PRSSTAT_CLSL           ((uint32_t)0x00800000)
#define SDHC_PRSSTAT_CINS           ((uint32_t)0x00010000)
#define SDHC_PRSSTAT_BREN           ((uint32_t)0x00000800)
#define SDHC_PRSSTAT_BWEN           ((uint32_t)0x00000400)
#define SDHC_PRSSTAT_RTA            ((uint32_t)0x00000200)
#define SDHC_PRSSTAT_WTA            ((uint32_t)0x00000100)
#define SDHC_PRSSTAT_SDOFF          ((uint32_t)0x00000080)
#define SDHC_PRSSTAT_PEROFF         ((uint32_t)0x00000040)
#define SDHC_PRSSTAT_HCKOFF         ((uint32_t)0x00000020)
#define SDHC_PRSSTAT_IPGOFF         ((uint32_t)0x00000010)
#define SDHC_PRSSTAT_SDSTB          ((uint32_t)0x00000008)
#define SDHC_PRSSTAT_DLA            ((uint32_t)0x00000004)
#define SDHC_PRSSTAT_CDIHB          ((uint32_t)0x00000002)
#define SDHC_PRSSTAT_CIHB           ((uint32_t)0x00000001)

#define SDHC_PROCTL_WECRM           ((uint32_t)0x04000000)
#define SDHC_PROCTL_WECINS          ((uint32_t)0x02000000)
#define SDHC_PROCTL_WECINT          ((uint32_t)0x01000000)
#define SDHC_PROCTL_IABG            ((uint32_t)0x00080000)
#define SDHC_PROCTL_RWCTL           ((uint32_t)0x00040000)
#define SDHC_PROCTL_CREQ            ((uint32_t)0x00020000)
#define SDHC_PROCTL_SABGREQ         ((uint32_t)0x00010000)
#define SDHC_PROCTL_DMAS_SHIFT      8
#define SDHC_PROCTL_DMAS_MASK       ((uint32_t)0x00000300)
#define SDHC_PROCTL_DMAS(x)         ((uint32_t)(((uint32_t)(x)<<SDHC_PROCTL_DMAS_SHIFT)&SDHC_PROCTL_DMAS_MASK))
#define SDHC_PROCTL_CDSS            ((uint32_t)0x00000080)
#define SDHC_PROCTL_CDTL            ((uint32_t)0x00000040)
#define SDHC_PROCTL_EMODE_SHIFT     4
#define SDHC_PROCTL_EMODE_MASK      ((uint32_t)0x00000030)
#define SDHC_PROCTL_EMODE(x)        ((uint32_t)(((uint32_t)(x)<<SDHC_PROCTL_EMODE_SHIFT)&SDHC_PROCTL_EMODE_MASK))
#define SDHC_PROCTL_D3CD            ((uint32_t)0x00000008)
#define SDHC_PROCTL_DTW_SHIFT       1
#define SDHC_PROCTL_DTW_MASK        ((uint32_t)0x00000006)
#define SDHC_PROCTL_DTW(x)          ((uint32_t)(((uint32_t)(x)<<SDHC_PROCTL_DTW_SHIFT)&SDHC_PROCTL_DTW_MASK))
#define SDHC_PROCTL_LCTL            ((uint32_t)0x00000001)

#define SDHC_SYSCTL_INITA           ((uint32_t)0x08000000)
#define SDHC_SYSCTL_RSTD            ((uint32_t)0x04000000)
#define SDHC_SYSCTL_RSTC            ((uint32_t)0x02000000)
#define SDHC_SYSCTL_RSTA            ((uint32_t)0x01000000)
#define SDHC_SYSCTL_DTOCV_SHIFT     16
#define SDHC_SYSCTL_DTOCV_MASK      ((uint32_t)0x000F0000)
#define SDHC_SYSCTL_DTOCV(x)        ((uint32_t)(((uint32_t)(x)<<SDHC_SYSCTL_DTOCV_SHIFT)&SDHC_SYSCTL_DTOCV_MASK))
#define SDHC_SYSCTL_SDCLKFS_SHIFT   8
#define SDHC_SYSCTL_SDCLKFS_MASK    ((uint32_t)0x0000FF00)
#define SDHC_SYSCTL_SDCLKFS(x)      ((uint32_t)(((uint32_t)(x)<<SDHC_SYSCTL_SDCLKFS_SHIFT)&SDHC_SYSCTL_SDCLKFS_MASK))
#define SDHC_SYSCTL_DVS_SHIFT       4
#define SDHC_SYSCTL_DVS_MASK        ((uint32_t)0x000000F0)
#define SDHC_SYSCTL_DVS(x)          ((uint32_t)(((uint32_t)(x)<<SDHC_SYSCTL_DVS_SHIFT)&SDHC_SYSCTL_DVS_MASK))
#define SDHC_SYSCTL_SDCLKEN         ((uint32_t)0x00000008)
#define SDHC_SYSCTL_PEREN           ((uint32_t)0x00000004)
#define SDHC_SYSCTL_HCKEN           ((uint32_t)0x00000002)
#define SDHC_SYSCTL_IPGEN           ((uint32_t)0x00000001)

#define SDHC_IRQSTAT_DMAE           ((uint32_t)0x10000000)
#define SDHC_IRQSTAT_AC12E          ((uint32_t)0x01000000)
#define SDHC_IRQSTAT_DEBE           ((uint32_t)0x00400000)
#define SDHC_IRQSTAT_DCE            ((uint32_t)0x00200000)
#define SDHC_IRQSTAT_DTOE           ((uint32_t)0x00100000)
#define SDHC_IRQSTAT_CIE            ((uint32_t)0x00080000)
#define SDHC_IRQSTAT_CEBE           ((uint32_t)0x00040000)
#define SDHC_IRQSTAT_CCE            ((uint32_t)0x00020000)
#define SDHC_IRQSTAT_CTOE           ((uint32_t)0x00010000)
#define SDHC_IRQSTAT_CINT           ((uint32_t)0x00000100)
#define SDHC_IRQSTAT_CRM            ((uint32_t)0x00000080)
#define SDHC_IRQSTAT_CINS           ((uint32_t)0x00000040)
#define SDHC_IRQSTAT_BRR            ((uint32_t)0x00000020)
#define SDHC_IRQSTAT_BWR            ((uint32_t)0x00000010)
#define SDHC_IRQSTAT_DINT           ((uint32_t)0x00000008)
#define SDHC_IRQSTAT_BGE            ((uint32_t)0x00000004)
#define SDHC_IRQSTAT_TC             ((uint32_t)0x00000002)
#define SDHC_IRQSTAT_CC             ((uint32_t)0x00000001)

#define SDHC_IRQSTATEN_DMAESEN      ((uint32_t)0x10000000)
#define SDHC_IRQSTATEN_AC12ESEN     ((uint32_t)0x01000000)
#define SDHC_IRQSTATEN_DEBESEN      ((uint32_t)0x00400000)
#define SDHC_IRQSTATEN_DCESEN       ((uint32_t)0x00200000)
#define SDHC_IRQSTATEN_DTOESEN      ((uint32_t)0x00100000)
#define SDHC_IRQSTATEN_CIESEN       ((uint32_t)0x00080000)
#define SDHC_IRQSTATEN_CEBESEN      ((uint32_t)0x00040000)
#define SDHC_IRQSTATEN_CCESEN       ((uint32_t)0x00020000)
#define SDHC_IRQSTATEN_CTOESEN      ((uint32_t)0x00010000)
#define SDHC_IRQSTATEN_CINTSEN      ((uint32_t)0x00000100)
#define SDHC_IRQSTATEN_CRMSEN       ((uint32_t)0x00000080)
#define SDHC_IRQSTATEN_CINSEN       ((uint32_t)0x00000040)
#define SDHC_IRQSTATEN_BRRSEN       ((uint32_t)0x00000020)
#define SDHC_IRQSTATEN_BWRSEN       ((uint32_t)0x00000010)
#define SDHC_IRQSTATEN_DINTSEN      ((uint32_t)0x00000008)
#define SDHC_IRQSTATEN_BGESEN       ((uint32_t)0x00000004)
#define SDHC_IRQSTATEN_TCSEN        ((uint32_t)0x00000002)
#define SDHC_IRQSTATEN_CCSEN        ((uint32_t)0x00000001)

#define SDHC_IRQSIGEN_DMAEIEN       ((uint32_t)0x10000000)
#define SDHC_IRQSIGEN_AC12EIEN      ((uint32_t)0x01000000)
#define SDHC_IRQSIGEN_DEBEIEN       ((uint32_t)0x00400000)
#define SDHC_IRQSIGEN_DCEIEN        ((uint32_t)0x00200000)
#define SDHC_IRQSIGEN_DTOEIEN       ((uint32_t)0x00100000)
#define SDHC_IRQSIGEN_CIEIEN        ((uint32_t)0x00080000)
#define SDHC_IRQSIGEN_CEBEIEN       ((uint32_t)0x00040000)
#define SDHC_IRQSIGEN_CCEIEN        ((uint32_t)0x00020000)
#define SDHC_IRQSIGEN_CTOEIEN       ((uint32_t)0x00010000)
#define SDHC_IRQSIGEN_CINTIEN       ((uint32_t)0x00000100)
#define SDHC_IRQSIGEN_CRMIEN        ((uint32_t)0x00000080)
#define SDHC_IRQSIGEN_CINSIEN       ((uint32_t)0x00000040)
#define SDHC_IRQSIGEN_BRRIEN        ((uint32_t)0x00000020)
#define SDHC_IRQSIGEN_BWRIEN        ((uint32_t)0x00000010)
#define SDHC_IRQSIGEN_DINTIEN       ((uint32_t)0x00000008)
#define SDHC_IRQSIGEN_BGEIEN        ((uint32_t)0x00000004)
#define SDHC_IRQSIGEN_TCIEN         ((uint32_t)0x00000002)
#define SDHC_IRQSIGEN_CCIEN         ((uint32_t)0x00000001)

#define SDHC_AC12ERR_CNIBAC12E      ((uint32_t)0x00000080)
#define SDHC_AC12ERR_AC12IE         ((uint32_t)0x00000010)
#define SDHC_AC12ERR_AC12CE         ((uint32_t)0x00000008)
#define SDHC_AC12ERR_AC12EBE        ((uint32_t)0x00000004)
#define SDHC_AC12ERR_AC12TOE        ((uint32_t)0x00000002)
#define SDHC_AC12ERR_AC12NE         ((uint32_t)0x00000001)

#define SDHC_HTCAPBLT_VS33          ((uint32_t)0x01000000)
#define SDHC_HTCAPBLT_SRS           ((uint32_t)0x00800000)
#define SDHC_HTCAPBLT_DMAS          ((uint32_t)0x00400000)
#define SDHC_HTCAPBLT_HSS           ((uint32_t)0x00200000)
#define SDHC_HTCAPBLT_ADMAS         ((uint32_t)0x00100000)
#define SDHC_HTCAPBLT_MBL_SHIFT     16
#define SDHC_HTCAPBLT_MBL_MASK      ((uint32_t)0x00070000)
#define SDHC_HTCAPBLT_MBL(x)        ((uint32_t)(((uint32_t)(x)<<SDHC_HTCAPBLT_MBL_SHIFT)&SDHC_HTCAPBLT_MBL_MASK))

#define SDHC_WML_WRWML_SHIFT        16
#define SDHC_WML_WRWML_MASK         ((uint32_t)0x00FF0000)
#define SDHC_WML_WRWML(x)           ((uint32_t)(((uint32_t)(x)<<SDHC_WML_WRWML_SHIFT)&SDHC_WML_WRWML_MASK))
#define SDHC_WML_RDWML_SHIFT        0
#define SDHC_WML_RDWML_MASK         ((uint32_t)0x000000FF)
#define SDHC_WML_RDWML(x)           ((uint32_t)(((uint32_t)(x)<<SDHC_WML_RDWML_SHIFT)&SDHC_WML_RDWML_MASK))

#define SDHC_FEVT_CINT              ((uint32_t)0x80000000)
#define SDHC_FEVT_DMAE              ((uint32_t)0x10000000)
#define SDHC_FEVT_AC12E             ((uint32_t)0x01000000)
#define SDHC_FEVT_DEBE              ((uint32_t)0x00400000)
#define SDHC_FEVT_DCE               ((uint32_t)0x00200000)
#define SDHC_FEVT_DTOE              ((uint32_t)0x00100000)
#define SDHC_FEVT_CIE               ((uint32_t)0x00080000)
#define SDHC_FEVT_CEBE              ((uint32_t)0x00040000)
#define SDHC_FEVT_CCE               ((uint32_t)0x00020000)
#define SDHC_FEVT_CTOE              ((uint32_t)0x00010000)
#define SDHC_FEVT_CNIBAC12E         ((uint32_t)0x00000080)
#define SDHC_FEVT_AC12IE            ((uint32_t)0x00000010)
#define SDHC_FEVT_AC12EBE           ((uint32_t)0x00000008)
#define SDHC_FEVT_AC12CE            ((uint32_t)0x00000004)
#define SDHC_FEVT_AC12TOE           ((uint32_t)0x00000002)
#define SDHC_FEVT_AC12NE            ((uint32_t)0x00000001)

#define SDHC_ADMAES_ADMADCE         ((uint32_t)0x00000008)
#define SDHC_ADMAES_ADMALME         ((uint32_t)0x00000004)
#define SDHC_ADMAES_ADMAES_SHIFT    0
#define SDHC_ADMAES_ADMAES_MASK     ((uint32_t)0x00000003)
#define SDHC_ADMAES_ADMAES(x)       ((uint32_t)(((uint32_t)(x)<<SDHC_ADMAES_ADMAES_SHIFT)&SDHC_ADMAES_ADMAES_MASK))

#define SDHC_ADSADDR_ADSADDR_MASK   ((uint32_t)0xFFFFFFFC)
#define SDHC_ADSADDR_ADSADDR_SHIFT  2
#define SDHC_ADSADDR_ADSADDR(x)     ((uint32_t)(((uint32_t)(x)<<SDHC_ADSADDR_ADSADDR_SHIFT)&SDHC_ADSADDR_ADSADDR_MASK))

#define SDHC_VENDOR_INTSTVAL_SHIFT  16
#define SDHC_VENDOR_INTSTVAL_MASK   ((uint32_t)0x00FF0000)
#define SDHC_VENDOR_INTSTVAL(x)     ((uint32_t)(((uint32_t)(x)<<SDHC_VENDOR_INTSTVAL_SHIFTE)&SDHC_VENDOR_INTSTVAL_MASK))
#define SDHC_VENDOR_EXBLKNU         ((uint32_t)0x00000002)
#define SDHC_VENDOR_EXTDMAEN        ((uint32_t)0x00000001)

#define SDHC_MMCBOOT_DTOCVACK_MASK  ((uint32_t)0x0000000F)
#define SDHC_MMCBOOT_DTOCVACK_SHIFT 0
#define SDHC_MMCBOOT_DTOCVACK(x)    ((uint32_t)(((uint32_t)(x)<<SDHC_MMCBOOT_DTOCVACK_SHIFT)&SDHC_MMCBOOT_DTOCVACK_MASK))
#define SDHC_MMCBOOT_BOOTBLKCNT_SHIFT 16
#define SDHC_MMCBOOT_BOOTBLKCNT_MASK  ((uint32_t)0xFFFF0000)
#define SDHC_MMCBOOT_BOOTBLKCNT(x)    ((uint32_t)(((uint32_t)(x)<<SDHC_MMCBOOT_BOOTBLKCNT_SHIFT)&SDHC_MMCBOOT_BOOTBLKCNT_MASK))
#define SDHC_MMCBOOT_AUTOSABGEN     ((uint32_t)0x00000080)
#define SDHC_MMCBOOT_BOOTEN         ((uint32_t)0x00000040)
#define SDHC_MMCBOOT_BOOTMODE       ((uint32_t)0x00000020)
#define SDHC_MMCBOOT_BOOTACK        ((uint32_t)0x00000010)

#define SDHC_HOSTVER_VVN_MASK       ((uint32_t)0x0000FF00)
#define SDHC_HOSTVER_VVN_SHIFT      8
#define SDHC_HOSTVER_VVN(x)         ((uint32_t)(((uint32_t)(x)<<SDHC_HOSTVER_VVN_SHIFT)&SDHC_HOSTVER_VVN_MASK))
#define SDHC_HOSTVER_SVN_MASK       ((uint32_t)0x000000FF)
#define SDHC_HOSTVER_SVN_SHIFT      0
#define SDHC_HOSTVER_SVN(x)         ((uint32_t)(((uint32_t)(x)<<SDHC_HOSTVER_SVN_SHIFT)&SDHC_HOSTVER_SVN_MASK))

/****************************************************************/
/*                                                              */
/*              System Integration Module (SIM)                 */
/*                                                              */
/****************************************************************/
typedef struct
{
  __IO uint32_t SOPT1;
  __IO uint32_t SOPT1CFG;
       uint32_t RESERVED0[1023];
  __IO uint32_t SOPT2;
       uint32_t RESERVED1[1];
  __IO uint32_t SOPT4;
  __IO uint32_t SOPT5;
       uint32_t RESERVED2[1];
  __IO uint32_t SOPT7;
       uint32_t RESERVED3[2];
  __I  uint32_t SDID;
  __IO uint32_t SCGC1;
  __IO uint32_t SCGC2;
  __IO uint32_t SCGC3;
  __IO uint32_t SCGC4;
  __IO uint32_t SCGC5;
  __IO uint32_t SCGC6;
  __IO uint32_t SCGC7;
  __IO uint32_t CLKDIV1;
  __IO uint32_t CLKDIV2;
  __I  uint32_t FCFG1;
  __I  uint32_t FCFG2;
  __I  uint32_t UIDH;
  __I  uint32_t UIDMH;
  __I  uint32_t UIDML;
  __I  uint32_t UIDL;
} SIM_TypeDef;

/****************************************************************/
/*                  Peripheral memory map                       */
/****************************************************************/
#define ADC0_BASE               ((uint32_t)0x4003B000)
#define ADC1_BASE               ((uint32_t)0x400BB000)
#define AIPS0_BASE              ((uint32_t)0x40000000)
#define AIPS1_BASE              ((uint32_t)0x40008000)
#define AXBS_BASE               ((uint32_t)0x40004000)
#define CAN0_BASE               ((uint32_t)0x40024000)
#define CAU_BASE                ((uint32_t)0xE0081000)
#define CMP0_BASE               ((uint32_t)0x40073000)
#define CMP1_BASE               ((uint32_t)0x40073008)
#define CMP2_BASE               ((uint32_t)0x40073010)
#define CMT_BASE                ((uint32_t)0x40062000)
#define CRC_BASE                ((uint32_t)0x40032000)
#define DAC0_BASE               ((uint32_t)0x400CC000)
#define DAC1_BASE               ((uint32_t)0x400CD000)
#define DMAMUX_BASE             ((uint32_t)0x40021000)
#define DMA_BASE                ((uint32_t)0x40008000)
#define ENET_BASE               ((uint32_t)0x400C0000)
#define EWM_BASE                ((uint32_t)0x40061000)
#define FB_BASE                 ((uint32_t)0x4000C000)
#define FMC_BASE                ((uint32_t)0x4001F000)
#define FTFE_BASE               ((uint32_t)0x40020000)
#define FTM0_BASE               ((uint32_t)0x40038000)
#define FTM1_BASE               ((uint32_t)0x40039000)
#define FTM2_BASE               ((uint32_t)0x4003A000)
#define FTM3_BASE               ((uint32_t)0x400B9000)
#define GPIOA_BASE              ((uint32_t)0x400FF000)
#define GPIOB_BASE              ((uint32_t)0x400FF040)
#define GPIOC_BASE              ((uint32_t)0x400FF080)
#define GPIOD_BASE              ((uint32_t)0x400FF0C0)
#define GPIOE_BASE              ((uint32_t)0x400FF100)
#define I2C0_BASE               ((uint32_t)0x40066000)
#define I2C1_BASE               ((uint32_t)0x40067000)
#define I2C2_BASE               ((uint32_t)0x400E6000)
#define I2S0_BASE               ((uint32_t)0x4002F000)
#define LLWU_BASE               ((uint32_t)0x4007C000)
#define LPTMR0_BASE             ((uint32_t)0x40040000)
#define MCG_BASE                ((uint32_t)0x40064000)
#define MCM_BASE                ((uint32_t)0xE0080000)
#define MPU_BASE                ((uint32_t)0x4000D000)
#define OSC0_BASE               ((uint32_t)0x40065000)
#define PDB_BASE                ((uint32_t)0x40036000)
#define PIT_BASE                ((uint32_t)0x40037000)
#define PMC_BASE                ((uint32_t)0x4007D000)
#define PORTA_BASE              ((uint32_t)0x40049000)
#define PORTB_BASE              ((uint32_t)0x4004A000)
#define PORTC_BASE              ((uint32_t)0x4004B000)
#define PORTD_BASE              ((uint32_t)0x4004C000)
#define PORTE_BASE              ((uint32_t)0x4004D000)
#define RCM_BASE                ((uint32_t)0x4007F000)
#define RNG_BASE                ((uint32_t)0x40029000)
#define RTC_BASE                ((uint32_t)0x4003D000)
#define SDHC_BASE               ((uint32_t)0x400B1000)
#define SIM_BASE                ((uint32_t)0x40047000)
#define SMC_BASE                ((uint32_t)0x4007E000)
#define SPI0_BASE               ((uint32_t)0x4002C000)
#define SPI1_BASE               ((uint32_t)0x4002D000)
#define SPI2_BASE               ((uint32_t)0x400AC000)
#define SRF_BASE                ((uint32_t)0x40041000)
#define UART0_BASE              ((uint32_t)0x4006A000)
#define UART1_BASE              ((uint32_t)0x4006B000)
#define UART2_BASE              ((uint32_t)0x4006C000)
#define UART3_BASE              ((uint32_t)0x4006D000)
#define UART4_BASE              ((uint32_t)0x400EA000)
#define UART5_BASE              ((uint32_t)0x400EB000)
#define USBDCD_BASE             ((uint32_t)0x40035000)
#define USBOTG_BASE             ((uint32_t)0x40072000)
#define VBAT_BASE               ((uint32_t)0x4003E000)
#define VREF_BASE               ((uint32_t)0x40074000)
#define WDOG_BASE               ((uint32_t)0x40052000)

/****************************************************************/
/*                 Peripheral declaration                       */
/****************************************************************/
#define ADC0                    ((ADC_TypeDef *)     ADC0_BASE)
#define ADC1                    ((ADC_TypeDef *)     ADC1_BASE)
#define AIPS0                   ((AIPS_TypeDef *)    AIPS0_BASE)
#define AIPS1                   ((AIPS_TypeDef *)    AIPS1_BASE)
#define AXBS                    ((AXBS_TypeDef *)    AXBS_BASE)
#define CAN0                    ((CAN_TypeDef *)     CAN0_BASE)
#define CAU                     ((CAU_TypeDef *)     CAU_BASE)
#define CMP0                    ((CMP_TypeDef *)     CMP0_BASE)
#define CMP1                    ((CMP_TypeDef *)     CMP1_BASE)
#define CMP2                    ((CMP_TypeDef *)     CMP2_BASE)
#define CRC0                    ((CRC_TypeDef *)     CRC_BASE)
#define DAC0                    ((DAC_TypeDef *)     DAC0_BASE)
#define DAC1                    ((DAC_TypeDef *)     DAC1_BASE)
#define DMA                     ((DMA_TypeDef *)     DMA_BASE)
#define DMAMUX                  ((DMAMUX_TypeDef *)  DMAMUX_BASE)
#define ENET                    ((ENET_TypeDef *)    ENET_BASE)
#define EWM                     ((EWM_TypeDef *)     EWM_BASE)
#define FB                      ((FB_TypeDef *)      FB_BASE)
#define FMC                     ((FMC_TypeDef *)     FMC_BASE)
#define FTFE                    ((FTFE_TypeDef *)    FTFE_BASE)
#define FTM0                    ((FTM_TypeDef *)     FTM0_BASE)
#define FTM1                    ((FTM_TypeDef *)     FTM1_BASE)
#define FTM2                    ((FTM_TypeDef *)     FTM2_BASE)
#define FTM3                    ((FTM_TypeDef *)     FTM3_BASE)
#define GPIOA                   ((GPIO_TypeDef  *)   GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef  *)   GPIOB_BASE)
#define GPIOC                   ((GPIO_TypeDef  *)   GPIOC_BASE)
#define GPIOD                   ((GPIO_TypeDef  *)   GPIOD_BASE)
#define GPIOE                   ((GPIO_TypeDef  *)   GPIOE_BASE)
#define I2C0                    ((I2C_TypeDef *)     I2C0_BASE)
#define I2C1                    ((I2C_TypeDef *)     I2C1_BASE)
#define I2C2                    ((I2C_TypeDef *)     I2C2_BASE)
#define I2S0                    ((I2S_TypeDef *)     I2S0_BASE)
#define LLWU                    ((LLWU_TypeDef  *)   LLWU_BASE)
#define LPTMR0                  ((LPTMR_TypeDef *)   LPTMR0_BASE)
#define MCG                     ((MCG_TypeDef  *)    MCG_BASE)
#define MCM                     ((MCM_TypeDef  *)    MCM_BASE)
#define MPU                     ((MPU_TypeDef  *)    MPU_BASE)
#define OSC0                    ((OSC_TypeDef  *)    OSC0_BASE)
#define PDB
#define PIT                     ((PIT_TypeDef *)     PIT_BASE)
#define PMC                     ((PMC_TypeDef  *)    PMC_BASE)
#define PORTA                   ((PORT_TypeDef  *)   PORTA_BASE)
#define PORTB                   ((PORT_TypeDef  *)   PORTB_BASE)
#define PORTC                   ((PORT_TypeDef  *)   PORTC_BASE)
#define PORTD                   ((PORT_TypeDef  *)   PORTD_BASE)
#define PORTE                   ((PORT_TypeDef  *)   PORTE_BASE)
#define RCM
#define RNG
#define RTC
#define SDHC
#define SIM                     ((SIM_TypeDef  *)    SIM_BASE)
#define SMC
#define SPI0                    ((SPI_TypeDef *)     SPI0_BASE)
#define SPI1                    ((SPI_TypeDef *)     SPI1_BASE)
#define SPI2                    ((SPI_TypeDef *)     SPI2_BASE)
#define SYSTEM_REGISTER_FILE    ((volatile uint8_t *)SRF_BASE) /* 32 bytes */
#define UART0                   ((UART_TypeDef *)    UART0_BASE)
#define UART1                   ((UART_TypeDef *)    UART1_BASE)
#define UART2                   ((UART_TypeDef *)    UART2_BASE)
#define UART3                   ((UART_TypeDef *)    UART3_BASE)
#define UART4                   ((UART_TypeDef *)    UART4_BASE)
#define UART5                   ((UART_TypeDef *)    UART5_BASE)
#define USB0                    ((USBOTG_TypeDef *)  USBOTG_BASE)
#define VBAT                    ((volatile uint8_t *)VBAT_BASE) /* 32 bytes */
#define VREF
#define WDOG                    ((WDOG_TypeDef  *)   WDOG_BASE)

/****************************************************************/
/*           Peripheral Registers Bits Definition               */
/****************************************************************/

/****************************************************************/
/*                                                              */
/*             System Integration Module (SIM)                  */
/*                                                              */
/****************************************************************/
/*********  Bits definition for SIM_SOPT1 register  *************/
#define SIM_SOPT1_USBREGEN           ((uint32_t)0x80000000)    /*!< USB voltage regulator enable */
#define SIM_SOPT1_USBSSTBY           ((uint32_t)0x40000000)    /*!< USB voltage regulator in standby mode during Stop, VLPS, LLS and VLLS modes */
#define SIM_SOPT1_USBVSTBY           ((uint32_t)0x20000000)    /*!< USB voltage regulator in standby mode during VLPR and VLPW modes */
#define SIM_SOPT1_OSC32KSEL_SHIFT    18                        /*!< 32K oscillator clock select (shift) */
#define SIM_SOPT1_OSC32KSEL_MASK     ((uint32_t)0x000C0000)    /*!< 32K oscillator clock select (mask) */
#define SIM_SOPT1_OSC32KSEL(x)       ((uint32_t)(((uint32_t)(x) << SIM_SOPT1_OSC32KSEL_SHIFT) & SIM_SOPT1_OSC32KSEL_MASK))  /*!< 32K oscillator clock select */
#define SIM_SOPT1_RAMSIZE_SHIFT      12
#define SIM_SOPT1_RAMSIZE_MASK       ((uint32_t)0x0000F000)
#define SIM_SOPT1_RAMSIZE(x)         ((uint32_t)(((uint32_t)(x) << SIM_SOPT1_RAMSIZE_SHIFT) & SIM_SOPT1_RAMSIZE_MASK))

/*******  Bits definition for SIM_SOPT1CFG register  ************/
#define SIM_SOPT1CFG_USSWE           ((uint32_t)0x04000000)    /*!< USB voltage regulator stop standby write enable */
#define SIM_SOPT1CFG_UVSWE           ((uint32_t)0x02000000)    /*!< USB voltage regulator VLP standby write enable */
#define SIM_SOPT1CFG_URWE            ((uint32_t)0x01000000)    /*!< USB voltage regulator voltage regulator write enable */

/*******  Bits definition for SIM_SOPT2 register  ************/
#define SIM_SOPT2_USBSRC             ((uint32_t)0x00040000)    /*!< USB clock source select */
#define SIM_SOPT2_PLLFLLSEL_SHIFT    16
#define SIM_SOPT2_PLLFLLSEL_MASK     ((uint32_t)0x00030000)
#define SIM_SOPT2_PLLFLLSEL(x)       ((uint32_t)(((uint32_t)(x) << SIM_SOPT2_PLLFLLSEL_SHIFT) & SIM_SOPT2_PLLFLLSEL_MASK))
#define SIM_SOPT2_TRACECLKSEL        ((uint32_t)0x00001000)
#define SIM_SOPT2_PTD7PAD            ((uint32_t)0x00000800)
#define SIM_SOPT2_FBSL_SHIFT         8
#define SIM_SOPT2_FBSL_MASK          ((uint32_t)0x00000300)
#define SIM_SOPT2_FBSL(x)            ((uint32_t)(((uint32_t)(x) << SIM_SOPT2_FBSL_SHIFT) & SIM_SOPT2_FBSL_MASK))
#define SIM_SOPT2_CLKOUTSEL_SHIFT    5
#define SIM_SOPT2_CLKOUTSEL_MASK     ((uint32_t)((uint32_t)0x7 << SIM_SOPT2_CLKOUTSEL_SHIFT))
#define SIM_SOPT2_CLKOUTSEL(x)       ((uint32_t)(((uint32_t)(x) << SIM_SOPT2_CLKOUTSEL_SHIFT) & SIM_SOPT2_CLKOUTSEL_MASK))
#define SIM_SOPT2_RTCCLKOUTSEL       ((uint32_t)0x00000010)    /*!< RTC clock out select */

/*******  Bits definition for SIM_SOPT4 register  ************/
#define SIM_SOPT4_FTM3TRG1SRC        ((uint32_t)0x80000000)
#define SIM_SOPT4_FTM3TRG0SRC        ((uint32_t)0x40000000)
#define SIM_SOPT4_FTM0TRG1SRC        ((uint32_t)0x20000000)
#define SIM_SOPT4_FTM0TRG0SRC        ((uint32_t)0x10000000)
#define SIM_SOPT4_FTM3CLKSEL         ((uint32_t)0x08000000)
#define SIM_SOPT4_FTM2CLKSEL         ((uint32_t)0x04000000)
#define SIM_SOPT4_FTM1CLKSEL         ((uint32_t)0x02000000)
#define SIM_SOPT4_FTM0CLKSEL         ((uint32_t)0x01000000)
#define SIM_SOPT4_FTM2CH0SRC_SHIFT   18
#define SIM_SOPT4_FTM2CH0SRC_MASK    ((uint32_t)0x00300000)
#define SIM_SOPT4_FTM2CH0SRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT4_FTM2CH0SRC_SHIFT) & SIM_SOPT4_FTM2CH0SRC_MASK))
#define SIM_SOPT4_FTM1CH0SRC_SHIFT   12
#define SIM_SOPT4_FTM1CH0SRC_MASK    ((uint32_t)0x000C0000)
#define SIM_SOPT4_FTM1CH0SRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT4_FTM1CH0SRC_SHIFT) & SIM_SOPT4_FTM1CH0SRC_MASK))
#define SIM_SOPT4_FTM3FLT0           ((uint32_t)0x00001000)
#define SIM_SOPT4_FTM2FLT0           ((uint32_t)0x00000100)
#define SIM_SOPT4_FTM1FLT0           ((uint32_t)0x00000010)
#define SIM_SOPT4_FTM0FLT2           ((uint32_t)0x00000004)
#define SIM_SOPT4_FTM0FLT1           ((uint32_t)0x00000002)
#define SIM_SOPT4_FTM0FLT0           ((uint32_t)0x00000001)

/*******  Bits definition for SIM_SOPT5 register  ************/
#define SIM_SOPT5_UART1RXSRC_SHIFT   6
#define SIM_SOPT5_UART1RXSRC_MASK    ((uint32_t)0x000000C0)
#define SIM_SOPT5_UART1RXSRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT5_UART1RXSRC_SHIFT) & SIM_SOPT5_UART1RXSRC_MASK))
#define SIM_SOPT5_UART1TXSRC_SHIFT   4
#define SIM_SOPT5_UART1TXSRC_MASK    ((uint32_t)0x00000030)
#define SIM_SOPT5_UART1TXSRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT5_UART1TXSRC_SHIFT) & SIM_SOPT5_UART1TXSRC_MASK))
#define SIM_SOPT5_UART0RXSRC_SHIFT   2
#define SIM_SOPT5_UART0RXSRC_MASK    ((uint32_t)0x0000000C)
#define SIM_SOPT5_UART0RXSRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT5_UART0RXSRC_SHIFT) & SIM_SOPT5_UART0RXSRC_MASK))
#define SIM_SOPT5_UART0TXSRC_SHIFT   0
#define SIM_SOPT5_UART0TXSRC_MASK    ((uint32_t)0x00000003)
#define SIM_SOPT5_UART0TXSRC(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT5_UART0TXSRC_SHIFT) & SIM_SOPT5_UART0TXSRC_MASK))

/*******  Bits definition for SIM_SOPT7 register  ************/
#define SIM_SOPT7_ADC1ALTTRGEN       ((uint32_t)0x00008000)
#define SIM_SOPT7_ADC1PRETRGSEL      ((uint32_t)0x00001000)
#define SIM_SOPT7_ADC1TRGSEL_SHIFT   8
#define SIM_SOPT7_ADC1TRGSEL_MASK    ((uint32_t)0x00000F00)
#define SIM_SOPT7_ADC1TRGSEL(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT7_ADC1TRGSEL_SHIFT) & SIM_SOPT7_ADC1TRGSEL_MASK))
#define SIM_SOPT7_ADC0ALTTRGEN       ((uint32_t)0x00000080)
#define SIM_SOPT7_ADC0PRETRGSEL      ((uint32_t)0x00000010)
#define SIM_SOPT7_ADC0TRGSEL_SHIFT   0
#define SIM_SOPT7_ADC0TRGSEL_MASK    ((uint32_t)0x0000000F)
#define SIM_SOPT7_ADC0TRGSEL(x)      ((uint32_t)(((uint32_t)(x) << SIM_SOPT7_ADC0TRGSEL_SHIFT) & SIM_SOPT7_ADC0TRGSEL_MASK))

/*******  Bits definition for SIM_SCGC1 register  ************/
#define SIM_SCGC1_UART5              ((uint32_t)0x00000800)    /*!< UART5 Clock Gate Control */
#define SIM_SCGC1_UART4              ((uint32_t)0x00000400)    /*!< UART4 Clock Gate Control */
#define SIM_SCGC1_I2C2               ((uint32_t)0x00000040)    /*!< I2C2 Clock Gate Control */

/*******  Bits definition for SIM_SCGC2 register  ************/
#define SIM_SCGC2_DAC1               ((uint32_t)0x00002000)    /*!< DAC1 Clock Gate Control */
#define SIM_SCGC2_DAC0               ((uint32_t)0x00001000)    /*!< DAC0 Clock Gate Control */
#define SIM_SCGC2_ENET               ((uint32_t)0x00000040)    /*!< ENET Clock Gate Control */

/*******  Bits definition for SIM_SCGC3 register  ************/
#define SIM_SCGC3_ADC1               ((uint32_t)0x08000000)    /*!< ADC1 Clock Gate Control */
#define SIM_SCGC3_FTM3               ((uint32_t)0x02000000)    /*!< FTM3 Clocl Gate Control */
#define SIM_SCGC3_FTM2               ((uint32_t)0x01000000)    /*!< FTM2 Clock Gate Control */
#define SIM_SCGC3_SDHC               ((uint32_t)0x00020000)    /*!< SDHC Clock Gate Control */
#define SIM_SCGC3_SPI2               ((uint32_t)0x00001000)    /*!< SPI2 Clock Gate Control */
#define SIM_SCGC3_RNGA               ((uint32_t)0x00000001)    /*!< RNGA Clock Gate Control */

/*******  Bits definition for SIM_SCGC4 register  ************/
#define SIM_SCGC4_VREF               ((uint32_t)0x00100000)    /*!< VREF Clock Gate Control */
#define SIM_SCGC4_CMP                ((uint32_t)0x00080000)    /*!< Comparator Clock Gate Control */
#define SIM_SCGC4_USBOTG             ((uint32_t)0x00040000)    /*!< USB Clock Gate Control */
#define SIM_SCGC4_UART3              ((uint32_t)0x00002000)    /*!< UART3 Clock Gate Control */
#define SIM_SCGC4_UART2              ((uint32_t)0x00001000)    /*!< UART2 Clock Gate Control */
#define SIM_SCGC4_UART1              ((uint32_t)0x00000800)    /*!< UART1 Clock Gate Control */
#define SIM_SCGC4_UART0              ((uint32_t)0x00000400)    /*!< UART0 Clock Gate Control */
#define SIM_SCGC4_I2C1               ((uint32_t)0x00000080)    /*!< I2C1 Clock Gate Control */
#define SIM_SCGC4_I2C0               ((uint32_t)0x00000040)    /*!< I2C0 Clock Gate Control */
#define SIM_SCGC4_CMT                ((uint32_t)0x00000004)    /*!< CMT Clock Gate Control */
#define SIM_SCGC4_EMW                ((uint32_t)0x00000002)    /*!< EWM Clock Gate Control */

/*******  Bits definition for SIM_SCGC5 register  ************/
#define SIM_SCGC5_PORTE              ((uint32_t)0x00002000)    /*!< Port E Clock Gate Control */
#define SIM_SCGC5_PORTD              ((uint32_t)0x00001000)    /*!< Port D Clock Gate Control */
#define SIM_SCGC5_PORTC              ((uint32_t)0x00000800)    /*!< Port C Clock Gate Control */
#define SIM_SCGC5_PORTB              ((uint32_t)0x00000400)    /*!< Port B Clock Gate Control */
#define SIM_SCGC5_PORTA              ((uint32_t)0x00000200)    /*!< Port A Clock Gate Control */
#define SIM_SCGC5_LPTIMER            ((uint32_t)0x00000001)    /*!< Low Power Timer Access Control */

/*******  Bits definition for SIM_SCGC6 register  ************/
#define SIM_SCGC6_DAC0               ((uint32_t)0x80000000)    /*!< DAC0 Clock Gate Control */
#define SIM_SCGC6_RTC                ((uint32_t)0x20000000)    /*!< RTC Access Control */
#define SIM_SCGC6_ADC0               ((uint32_t)0x08000000)    /*!< ADC0 Clock Gate Control */
#define SIM_SCGC6_FTM2               ((uint32_t)0x04000000)    /*!< FTM2 Clock Gate Control */
#define SIM_SCGC6_FTM1               ((uint32_t)0x02000000)    /*!< FTM1 Clock Gate Control */
#define SIM_SCGC6_FTM0               ((uint32_t)0x01000000)    /*!< FTM0 Clock Gate Control */
#define SIM_SCGC6_PIT                ((uint32_t)0x00800000)    /*!< PIT Clock Gate Control */
#define SIM_SCGC6_PDB                ((uint32_t)0x00400000)    /*!< PDB Clock Gate Control */
#define SIM_SCGC6_USBDCD             ((uint32_t)0x00200000)    /*!< USB DCD Clock Gate Control */
#define SIM_SCGC6_CRC                ((uint32_t)0x00040000)    /*!< Low Power Timer Access Control */
#define SIM_SCGC6_I2S                ((uint32_t)0x00008000)    /*!< CRC Clock Gate Control */
#define SIM_SCGC6_SPI1               ((uint32_t)0x00002000)    /*!< SPI1 Clock Gate Control */
#define SIM_SCGC6_SPI0               ((uint32_t)0x00001000)    /*!< SPI0 Clock Gate Control */
#define SIM_SCGC6_RNGA               ((uint32_t)0x00000200)
#define SIM_SCGC6_FCAN0              ((uint32_t)0x00000010)    /*!< FlexCAN 0 Clock Gate Control */
#define SIM_SCGC6_DMAMUX             ((uint32_t)0x00000002)    /*!< DMA Mux Clock Gate Control */
#define SIM_SCGC6_FTF                ((uint32_t)0x00000001)    /*!< Flash Memory Clock Gate Control */

/*******  Bits definition for SIM_SCGC6 register  ************/
#define SIM_SCGC7_MPU                ((uint32_t)0x00000004)    /*!< MPU Clock Gate Control */
#define SIM_SCGC7_DMA                ((uint32_t)0x00000002)    /*!< DMA Clock Gate Control */
#define SIM_SCGC7_FLEXBUS            ((uint32_t)0x00000001)    /*!< FlexBus Clock Gate Control */

/******  Bits definition for SIM_CLKDIV1 register  ***********/
#define SIM_CLKDIV1_OUTDIV1_SHIFT    28
#define SIM_CLKDIV1_OUTDIV1_MASK     ((uint32_t)0xF0000000)
#define SIM_CLKDIV1_OUTDIV1(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV1_SHIFT) & SIM_CLKDIV1_OUTDIV1_MASK))
#define SIM_CLKDIV1_OUTDIV2_SHIFT    24
#define SIM_CLKDIV1_OUTDIV2_MASK     ((uint32_t)0x0F000000)
#define SIM_CLKDIV1_OUTDIV2(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV2_SHIFT) & SIM_CLKDIV1_OUTDIV2_MASK))
#define SIM_CLKDIV1_OUTDIV3_SHIFT    20
#define SIM_CLKDIV1_OUTDIV3_MASK     ((uint32_t)0x00F00000)
#define SIM_CLKDIV1_OUTDIV3(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV3_SHIFT) & SIM_CLKDIV1_OUTDIV3_MASK))
#define SIM_CLKDIV1_OUTDIV4_SHIFT    16
#define SIM_CLKDIV1_OUTDIV4_MASK     ((uint32_t)0x000F0000)
#define SIM_CLKDIV1_OUTDIV4(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV4_SHIFT) & SIM_CLKDIV1_OUTDIV4_MASK))

/******  Bits definition for SIM_CLKDIV2 register  ***********/
#define SIM_CLKDIV2_USBDIV_SHIFT     1
#define SIM_CLKDIV2_USBDIV_MASK      ((uint32_t)0x0000000E)
#define SIM_CLKDIV2_USBDIV(x)        ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV2_USBDIV_SHIFT) & SIM_CLKDIV2_USBDIV_MASK))
#define SIM_CLKDIV2_USBFRAC          ((uint32_t)0x00000001)

#endif
