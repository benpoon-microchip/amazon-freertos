/*******************************************************************************
  System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include "ISR_Support.h"

    .extern  TIMER_2_Handler

    .section   .vector_9,code, keep
    .equ     __vector_dispatch_9, IntVectorTIMER_2_Handler
    .global  __vector_dispatch_9
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorTIMER_2_Handler

IntVectorTIMER_2_Handler:
    portSAVE_CONTEXT
    la    s6,  TIMER_2_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorTIMER_2_Handler
    .extern  I2C1_BUS_Handler

    .section   .vector_115,code, keep
    .equ     __vector_dispatch_115, IntVectorI2C1_BUS_Handler
    .global  __vector_dispatch_115
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorI2C1_BUS_Handler

IntVectorI2C1_BUS_Handler:
    portSAVE_CONTEXT
    la    s6,  I2C1_BUS_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorI2C1_BUS_Handler
    .extern  I2C1_MASTER_Handler

    .section   .vector_117,code, keep
    .equ     __vector_dispatch_117, IntVectorI2C1_MASTER_Handler
    .global  __vector_dispatch_117
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorI2C1_MASTER_Handler

IntVectorI2C1_MASTER_Handler:
    portSAVE_CONTEXT
    la    s6,  I2C1_MASTER_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorI2C1_MASTER_Handler
    .extern  CHANGE_NOTICE_K_Handler

    .section   .vector_127,code, keep
    .equ     __vector_dispatch_127, IntVectorCHANGE_NOTICE_K_Handler
    .global  __vector_dispatch_127
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorCHANGE_NOTICE_K_Handler

IntVectorCHANGE_NOTICE_K_Handler:
    portSAVE_CONTEXT
    la    s6,  CHANGE_NOTICE_K_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorCHANGE_NOTICE_K_Handler
    .extern  SPI3_RX_Handler

    .section   .vector_155,code, keep
    .equ     __vector_dispatch_155, IntVectorSPI3_RX_Handler
    .global  __vector_dispatch_155
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorSPI3_RX_Handler

IntVectorSPI3_RX_Handler:
    portSAVE_CONTEXT
    la    s6,  SPI3_RX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorSPI3_RX_Handler
    .extern  SPI3_TX_Handler

    .section   .vector_156,code, keep
    .equ     __vector_dispatch_156, IntVectorSPI3_TX_Handler
    .global  __vector_dispatch_156
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorSPI3_TX_Handler

IntVectorSPI3_TX_Handler:
    portSAVE_CONTEXT
    la    s6,  SPI3_TX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorSPI3_TX_Handler
    .extern  FLASH_CONTROL_Handler

    .section   .vector_167,code, keep
    .equ     __vector_dispatch_167, IntVectorFLASH_CONTROL_Handler
    .global  __vector_dispatch_167
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorFLASH_CONTROL_Handler

IntVectorFLASH_CONTROL_Handler:
    portSAVE_CONTEXT
    la    s6,  FLASH_CONTROL_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorFLASH_CONTROL_Handler
    .extern  UART4_FAULT_Handler

    .section   .vector_170,code, keep
    .equ     __vector_dispatch_170, IntVectorUART4_FAULT_Handler
    .global  __vector_dispatch_170
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART4_FAULT_Handler

IntVectorUART4_FAULT_Handler:
    portSAVE_CONTEXT
    la    s6,  UART4_FAULT_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART4_FAULT_Handler
    .extern  UART4_RX_Handler

    .section   .vector_171,code, keep
    .equ     __vector_dispatch_171, IntVectorUART4_RX_Handler
    .global  __vector_dispatch_171
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART4_RX_Handler

IntVectorUART4_RX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART4_RX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART4_RX_Handler
    .extern  UART4_TX_Handler

    .section   .vector_172,code, keep
    .equ     __vector_dispatch_172, IntVectorUART4_TX_Handler
    .global  __vector_dispatch_172
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorUART4_TX_Handler

IntVectorUART4_TX_Handler:
    portSAVE_CONTEXT
    la    s6,  UART4_TX_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorUART4_TX_Handler

