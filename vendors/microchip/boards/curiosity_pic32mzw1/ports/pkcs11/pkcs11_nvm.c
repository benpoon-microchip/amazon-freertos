/*
  * Copyright 2017 Microchip Technology Incorporated and its subsidiaries.
  * 
  * Amazon FreeRTOS PKCS#11 for Curiosity PIC32MZEF V1.0.4
  * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy of 
  * this software and associated documentation files (the "Software"), to deal in 
  * the Software without restriction, including without limitation the rights to 
  * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
  * of the Software, and to permit persons to whom the Software is furnished to do 
  * so, subject to the following conditions:
  * The above copyright notice and this permission notice shall be included in all 
  * copies or substantial portions of the Software.

  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
  * SOFTWARE
  ******************************************************************************
 */

/* PKCS11 storage implementation for Microchip PIC32MZ platform. */

#include <stdlib.h>
#include <string.h>
#include <sys/kmem.h>

#include "stdbool.h"
#include "pkcs11_nvm.h"

#include "peripheral/nvm/plib_nvm.h"



/* NVM Flash Memory programming
 * key 1*/

#define NVM_PROGRAM_UNLOCK_KEY1    0xAA996655

/* NVM Driver Flash Memory programming
 * key 1*/

#define NVM_PROGRAM_UNLOCK_KEY2    0x556699AA

#define AWS_NVM_QUAD_MASK          0xfffffff0
// data

 //void (*NVM_CALLBACK)(uintptr_t context);

void nvm_callback(uintptr_t context);
// prototypes
//

volatile bool bTaskCompleted = false;

// implementation
void nvm_callback(uintptr_t context)
{
    //printf("[%s] log 1\r\n", __func__);
    bTaskCompleted=true;
}


bool AWS_UpperBootPage4Erase(uint32_t addr)
{
    printf("[%s] log 1\r\n", __func__);
    bool bResult=true;
    int i = 0;
    uint32_t phys_addr = KVA0_TO_KVA1(addr);
    
    NVM_CallbackRegister(nvm_callback, (uintptr_t)NULL);
    
    for (i=0; i<4; i++)
    {
        bTaskCompleted=false; 
        while(NVM_IsBusy() == true);

        /* Erase the Page */
        NVM_PageErase(phys_addr);

        while(bTaskCompleted == false);
        bResult = (NVM_ErrorGet() == 0)?true:false;
        if ( false == bResult)
            break;

        phys_addr += NVM_FLASH_PAGESIZE;
    }
    
    return bResult;
    
}

bool AWS_UpperBootWriteRow(const uint32_t ptrFlash, const uint32_t* rowData)
{
    //printf("[%s] log 1 ptrFlash = 0x%p\r\n", __func__, ptrFlash);
    //printf("[%s] data = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", __func__, rowData[0], rowData[1], rowData[2], rowData[3], rowData[4], rowData[5]);
    bool bResult=true;
        /* Program a row of data */
        bTaskCompleted=false;
        while(NVM_IsBusy() == true);
        NVM_RowWrite((uint32_t *)rowData, ptrFlash);

        while(bTaskCompleted == false);

        bTaskCompleted = false;
    bResult = (NVM_ErrorGet() == 0)?true:false;
    return bResult;
}


void AWS_UpperBootPage4ProtectionDisable(void)
{
    printf("[%s] log 1\r\n", __func__);
    // No Support now!!!
}

void AWS_UpperBootPage4ProtectionEnable(void)
{
    printf("[%s] log 1\r\n", __func__);
    // No Support now!!!
}


bool AWS_FlashProgramBlock(const uint8_t* address, const uint8_t* pData, uint32_t size)
{
    printf("[%s] log 1\r\n", __func__);
    uint32_t quad_buff[AWS_NVM_QUAD_SIZE / sizeof(uint32_t)];

    const uint8_t* start_quad = (const uint8_t*)((uint32_t)address & AWS_NVM_QUAD_MASK);    // quad to start with
    const uint8_t* first_quad = start_quad + AWS_NVM_QUAD_SIZE; // first full quad
    const uint8_t* end_quad = (const uint8_t*)(((uint32_t)address + size + AWS_NVM_QUAD_SIZE - 1) & AWS_NVM_QUAD_MASK); // last quad pointer, past the memory area
    const uint8_t* last_quad = end_quad - AWS_NVM_QUAD_SIZE; // last full quad pointer

    uint32_t n_quads = (end_quad - start_quad ) / AWS_NVM_QUAD_SIZE;

    bool flash_fault = false;

    while(n_quads)
    {
        uint32_t start_gap = address - start_quad;    // not used bytes in the start quad
        uint32_t last_gap = end_quad - (address + size);// not used bytes in the last quad
        uint32_t last_fill = (address + size) - last_quad ;    // payload bytes in the last quad
        uint32_t start_load = AWS_NVM_QUAD_SIZE - start_gap; // useful bytes at beginning

        if(start_gap != 0)
        {   // not complete quad at the beginning
            memset(quad_buff, 0xff, sizeof(quad_buff));
            memcpy((uint8_t*)quad_buff + start_gap, pData, start_load);
            if(!AWS_NVM_QuadWordWrite((uint32_t*)start_quad, quad_buff, 1))
            {
                flash_fault = true;
                break;
            }
            start_quad = first_quad;
            pData += start_load;
            n_quads--;
        }

        if(last_gap == 0)
        {
            if(!AWS_NVM_QuadWordWrite((uint32_t*)start_quad, (uint32_t*)pData, n_quads))
            {
                flash_fault = true;
            }
            break;
        }

        // there is left payload data in the last quad
        // program full quads first
        if(!AWS_NVM_QuadWordWrite((uint32_t*)start_quad, (uint32_t*)pData, n_quads - 1))
        {
            flash_fault = true;
            break;
        }

        // now program the last quad
        memset(quad_buff, 0xff, sizeof(quad_buff));
        memcpy((uint8_t*)quad_buff, pData + (n_quads - 1) * AWS_NVM_QUAD_SIZE, last_fill);
        if(!AWS_NVM_QuadWordWrite((uint32_t*)last_quad, quad_buff , 1))
        {
            flash_fault = true;
        }

        break;
    }


    return !flash_fault;

}



bool AWS_FlashEraseUpdateBank()
{
    printf("[%s] log 1\r\n", __func__);
    volatile uint32_t processorStatus;
    uint32_t operation = 6; // Upper Boot Region
    bool bResult=true;
    processorStatus = __builtin_disable_interrupts();

    bTaskCompleted=false;
    while(NVM_IsBusy() == true);

    // Set the flash operation:
    /***************************************************************************
     * Page erase: Erases the entire page which includes the target address
     *    (NVMADDR) if it is not write-protected.
     * Word (32-bit) program: Programs the 32 bit word in NVMDATA0 to the flash
     *    address selected by NVMADDR, if it is not write-protected.
     * Quad Word (128-bit) program: Programs the 128 bit quad words in NVMDATA0
     *    through NVMDATA3 to flash address selected by NVMADDR, if it they are
     *    not write-protected.
     * Row program: Programs the entire row from the physical address in
     *    NVMSRCADDR to the flash address selected by NVMADDR if it is not
     *    write-protected
     **************************************************************************/

    // NVMOP can be written only when WREN is zero. So, clear WREN.
    NVMCONCLR = _NVMCON_WREN_MASK;

    /* Clear and Set, as NVMCON contains status bits and hence need to be accessed atomically.
     * Using bit field access may erroneously cause status bits to get cleared */
    NVMCONCLR = _NVMCON_NVMOP_MASK;
    NVMCONSET = ( _NVMCON_NVMOP_MASK & (((uint32_t)operation) << _NVMCON_NVMOP_POSITION) );

    // Set WREN to enable writes to the WR bit and to prevent NVMOP modification
    NVMCONSET = _NVMCON_WREN_MASK;

    // Write the unlock key sequence
    NVMKEY = 0x0;
    NVMKEY = NVM_PROGRAM_UNLOCK_KEY1;
    NVMKEY = NVM_PROGRAM_UNLOCK_KEY2;

    // Start the operation
    NVMCONSET = _NVMCON_WR_MASK;

    __builtin_mtc0(12, 0, processorStatus);

    IEC5SET   = 0x80;
    while(bTaskCompleted == false);

    bTaskCompleted = false;
    bResult = (NVM_ErrorGet() == 0)?true:false;
    return bResult;

}

bool AWS_NVM_QuadWordWrite(uint32_t * address,
                           uint32_t * data,
                            int nQuads )
{
    printf("[%s] log 1\r\n", __func__);
     bool bResult=true;
     int i;
     uint32_t flash_phys_addr = (uint32_t) address;
     uint32_t *flash_data = data;
        /* Program a row of data */
     for (i=0;i<nQuads && bResult; i++)
     {
        bTaskCompleted=false;
        while(NVM_IsBusy() == true);
        NVM_QuadWordWrite((uint32_t *)flash_data, (uint32_t) flash_phys_addr);
        
        while(bTaskCompleted == false);
        
        flash_phys_addr += AWS_NVM_QUAD_SIZE;
        flash_data += AWS_NVM_QUAD_SIZE / sizeof(*flash_data);

        bTaskCompleted = false;
        bResult = (NVM_ErrorGet() == 0)?true:false;
     }
    return bResult;
}

int mbedtls_hardware_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{
    printf("[%s] log 1\r\n", __func__);
    ((void) data);

    union
    {
        uint64_t    v64;
        uint8_t     v8[8];
    }suint_64;

    int n8Chunks = len / 8;
    int nLeft = len % 8;

    while(n8Chunks--)
    {
        suint_64.v64 = RNG_NumGen1Get();
        suint_64.v64 = suint_64.v64 << 32;
        suint_64.v64 |= RNG_NumGen2Get();
        memcpy(output, suint_64.v8, sizeof(suint_64.v8));
        output += sizeof(suint_64.v8);
    }

    if(nLeft)
    {
        suint_64.v64 = RNG_NumGen1Get();
        suint_64.v64 = suint_64.v64 << 32;
        suint_64.v64 |= RNG_NumGen2Get();
        memcpy(output, suint_64.v8, nLeft);
    }


  *olen = len;
  
  return 0;
}





