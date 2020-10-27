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
#include <stdbool.h>
#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "list.h"
#include "semphr.h"
#include "FreeRTOS_IP.h"
#include "pkcs11_nvm.h"
#include "device.h"
#include "system/int/sys_int.h"
#include "peripheral/efc/plib_efc.h"


// mask corresponding to AWS_NVM_PAGE_SIZE == 512 = 2^9
#define AWS_NVM_PAGE_MASK   (~((1 << 9) - 1))


// page command args for erasing min/max pages
#define AWS_NVM_ERASE_COMMAND_ARG_MIN   2
#define AWS_NVM_ERASE_COMMAND_ARG_MAX   3

// force erase in MIN pages, if needed for some reason
#define AWS_NVM_ERASE_FORCE_MIN         0

static uint32_t GetRandomNum();

int mbedtls_hardware_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{

 

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
        suint_64.v64 = GetRandomNum();
        suint_64.v64 = suint_64.v64 << 32;
        suint_64.v64 |= GetRandomNum();
        memcpy(output, suint_64.v8, sizeof(suint_64.v8));
        output += sizeof(suint_64.v8);
    }

 

    if(nLeft)
    {
        suint_64.v64 = GetRandomNum();
        suint_64.v64 = suint_64.v64 << 32;
        suint_64.v64 |= GetRandomNum();
        memcpy(output, suint_64.v8, nLeft);
    }

 


  *olen = len;
  
  return 0;
}

static uint32_t GetRandomNum()
{
    uint32_t rand_num;
    //xApplicationGetRandomNumber(&rand_num);
    return rand_num;
}
