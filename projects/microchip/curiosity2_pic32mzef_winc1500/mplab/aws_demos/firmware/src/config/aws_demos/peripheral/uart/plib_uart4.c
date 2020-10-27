/*******************************************************************************
  UART4 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart4.c

  Summary:
    UART4 PLIB Implementation File

  Description:
    None

*******************************************************************************/

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

#include "device.h"
#include "plib_uart4.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART4 Implementation
// *****************************************************************************
// *****************************************************************************

UART_RING_BUFFER_OBJECT uart4Obj;

#define UART4_READ_BUFFER_SIZE      128
#define UART4_RX_INT_DISABLE()      IEC5CLR = _IEC5_U4RXIE_MASK;
#define UART4_RX_INT_ENABLE()       IEC5SET = _IEC5_U4RXIE_MASK;

static uint8_t UART4_ReadBuffer[UART4_READ_BUFFER_SIZE];

#define UART4_WRITE_BUFFER_SIZE     128
#define UART4_TX_INT_DISABLE()      IEC5CLR = _IEC5_U4TXIE_MASK;
#define UART4_TX_INT_ENABLE()       IEC5SET = _IEC5_U4TXIE_MASK;

static uint8_t UART4_WriteBuffer[UART4_WRITE_BUFFER_SIZE];

void static UART4_ErrorClear( void )
{
    /* rxBufferLen = (FIFO level + RX register) */
    uint8_t rxBufferLen = UART_RXFIFO_DEPTH;
    uint8_t dummyData = 0u;

    /* If it's a overrun error then clear it to flush FIFO */
    if(U4STA & _U4STA_OERR_MASK)
    {
        U4STACLR = _U4STA_OERR_MASK;
    }

    /* Read existing error bytes from FIFO to clear parity and framing error flags */
    while(U4STA & (_U4STA_FERR_MASK | _U4STA_PERR_MASK))
    {
        dummyData = (uint8_t )(U4RXREG );
        rxBufferLen--;

        /* Try to flush error bytes for one full FIFO and exit instead of
         * blocking here if more error bytes are received */
        if(rxBufferLen == 0u)
        {
            break;
        }
    }

    // Ignore the warning
    (void)dummyData;

    /* Clear error interrupt flag */
    IFS5CLR = _IFS5_U4EIF_MASK;

    /* Clear up the receive interrupt flag so that RX interrupt is not
     * triggered for error bytes */
    IFS5CLR = _IFS5_U4RXIF_MASK;

    return;
}

void UART4_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0 */
    /* PDSEL = 0 */

    U4MODE = 0x8;

    /* Enable UART4 Receiver and Transmitter */
    U4STASET = (_U4STA_UTXEN_MASK | _U4STA_URXEN_MASK | _U4STA_UTXISEL1_MASK);

    /* BAUD Rate register Setup */
    U4BRG = 216;

    /* Disable Interrupts */
    IEC5CLR = _IEC5_U4EIE_MASK;

    IEC5CLR = _IEC5_U4RXIE_MASK;

    IEC5CLR = _IEC5_U4TXIE_MASK;

    /* Initialize instance object */
    uart4Obj.rdCallback = NULL;
    uart4Obj.rdInIndex = 0;
    uart4Obj.rdOutIndex = 0;
    uart4Obj.isRdNotificationEnabled = false;
    uart4Obj.isRdNotifyPersistently = false;
    uart4Obj.rdThreshold = 0;

    uart4Obj.wrCallback = NULL;
    uart4Obj.wrInIndex = 0;
    uart4Obj.wrOutIndex = 0;
    uart4Obj.isWrNotificationEnabled = false;
    uart4Obj.isWrNotifyPersistently = false;
    uart4Obj.wrThreshold = 0;

    /* Turn ON UART4 */
    U4MODESET = _U4MODE_ON_MASK;

    /* Enable UART4_FAULT Interrupt */
    IEC5SET = _IEC5_U4EIE_MASK;

    /* Enable UART4_RX Interrupt */
    IEC5SET = _IEC5_U4RXIE_MASK;
}

bool UART4_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    int32_t brgValHigh = 0;
    int32_t brgValLow = 0;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if (baud == 0)
        {
            return status;
        }

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART4_FrequencyGet();
        }

        /* Calculate BRG value */
        brgValLow = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        brgValHigh = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;

        /* Check if the baud value can be set with low baud settings */
        if((brgValLow >= 0) && (brgValLow <= UINT16_MAX))
        {
            brgVal =  brgValLow;
            U4MODECLR = _U4MODE_BRGH_MASK;
        }
        else if ((brgValHigh >= 0) && (brgValHigh <= UINT16_MAX))
        {
            brgVal = brgValHigh;
            U4MODESET = _U4MODE_BRGH_MASK;
        }
        else
        {
            return status;
        }

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            if(setup->parity != UART_PARITY_NONE)
            {
               return status;
            }
            else
            {
               /* Configure UART4 mode */
               uartMode = U4MODE;
               uartMode &= ~_U4MODE_PDSEL_MASK;
               U4MODE = uartMode | setup->dataWidth;
            }
        }
        else
        {
            /* Configure UART4 mode */
            uartMode = U4MODE;
            uartMode &= ~_U4MODE_PDSEL_MASK;
            U4MODE = uartMode | setup->parity ;
        }

        /* Configure UART4 mode */
        uartMode = U4MODE;
        uartMode &= ~_U4MODE_STSEL_MASK;
        U4MODE = uartMode | setup->stopBits ;

        /* Configure UART4 Baud Rate */
        U4BRG = brgVal;

        status = true;
    }

    return status;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static inline bool UART4_RxPushByte(uint8_t rdByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart4Obj.rdInIndex + 1;

    if (tempInIndex >= UART4_READ_BUFFER_SIZE)
    {
        tempInIndex = 0;
    }

    if (tempInIndex == uart4Obj.rdOutIndex)
    {
        /* Queue is full - Report it to the application. Application gets a chance to free up space by reading data out from the RX ring buffer */
        if(uart4Obj.rdCallback != NULL)
        {
            uart4Obj.rdCallback(UART_EVENT_READ_BUFFER_FULL, uart4Obj.rdContext);

            /* Read the indices again in case application has freed up space in RX ring buffer */
            tempInIndex = uart4Obj.rdInIndex + 1;

            if (tempInIndex >= UART4_READ_BUFFER_SIZE)
            {
                tempInIndex = 0;
            }
        }
    }

    /* Attempt to push the data into the ring buffer */
    if (tempInIndex != uart4Obj.rdOutIndex)
    {
        UART4_ReadBuffer[uart4Obj.rdInIndex] = rdByte;
        uart4Obj.rdInIndex = tempInIndex;
        isSuccess = true;
    }
    else
    {
        /* Queue is full. Data will be lost. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART4_ReadNotificationSend(void)
{
    uint32_t nUnreadBytesAvailable;

    if (uart4Obj.isRdNotificationEnabled == true)
    {
        nUnreadBytesAvailable = UART4_ReadCountGet();

        if(uart4Obj.rdCallback != NULL)
        {
            if (uart4Obj.isRdNotifyPersistently == true)
            {
                if (nUnreadBytesAvailable >= uart4Obj.rdThreshold)
                {
                    uart4Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart4Obj.rdContext);
                }
            }
            else
            {
                if (nUnreadBytesAvailable == uart4Obj.rdThreshold)
                {
                    uart4Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart4Obj.rdContext);
                }
            }
        }
    }
}

size_t UART4_Read(uint8_t* pRdBuffer, const size_t size)
{
    size_t nBytesRead = 0;
    uint32_t rdOutIndex;
    uint32_t rdInIndex;

    while (nBytesRead < size)
    {
        UART4_RX_INT_DISABLE();

        rdOutIndex = uart4Obj.rdOutIndex;
        rdInIndex = uart4Obj.rdInIndex;

        if (rdOutIndex != rdInIndex)
        {
            pRdBuffer[nBytesRead++] = UART4_ReadBuffer[uart4Obj.rdOutIndex++];

            if (uart4Obj.rdOutIndex >= UART4_READ_BUFFER_SIZE)
            {
                uart4Obj.rdOutIndex = 0;
            }
            UART4_RX_INT_ENABLE();
        }
        else
        {
            UART4_RX_INT_ENABLE();
            break;
        }
    }

    return nBytesRead;
}

size_t UART4_ReadCountGet(void)
{
    size_t nUnreadBytesAvailable;
    uint32_t rdInIndex;
    uint32_t rdOutIndex;

    /* Take a snapshot of indices to avoid processing in critical section */
    rdInIndex = uart4Obj.rdInIndex;
    rdOutIndex = uart4Obj.rdOutIndex;

    if ( rdInIndex >=  rdOutIndex)
    {
        nUnreadBytesAvailable =  rdInIndex -  rdOutIndex;
    }
    else
    {
        nUnreadBytesAvailable =  (UART4_READ_BUFFER_SIZE -  rdOutIndex) + rdInIndex;
    }

    return nUnreadBytesAvailable;
}

size_t UART4_ReadFreeBufferCountGet(void)
{
    return (UART4_READ_BUFFER_SIZE - 1) - UART4_ReadCountGet();
}

size_t UART4_ReadBufferSizeGet(void)
{
    return (UART4_READ_BUFFER_SIZE - 1);
}

bool UART4_ReadNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart4Obj.isRdNotificationEnabled;

    uart4Obj.isRdNotificationEnabled = isEnabled;

    uart4Obj.isRdNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART4_ReadThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart4Obj.rdThreshold = nBytesThreshold;
    }
}

void UART4_ReadCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart4Obj.rdCallback = callback;

    uart4Obj.rdContext = context;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static bool UART4_TxPullByte(uint8_t* pWrByte)
{
    bool isSuccess = false;
    uint32_t wrOutIndex = uart4Obj.wrOutIndex;
    uint32_t wrInIndex = uart4Obj.wrInIndex;

    if (wrOutIndex != wrInIndex)
    {
        *pWrByte = UART4_WriteBuffer[uart4Obj.wrOutIndex++];

        if (uart4Obj.wrOutIndex >= UART4_WRITE_BUFFER_SIZE)
        {
            uart4Obj.wrOutIndex = 0;
        }
        isSuccess = true;
    }

    return isSuccess;
}

static inline bool UART4_TxPushByte(uint8_t wrByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart4Obj.wrInIndex + 1;

    if (tempInIndex >= UART4_WRITE_BUFFER_SIZE)
    {
        tempInIndex = 0;
    }
    if (tempInIndex != uart4Obj.wrOutIndex)
    {
        UART4_WriteBuffer[uart4Obj.wrInIndex] = wrByte;
        uart4Obj.wrInIndex = tempInIndex;
        isSuccess = true;
    }
    else
    {
        /* Queue is full. Report Error. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART4_WriteNotificationSend(void)
{
    uint32_t nFreeWrBufferCount;

    if (uart4Obj.isWrNotificationEnabled == true)
    {
        nFreeWrBufferCount = UART4_WriteFreeBufferCountGet();

        if(uart4Obj.wrCallback != NULL)
        {
            if (uart4Obj.isWrNotifyPersistently == true)
            {
                if (nFreeWrBufferCount >= uart4Obj.wrThreshold)
                {
                    uart4Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart4Obj.wrContext);
                }
            }
            else
            {
                if (nFreeWrBufferCount == uart4Obj.wrThreshold)
                {
                    uart4Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart4Obj.wrContext);
                }
            }
        }
    }
}

static size_t UART4_WritePendingBytesGet(void)
{
    size_t nPendingTxBytes;

    /* Take a snapshot of indices to avoid processing in critical section */
    uint32_t wrOutIndex = uart4Obj.wrOutIndex;
    uint32_t wrInIndex = uart4Obj.wrInIndex;

    if ( wrInIndex >=  wrOutIndex)
    {
        nPendingTxBytes =  wrInIndex - wrOutIndex;
    }
    else
    {
        nPendingTxBytes =  (UART4_WRITE_BUFFER_SIZE -  wrOutIndex) + wrInIndex;
    }

    return nPendingTxBytes;
}

size_t UART4_WriteCountGet(void)
{
    size_t nPendingTxBytes;

    nPendingTxBytes = UART4_WritePendingBytesGet();

    return nPendingTxBytes;
}

size_t UART4_Write(uint8_t* pWrBuffer, const size_t size )
{
    size_t nBytesWritten  = 0;

    UART4_TX_INT_DISABLE();

    while (nBytesWritten < size)
    {
        if (UART4_TxPushByte(pWrBuffer[nBytesWritten]) == true)
        {
            nBytesWritten++;
        }
        else
        {
            /* Queue is full, exit the loop */
            break;
        }
    }

    /* Check if any data is pending for transmission */
    if (UART4_WritePendingBytesGet() > 0)
    {
        /* Enable TX interrupt as data is pending for transmission */
        UART4_TX_INT_ENABLE();
    }

    return nBytesWritten;
}

size_t UART4_WriteFreeBufferCountGet(void)
{
    return (UART4_WRITE_BUFFER_SIZE - 1) - UART4_WriteCountGet();
}

size_t UART4_WriteBufferSizeGet(void)
{
    return (UART4_WRITE_BUFFER_SIZE - 1);
}

bool UART4_WriteNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart4Obj.isWrNotificationEnabled;

    uart4Obj.isWrNotificationEnabled = isEnabled;

    uart4Obj.isWrNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART4_WriteThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart4Obj.wrThreshold = nBytesThreshold;
    }
}

void UART4_WriteCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart4Obj.wrCallback = callback;

    uart4Obj.wrContext = context;
}

UART_ERROR UART4_ErrorGet( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint32_t status = U4STA;

    errors = (UART_ERROR)(status & (_U4STA_OERR_MASK | _U4STA_FERR_MASK | _U4STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        UART4_ErrorClear();
    }

    /* All errors are cleared, but send the previous error state */
    return errors;
}

void UART4_FAULT_InterruptHandler (void)
{
    /* Disable the fault interrupt */
    IEC5CLR = _IEC5_U4EIE_MASK;
    /* Disable the receive interrupt */
    IEC5CLR = _IEC5_U4RXIE_MASK;

    /* Client must call UARTx_ErrorGet() function to clear the errors */
    if( uart4Obj.rdCallback != NULL )
    {
        uart4Obj.rdCallback(UART_EVENT_READ_ERROR, uart4Obj.rdContext);
    }
}

void UART4_RX_InterruptHandler (void)
{
    /* Keep reading until there is a character availabe in the RX FIFO */
    while((U4STA & _U4STA_URXDA_MASK) == _U4STA_URXDA_MASK)
    {
        if (UART4_RxPushByte( (uint8_t )(U4RXREG) ) == true)
        {
            UART4_ReadNotificationSend();
        }
        else
        {
            /* UART RX buffer is full */
        }
    }

    /* Clear UART4 RX Interrupt flag */
    IFS5CLR = _IFS5_U4RXIF_MASK;
}

void UART4_TX_InterruptHandler (void)
{
    uint8_t wrByte;

    /* Check if any data is pending for transmission */
    if (UART4_WritePendingBytesGet() > 0)
    {
        /* Keep writing to the TX FIFO as long as there is space */
        while(!(U4STA & _U4STA_UTXBF_MASK))
        {
            if (UART4_TxPullByte(&wrByte) == true)
            {
                U4TXREG = wrByte;

                /* Send notification */
                UART4_WriteNotificationSend();
            }
            else
            {
                /* Nothing to transmit. Disable the data register empty interrupt. */
                UART4_TX_INT_DISABLE();
                break;
            }
        }

        /* Clear UART4TX Interrupt flag */
        IFS5CLR = _IFS5_U4TXIF_MASK;
    }
    else
    {
        /* Nothing to transmit. Disable the data register empty interrupt. */
        UART4_TX_INT_DISABLE();
    }
}

