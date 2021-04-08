/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"

/* Library includes. */
//#include "stm32f10x_lib.h"

#include "usbd_cdc_if.h"
//#include "usart.h"
/* Demo application includes. */
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )


/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
//QueueHandle_t xRxedChars;
//static QueueHandle_t xCharsForTx;
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
//void vUARTInterruptHandler( void );
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    xComPortHandle xReturn;


	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	{
		
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/

////signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
////{
////	/* The port handle is not required as this driver only supports one port. */
////	( void ) pxPort;

////	/* Get the next character from the buffer.  Return false if no characters
////	are available, or arrive before xBlockTime expires. */
////	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
////	{
////		return pdTRUE;
////	}
////	else
////	{
////		return pdFALSE;
////	}
////}
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* The port handle is not required as this driver only supports one port. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
    ///if( osMessageQueueGet(xRxedChars, (void*)&pcRxedChar, NULL, xBlockTime) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
 
/*-----------------------------------------------------------*/
////void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
////{
////signed char *pxNext;

////	/* A couple of parameters that this port does not use. */
////	( void ) usStringLength;
////	( void ) pxPort;

////	/* NOTE: This implementation does not handle the queue being full as no
////	block time is used! */

////	/* The port handle is not required as this driver only supports UART1. */
////	( void ) pxPort;

////	/* Send each character in the string, one at a time. */
////	pxNext = ( signed char * ) pcString;
////	while( *pxNext )
////	{
////		xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
////		pxNext++;
////	}
////}
void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
  signed char *pxNext;

  /* A couple of parameters that this port does not use. */
  ( void ) usStringLength;
  ( void ) pxPort;

  /* NOTE: This implementation does not handle the queue being full as no
  block time is used! */

  /* The port handle is not required as this driver only supports UART1. */
  ( void ) pxPort;

  /* Send each character in the string, one at a time. */
  pxNext = ( signed char * ) pcString;
/*
    while( *pxNext )
    {
        xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
        pxNext++;
    }
*/
  CDC_Transmit_FS((uint8_t* )pxNext, usStringLength);
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
    signed portBASE_TYPE xReturn = pdPASS;

      /*
      if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
      {
          xReturn = pdPASS;
          USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
      }
      else
      {
          xReturn = pdFAIL;
      }
      */
  
      if (CDC_Transmit_FS((uint8_t* )&cOutChar, 1) == USBD_OK)
      {
        xReturn = pdPASS;
      } 
      else 
      {
        xReturn = pdFAIL;
      }

      return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

//void vUARTInterruptHandler( void )
//{
//    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//    char cChar;

//	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
//	{
//		/* The interrupt was caused by the THR becoming empty.  Are there any
//		more characters to transmit? */
//		if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
//		{
//			/* A character was retrieved from the queue so can be sent to the
//			THR now. */
//			USART_SendData( USART1, cChar );
//		}
//		else
//		{
//			USART_ITConfig( USART1, USART_IT_TXE, DISABLE );		
//		}		
//	}
//	
//	if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET )
//	{
//		cChar = USART_ReceiveData( USART1 );
//		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
//	}	
//	
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}

void vUARTInterruptHandler(uint8_t* Buf, uint32_t *Len)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  uint32_t Cnt = 0;
  while(Cnt < *Len)
  {
    xQueueSendFromISR( xRxedChars, Buf+Cnt, &xHigherPriorityTaskWoken );
    Cnt ++;
  }
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



	
