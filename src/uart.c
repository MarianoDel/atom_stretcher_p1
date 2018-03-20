//--------------------------------
// #### PROYECTO STRETCHER P1 ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### UART.C ###################
//--------------------------------

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "stm32f0xx.h"
#include "uart.h"


#include <string.h>




//--- Private typedef ---//
//--- Private define ---//
//--- Private macro ---//



//--- VARIABLES EXTERNAS ---//

extern volatile unsigned char usart1_have_data;
extern volatile unsigned char usart2_have_data;


//--- Private variables ---//
volatile unsigned char * ptx1;
volatile unsigned char * ptx1_pckt_index;
volatile unsigned char * prx1;
volatile unsigned char tx1buff[SIZEOF_DATA];
volatile unsigned char rx1buff[SIZEOF_DATA];

volatile unsigned char * ptx2;
volatile unsigned char * ptx2_pckt_index;
volatile unsigned char * prx2;
volatile unsigned char tx2buff[SIZEOF_DATA];
volatile unsigned char rx2buff[SIZEOF_DATA];


//Reception buffer.

//Transmission buffer.

//--- Private function prototypes ---//
//--- Private functions ---//
unsigned char ReadUsart1Buffer (unsigned char * bout, unsigned short max_len)
{
    unsigned int len;

    len = prx1 - rx1buff;

    if (len < max_len)
    {
        //el prx1 siempre llega adelantado desde la int, lo corto con un 0
        *prx1 = '\0';
        prx1++;
        len += 1;
        memcpy(bout, (unsigned char *) rx1buff, len);
    }
    else
    {
        memcpy(bout, (unsigned char *) rx1buff, len);
        len = max_len;
    }

    //ajusto punteros de rx luego de la copia
    prx1 = rx1buff;
    return (unsigned char) len;
}

unsigned char ReadUsart2Buffer (unsigned char * bout, unsigned short max_len)
{
    unsigned int len;

    len = prx2 - rx2buff;

    if (len < max_len)
    {
        //el prx2 siempre llega adelantado desde la int, lo corto con un 0
        *prx2 = '\0';
        prx2++;
        len += 1;
        memcpy(bout, (unsigned char *) rx2buff, len);
    }
    else
    {
        memcpy(bout, (unsigned char *) rx2buff, len);
        len = max_len;
    }

    //ajusto punteros de rx luego de la copia
    prx2 = rx2buff;
    return (unsigned char) len;
}

void USART1_IRQHandler(void)
{
    unsigned short i;
    unsigned char dummy;

    /* USART in mode Receiver --------------------------------------------------*/
    if (USART1->ISR & USART_ISR_RXNE)
    {
        dummy = USART1->RDR & 0x0FF;
        
        if (prx1 < &rx1buff[SIZEOF_RXDATA])
        {
            if ((dummy == '\n') || (dummy == '\r') || (dummy == 26))		//26 es CTRL-Z
            {
                *prx1 = '\0';
                usart1_have_data = 1;
            }
            else
            {
                *prx1 = dummy;
                prx1++;
            }
        }        
    }

    /* USART in mode Transmitter -------------------------------------------------*/
    //if (USART_GetITStatus(USARTx, USART_IT_TXE) == SET)


    if (USART1->CR1 & USART_CR1_TXEIE)
    {
        if (USART1->ISR & USART_ISR_TXE)
        {
            if ((ptx1 < &tx1buff[SIZEOF_DATA]) && (ptx1 < ptx1_pckt_index))
            {
                USART1->TDR = *ptx1;
                ptx1++;
            }
            else
            {
                ptx1 = tx1buff;
                ptx1_pckt_index = tx1buff;
                USART1->CR1 &= ~USART_CR1_TXEIE;
            }
        }
    }

    if ((USART1->ISR & USART_ISR_ORE) || (USART1->ISR & USART_ISR_NE) || (USART1->ISR & USART_ISR_FE))
    {
        USART1->ICR |= 0x0e;
        dummy = USART1->RDR;
    }
}

void USART2_IRQHandler(void)
{
    unsigned char dummy;

    /* USART in mode Receiver --------------------------------------------------*/
    if (USART2->ISR & USART_ISR_RXNE)
    {
        dummy = USART2->RDR & 0x0FF;

        if (prx2 < &rx2buff[SIZEOF_RXDATA])
        {
            if ((dummy == '\n') || (dummy == '\r') || (dummy == 26))		//26 es CTRL-Z
            {
                *prx2 = '\0';
                usart2_have_data = 1;
            }
            else
            {
                *prx2 = dummy;
                prx2++;
            }
        }
    }
    /* USART in mode Transmitter -------------------------------------------------*/

    if (USART2->CR1 & USART_CR1_TXEIE)
    {
        if (USART2->ISR & USART_ISR_TXE)
        {
            if ((ptx2 < &tx2buff[SIZEOF_DATA]) && (ptx2 < ptx2_pckt_index))
            {
                USART2->TDR = *ptx2;
                ptx2++;
            }
            else
            {
                ptx2 = tx2buff;
                ptx2_pckt_index = tx2buff;
                USART2->CR1 &= ~USART_CR1_TXEIE;
            }
        }
    }

    if ((USART2->ISR & USART_ISR_ORE) || (USART2->ISR & USART_ISR_NE) || (USART2->ISR & USART_ISR_FE))
    {
        USART2->ICR |= 0x0e;
        dummy = USART2->RDR;
    }
}

void Usart2Send (char * send)
{
    unsigned char i;

    i = strlen(send);
    Usart2SendUnsigned(send, i);
}

void Usart2SendUnsigned(unsigned char * send, unsigned char size)
{
    if ((ptx2_pckt_index + size) < &tx2buff[SIZEOF_DATA])
    {
        memcpy((unsigned char *)ptx2_pckt_index, send, size);
        ptx2_pckt_index += size;
        USART2->CR1 |= USART_CR1_TXEIE;
    }
}

void Usart1Send (char * send)
{
    unsigned char i;

    i = strlen(send);
    Usart1SendUnsigned(send, i);
}

void Usart1SendUnsigned(unsigned char * send, unsigned char size)
{
    if ((ptx1_pckt_index + size) < &tx1buff[SIZEOF_DATA])
    {
        memcpy((unsigned char *)ptx1_pckt_index, send, size);
        ptx1_pckt_index += size;
        USART1->CR1 |= USART_CR1_TXEIE;
    }
}


#ifdef VER_1_3
void USART2Config(void)
{
    if (!USART2_CLK)
        USART2_CLK_ON;

    GPIOA->AFR[0] |= 0x0001100;	//PA2 -> AF1 PA3 -> AF1

    USART2->BRR = USART_9600;
#ifdef DATALOGGER
    //doy vuelta el pin TX porque usa OPTO para conectarse a la otra placa
    USART2->CR2 |= USART_CR2_TXINV;
#endif
    USART2->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

    ptx2 = tx2buff;
    ptx2_pckt_index = tx2buff;
    prx2 = rx2buff;
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 7);
}
#endif

void USART1Config(void)
{
    if (!USART1_CLK)
        USART1_CLK_ON;

#ifdef VER_1_3
    GPIOB->AFR[0] |= 0x00000000;	//PB7 -> AF0 PB6 -> AF0
#endif
#ifdef VER_1_2
    GPIOA->AFR[1] |= 0x00000110;	//PA10 -> AF1 PA9 -> AF1
#endif

    USART1->BRR = USART_9600;
    // USART1->CR2 |= USART_CR2_STOP_1;	//2 bits stop
//	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
//	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;	//SIN TX
    USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;	//para pruebas TX

    ptx1 = tx1buff;
    ptx1_pckt_index = tx1buff;
    prx1 = rx1buff;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 5);
}

//--- end of file ---//
