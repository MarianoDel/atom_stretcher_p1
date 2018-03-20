//--------------------------------
// #### PROYECTO STRETCHER P1 ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### MAIN.C ###################
//--------------------------------


/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_adc.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//--- My includes ---//
#include "stm32f0x_gpio.h"
#include "stm32f0x_tim.h"
#include "uart.h"
#include "hard.h"

#include "core_cm0.h"
#include "adc.h"
#include "flash_program.h"


//--- VARIABLES EXTERNAS ---//
volatile unsigned char timer_1seg = 0;

volatile unsigned short timer_led_comm = 0;
volatile unsigned short timer_for_cat_switch = 0;
volatile unsigned short timer_for_cat_display = 0;

volatile unsigned short wait_ms_var = 0;

unsigned char new_ldr_sample;

// ------- Externals del Puerto serie  -------
volatile unsigned char usart1_have_data = 0;
volatile unsigned char usart2_have_data = 0;

// ------- Externals de los timers -------
//volatile unsigned short prog_timer = 0;
//volatile unsigned short mainmenu_timer = 0;
volatile unsigned short show_select_timer = 0;
volatile unsigned char switches_timer = 0;
volatile unsigned char acswitch_timer = 0;

volatile unsigned short scroll1_timer = 0;
volatile unsigned short scroll2_timer = 0;

volatile unsigned short standalone_timer;
volatile unsigned short standalone_enable_menu_timer;
//volatile unsigned short standalone_menu_timer;
volatile unsigned char grouped_master_timeout_timer;
volatile unsigned short take_temp_sample = 0;
volatile unsigned short take_ldr_sample = 0;
volatile unsigned short minutes = 0;
volatile unsigned char timer_wifi_bright = 0;

// ------- Externals de los modos -------
unsigned char saved_mode;

// ------- para determinar igrid -------
volatile unsigned char igrid_timer = 0;
volatile unsigned char vgrid_timer = 0;

// ------- del display LCD -------
const char s_blank_line [] = {"                "};

// ------- Externals de los switches -------
unsigned short s1;
unsigned short s2;
unsigned short sac;
unsigned char sac_aux;

//#ifdef WIFI_TO_CEL_PHONE_PROGRAM
unsigned char messages [100];
//#endif
//--- VARIABLES GLOBALES ---//
parameters_typedef param_struct;

// ------- de los timers -------
volatile unsigned short timer_standby;
volatile unsigned char filter_timer;


// ------- de los filtros ADC -------
#ifdef DATALOGGER
unsigned short v_adc0 [DATALOGGER_FILTER];
unsigned short v_adc1 [DATALOGGER_FILTER];
#endif


#define IDLE	0
#define LOOK_FOR_BREAK	1
#define LOOK_FOR_MARK	2
#define LOOK_FOR_START	3


//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);


// ------- del DMX -------
extern void EXTI4_15_IRQHandler(void);
#define DMX_TIMEOUT	20

//--- FILTROS DE SENSORES ---//
#define LARGO_FILTRO 16
#define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
unsigned short vtemp [LARGO_FILTRO + 1];
unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    unsigned char i,ii;
    unsigned char bytes_remain, bytes_readed, need_ack = 0;
    unsigned short local_meas, local_meas_last;
    unsigned char main_state = 0;


#ifdef DATALOGGER
    char s_to_send [100];
#endif

#ifdef PULSE_GENERATOR
    unsigned short pulse_size = 0;
    char s_to_send [100];
#endif

#ifdef STRETCHER_P1
    char s_to_senda [100];
    char s_to_sendb [100];
#endif

    parameters_typedef * p_mem_init;
    //!< At this stage the microcontroller clock setting is already configured,
    //   this is done through SystemInit() function which is called from startup
    //   file (startup_stm32f0xx.s) before to branch to application main.
    //   To reconfigure the default setting of SystemInit() function, refer to
    //   system_stm32f0xx.c file

    //GPIO Configuration.
    GPIO_Config();


    //ACTIVAR SYSTICK TIMER
    if (SysTick_Config(48000))
    {
        while (1)	/* Capture error */
        {
            if (LED)
                LED_OFF;
            else
                LED_ON;

            for (i = 0; i < 255; i++)
            {
                asm (	"nop \n\t"
                        "nop \n\t"
                        "nop \n\t" );
            }
        }
    }

//------- PROGRAMA DE PRUEBA DATALOGGER -----//

//------- PROGRAMA P1 STRETCHER -----//
#ifdef STRETCHER_P1
    // TIM_15_Init();

    //--- PRUEBA DISPLAY LCD ---
    EXTIOff ();

    //TODO: enviar tipo de programa al puerto serie    
    USART2Config();
    USART1Config();
    Wait_ms(1000);

// #ifdef STRETCHER_P1
    Usart1Send((char *) (const char *) "\r\nKirno Stretcher over P1\r\n");
    Usart1Send((char *) (const char *) "HW Ver: 1.3\n");
    Usart1Send((char *) (const char *) "SW Ver: 1.0\n");
// #endif

    while (1)
    {
        if (usart2_have_data)
        {
            usart2_have_data = 0;
            bytes_readed = ReadUsart2Buffer(s_to_sendb, sizeof(s_to_sendb));

            if ((bytes_readed + 1) < sizeof(s_to_sendb))
            {
                *(s_to_sendb + bytes_readed) = '\n';
                *(s_to_sendb + bytes_readed + 1) = '\0';
                Usart1Send(s_to_sendb);
            }
            
            // if (LED)
            //     LED_OFF;
            // else
            //     LED_ON;
        }

        if (usart1_have_data)
        {
            usart1_have_data = 0;
            bytes_readed = ReadUsart1Buffer(s_to_senda, sizeof(s_to_senda));

            if ((bytes_readed + 1) < sizeof(s_to_senda))
            {
                *(s_to_senda + bytes_readed) = '\n';
                *(s_to_senda + bytes_readed + 1) = '\0';
                Usart2Send(s_to_senda);
            }

            if (LED)
                LED_OFF;
            else
                LED_ON;
        }
            
        // Usart2Send("prueba\n");
        // Wait_ms(1000);
        // LED_OFF;
        // Wait_ms(1000);

        // UpdateSwitches();

    }


#endif    //end of Stretcher
//------- FIN PROGRAMA P1 STRETCHER -----//    

#ifdef DATALOGGER
    //ADC Configuration
    AdcConfig();

    USART2Config();

    //Pre cargo los filtros
    local_meas = ReadADC1_SameSampleTime (ADC_Channel_5);
    local_meas_last = ReadADC1_SameSampleTime (ADC_Channel_8);
    for (i = 0; i < DATALOGGER_FILTER; i++)
    {
        v_adc0[i] = local_meas;
        v_adc1[i] = local_meas_last;
    }
    LED_ON;

    while (1)
    {
        //Prueba LED
        // if (LED)
        // 	LED_OFF;
        // else
        // 	LED_ON;
        //
        // Wait_ms(1000);
        //Fin Prueba LED
        for (i = 0; i < DATALOGGER_FILTER; i++)
        {
            local_meas = ReadADC1_SameSampleTime (ADC_Channel_5);
            local_meas = MAFilter32(local_meas, v_adc0);
            local_meas_last = ReadADC1_SameSampleTime (ADC_Channel_8);
            local_meas_last = MAFilter32(local_meas_last, v_adc1);

            Wait_ms(UPDATE_FILTER);
        }

        if (LOGGER_INPUT)
            sprintf(s_to_send, "%04d,%04d,1,\r\n",local_meas,local_meas_last);
        else
            sprintf(s_to_send, "%04d,%04d,0,\r\n",local_meas,local_meas_last);
        Usart2Send(s_to_send);
    }

#endif    //end of Datalogger
//------- FIN DE PROGRAMA DATALLOGER -----------------//

//------- PROGRAMA DE PRUEBA GENERADOR DE PULSOS -----//
#ifdef PULSE_GENERATOR
    TIM_15_Init();

    //--- PRUEBA DISPLAY LCD ---
    EXTIOff ();

    LCDInit();
    LED_ON;

    //--- Welcome code ---//
    Lcd_Command(CLEAR);
    Wait_ms(100);
    Lcd_Command(CURSOR_OFF);
    Wait_ms(100);
    Lcd_Command(BLINK_OFF);
    Wait_ms(100);
    CTRL_BKL_ON;

    while (FuncShowBlink ((const char *) "Kirno Technology", (const char *) " Pulse Generator", 2, BLINK_NO) != RESP_FINISH);
    LED_OFF;
    // LCD_1ER_RENGLON;
    // LCDTransmitStr(s_blank_line);
    LCD_2DO_RENGLON;
    LCDTransmitStr(s_blank_line);

    LCD_1ER_RENGLON;
    LCDTransmitStr(" 1PPS           ");


    while (1)
    {
        if (!timer_standby)
        {
            timer_standby = 1000;	//pulsos cada 1 segundo
            OneShootTIM15 (pulse_size);

            if (LED)
                LED_OFF;
            else
                LED_ON;

            // LCD_1ER_RENGLON;
            Lcd_SetDDRAM(0x00 + 7);
            sprintf(s_to_send, "p: %5d", pulse_size);
            LCDTransmitStr(s_to_send);

            CalculateTime(pulse_size, s_to_send);
            LCD_2DO_RENGLON;
            LCDTransmitStr(s_to_send);
        }

        if (TIM15->CNT > TIM15->CCR1)
        {
            //me fijo si corre el timer
            if (TIM15->CR1 & TIM_CR1_CEN)
                TIM15->CR1 &= ~TIM_CR1_CEN;
        }





        if (CheckS1() > S_NO)
        {
            if (pulse_size > 0)
            {
                pulse_size--;
                Wait_ms(60);
            }

            if (CheckS1() > S_HALF)
            {
                if (pulse_size > 25)
                    pulse_size -= 25;
                else if (pulse_size > 0)
                    pulse_size--;
            }
        }

        if (CheckS2() > S_NO)
        {
            if (pulse_size < 0xFFFF)
            {
                pulse_size++;
                Wait_ms(60);
            }


            if (CheckS2() > S_HALF)
                if (pulse_size < 0xFFFF)
                    pulse_size += 25;


        }

        UpdateSwitches();

    }


#endif    //end of Pulse Genetator

    //---------- Fin Programa de Procduccion --------//

    return 0;
}
//--- End of Main ---//





// void EXTI4_15_IRQHandler(void)
// {
// 	unsigned short aux;

// //--- SOLO PRUEBA DE INTERRUPCIONES ---//
// //	if (DMX_INPUT)
// //		LED_ON;
// //	else
// //		LED_OFF;
// //
// //	EXTI->PR |= 0x0100;

// 	if(EXTI->PR & 0x0100)	//Line8
// 	{

// 		//si no esta con el USART detecta el flanco	PONER TIMEOUT ACA?????
// 		if ((dmx_receive_flag == 0) || (dmx_timeout_timer == 0))
// 		//if (dmx_receive_flag == 0)
// 		{
// 			switch (signal_state)
// 			{
// 				case IDLE:
// 					if (!(DMX_INPUT))
// 					{
// 						//Activo timer en Falling.
// 						TIM14->CNT = 0;
// 						TIM14->CR1 |= 0x0001;
// 						signal_state++;
// 					}
// 					break;

// 				case LOOK_FOR_BREAK:
// 					if (DMX_INPUT)
// 					{
// 						//Desactivo timer en Rising.
// 						aux = TIM14->CNT;

// 						//reviso BREAK
// 						//if (((tim_counter_65ms) || (aux > 88)) && (tim_counter_65ms <= 20))
// 						if ((aux > 87) && (aux < 210))	//Consola STARLET 6
// 						//if ((aux > 87) && (aux < 2000))		//Consola marca CODE tiene break 1.88ms
// 						{
// 							LED_ON;
// 							//Activo timer para ver MARK.
// 							//TIM2->CNT = 0;
// 							//TIM2->CR1 |= 0x0001;

// 							signal_state++;
// 							//tengo el break, activo el puerto serie
// 							DMX_channel_received = 0;
// 							//dmx_receive_flag = 1;

// 							dmx_timeout_timer = DMX_TIMEOUT;		//activo el timer cuando prendo el puerto serie
// 							//USARTx_RX_ENA;
// 						}
// 						else	//falso disparo
// 							signal_state = IDLE;
// 					}
// 					else	//falso disparo
// 						signal_state = IDLE;

// 					TIM14->CR1 &= 0xFFFE;
// 					break;

// 				case LOOK_FOR_MARK:
// 					if ((!(DMX_INPUT)) && (dmx_timeout_timer))	//termino Mark after break
// 					{
// 						//ya tenia el serie habilitado
// 						//if ((aux > 7) && (aux < 12))
// 						dmx_receive_flag = 1;
// 					}
// 					else	//falso disparo
// 					{
// 						//termine por timeout
// 						dmx_receive_flag = 0;
// 						//USARTx_RX_DISA;
// 					}
// 					signal_state = IDLE;
// 					LED_OFF;
// 					break;

// 				default:
// 					signal_state = IDLE;
// 					break;
// 			}
// 		}

// 		EXTI->PR |= 0x0100;
// 	}
// }
//#endif

void TimingDelay_Decrement(void)
{
	if (wait_ms_var)
		wait_ms_var--;

//	if (display_timer)
//		display_timer--;

	if (timer_standby)
		timer_standby--;

	if (switches_timer)
		switches_timer--;


}

//------ EOF -------//
