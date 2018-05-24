//---------------------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### COMMS_FROM_RASP.C ############################################
//---------------------------------------------------------

/* Includes ------------------------------------------------------------------*/
// #include "stm32f10x.h"
#include "hard.h"

#include "comms_from_rasp.h"
#include "comms.h"
#include "uart.h"
#include "treatment.h"

#include <string.h>
#include <stdio.h>



/* Externals ------------------------------------------------------------------*/
extern volatile unsigned char rpi_have_data;
extern unsigned short comms_messages;



/* Globals ------------------------------------------------------------------*/
char local_rasp_buff [SIZEOF_RXDATA];

const char s_getall [] = {"get all conf"};

/* Module Private Functions -----------------------------------------------------------*/
static void RaspBerry_Messages (char *);
static void SendAllConf (void);

/* Module Exported Functions -----------------------------------------------------------*/
void UpdateRaspberryMessages (void)
{
    if (rpi_have_data)
    {
        rpi_have_data = 0;
        // L_ALARMA_ON;
        ReadRPIBuffer((unsigned char *) local_rasp_buff, SIZEOF_RXDATA);
        RaspBerry_Messages(local_rasp_buff);
        // L_ALARMA_OFF;
    }
}


static void RaspBerry_Messages (char * msg)
{
    unsigned short hours = 0;
    unsigned short minutes = 0;
    unsigned short seconds = 0;
    
    if (!strncmp(msg, (const char *)"get_temp,", (sizeof("get_temp,") - 1)))
    {
        switch (*(msg+9))
        {
        case '1':
            // UART_CH1_Send("get_temp\r\n");
            RPI_Send("OK\r\n");
            break;

        case '2':
            // UART_CH2_Send("get_temp\r\n");
            RPI_Send("OK\r\n");
            break;

        case '3':
            // UART_CH3_Send("get_temp\r\n");
            RPI_Send("OK\r\n");
            break;

        default:
            RPI_Send("ERROR\r\n");
            break;
        }
    }

    else if (!strncmp(msg, (const char *)"signal,", (sizeof("signal,") - 1)))
    {
        //signal,%03d,%03d,0%x%x%d,%04d,%04d,%04d,%04d,%04d,%04d,1\r\n",
        if (((*(msg + 50) -'0') > 0) && ((*(msg + 50) - '0') < 4) && (*(msg + 49) == ','))
        {
            if (TreatmentTranslateOldMsg( msg ) == resp_ok)
            {
                RPI_Send((char *)"OK\r\n");
                comms_messages |= COMM_CONF_CHANGE;
            }
            else
                RPI_Send((char *)"ERROR\r\n");            
        }
        else
            RPI_Send((char *)"ERROR\r\n");

    }
//     //example.	duration,00,10,00,1
    else if (!strncmp(msg, (const char *)"duration,", (sizeof("duration,") - 1)))
    {
        if ((*(msg + 18) > '0') && ((*(msg + 18)) < '4') && (*(msg + 17) == ','))
        {
            hours = (*(msg + 9) - '0') * 10 + *(msg + 10) - '0';
            minutes = (*(msg + 12) - '0') * 10 + *(msg + 13) - '0';
            seconds = (*(msg + 15) - '0') * 10 + *(msg + 16) - '0';

            if (TreatmentSetTime(hours, minutes, seconds) == resp_ok)
            {
                RPI_Send((char *)"OK\r\n");
                comms_messages |= COMM_CONF_CHANGE;
            }
            else
                RPI_Send((char *)"ERROR\r\n");
        }
        else
            RPI_Send((char *)"ERROR\r\n");
    }

    else if (!strncmp(msg, (const char *)"save,01", (sizeof("save,01") - 1)))
    {
        // i = FLASH_Program(&session_slot_aux, ((buffUART1rx2[5] - 48) * 10 + (buffUART1rx2[6] - 48)));

        // if (i == FIN_OK)
        // {
        //     RPI_Send("Guardado OK\r\n");
        //     Wait_ms(1000);
        //     //Reset.
        //     NVIC_SystemReset();
        // }

        // if (i == FIN_ERROR)
        // {
        //     RPI_Send("Guardado ERROR\r\n");
        //     Wait_ms(1000);
        //     //Reset.
        //     NVIC_SystemReset();
        // }
        RPI_Send((char *)"OK\r\n");
    }

    else if (!strncmp(msg, (const char *)"load,01", (sizeof("load,01") - 1)))
    {
        RPI_Send((char *)"OK\r\n");
    }

    else if (!strncmp(msg, (const char *)"stop,", (sizeof("stop,") - 1)))
    {
        comms_messages |= COMM_STOP_TREAT;
        
        // RPI_Send((const char *)"OK\r\n");
    }

    else if (!strncmp(msg, (const char *)"pause,", (sizeof("pause,") - 1)))
    {
        comms_messages |= COMM_PAUSE_TREAT;
        
        // RPI_Send((const char *)"OK\r\n");
    }
    
    else if (!strncmp(msg, (const char *)"reset", (sizeof("reset") - 1)))
    {
        RPI_Send((const char *) "Restarting...\r\n");
        // Wait_ms(1000);
        // NVIC_SystemReset();
    }

    else if (!strncmp(msg,(const char *)"start,", (sizeof("start,") - 1)))
    {
        comms_messages |= COMM_START_TREAT;

        // RPI_Send((char *)"OK\r\n");
    }



//     //example. state_of_stage,0,1
//     else if (!strncmp(msg, (const char *)"state_of_stage,", (sizeof("state_of_stage,") - 1)))
//     {

//         if (((buffUART1rx2[15] - 48) == 0) || ((buffUART1rx2[15] - 48) == 1))
//         {

//             Session_Set_Status (&session_slot_aux, (buffUART1rx2[17] - 48),(buffUART1rx2[15] - 48));

//             RPI_Send((char *)"OK\r\n");
//         }
//         else
//             RPI_Send((char *)"ERROR\r\n");

//     }

//     else if (!strncmp(msg, (const char *)"special_function,", (sizeof("special_function,") - 1)))
//     {

//         RPI_Send((char *)"OK\r\n");
//     }


//     else if (!strncmp(msg, (const char *)"stop,", (sizeof("stop,") - 1)))
//     {
// /*			if((buffUART3rx2[5] - 48) == 1)
//                         Session_Channel_1_Stop();
// 			if((buffUART3rx2[5] - 48) == 2)
//                         Session_Channel_2_Stop();
// 			if((buffUART3rx2[5] - 48) == 3)
//                         Session_Channel_3_Stop();
// 			if((buffUART3rx2[5] - 48) == 4)
//                         Session_Channel_4_Stop();
// */
//         Session_Channel_1_Stop();
//         Session_Channel_2_Stop();
//         Session_Channel_3_Stop();
//         Session_Channel_4_Stop();

//         RPI_Send((char *)"OK\r\n");
//     }

// #ifdef HARDWARE_VERSION_2_1
//     else if (!strncmp(msg, (const char *)"finish_ok,", (sizeof("finish_ok,") - 1)))
//     {
//         Session_Channel_1_Stop();
//         Session_Channel_2_Stop();
//         Session_Channel_3_Stop();
//         Session_Channel_4_Stop();

//         BuzzerCommands(BUZZER_MULTIPLE_SHORT, 3);
//         RPI_Send((char *)"OK\r\n");
//     }
// #endif

//     else if (!strncmp(msg, (const char *)"read_channel,", (sizeof("read_channel,") - 1)))
//     {
//         if (((buffUART1rx2[13] - 48) < 5) && ((buffUART1rx2[13] - 48) > 0))
//         {
//             Channel_Load (&session_slot_aux, (buffUART1rx2[13] - 48));

//             //--- Send slot content ---//
//             SessionSend(&session_slot_aux);

//             RPI_Send((char *)"OK\r\n");
//         }
//         else
//             RPI_Send((char *)"ERROR\r\n");
//     }

//     else if (!strncmp(msg, (const char *)"read_slot,", (sizeof("read_slot,") - 1)))
//     {
//         //load,slot[2],destino[1]
//         if ((((buffUART1rx2[11] - 48) + (buffUART1rx2[10] - 48) * 10) < 11))
//         {
//             Session_Load (&session_slot_aux, (((buffUART1rx2[10] - 48) * 10) + buffUART1rx2[11] - 48), 0);

//             //--- Send slot content ---//
//             SessionSend(&session_slot_aux);

//             RPI_Send((char *)"OK\r\n");
//         }
//         else
//             RPI_Send((char *)"ERROR\r\n");
//     }

//     else
//         RPI_Send((char *)"ERROR\r\n");
    else if (strncmp(msg, s_getall, sizeof(s_getall) - 1) == 0)
        SendAllConf();


//     //--- end ---//

}

void RaspBerry_Report_Errors (unsigned short * errors)
{
    //reporta errores como "ERROR(0xNN)\r\n"
    //0x1N antena desconectada
    //0x2N antena perdida
    //0x4N sobre temperatura
    //0x5N sobre corriente
    //N num de canal 1-4

    char buff [20];
    unsigned short err = 0;
    unsigned short chn = 0;

    sprintf(buff, "0x%04x\r\n", *errors);
    RPI_Send(buff);
    
    err = *errors & COMM_POWER_ERR_MASK;
    chn = *errors & COMM_POWER_CHN_MASK;

    
    while (err | chn)
    {
        switch (err)
        {
        case COMM_ERROR_OVERCURRENT:
            err &= ~COMM_ERROR_OVERCURRENT;
            if (chn == COMM_ERROR_IN_CH1)
            {
                chn &= ~COMM_ERROR_IN_CH1;
                RPI_Send("ERROR(0x51)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH2)
            {
                chn &= ~COMM_ERROR_IN_CH2;
                RPI_Send("ERROR(0x52)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH3)
            {
                chn &= ~COMM_ERROR_IN_CH3;
                RPI_Send("ERROR(0x53)\r\n");
            }            
            break;

        case COMM_ERROR_NO_CURRENT:
            err &= ~COMM_ERROR_NO_CURRENT;
            if (chn == COMM_ERROR_IN_CH1)
            {
                chn &= ~COMM_ERROR_IN_CH1;
                RPI_Send("ERROR(0x21)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH2)
            {
                chn &= ~COMM_ERROR_IN_CH2;
                RPI_Send("ERROR(0x22)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH3)
            {
                chn &= ~COMM_ERROR_IN_CH3;
                RPI_Send("ERROR(0x23)\r\n");
            }                        
            break;

        case COMM_ERROR_SOFT_OVERCURRENT:
            err &= ~COMM_ERROR_SOFT_OVERCURRENT;
            if (chn == COMM_ERROR_IN_CH1)
            {
                chn &= ~COMM_ERROR_IN_CH1;
                RPI_Send("ERROR(0x51)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH2)
            {
                chn &= ~COMM_ERROR_IN_CH2;
                RPI_Send("ERROR(0x52)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH3)
            {
                chn &= ~COMM_ERROR_IN_CH3;
                RPI_Send("ERROR(0x53)\r\n");
            }                                    
            break;

        case COMM_ERROR_OVERTEMP:
            err &= ~COMM_ERROR_OVERTEMP;
            if (chn == COMM_ERROR_IN_CH1)
            {
                chn &= ~COMM_ERROR_IN_CH1;
                RPI_Send("ERROR(0x41)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH2)
            {
                chn &= ~COMM_ERROR_IN_CH2;
                RPI_Send("ERROR(0x42)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH3)
            {
                chn &= ~COMM_ERROR_IN_CH3;
                RPI_Send("ERROR(0x43)\r\n");
            }                                                
            break;

        default:
            //debe ser falta de comunicacion
            if (chn == COMM_ERROR_IN_CH1)
            {
                chn &= ~COMM_ERROR_IN_CH1;
                RPI_Send("ERROR(0x11)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH2)
            {
                chn &= ~COMM_ERROR_IN_CH2;
                RPI_Send("ERROR(0x12)\r\n");
            }

            if (chn == COMM_ERROR_IN_CH3)
            {
                chn &= ~COMM_ERROR_IN_CH3;
                RPI_Send("ERROR(0x13)\r\n");
            }                                                            
            break;
        }

        sprintf(buff, "err: 0x%04x chn: 0x%04x\r\n", err, chn);
        RPI_Send(buff);

    }            
}

static void SendAllConf (void)
{
    char b [128];
    TreatmentGetAllConf(b);
    RPI_Send(b);
}























//---- End of File ----//
