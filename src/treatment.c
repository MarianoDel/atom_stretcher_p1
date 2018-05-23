//---------------------------------------------------------
// #### PROYECTO STRETCHER MAGNETO F103 - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### TREATMENT.C ###############################
//---------------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "treatment.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// #include "hard.h"


//--- VARIABLES EXTERNAS ---//

//--- VARIABLES GLOBALES ---//
treatment_conf_t treatment_conf;
unsigned char global_error = 0;

//--- FUNCIONES DEL MODULO ---//

resp_t TreatmentSetSignalType (signal_type_t a)
{
    if ((a == SQUARE_SIGNAL) ||
        (a == TRIANGULAR_SIGNAL) ||
        (a == SINUSOIDAL_SIGNAL))

        treatment_conf.treatment_signal.signal = a;
    else
        return resp_error;

    return resp_ok;
}

signal_type_t TreatmentGetSignalType (void)
{
    return treatment_conf.treatment_signal.signal;
}

resp_t TreatmentSetFrequency (frequency_t a)
{
    if ((a == TEN_HZ) ||
        (a == THIRTY_HZ) ||
        (a == SIXTY_HZ))

        treatment_conf.treatment_signal.frequency = a;
    else
        return resp_error;

    return resp_ok;
}

frequency_t TreatmentGetFrequency (void)
{
    return treatment_conf.treatment_signal.frequency;
}

resp_t TreatmentSetPower (unsigned char a)
{
    if (a > 100)
        treatment_conf.treatment_signal.power = 100;
    else if (a < 10)
        treatment_conf.treatment_signal.power = 10;
    else
        treatment_conf.treatment_signal.power = a;

    return resp_ok;
}

unsigned char TreatmentGetPower (void)
{
    return treatment_conf.treatment_signal.power;
}

resp_t TreatmentSetTime (unsigned char h, unsigned char m, unsigned char s)
{
    if ((h > 1) || (m > 60) || (s > 60))
        return resp_error;

    treatment_conf.treatment_time = h * 3600;
    treatment_conf.treatment_time += m * 60;
    treatment_conf.treatment_time += s;
    
    return resp_ok;
}

unsigned short TreatmentGetTime (void)
{
    return treatment_conf.treatment_time;
}

void TreatmentGetAllConf (char * tosend)
{
    char buf[30];
    
    sprintf(buf, "signal: %d\n", treatment_conf.treatment_signal.signal);
    strcpy(tosend, buf);
    sprintf(buf, "freq: %d\n", treatment_conf.treatment_signal.frequency);
    strcat(tosend, buf);
    sprintf(buf, "power: %d\n", treatment_conf.treatment_signal.power);
    strcat(tosend, buf);
    sprintf(buf, "synchro: %d\n", treatment_conf.treatment_signal.synchro_needed);
    strcat(tosend, buf);
    sprintf(buf, "time in s: %d\n", treatment_conf.treatment_time);
    strcat(tosend, buf);
}

resp_t TreatmentTranslateOldMsg (char * msg)
{
    resp_t resp = resp_error;
    char num [10];
    unsigned short power;

    //signal,070,070,0000,0049,0001,0001,0049,0000,0000,1
    //       *
    strncpy(num, (msg + 7), 3);
    power = atoi(num);

    if (power > 100)
        return resp;

    //signal,070,070,0000,0049,0001,0001,0049,0000,0000,1
    //               *
    if (strncmp((msg + 15),
                (const char *) "0000,0049,0001,0001,0049,0000,0000,1",
                sizeof("0000,0049,0001,0001,0049,0000,0000,1") - 1) == 0)        
    {
        //esto es triangular 10Hz
        treatment_conf.treatment_signal.signal = TRIANGULAR_SIGNAL;
        treatment_conf.treatment_signal.frequency = TEN_HZ;
        treatment_conf.treatment_signal.power = power;        
    }

    else if (strncmp((msg + 15),
                (const char *) "0000,0016,0001,0001,0015,0000,0000,1",
                sizeof("0000,0016,0001,0001,0015,0000,0000,1") - 1) == 0)
    {
        //esto es triangular 30Hz
        treatment_conf.treatment_signal.signal = TRIANGULAR_SIGNAL;
        treatment_conf.treatment_signal.frequency = THIRTY_HZ;
        treatment_conf.treatment_signal.power = power;         
    }

    else if (strncmp((msg + 15),
                (const char *) "0000,0007,0001,0001,0007,0000,0000,1",
                sizeof("0000,0007,0001,0001,0007,0000,0000,1") - 1) == 0)
    {
        //esto es triangular 60Hz
        treatment_conf.treatment_signal.signal = TRIANGULAR_SIGNAL;
        treatment_conf.treatment_signal.frequency = SIXTY_HZ;
        treatment_conf.treatment_signal.power = power;         
    }

    else if (strncmp((msg + 15),
                (const char *) "0000,0001,0049,0001,0049,0000,0000,1",
                sizeof("0000,0001,0049,0001,0049,0000,0000,1") - 1) == 0)
    {
        //esto es cuadrada 10Hz
        treatment_conf.treatment_signal.signal = SQUARE_SIGNAL;
        treatment_conf.treatment_signal.frequency = TEN_HZ;
        treatment_conf.treatment_signal.power = power;         
    }

    else if (strncmp((msg + 15),
                (const char *) "0000,0001,0016,0001,0015,0000,0000,1",
                sizeof("0000,0001,0016,0001,0015,0000,0000,1") - 1) == 0)
    {
        //esto es cuadrada 30Hz
        treatment_conf.treatment_signal.signal = SQUARE_SIGNAL;
        treatment_conf.treatment_signal.frequency = THIRTY_HZ;
        treatment_conf.treatment_signal.power = power;         
    }

    else if (strncmp((msg + 15),
                (const char *) "0000,0001,0007,0001,0007,0000,0000,1",
                sizeof("0000,0007,0001,0001,0007,0000,0000,1") - 1) == 0)
    {
        //esto es cuadrada 60Hz
        treatment_conf.treatment_signal.signal = SQUARE_SIGNAL;
        treatment_conf.treatment_signal.frequency = SIXTY_HZ;
        treatment_conf.treatment_signal.power = power;         
    }
    
    return resp_ok;
}

//verifica que se cumplan con todos los parametros para poder enviar una senial coherente
resp_t TreatmentAssertParams (void)
{
    resp_t resp = resp_error;

    if ((treatment_conf.treatment_signal.power > 100) || (treatment_conf.treatment_signal.power < 10))
        return resp;

    if ((treatment_conf.treatment_signal.frequency != TEN_HZ) &&
        (treatment_conf.treatment_signal.frequency != THIRTY_HZ) &&
        (treatment_conf.treatment_signal.frequency != SIXTY_HZ))
        return resp;

    if ((treatment_conf.treatment_signal.signal != SQUARE_SIGNAL) &&
        (treatment_conf.treatment_signal.signal != TRIANGULAR_SIGNAL) &&
        (treatment_conf.treatment_signal.signal != SINUSOIDAL_SIGNAL))
        return resp;

    return resp_ok;
}


//--- end of file ---//
