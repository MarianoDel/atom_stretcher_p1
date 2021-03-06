
/* Includes ------------------------------------------------------------------*/
#include "comm.h"
#include "uart.h"

#include "utils.h"

#include <string.h>
#include <stdio.h>



//--- VARIABLES EXTERNAS ---//
//del main
extern const char s_ok [];
// ------- Externals del Puerto serie  -------
extern unsigned char usart1_have_data;

//--- VARIABLES GLOBALES ---//
//globales de este modulo

//strings de comienzo o lineas intermedias
const char s_ch1 [] = {"ch1"};
const char s_ch2 [] = {"ch2"};
const char s_ch3 [] = {"ch3"};
const char s_chf [] = {"chf"};
const char s_set_signal [] = {"signal"};
const char s_frequency [] = {"frequency"};
const char s_power [] = {"power"};
const char s_square [] = {"square"};
const char s_triangular [] = {"triangular"};
const char s_sinusoidal [] = {"sinusoidal"};
const char s_ten_hz [] = {"10Hz"};
const char s_thirty_hz [] = {"30Hz"};
const char s_sixty_hz [] = {"60Hz"};
const char s_start_treatment [] = {"start treatment"};
const char s_stop_treatment [] = {"stop treatment"};
const char s_status [] = {"status"};
const char s_flush_errors [] = {"flush errors"};
const char s_getall [] = {"get all conf"};


char buffMessages [100];
const char * p_own_channel;


//--- FUNCIONES DEL MODULO ---//
void SetOwnChannel (unsigned char ch)
{
	if (ch == 1)
		p_own_channel = s_ch1;
	else if (ch == 2)
		p_own_channel = s_ch2;
	else
		p_own_channel = s_ch3;

}

char * GetOwnChannel (void)
{
	return (char *) p_own_channel;
}

void UpdateCommunications (void)
{
	if (SerialProcess() > 2)	//si tiene algun dato significativo
	{
		InterpretarMsg();
	}
}

//Procesa consultas desde el micro principal
//carga el buffer buffMessages y avisa con el flag msg_ready
unsigned char SerialProcess (void)
{
	unsigned char bytes_readed = 0;

	if (usart1_have_data)
	{
		usart1_have_data = 0;
		bytes_readed = ReadUsart1Buffer((unsigned char *) buffMessages, sizeof(buffMessages));
	}
	return bytes_readed;
}

resp_t InterpretarMsg (void)
{
	resp_t resp = resp_not_own;
	unsigned char broadcast = 0;
	char * pStr = buffMessages;
	unsigned short new_power = 0;
	unsigned char decimales = 0;
	char b [30];

	//reviso canal propio o canal broadcast
	if ((strncmp(pStr, p_own_channel, sizeof(s_chf) - 1) == 0) ||
	    (strncmp(pStr, s_chf, sizeof(s_chf) - 1) == 0))
	{
		resp = resp_ok;

		//es broadcast?
		if (*(pStr + 2) == 'f')
			broadcast = 1;

		pStr += sizeof(s_chf);	//normalizo al mensaje, hay un espacio

		//-- Signal Setting
		if (strncmp(pStr, s_set_signal, sizeof(s_set_signal) - 1) == 0)
		{
			pStr += sizeof(s_set_signal);		//normalizo al payload, hay un espacio

			if (strncmp(pStr, s_square, sizeof(s_square) - 1) == 0)
				resp = SetSignalType (SQUARE_SIGNAL);
			else if (strncmp(pStr, s_triangular, sizeof(s_triangular) - 1) == 0)
				resp = SetSignalType (TRIANGULAR_SIGNAL);
			else if (strncmp(pStr, s_sinusoidal, sizeof(s_sinusoidal) - 1) == 0)
				resp = SetSignalType (SINUSOIDAL_SIGNAL);
			else
				resp = resp_error;
		}

		//-- Frequency Setting
		else if (strncmp(pStr, s_frequency, sizeof(s_frequency) - 1) == 0)
		{
			pStr += sizeof(s_frequency);		//normalizo al payload, hay un espacio

			if (strncmp(pStr, s_ten_hz, sizeof(s_ten_hz) - 1) == 0)
				resp = SetFrequency (TEN_HZ);
			else if (strncmp(pStr, s_thirty_hz, sizeof(s_thirty_hz) - 1) == 0)
				resp = SetFrequency (THIRTY_HZ);
			else if (strncmp(pStr, s_sixty_hz, sizeof(s_sixty_hz) - 1) == 0)
				resp = SetFrequency (SIXTY_HZ);
			else
				resp = resp_error;
		}

		//-- Power Setting
		else if (strncmp(pStr, s_power, sizeof(s_power) - 1) == 0)
		{
			pStr += sizeof(s_power);		//normalizo al payload, hay un espacio

			//lo que viene son 2 o 3 bytes
			decimales = StringIsANumber(pStr, &new_power);
			if ((decimales > 1) && (decimales < 4))
			{
				resp = SetPower (new_power);
				// sprintf(b, "dec: %d, power: %d", decimales, new_power);
				// Usart1Send(b);
			}
			else
				resp = resp_error;
		}

		//-- Start Treatment
		else if (strncmp(pStr, s_start_treatment, sizeof(s_start_treatment) - 1) == 0)
		{
			//se puede empezar
			if (GetTreatmentState() == TREATMENT_STANDBY)
			{
				resp = StartTreatment();
			}
			else
				resp = resp_error;
		}

		//-- Stop Treatment
		else if (strncmp(pStr, s_stop_treatment, sizeof(s_stop_treatment) - 1) == 0)
		{
			StopTreatment();
		}

		//-- Status
		else if (strncmp(pStr, s_status, sizeof(s_status) - 1) == 0)
		{
			switch (GetErrorStatus())
			{
				case ERROR_OK:
					sprintf(b, "Manager status: %d\n", GetTreatmentState());
				  	Usart1Send(b);
				  	break;

				case ERROR_OVERCURRENT:
					Usart1Send("Error: Overcurrent\n");
					break;

				case ERROR_NO_CURRENT:
					Usart1Send("Error: No current\n");
					break;

				case ERROR_SOFT_OVERCURRENT:
					Usart1Send("Error: Soft Overcurrent\n");
					break;

				case ERROR_OVERTEMP:
					Usart1Send("Error: Overtemp\n");
					break;

			}
		}

		//-- Flush Errors
		else if (strncmp(pStr, s_flush_errors, sizeof(s_flush_errors) - 1) == 0)
		{
			SetErrorStatus(ERROR_FLUSH_MASK);
		}

			//reviso errores y envio
		// 	error_t e;
		//
		// 	e = GetErrorStatus();
		// 	if (e == ERROR_OK)
		// 	{
		// 		sprintf(b, "Manager status: %d\n", GetTreatmentState());
		// 		Usart1Send(b);
		// 	}
		// 	else
		// 	{
		// 		//tengo algun error, los mando en secuencias
		// 		if (e & ERROR_OVERCURRENT)
		// 			Usart1Send("Error: Overcurrent\n");
		//
		// 		if (e & ERROR_NO_CURRENT)
		// 			Usart1Send("Error: No current\n");
		//
		// 		if (e & ERROR_SOFT_OVERCURRENT)
		// 			Usart1Send("Error: Soft Overcurrent\n");
		//
		// 		if (e & ERROR_OVERTEMP)
		// 			Usart1Send("Error: Overtemp\n");
		// 	}
		// }

		//-- Get All Configuration
		else if (strncmp(pStr, s_getall, sizeof(s_getall) - 1) == 0)
		{
			SendAllConf();
		}

		//-- Ninguno de los anteriores
		else
			resp = resp_error;

	}	//fin if chx

	if (!broadcast)
	{
		if (resp == resp_ok)
			Usart1Send("OK\n");

		if (resp == resp_error)
			Usart1Send("NOK\n");
	}

	return resp;
}
