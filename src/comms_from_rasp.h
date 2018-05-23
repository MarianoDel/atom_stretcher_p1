//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### COMMS_FROM_RASP.H ################################
//---------------------------------------------

#ifndef _COMMS_FROM_RASP_H_
#define _COMMS_FROM_RASP_H_

//---- Includes to help the Defines ----------



//---- Configurations Defines --------------------
#define RPI_Send(X)    Usart1Send(X)
#define rpi_have_data    usart1_have_data
#define ReadRPIBuffer(X,Y)    ReadUsart1Buffer(X,Y)

//---- Common Defines --------------------
// #define COMM_NONE_TREAT     0x0000
// #define COMM_START_TREAT    0x0001
// #define COMM_STOP_TREAT     0x0002
// #define COMM_PAUSE_TREAT    0x0004
// #define COMM_CONF_CHANGE    0x0008
// #define COMM_NO_COMM_CH1    0x0100
// #define COMM_NO_COMM_CH2    0x0200
// #define COMM_NO_COMM_CH3    0x0400





//--- Exported functions ---//
void UpdateRaspberryMessages (void);
void RaspBerry_Report_Errors (unsigned short *);


#endif
