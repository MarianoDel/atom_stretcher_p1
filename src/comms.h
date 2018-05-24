//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### COMMS.H ################################
//---------------------------------------------

#ifndef _COMMS_H_
#define _COMMS_H_

//---- Includes to help the Defines ----------



//---- Configurations Defines --------------------



//---- Common Defines --------------------
#define COMM_NONE_TREAT     0x0000
#define COMM_START_TREAT    0x0001
#define COMM_STOP_TREAT     0x0002
#define COMM_PAUSE_TREAT    0x0004
#define COMM_CONF_CHANGE    0x0008

#define COMM_POWER_ERR_MASK            0x00F0
#define COMM_ERROR_OVERCURRENT         0x0010
#define COMM_ERROR_NO_CURRENT          0x0020
#define COMM_ERROR_SOFT_OVERCURRENT    0x0040
#define COMM_ERROR_OVERTEMP            0x0080

#define COMM_NO_COMM_CH1    0x0100
#define COMM_NO_COMM_CH2    0x0200
#define COMM_NO_COMM_CH3    0x0400

#define COMM_POWER_CHN_MASK            0x0F00
#define COMM_ERROR_IN_CH1    0x0100
#define COMM_ERROR_IN_CH2    0x0200
#define COMM_ERROR_IN_CH3    0x0400






//--- Exported functions ---//




#endif
