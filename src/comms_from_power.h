//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### COMMS_FROM_POWER.H ################################
//---------------------------------------------

#ifndef _COMMS_FROM_POWER_H_
#define _COMMS_FROM_POWER_H_

//---- Includes to help the Defines ----------



//---- Configurations Defines --------------------
#define Power_Send(X)    Usart2Send(X)
#define power_have_data    usart2_have_data
#define ReadPowerBuffer(X,Y)    ReadUsart2Buffer(X,Y)

typedef enum {
    ASK_CH1 = 0,
    WAIT_ANSWER_CH1,
    ASK_CH2,
    WAIT_ANSWER_CH2,
    ASK_CH3,
    WAIT_ANSWER_CH3,
    CHECK_COUNTERS
    
} comm_state_t;


//---- Common Defines --------------------
#define MAX_NO_COMM_ERRORS        15
#define POWER_COMMS_TT            1000



//--- Exported functions ---//
void UpdatePowerMessages (void);
void PowerSendConf (void);
void PowerSendStart( void );
void PowerSendStop( void );
void PowerCommunicationStack (void);
void PowerCommunicationStackReset (void);

#endif
