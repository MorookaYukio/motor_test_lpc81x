#ifndef _MY_SCTIMER_H_
#define _MY_SCTIMER_H_

#include "my_bool.h"

#define SCT_OUTPUTNUM_MAX (3)
#define SCT_PIN_MAX (17)//pinの数は他とも共有したいのでもう少し頭のいい方法を考えるべき




typedef enum{
    eUNIFY,
    eLOW,
    eHIGH
} E_SCT_CNTR;

typedef enum{
    eMATCH_OR_IO = 0,
    eMATH = 1,
    eIO = 2,
    eMATH_AND_IO = 3
} E_SCT_EVENT_TRIGGER;

typedef enum{
    eSET,
    eCLEAR
} E_SCT_OUT_ACTION;


typedef struct{
    E_SCT_CNTR counter;
    uint8_t output_num;
    uint8_t output_pin[4];
    uint16_t cycle;
    uint8_t duty_ratio[4];
} T_SCT_PWMPARAM;


int sctSetupPWM( T_SCT_PWMPARAM* pwm_param );
void sctChangePWMDuty( T_SCT_PWMPARAM* pwm, uint8_t output_num, uint8_t duty );
void sctEnableAutoLimit( E_SCT_CNTR counter, BOOL is_enable );
void sctUnifyTimer( BOOL is_unify );
void sctClearReset( void );
void sctAssignOutPin( uint8_t output_id, uint8_t pin_num );
void sctSetMatchVal( uint8_t match_num, E_SCT_CNTR counter, uint32_t val );
void sctSetMatchRelVal( uint8_t match_rel_num, E_SCT_CNTR counter, uint32_t val );
void sctReleseEventFromState( uint8_t event, uint8_t state );
void sctBindEventToState( uint8_t event, uint8_t state );

void sctBindEventToMatch( uint8_t event, uint8_t match );
void sctSetEventCounter( uint8_t event, E_SCT_CNTR counter );
void sctSetEventTrigger( uint8_t event, E_SCT_EVENT_TRIGGER trigger );

void sctSetOutputAction( uint8_t output_num, uint8_t event_num, E_SCT_OUT_ACTION action );

void sctHalt( E_SCT_CNTR counter, BOOL is_halt );





#endif
