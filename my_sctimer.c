#include "../../common/include/LPC8xx.h"
#include "../../common/include/my_uart.h"
#include "my_sctimer.h"
#include "my_bool.h"




int sctSetupPWM( T_SCT_PWMPARAM* pwm_param ){
    //TODO:不正値チェックを書く

    //SCTリセットクリア
    sctClearReset();
	
    //  SWMのclockを有効化
    LPC_SYSCON->SYSAHBCLKCTRL |= ( 1 << 7 );
    //SCTに対してclockを有効化
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);

    //counterの設定
    if( pwm_param->counter == eUNIFY ){
    	sctUnifyTimer( TRUE );
    }else{
    	sctUnifyTimer( FALSE );
    }

    //出力ピンの設定
    int itr = 0;
    for( itr=0; itr<pwm_param->output_num; itr++){
    	sctAssignOutPin( itr, pwm_param->output_pin[itr] );
    }
    //Limit時の自動リロードを有効に
    sctEnableAutoLimit( pwm_param->counter, TRUE );

    //pwm周期とduty比に対応したカウンタ値の算出
    int frequency_k = 12000;
    int cycle_count = frequency_k * pwm_param->cycle;
    int at_duty_ratio_count;
    
    //uartSendF( LPC_USART0, "frequency_k:%d\n\0" , frequency_k);
    //uartSendF( LPC_USART0, "cycle:%d\n\0" , pwm_param->cycle);
    //uartSendF( LPC_USART0, "cycle_count:%d\n\0" , cycle_count);
    
    sctSetMatchRelVal( 0, pwm_param->counter, cycle_count );
    
    // event0番(match 0、1秒に対応して発生)
    sctBindEventToState( 0, 0 ); //state0で発生
    sctBindEventToMatch( 0, 0 ); //match0番で発生
    sctSetEventCounter( 0, pwm_param->counter );//counterの設定
    sctSetEventTrigger( 0, eMATH ); //matchの時のみ発生
    
    for( itr=1; itr<pwm_param->output_num+1; itr++ ){ 
	//match_reloadレジスタの設定
	at_duty_ratio_count = (cycle_count / 100) * pwm_param->duty_ratio[itr-1];	
	sctSetMatchRelVal( itr, pwm_param->counter, at_duty_ratio_count );
	//uartSendF( LPC_USART0, "duty_ratio_count[%d]:%d\n\0" , itr, at_duty_ratio_count);

	//eventの設定
	sctBindEventToState( itr, 0 ); // stateは使用しないので0に充てておく
	sctBindEventToMatch( itr, itr );
	sctSetEventCounter( itr, pwm_param->counter );
	sctSetEventTrigger( itr, eMATH );

	//すべての出力をevent0でsetする
	sctSetOutputAction( itr-1, 0, eSET );
	//出力のclearタイミングを設定する
	sctSetOutputAction( itr-1, itr, eCLEAR );
    }

    // 以上で設定完了、haltを解除してstart
    sctHalt( pwm_param->counter, FALSE );
}

void sctChangePWMDuty( T_SCT_PWMPARAM* pwm_param, uint8_t output_num, uint8_t duty ){
    pwm_param->duty_ratio[output_num] = duty;
    int frequency_k = 12000;
    int cycle_count = frequency_k * pwm_param->cycle;
    int at_duty_ratio_count;

    at_duty_ratio_count = (cycle_count / 100) * pwm_param->duty_ratio[output_num];
    sctSetMatchRelVal( output_num+1, pwm_param->counter, at_duty_ratio_count );
}

void sctEnableAutoLimit( E_SCT_CNTR counter, BOOL is_enable ){
    switch(counter){
    case eUNIFY:
    case eLOW:
	if( is_enable == TRUE ){
	    //17bit目を1に
	    LPC_SCT->CONFIG |= ( 1 << 17 );
	}else{
	    //17bit目を1に
	    LPC_SCT->CONFIG &= ~( 1 << 17 );
	}
	break;
    case eHIGH:
	if( is_enable == TRUE ){
	    //18bit目を1に
	    LPC_SCT->CONFIG |= ( 1 << 18 );
	}else{
	    //18bit目を1に
	    LPC_SCT->CONFIG &= ~( 1 << 18 );
	}
	break;
    default:
	break;
    }
}

void sctUnifyTimer( BOOL is_unify ){
    if( is_unify == TRUE ){
	//1bit目を1に
	LPC_SCT->CONFIG |= (1 << 0);
    }else{
	//1bit目を0に
	LPC_SCT->CONFIG &= ~(1 << 0);
    }
}



void sctClearReset( void ){
    LPC_SYSCON->PRESETCTRL &= ~(0x1<<8);
    LPC_SYSCON->PRESETCTRL |= (0x1<<8);
}


void sctAssignOutPin( uint8_t output_id, uint8_t pin_num ){

    //不正なpinの例外処理
    if( output_id < 0 || SCT_OUTPUTNUM_MAX < output_id ){
	return;
    }
    if( pin_num < 0 || SCT_PIN_MAX < pin_num ){
	return;
    }

    switch( output_id ){
    case 0:
	LPC_SWM->PINASSIGN6 &= ~(0xFF << 24); 
	LPC_SWM->PINASSIGN6 |= (pin_num << 24);
	//uartSendF( LPC_USART0, "pinassign6: %d\n\0", LPC_SWM->PINASSIGN6);
	break;
    case 1:
	LPC_SWM->PINASSIGN7 &= ~(0xFF << 0);
	LPC_SWM->PINASSIGN7 |= (pin_num << 0);
	break;
    case 2:
	LPC_SWM->PINASSIGN7 &= ~(0xFF << 8);
	LPC_SWM->PINASSIGN7 |= (pin_num << 8);
	break;
    case 3:
	LPC_SWM->PINASSIGN7 &= ~(0xFF << 16);
	LPC_SWM->PINASSIGN7 |= (pin_num << 16);
	break;
    default:
	//不正な出力番号の例外処理
	return;
    }
}

void sctSetMatchVal( uint8_t match_num, E_SCT_CNTR counter, uint32_t val ){
    //不正なMATCHRELレジスタの指定をはじく
    if( match_num < 0 || 4< match_num ){
	return;
    }
    
    switch( counter ){
    case eUNIFY:	
	LPC_SCT->MATCH[match_num].U = val;
	break;
    case eLOW:
	LPC_SCT->MATCH[match_num].L = val;
	break;
    case eHIGH:
	LPC_SCT->MATCH[match_num].H = val;
	break;
    default:
	break;
    }
}

void sctSetMatchRelVal( uint8_t match_rel_num, E_SCT_CNTR counter, uint32_t val ){
    //不正なMATCHRELレジスタの指定をはじく
    if( match_rel_num < 0 || 4< match_rel_num ){
	return;
    }
    
    switch( counter ){
    case eUNIFY:	
	LPC_SCT->MATCHREL[match_rel_num].U = val;
	break;
    case eLOW:
	LPC_SCT->MATCHREL[match_rel_num].L = val;
	break;
    case eHIGH:
	LPC_SCT->MATCHREL[match_rel_num].H = val;
	break;
    default:
	break;
    }
}


void actReleseEventFromState( uint8_t event, uint8_t state ){
    //不正なstate値をはじく
    if( state < 0 || 1 < state ){
	return;
    }
    LPC_SCT->EVENT[event].STATE &= ~(1 << state);
}


void sctBindEventToState( uint8_t event, uint8_t state ){
    //不正なstate値をはじく
    if( state < 0 || 1 < state ){
	return;
    }
    LPC_SCT->EVENT[event].STATE |= (1 << state);
}


void sctBindEventToMatch( uint8_t event, uint8_t match ){
    //不正なmatch値をはじく
    if( match < 0 || 5 < match ){
	return;
    }
    
    LPC_SCT->EVENT[event].CTRL &= ~( 0b0000 << 0 );
    LPC_SCT->EVENT[event].CTRL |= ( match << 0 );
}

void sctSetEventCounter( uint8_t event, E_SCT_CNTR counter ){
    switch( counter ){
    case eLOW:
	//4bit目を0に設定
	LPC_SCT->EVENT[event].CTRL &= ~( 1 << 4 );
	break;
    case eHIGH:
	//4bit目を0に設定
	LPC_SCT->EVENT[event].CTRL |= ( 1 << 4 );
	break;

    case eUNIFY:
    default:
	//この2caseでは何もしない
	break;
    }
}

void sctSetEventTrigger( uint8_t event, E_SCT_EVENT_TRIGGER trigger ){
    LPC_SCT->EVENT[event].CTRL &= ~(0b00 << 12);
    LPC_SCT->EVENT[event].CTRL |= (trigger << 12);
}

void sctSetOutputAction( uint8_t output_num, uint8_t event_num, E_SCT_OUT_ACTION action ){
    //不正なoutput_num値をはじく
    if( output_num < 0 || 3 < output_num ){
	return;
    }
    //不正なevent_num値をはじく
    if( event_num < 0 || 5 < event_num ){
	return;
    }

    switch( action ){
    case eSET:
	LPC_SCT->OUT[output_num].SET |= ( 1 << event_num );
	break;
    case eCLEAR:
	LPC_SCT->OUT[output_num].CLR |= ( 1 << event_num );
	break;
    default:
	break;
    }
}


void sctHalt( E_SCT_CNTR counter, BOOL is_halt ){
    switch( counter ){
    case eUNIFY:
	if( is_halt == TRUE ){
	    LPC_SCT->CTRL_U |= (1 << 2);
	}else{
	    LPC_SCT->CTRL_U &= ~(1 << 2);
	}
	break;
    case eLOW:
	if( is_halt == TRUE ){
	    LPC_SCT->CTRL_L |= (1 << 2);
	}else{
	    LPC_SCT->CTRL_L &= ~(1 << 2);
	}
	break;
    case eHIGH:
	if( is_halt == TRUE ){
	    LPC_SCT->CTRL_H |= (1 << 2);
	}else{
	    LPC_SCT->CTRL_H &= ~(1 << 2);
	}
	break;
    default:
	break;
    }
}
