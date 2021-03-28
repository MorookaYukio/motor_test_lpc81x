#include "LPC8xx.h"
#include "my_sctimer.h"
#include "../../common/include/my_queue.h"

#include "lpc8xx_uart.h"
#include "../../common/include/my_uart.h"


// モータ動作定義.
typedef enum{
    eMT_ACTION_CW,
    eMT_ACTION_CCW,
    eMT_ACTION_BRAKE,
    eMT_ACTION_FREE,
    eMT_ACTION_MAX
}E_MT_ACTION;


// モータ情報構造体.
typedef struct{
    // モータ識別用に一応IDを用意.
    uint8_t motor_id;

    // ピン設定.
    uint8_t pin_in_1;
    uint8_t pin_in_2;
    uint8_t pin_pwm;
    uint8_t pin_stby;

    // モータの動作.
    E_MT_ACTION action;
    // モータの回転速度.
    uint8_t rotation_speed;
    // PWM制御用構造体.
    T_SCT_PWMPARAM pwm;
}T_MOTOR;

// グローバル変数.
T_MOTOR g_motor;



// 回転速度変更(秘匿された関数にするイメージ).
void setMotorSpeed( T_MOTOR* rv_motor, uint8_t rv_rotation_speed ){
    // メンバの設定.
    rv_motor->rotation_speed = rv_rotation_speed;

    // PWM Dutyの設定.
    sctChangePWMDuty( &(rv_motor->pwm), 0, rv_motor->rotation_speed);

}

void runMotor( T_MOTOR* rv_motor, E_MT_ACTION rv_motor_action, uint8_t rv_rotation_speed ){
    // モータ情報構造体のメンバ設定.
    rv_motor->action = rv_motor_action;
    rv_motor->rotation_speed = rv_rotation_speed;
    
    // 回転速度の設定(PWMの設定).
    setMotorSpeed( rv_motor, rv_rotation_speed );
    
    switch( rv_motor->action ){
    case eMT_ACTION_CW:
	LPC_GPIO_PORT->SET0 |= ( 1 << rv_motor->pin_in_1);
	LPC_GPIO_PORT->CLR0 |= ( 1 << rv_motor->pin_in_2);
	break;
    case eMT_ACTION_CCW:
	LPC_GPIO_PORT->CLR0 |= ( 1 << rv_motor->pin_in_1);
	LPC_GPIO_PORT->SET0 |= ( 1 << rv_motor->pin_in_2);
	break;
    case eMT_ACTION_BRAKE:
	LPC_GPIO_PORT->SET0 |= ( 1 << rv_motor->pin_in_1);
	LPC_GPIO_PORT->SET0 |= ( 1 << rv_motor->pin_in_2);
	break;
    case eMT_ACTION_FREE:
	LPC_GPIO_PORT->CLR0 |= ( 1 << rv_motor->pin_in_1);
	LPC_GPIO_PORT->CLR0 |= ( 1 << rv_motor->pin_in_2);
	break;
    default:
	break;
    }
}

T_MOTOR initMotor( uint8_t rv_motor_id, uint8_t rv_pin_in_1, uint8_t rv_pin_in_2, uint8_t rv_pin_pwm, uint8_t rv_pin_stby ){
    T_MOTOR at_motor;

    // モータ情報構造体のメンバ設定.
    at_motor.pin_in_1 = rv_pin_in_1;
    at_motor.pin_in_2 = rv_pin_in_2;
    at_motor.pin_pwm = rv_pin_pwm;
    at_motor.pin_stby = rv_pin_stby;
    
    // GPIO設定.
    LPC_SYSCON->SYSAHBCLKCTRL |= (0x1 << 6);// GPIOに対してclock供給.
    // 出力ピン設定.
    LPC_GPIO_PORT->DIR0 |= ( 1 << rv_pin_in_1 ); // モタドラ制御入力BIN1.
    LPC_GPIO_PORT->DIR0 |= ( 1 << rv_pin_in_2 ); // モタドラ制御入力BIN2.
    LPC_GPIO_PORT->DIR0 |= ( 1 << rv_pin_pwm );// モタドラ制御入力PWMB.
    LPC_GPIO_PORT->DIR0 |= ( 1 << rv_pin_stby );// モタドラ制御入力STBY.

    // STBYピンはHIGHにする.
    LPC_GPIO_PORT->SET0 |= ( 1 << at_motor.pin_stby );

    // PWM制御用構造体の準備.
    at_motor.pwm.counter = eUNIFY;
    at_motor.pwm.output_num = 1;
    at_motor.pwm.output_pin[0] = at_motor.pin_pwm;
    at_motor.pwm.cycle = 10;
    at_motor.pwm.duty_ratio[0] = 0;
    sctSetupPWM( &(at_motor.pwm) );
    
    // 初期はモータをフリーにしておく.
    runMotor( &at_motor , eMT_ACTION_FREE, 0);

    return at_motor;
}




void SysTick_Handler( void ){
    static uint16_t cnt = 0;
    static E_MT_ACTION mt_action = eMT_ACTION_CW;
    static uint8_t rotation_speed = 0;

    cnt++;
    if(cnt % 30 == 0){
	rotation_speed = cnt / 30;
	runMotor( &(g_motor), mt_action, rotation_speed );
	uartSendF(LPC_USART0, "action:[%d], speed:[%d]\n\0",
		  mt_action, rotation_speed);
    }
    if(cnt >= 3000){
	// 一定時間ごとにモータ動作を切り替える.
	mt_action++;
	if( mt_action == eMT_ACTION_MAX ){
	    mt_action = eMT_ACTION_CW;
	}
	rotation_speed = 0;
	
	runMotor( &(g_motor), mt_action, rotation_speed );
	
	cnt = 0;
	uartSendF(LPC_USART0, "action:[%d], speed:[%d]\n\0",
		  mt_action, rotation_speed);
    }
}


int main(void) {

    SystemCoreClockUpdate();
    SysTick_Config( SystemCoreClock / 1000 );
    
    int32_t at_val;
    at_val = LPC_SYSCON->MAINCLKSEL & ~(0b00);
    LPC_SYSCON->MAINCLKSEL = at_val | ( 0 );
    LPC_SYSCON->MAINCLKUEN &= ~( 1 << 0 );
    LPC_SYSCON->MAINCLKUEN |= ( 1 << 0 );

    /* // GPIO設定. */
    /* LPC_SYSCON->SYSAHBCLKCTRL |= (0x1 << 6);// GPIOに対してclock供給. */
    /* // 出力ピン設定. */
    /* LPC_GPIO_PORT->DIR0 |= ( 1 << eBIN1 ); // モタドラ制御入力BIN2. */
    /* LPC_GPIO_PORT->DIR0 |= ( 1 << eBIN2 ); // モタドラ制御入力BIN1. */
    /* LPC_GPIO_PORT->DIR0 |= ( 1 << ePWMB );// モタドラ制御入力PWMB. */
    /* LPC_GPIO_PORT->DIR0 |= ( 1 << eSTBY );// モタドラ制御入力STBY. */

    /* // BIN1/2の設定（初期はLOWとする）. */
    /* LPC_GPIO_PORT->CLR0 |= ( 1 << eBIN1 ); */
    /* LPC_GPIO_PORT->CLR0 |= ( 1 << eBIN2 ); */
    
    /* // PWMBとSTBYの設定（暫定的に常時HIGH）. */
    /* LPC_GPIO_PORT->SET0 |= ( 1 << ePWMB ); */
    /* LPC_GPIO_PORT->SET0 |= ( 1 << eSTBY ); */
    
    g_motor = initMotor( 0, 7, 6, 14, 17);
    
    //uartの設定
    uartAssignTxPin( LPC_USART0, 4 );
    uartAssignRxPin( LPC_USART0, 0 );
    UARTInit( LPC_USART0, 9600 );
    uartSendF( LPC_USART0, "uart init!\n\0" );

    /* T_SCT_PWMPARAM pwm; */
    /* pwm.counter = eUNIFY; */
    /* pwm.output_num = 3; */
    /* pwm.output_pin[0] = 16; */
    /* pwm.output_pin[1] = 17; */
    /* pwm.output_pin[2] = 15; */
    /* pwm.cycle = 10; */
    /* pwm.duty_ratio[0] = 10; */
    /* pwm.duty_ratio[1] = 50; */
    /* pwm.duty_ratio[2] = 90; */
    
    /* sctSetupPWM( &pwm ); */

    /* uartSendF(LPC_USART0, "mainclksel->sel:%d\n\0", */
    /* 	      LPC_SYSCON->MAINCLKSEL); */
    
    int i;
    int duty0 = 1;
    int duty1 = 1;
    int duty2 = 1;
    while(1){
	for(i=0; i<100000; i++){
	}
	if(duty0 < 90){
	    duty0+=1;
	}else{
	    duty0 = 1;
	}
	if(duty1 < 90){
	    duty1+=2;
	}else{
	    duty1 = 1;
	}
	if(duty2 < 90){
	    duty2+=3;
	}else{
	    duty2 = 1;
	}
	//uartSendF(LPC_USART0, "duty:%d\n\0", duty);
	/* sctChangePWMDuty( &pwm, 0, duty0 ); */
	/* sctChangePWMDuty( &pwm, 1, duty1 ); */
	/* sctChangePWMDuty( &pwm, 2, duty2 ); */
    }

    return 0 ;
}
