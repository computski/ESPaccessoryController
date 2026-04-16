// 
// 
// 

#include "ESPservo.h"
#include <c_types.h>
#include <pwm.h>
#include <eagle_soc.h>
#include <ets_sys.h>
//now adding gpio.h for function GPIO_PIN_ADDR
#include <gpio.h>

#ifndef SDK_PWM_PERIOD_COMPAT_MODE
#define SDK_PWM_PERIOD_COMPAT_MODE 0
#endif
#define PWM_USE_NMI 0




//timer div16 gives 200nS ticks, and div256 gives 3.2uS ticks
//servo pulse is between 0.5mS and 2mS, or 156 through 625 counts, with 468 being neutral

// from SDK hw_timer.c
#define TIMER1_DIVIDE_BY_16             0x0004
#define TIMER1_DIVIDE_BY_256            0x0008
#define TIMER1_ENABLE_TIMER             0x0080

//https://esp8266.ru/esp8266-gpio-register/
struct gpio_regs {
	uint32_t out;         /* 0x60000300 entire output reg*/
	uint32_t out_w1ts;    /* 0x60000304 selective outputs hi*/
	uint32_t out_w1tc;    /* 0x60000308 selective outputs low*/
	uint32_t enable;      /* 0x6000030C enable outputs (hi) or inputs (low)*/
	uint32_t enable_w1ts; /* 0x60000310 seletive IO as output*/
	uint32_t enable_w1tc; /* 0x60000314 selective IO as input*/
	uint32_t in;          /* 0x60000318 Input level when IO is an input*/
	uint32_t status;      /* 0x6000031C interrupt status*/
	uint32_t status_w1ts; /* 0x60000320 */
	uint32_t status_w1tc; /* 0x60000324 */
};
static struct gpio_regs* gpio = (struct gpio_regs*)(0x60000300);

struct timer_regs {
	uint32_t frc1_load;   /* 0x60000600 */
	uint32_t frc1_count;  /* 0x60000604 */
	uint32_t frc1_ctrl;   /* 0x60000608 */
	uint32_t frc1_int;    /* 0x6000060C */
	uint8_t  pad[16];
	uint32_t frc2_load;   /* 0x60000620 */
	uint32_t frc2_count;  /* 0x60000624 */
	uint32_t frc2_ctrl;   /* 0x60000628 */
	uint32_t frc2_int;    /* 0x6000062C */
	uint32_t frc2_alarm;  /* 0x60000630 */
};
static struct timer_regs* timer = (struct timer_regs*)(0x60000600);







//rotate through each servo and assert each servo pulse in turn, with all of them repeating on a 20mS (6250 ticks) cycle
//


static void IRAM_ATTR servo_handler(void) {
	static uint8_t servoIndex = 0;
	static uint16_t periodPadding = 6250;
	static uint32_t servoGPIOmask = 0;

	gpio->out_w1tc = servoGPIOmask;  //clear down the last servo hi period

	if (servoIndex < 9) {
		//0-8 we load up the servo specific pulse period
		servoGPIOmask = 1 << servoPool[servoIndex].gpioPin;
		WRITE_PERI_REG(&timer->frc1_load, servoPool[servoIndex].hiPulseLen);  //1.5mS test
		periodPadding -= servoPool[servoIndex].hiPulseLen;
		gpio->out_w1ts = servoGPIOmask;

		//if detached then i think we hold the pin low, but maybe we release it entirely to float as an input
		//basically we don't want to put it in the mask nor deduct it from Padding


	}
	else if (servoIndex >= 9) {
	//9, we stay low for the remainder of periodPadding and reset index and padding
		WRITE_PERI_REG(&timer->frc1_load, periodPadding);
		periodPadding = 6250;
		servoIndex = 0;
	}

	servoIndex++;

	//reset the timer with the load value recently given to it
	asm volatile ("" : : : "memory");//memory barrier compiler instruction
	timer->frc1_int &= ~FRC1_INT_CLR_MASK;
	
}

void ESPservoInit() {
	//populate GPIO pins
	uint8_t NodeMCUmap[] = { 16,5,4,0,2,14,12,13,15 };
	uint8_t* gp = NodeMCUmap;


	for (auto& s : servoPool) {
		s.hiPulseLen = 468;  //neutral
		s.isAttached = false;
		s.position = 0;
		s.gpioPin = *gp;
		gp++;
	}


	ETS_FRC_TIMER1_INTR_ATTACH(servo_handler, NULL);
	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();
	TIMER_REG_WRITE(FRC1_LOAD_ADDRESS, 0);  //This starts timer.  +++++++++ RTC_REG_WRITE is deprecated ++++++
	timer->frc1_ctrl = TIMER1_DIVIDE_BY_256 | TIMER1_ENABLE_TIMER;

	pinMode(5, OUTPUT);
	pinMode(4, OUTPUT);

}