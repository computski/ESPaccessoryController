// IMPORTANT: GPIO16 is part of the RTC module, and cannot be controlled via out_w1ts/out_w1tc
// 
// 



#include "ESPservo.h"
#include <c_types.h>
#include <eagle_soc.h>
#include <ets_sys.h>

extern "C" {
#include <osapi.h>
#include <os_type.h>
}



#ifndef SDK_PWM_PERIOD_COMPAT_MODE
#define SDK_PWM_PERIOD_COMPAT_MODE 0
#endif
#define PWM_USE_NMI 0

const uint8_t NodeMCUmap[] = { 16,5,4,0,2,14,12,13,15 };  
volatile static SERVO servoPool[9];  //9 pins


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


#define RANGE90
#ifdef RANGE90
const int MIN_PULSE = 312;  //1ms
const int MAX_PULSE = 625;  //2ms
const int PAD_PULSE = 6250;  //20ms notional, but must be > 9 x MAX
const int NEUTRAL_PULSE = 428;

#else
const int MIN_PULSE = 156;  //0.5ms
const int MAX_PULSE = 780;  //2.5ms
const int PAD_PULSE = 7030;
const int NEUTRAL_PULSE = 428;
#endif // RANGE90





//cascade through each servo and assert each servo pulse in turn, with all of them repeating on a 20mS (6250 ticks) cycle
//index 0 is not used as GPIO16 cannot be driven with out_w1ts/c
static void IRAM_ATTR servo_handler(void) {
	static uint8_t servoIndex = 1;
	static uint16_t periodPadding = PAD_PULSE;
	static uint32_t servoGPIOmask = 0;

	gpio->out_w1tc = servoGPIOmask;  //clear down the last servo hi period
	servoGPIOmask = 0;

	//1-8 we load up the servo specific pulse period
	//if a servo is not attached, then we clear servoGPIOmask and we still need to load the pulse period
	//to ensure we have another int and advance through all pins

	if (servoIndex < 9) {
		if (servoPool[servoIndex].isAttached) {
			servoGPIOmask = 1 << servoPool[servoIndex].gpioPin;
			gpio->out_w1ts = servoGPIOmask;
		}

		WRITE_PERI_REG(&timer->frc1_load, servoPool[servoIndex].hiPulseLen);
		periodPadding -= servoPool[servoIndex++].hiPulseLen;
	}else {
		//9, we stay low for the remainder of periodPadding and reset index and padding
		WRITE_PERI_REG(&timer->frc1_load, periodPadding);
		periodPadding = PAD_PULSE;
		servoIndex = 1;
	}

	//reset the timer with the load value recently given to it
	asm volatile ("" : : : "memory");//memory barrier compiler instruction
	timer->frc1_int &= ~FRC1_INT_CLR_MASK;

}

void ESPservoInit() {
	//populate GPIO pins
	uint8_t i;

	for (auto& s : servoPool) {
		s.hiPulseLen = NEUTRAL_PULSE;
		s.gpioPin = NodeMCUmap[i++];
		s.isAttached = false;
	}

	ETS_FRC_TIMER1_INTR_ATTACH(servo_handler, NULL);
	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();
	TIMER_REG_WRITE(FRC1_LOAD_ADDRESS, 0);  //This starts timer.  +++++++++ RTC_REG_WRITE is deprecated ++++++
	timer->frc1_ctrl = TIMER1_DIVIDE_BY_256 | TIMER1_ENABLE_TIMER;

}




/// <summary>
/// Command a servo to a position
/// </summary>
/// <param name="pin">Pin 1-8</param>
/// <param name="position">0-180 degrees</param>
void ESPservoWrite(uint8_t pin, uint8_t position) {
	//find which index we are dealing with
	if ((pin==0) ||(pin > 8)) return;
	for (auto &s : servoPool) {
		if (s.gpioPin == NodeMCUmap[pin]) {
			//calculate new delay from position. 0.5ms = 156, 2.5mS = 781
			float d = position / 180.0;
			d *= (MAX_PULSE-MIN_PULSE);  //full range
			d += MIN_PULSE;  //add min pulse offset
			s.hiPulseLen = (uint16_t)d;
			if (s.hiPulseLen > MAX_PULSE) s.hiPulseLen = MAX_PULSE;
			return;
		}
	}
}

/// <summary>
/// If not already attached, set the pin as an output and drive with pwm.
/// Detach does not change the pinMode
/// </summary>
/// <param name="pin">Pin 1-8</param>
/// <param name="attach">desired attach state</param>
void ESPservoAttach(uint8_t pin, bool attach) {
	if ((pin==0)||(pin > 8)) return;
	for (auto& s : servoPool) {
		if (s.gpioPin == NodeMCUmap[pin]) {
						
			//if commanding attach, then assert an output if current state is detach
			if (attach) { 
				if (!s.isAttached) pinMode(NodeMCUmap[pin], OUTPUT);
			}
			else { //if commanding detach then clear pin down if current state is attach
				if (s.isAttached) gpio->out_w1tc = 1 << NodeMCUmap[pin];
			}

			//capture newly commanded state
			s.isAttached = attach;
			//for detach, we leave pin as an output driving low.  Main program can take over the 
			//assignment of that pin
		}
	}
}

/// <summary>
/// returns current servo pin attachment state
/// </summary>
/// <param name="pin">pin 1-8</param>
/// <returns>true if driving the servo</returns>
bool ESPservoIsAttached(uint8_t pin) {
	if (pin > 8) return false;
	for (const auto &s : servoPool) {
		if (s.gpioPin == NodeMCUmap[pin]) {
			return s.isAttached;
		}
	}
	return false;
}