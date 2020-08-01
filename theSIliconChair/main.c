/***************************************************************************//**
* @file
* @brief Simple GPIO interrupt dispatcher Demo for EFM32GG_STK3700
*******************************************************************************
* # License
* <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "gpiointerrupt.h"
#include "udelay.h"


/* Set to 1 if you want to allow for "infinite" on-time, i.e. to
 * disable any timers, such that the LED strip may be on indefinitely */
#define INFINITE_ACTIVE_TIME_ENABLE 0

#define PWM_FREQ 1000
#define TIMER_FREQ 100000

static volatile uint32_t ticks = 0;
static volatile uint32_t usTicks;

static int green_btn_in  = 11; //PB11
static int red_btn_in    = 12; //PB12
static int green_sig_out =  1; //PD1
static int red_sig_out   =  6; //PD6
static int sensor_out 	 =  5; //PD5 ( trig )
static int sensor_in 	 =  4; //PD4 ( echo )

static bool red;
static bool green;
static bool ACTIVE = false;

static uint32_t topValue;

static volatile float currentDistance;

static volatile int intensity_red       = 100;
static volatile int intensity_green     = 100;
static int 			intensity_low_limit =  10;
/* Sleep time in seconds */
static uint32_t sleepTime = 5;


/*****************************************************************************
* @brief  Gpio callback
* @param  pin - pin which triggered interrupt
******************************************************************************/
void gpioCallback(uint8_t pin)
{
  if (pin == red_btn_in) {
      if (!red){
          BSP_LedSet(0);
          red = true;
          TIMER_Enable( TIMER1, true );
      }
      if (green){
          BSP_LedClear(1);
          green = false;
          TIMER_Enable( TIMER0, false );
      }

  }
  else if (pin == green_btn_in) {
      if (!green){
    	  BSP_LedSet(1);
    	  green = true;
    	  TIMER_Enable( TIMER0, true );
      }
      if (red) {
          BSP_LedClear(0);
          red = false;
          TIMER_Enable( TIMER1, false );
      }
  }

  /* Set status */
  ACTIVE = true;
  /* Reset counter */
  ticks  = 0;

}



/***************************************************************************//**
* @brief  Gpio setup. Setup button pins to trigger falling edge interrupts.
*  Register callbacks for that interrupts.
******************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable( cmuClock_GPIO, true );

  /* Initialize GPIO interrupt dispatcher */
  GPIOINT_Init();

  /* Configure buttons as input and transistor base pins as push/pull */
  GPIO_PinModeSet( gpioPortB, green_btn_in,  gpioModeInput,    0 );
  GPIO_PinModeSet( gpioPortB, red_btn_in,    gpioModeInput,    0 );
  GPIO_PinModeSet( gpioPortD, red_sig_out,   gpioModePushPull, 0 );
  GPIO_PinModeSet( gpioPortD, green_sig_out, gpioModePushPull, 0 );
  GPIO_PinModeSet( gpioPortD, sensor_in,     gpioModeInput,    0 );
  GPIO_PinModeSet( gpioPortD, sensor_out,    gpioModePushPull, 0 );

  /* Register callbacks before setting up and enabling pin interrupt. */
  GPIOINT_CallbackRegister( green_btn_in, gpioCallback );
  GPIOINT_CallbackRegister( red_btn_in,   gpioCallback );

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig( gpioPortB, green_btn_in, false, true, true );
  GPIO_IntConfig( gpioPortB, red_btn_in,   false, true, true );
}


void PWMsetup( void ) {

   /* Set up PWM dimming */
   /* TIMER0 is for green LED, TIMER1 is for red */
   CMU_ClockEnable( cmuClock_TIMER0, true );
   CMU_ClockEnable( cmuClock_TIMER1, true );

   /* Initialize timers */
   TIMER_InitCC_TypeDef PWM_Green = TIMER_INITCC_DEFAULT;
   PWM_Green.mode = timerCCModePWM;

   TIMER_InitCC_TypeDef PWM_Red = TIMER_INITCC_DEFAULT;
   PWM_Red.mode   = timerCCModePWM;

   TIMER_InitCC( TIMER0, 0, &PWM_Green );
   TIMER_InitCC( TIMER1, 0, &PWM_Red );

   /* Route Timer0 cc0 output to LOC3 = PD1 */
   TIMER0->ROUTE |= ( TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3 );

   /* Route Timer1 cc0 output to LOC4 = PD6 */
   TIMER1->ROUTE |= ( TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC4 );

   topValue = CMU_ClockFreqGet( cmuClock_HFPER ) / PWM_FREQ;
   TIMER_TopSet( TIMER0, topValue );
   TIMER_TopSet( TIMER1, topValue );

   TIMER_CompareSet( TIMER0, 0, ( topValue * intensity_green ) / 100 );
   TIMER_CompareSet( TIMER1, 0, ( topValue * intensity_red   ) / 100 );

   TIMER_Init_TypeDef timer0Init = TIMER_INIT_DEFAULT;
   TIMER_Init_TypeDef timer1Init = TIMER_INIT_DEFAULT;
   TIMER_Init( TIMER0, &timer0Init );
   TIMER_Init( TIMER1, &timer1Init );

   TIMER_IntEnable( TIMER0, TIMER_IEN_CC0 );
   TIMER_IntEnable( TIMER1, TIMER_IEN_CC0 );
   NVIC_EnableIRQ( TIMER0_IRQn );
   NVIC_EnableIRQ( TIMER1_IRQn );

   /* Don't start the timers yet */
   TIMER_Enable( TIMER0, false );
   TIMER_Enable( TIMER1, false );
}
void TIMER0_IRQHandler( void ) {
	/* The PWM handler for GREEN */
	/* Clear the interrupt flags */

	uint32_t flags = TIMER_IntGet( TIMER0 );
	TIMER_IntClear( TIMER0, flags );

}

void TIMER1_IRQHandler( void ) {
	/* The PWM handler for RED */
	/* Clear the interrupt flags */

	uint32_t flags = TIMER_IntGet( TIMER1 );
	TIMER_IntClear( TIMER1, flags );

}

void TIMER2_IRQHandler( void ) {

	/* Do something */
	usTicks++;



	uint32_t flags = TIMER_IntGet( TIMER2 );
	TIMER_IntClear( TIMER2, flags );
}

void SysTick_Handler( void ) {

	ticks++;
#if 0
	GPIO_PinOutClear(gpioPortD, sensor_out);
	UDELAY_Delay( 5 );
	GPIO_PinOutSet( gpioPortD, sensor_out );
	UDELAY_Delay( 10 );
	GPIO_PinOutClear( gpioPortD, sensor_out );

	uint32_t max_iterations = 100000;
	uint32_t i = 0;
	while ( !GPIO_PinInGet( gpioPortD, sensor_in )) {
		//if ( i++ > max_iterations ) {
			//return;
		//}
	}
	uint32_t startTick = usTicks;
	i = 0;
	while ( GPIO_PinInGet( gpioPortD, sensor_in ) ) {
		if ( i++ > max_iterations ) {
			return;
		}
	}
	uint32_t stopTick = usTicks;

	currentDistance = (float) ( stopTick - startTick ) * 0.034 / 2;

#endif
}


/***************************************************************************//**
* @brief  Main function
******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize gpio */
  gpioSetup();

  /* Initialize LED driver */
  BSP_LedsInit();

  red    = false;
  green  = false;
  ACTIVE = true;


  /* Set up SysTick to increment the tick once per second */
  if ( SysTick_Config( CMU_ClockFreqGet( cmuClock_CORE ) ) ) {
	  /* Something went wrong in the setup */
	  while(1);
  }

  /* Set up PWM control */
  PWMsetup();


  /* Set up timer for ultrasonic sensor */
  UDELAY_Calibrate();

  #if 0
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER2;
  uint32_t freq = CMU_ClockFreqGet( cmuClock_TIMER2 );
  TIMER2->CTRL |= TIMER_CTRL_PRESC_DIV1024;

  TIMER2->TOP |= freq / ( 1000 * 1024 );
  TIMER2->IEN |= TIMER_IEN_OF;
  NVIC_EnableIRQ( TIMER2_IRQn );
  TIMER2->CMD |= TIMER_CMD_START;
#endif

  CMU_ClockEnable(cmuClock_TIMER2, true);

  // Configure TIMER1 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timer2CCInit = TIMER_INITCC_DEFAULT;
  timer2CCInit.mode = timerCCModeCompare;
  timer2CCInit.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER2, 0, &timer2CCInit);

  uint32_t topValue2 = CMU_ClockFreqGet(cmuClock_HFPER) / (2*TIMER_FREQ * (1 << timerPrescale1))-1;
  TIMER_TopSet(TIMER2, topValue2);

  // Initialize and start timer with defined prescale
  TIMER_Init_TypeDef timer2Init = TIMER_INIT_DEFAULT;
  timer2Init.prescale = timerPrescale1;
  TIMER_Init(TIMER2, &timer2Init);

  TIMER_IntEnable(TIMER2, TIMER_IEN_CC0);
  NVIC_EnableIRQ( TIMER2_IRQn );

  /* Infinite loop */
  while ( 1 ) {
	  if ( INFINITE_ACTIVE_TIME_ENABLE ) {
		  /* Let the LED strip shine forever! */
		  /* Force the tick count the be < sleepTime, always,
		   * thus keeping the LED strip to shine with full brightness */
		  ticks = 0;
	  }
	  if ( ACTIVE && ( ticks < sleepTime ) ) {
		  /* Shine with full brightness */
		  if ( red && ( intensity_red != 100 ) ) {
			  intensity_red = 100;
			  TIMER_CompareBufSet(TIMER1, 0, ( topValue * intensity_red   ) / 100 );
		  }
		  else if ( green && ( intensity_green != 100 ) ) {
			  intensity_green = 100;
			  TIMER_CompareBufSet(TIMER0, 0, ( topValue * intensity_green ) / 100 );
		  }

	  }
	  else if ( ACTIVE && ticks > sleepTime ) {
		/* If strip is turned on AND we are above sleepTime... */
		/* Dim led strip */

		if ( red ) {
			TIMER_Enable( TIMER1, true );
			intensity_red = intensity_low_limit;
			TIMER_CompareBufSet(TIMER1, 0, ( topValue * intensity_red   ) / 100);
		}
		else if ( green ) {
			TIMER_Enable( TIMER0, true );
			intensity_green = intensity_low_limit;
			TIMER_CompareBufSet(TIMER0, 0, ( topValue * intensity_green ) / 100);
		}

		ACTIVE = false;

	  }
  }
}
