/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Timer routines.
 *
 * \par Application note:
 *      AVR453: Smart Battery Reference Design
 *
 * \par Documentation:
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com \n
 *      Original author: Rob G. Fries - Apt Inc.\n
 *
 * $Revision: 2687 $
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/timer.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)

#define MODULE_TIMER
#include "timer.h"

#include "main.h"	//for action_flags
#include "smbus.h"
#include "inavr.h"
#include "pwrmgmt.h"	//for sleep mode definitions
#include "analog.h"     //for access to RunningAcc
#include "pack.h"


/* *****************************************************
 *
 * Timer Usage:
 *
 * T0: serves several functions...
 *     1. basic tick rate for generic timers
 *     2. T0CMPA is optional LED PWM function
 *     3. T0CMPB is Charge FET PWM
 *   The clock src for T0 is main clk, 1MHz.
 *   Therefore, base period is 1MHz/256 = 3.9KHz = 256uS
 *   if prescaler = 1.
 *   For normal operation, we need both the LEDs on CMPA
 *   and the overflow as a timer tick. We will therefore
 *   use prescale = 8. This yields a 2.048mS tick, and
 *   a 100Hz update rate across all 5 LEDs.  The PWM for
 *   the FETs is available in this design, although it is
 *   not used.
 *
 * T1: not used in this design.
 *
 *
 * *****************************************************/

void T0init(void)
{
  DDRB |= (1<<6);	//set up PB6 output as OC0A
  TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<WGM01) | (1<<WGM00);
  TCCR0B = 2;		//prescale = 8
  OCR0A = 192;		//use 75% brightness level so we can see the waveform for testing
  OCR0B = 0;		//available for PWM use by FETs

  TIFR0 = 7;		//force all timer interrupt flags to be cleared.
  TIMSK0 = (1<<OCIE0A) | (1<<TOIE0);		//enable interrupts as needed.
}


void SetLEDbrightness(unsigned char value)
{
  //Don't let the LED brightness go higher than 250, just so that
  // we don't collide with the overflow interrupt.
  if(value <= 250)
    OCR0A = value;
  else
    OCR0A = 250;
}

void SetLEDs(unsigned char LEDflags)
{
  LEDs = LEDflags;

  //Each time the user updates the LED settings,
  // refresh the directional state of the I/O pins.
  DDRB |= (1<<4) | (1<<5) | (1<<7);
  DDRD |= (1<<0) | (1<<1);

}



// This ISR rotates between the five LEDs in this design, assuming the user jumpered
// the option on the SB100 PCB properly. The PWM function is used along with tripping
// this interrupt.  Active High output on the PWM pin is what drives the LEDs.  We rotate
// the LED outputs when the match occurs, which turns OFF the output at the same time;
// thus, we switch while the LEDs are off. (unless the user asserts maximum brightness,
// in which case there is no OFF time.

//! \todo  Some additional functionality could be added to allow the battery to wake up
//!        from all sleep modes when the pushbutton is pressed.  Presently it only
//!        works when the pack is not in Hibernate mode.

//extern unsigned char LEDflags;

#pragma vector = TIMER0_COMPA_vect
__interrupt void T0CMPA_ISR(void)
{
  static unsigned char select = 0;
  unsigned char temp = select;

  if(PINA & (1<<7))    //PA7 high?
  {
    DDRB &= ~(1<<6);  //shut off the DRIVE to the LEDs
    PORTB &= ~(1<<6);
    return;
  }
  else
    DDRB |= (1<<6);   //enable PB6 output drive

  temp++;
  if(6 == temp)
    temp = 1;
  select = temp;

  //First set ALL led ports High.
  PORTB |= (1<<4) | (1<<5) | (1<<7);
  PORTD |= (1<<0) | (1<<1);

  //Next, only set ONE low, based on the SELECT value and the LED bit.
  if((temp == 1) && (LEDs & (1<<0)))
      PORTB &= ~(1<<4);
  else
  if((temp == 2) && (LEDs & (1<<1)))
      PORTB &= ~(1<<5);
  else
  if((temp == 3) && (LEDs & (1<<2)))
      PORTB &= ~(1<<7);
  else
  if((temp == 4) && (LEDs & (1<<3)))
      PORTD &= ~(1<<0);
  else
  if((temp == 5) && (LEDs & (1<<4)))
      PORTD &= ~(1<<1);

  return;
}


//Since this drives the Charge and Precharge FET PWM in this design, there is no need
// to use this interrupt.  Its interrupt is not normally enabled, but this code shell
// is here just in case...
#pragma vector = TIMER0_COMPB_vect
__interrupt void T0_COMPB_ISR(void)
{
  return;	//! \todo  Add code here if you need to PWM the Charging FETs to control average charge current.
}




//NOTE:  all of these fire from within the ISR, so keep them short!!

void generictimer0expired(void)
{
  SMB_RestoreBus();
  return;
}

void generictimer1expired(void)
{
  SetQtrSec;
  return;
}

void generictimer2expired(void)
{
		//open for your use
  return;
}

void generictimer3expired(void)
{
		//open for your use
  return;
}

void generictimer4expired(void)
{
		//open for your use
  return;
}

void generictimer5expired(void)
{
		//open for your use
  return;
}

void generictimer6expired(void)
{
		//open for your use
  return;
}

void generictimer7expired(void)
{
		//open for your use
  return;
}




//Delay is given in 1mS intervals, but timed in 2mS intervals.
void SetGenericTimer(unsigned char index, unsigned int delay)
{
  ++delay;
  delay >>= 1;	//convert from 1mS interval to 2mS resolution
  generictimer[index] = (unsigned char) delay;
}

unsigned char GetGenericTimer(unsigned char index)
{
  return generictimer[index];
}


typedef void (*ptr2funcV_V)(void);

//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcV_V GenericExpire[8] =
{
  generictimer0expired,
  generictimer1expired,
  generictimer2expired,
  generictimer3expired,
  generictimer4expired,
  generictimer5expired,
  generictimer6expired,
  generictimer7expired
};



//This interrupt functions as the main timer tick for the code.  Each time, a list
// of generic timers is decremented.  Whenever one reaches zero, its expiration function
// is called.  As the prescaler for T0 is initialized to 8, this fires every 2.048mS.
#pragma vector = TIMER0_OVF_vect
__interrupt void T0OVF_ISR(void)
{
  unsigned int temp;
  unsigned char i;

  for(i=0; i<8; i++)
  {
    temp = generictimer[i];
    if(0 != temp)
    {
      temp--;
      generictimer[i] = temp;
      if(temp == 0)	//when it hits 0, flag it!
        (GenericExpire[i])();
    }
  }

  return;
}











//Timer 1 is not used in this design.

void T1init(void)
{
  return;
}

#pragma vector = TIMER1_COMP_vect
__interrupt void T1CMP_ISR(void)
{
  return;
}

#pragma vector = TIMER1_OVF_vect
__interrupt void T1OVF_ISR(void)
{
  return;
}






/* *****************************************************
 *
 *
 *
 *
 * *****************************************************/



void Wdog_init(unsigned char mode, unsigned char rate)
{
  unsigned char temp;
  unsigned char flags;

  if(mode == WDOG_MODE_DISABLED)
  {
    MCUSR &= ~(1<<WDRF);
    flags = SREG;
    __disable_interrupt();
    __watchdog_reset();
    WDTCSR = (1<<WDCE);
    WDTCSR = 0;
    if(flags & (1<<7))		//Were interrupts enabled before?
      __enable_interrupt();	// Then re-enable them.
    return;
  }

  // Clear all other bits from WDT tining setting.
  temp  = rate & ( (1 << WDP2) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0) );



  if(mode & WDOG_MODE_INTERRUPT)
  {
    temp |= 1<<WDIE;
  }

//  if(mode & WDOG_MODE_RESET)	//no special code required for this mode...
//  {
//    ;
//  }

  temp |= (1<<WDIF) | (1<<WDE);	//always try to clear the Int flag.


  flags = SREG;
    __disable_interrupt();
    __watchdog_reset();
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = temp;
  if(flags & (1<<7))		//Were interrupts enabled before?
      __enable_interrupt();	// Then re-enable them.
}



#pragma vector = WDT_vect
__interrupt void WDT_ISR(void)
{
  return;	//! \todo  Not used in this implementation
}



/* *****************************************************
 *
 * Set up and start the Wakeup timer, pushing into Sleep
 * mode as well.  If it's already running, this will not
 * reset the timer, allowing it to continue at the rate
 * previously defined.
 *
 * To prevent 4-second delays if you change the wakeup
 * interval after this is already running, a local static
 * copy of the current setting is kept and compared.
 * Whenever the rate is changed, the counter will be
 * forcibly reset.
 *
 * Return: 0 if OK, 1 if bad delay value or sleep mode requested.
 *
 * *****************************************************/
char setWakeup(unsigned char delay, unsigned char sleepmode)
{
  static unsigned char LastDelay = 0;
  unsigned char temp;

  if(delay > WU4sec)
    return 1;

  if(WUoff == delay)
  {
    WUTCSR = (WUTIF | WUTR);
    LastDelay = delay;
    return 0;
  }
  else
  if(LastDelay != delay)
    WUTCSR = ((delay-1) | WUTE | WUTIE | WUTR);
  else
    WUTCSR = ((delay-1) | WUTE | WUTIE);
  LastDelay = delay;

  //Assert sleep mode here, but only if it was requested.
  if(SLEEP_NONE == sleepmode)
    return 0;
  if(SLEEP_POWEROFF < sleepmode)
    return 1;
  temp = sleepmode & ~(1<<SE);
  SMCR = sleepmode | (1<<SE);
  __sleep();

  //After waking up, the datasheet indicates you should clear the SE bit.
  SMCR = temp;
  return 0;
}




#pragma vector = WAKE_UP_vect
__interrupt void WakeUp_ISR(void)
{
  RunningAcc -= PACK_HIBERNATEAMOUNT;
  return;
}

