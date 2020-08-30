/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for timer.c
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/timer.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


void T0init(void);
void SetLEDbrightness(unsigned char value);
void SetLEDs(unsigned char flags);
void SetGenericTimer(unsigned char index, unsigned int delay);
unsigned char GetGenericTimer(unsigned char index);
void T1init(void);
void Wdog_init(unsigned char mode, unsigned char rate);
char setWakeup(unsigned char delay, unsigned char sleepmode);




#ifdef MODULE_TIMER

  unsigned char LEDs = 0;		//bit 0 = LED1, bit 4 = LED5
  unsigned char generictimer[8] = {0};
  unsigned char Timer32KHz = 0;		//this times out the 2-sec. startup of the xtal osc.

#else

  extern unsigned char LEDs;
  extern unsigned char generictimer[8];
  extern unsigned char Timer32KHz;

#endif



//List of assignments for generic timer channels
#define SMBfaultTimer 0
#define OneQtrSecond  1
#define genericTimer2 2
#define genericTimer3 3
#define genericTimer4 4
#define genericTimer5 5
#define genericTimer6 6
#define genericTimer7 7


//WATCHDOG TIMER DEFINITIONS
#define WDOG_MODE_DISABLED 0
#define WDOG_MODE_INTERRUPT 1
#define WDOG_MODE_RESET 2
#define WDOG_MODE_RSTINT 3

// Delay values for Watchdog timer
#define WD16ms  0
#define WD32ms  ( (1 << WDP0) )
#define WD64ms  ( (1 << WDP1) )
#define WD125ms ( (1 << WDP1) | (1 << WDP0) )
#define WD250ms ( (1 << WDP2) )
#define WD500ms ( (1 << WDP2) | (1 << WDP0) )
#define WD1sec  ( (1 << WDP2) | (1 << WDP1) )
#define WD2sec  ( (1 << WDP2) | (1 << WDP1) | (1 << WDP0) )
#define WD4sec  ( (1 << WDP3) )
#define WD8sec  ( (1 << WDP3) | (1 << WDP0) )



// Delay values for Wakeup timer
#define WUoff   0
#define WU31ms  1
#define WU62ms  2
#define WU125ms 3
#define WU250ms 4
#define WU500ms 5
#define WU1sec  6
#define WU2sec  7
#define WU4sec  8


//for Sleep Mode definitions, see pwrmgmt.h




