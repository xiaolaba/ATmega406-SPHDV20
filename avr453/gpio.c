/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *	I/O routines.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/gpio.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)
#include "gpio.h"
#include "timer.h"




// Configure all I/O
void PinInit(void)
{
  ; //! \todo  Add code here for any special I/O initialization required.

}



//Activate pushbutton interrupt.
void PBinit(void)
{
  SetLEDbrightness(192);  //use 75% high-time so we can see it changing on an oscilloscope.
  DDRA &= ~(1<<7);        //make PA7 be input
  PORTA |= 1<<7;          //turn on PA7 pullup
  SetLEDs(0x1F);
}



//Disable the PB interrupt to deactivate the function.
void PBdisable(void)
{
  ;	// not used
}



#pragma vector = INT3_vect
__interrupt void PB_ISR(void)
{
  ;	// not used
}





