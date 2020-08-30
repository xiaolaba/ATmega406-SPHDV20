/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Safety routines.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/safety.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


#define MODULE_SAFETY
#include "pack.h"	//get macros we need
#include "safety.h"
//#include "iom406_320.h"
#include "iom406.h"     // IAR headerfile for Mega406 (EW 410)
#include "pwrmgmt.h"

//Sets up h/w protection registers based on EEPROM values from Calibration procedure.
//  Also sets up necessary pieces for using the HWP interrupt.
void HWProtectinit(void)
{
  PACK_PROTECT_DUV		//see pack.h for definitions & to make adjustments
  PACK_PROTECT_SHORT
  PACK_PROTECT_OVERC
  PACK_PROTECT_TIME

  BPCR = 0;

  BPIR = (1<<DUVIE) | (1<<COCIE) | (1<<DOCIE) | (1<<SCIE);

  //Interrupts should really be disabled before attempting to lock!
//  BPPLR = (1<<BPPLE) | (1<<BPPL);
//  BPPLR = (1<<BPPL);

}




// Handle hardware-protection trip.

#pragma vector = BPINT_vect
__interrupt void HWP_int(void)
{
  unsigned char temp = BPIR;

  if(temp & (1<<DUVIF))
    DoShutdown(SHUTDOWN_REASON_UNDERVOLTAGE);
  if(temp & (1<<COCIF))
    DoShutdown(SHUTDOWN_REASON_CHARGE_OVERCURRENT);
  if(temp & (1<<DOCIF))
    DoShutdown(SHUTDOWN_REASON_DISCHARGE_OVERCURRENT);
  if(temp & (1<<SCIF))
    DoShutdown(SHUTDOWN_REASON_SHORTCIRCUIT);

  BPIR |= 0xF0;	//this actually won't execute, but is here as a reminder
  		// that you need to MANUALLY reset these flags if you
  		// choose to not shut down in here.
}




//! \todo  Should be called periodically to double-check all h/w config & safety parameters
void SafetyScan(void)
{
  ; //! \todo  You can add more safety-monitoring code here.

}



