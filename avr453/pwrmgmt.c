/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Power mangagement.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/pwrmgmt.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)
#include <inavr.h>

#define MODULE_PWRMGMT
#include "pwrmgmt.h"

#include "ee.h"
#include "analog.h"
#include "timer.h"


void SetPowerSaveMode(void);
void SetIdleMode(void);
void SetActiveMode(void);



//All Mode changes should use this routine in order to ensure that only
// valid state transitions are made.  See AVR453 for details.
void ChangePowerMode(unsigned char newmode, unsigned char shutdown_reason)
{
  switch (PowerMode)	//what mode are we presently in?
  {
    case POWERMODE_POWERSAVE:
      if(newmode == POWERMODE_ACTIVE)
        SetActiveMode();
      else
      if(newmode == POWERMODE_POWEROFF)
        DoShutdown(shutdown_reason);
      break;


    case POWERMODE_IDLE:
      if(newmode == POWERMODE_ACTIVE)
        SetActiveMode();
      else
      if(newmode == POWERMODE_POWERSAVE)
        SetPowerSaveMode();
      else
      if(newmode == POWERMODE_POWEROFF)
        DoShutdown(shutdown_reason);
      break;


    case POWERMODE_ACTIVE:
      if(newmode == POWERMODE_IDLE)
        SetIdleMode();
      else
      if(newmode == POWERMODE_POWERSAVE)
        SetPowerSaveMode();
      else
      if(newmode == POWERMODE_POWEROFF)
        DoShutdown(shutdown_reason);
      break;


    case POWERMODE_POWEROFF:
    default:
      if(newmode == POWERMODE_POWEROFF)
        DoShutdown(SHUTDOWN_REASON_UNKNOWNSTATE);
      else
        SetPowerSaveMode();
      break;
  }
}



void DoShutdown(unsigned char reason)
{
  __disable_interrupt();

  //store the REASON code in EEPROM
  while(EECR & (1<<EEPE));
  while(SPMCSR & (1<<SPMEN));
  EEAR = EESTORAGE_SHUTDOWNREASON;
  EEDR = reason;
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go
  while(EECR & (1<<EEPE));								//wait until done

  SMCR = SLEEP_POWEROFF | (1<<SE);	//prep for power-off
  __sleep();
}


void SetPowerSaveMode(void)
{
  CCmode(CC_DISABLED);
  FCSR = 0;
  SMCR = SLEEP_POWERSAVE;
  PowerMode = POWERMODE_POWERSAVE;
  setWakeup(WU4sec, SLEEP_NONE);
}


void SetIdleMode(void)
{
  CCmode(CC_REGULAR);
  FCSR = ((1<<DFE) | (1<<CFE) | (1<<PFD));
  SMCR = SLEEP_IDLE;
  PowerMode = POWERMODE_IDLE;
}




void SetActiveMode(void)
{
  CCmode(CC_ACCUMULATE);
  FCSR = ((1<<DFE) | (1<<CFE) | (1<<PFD));
  SMCR = SLEEP_IDLE;
  PowerMode = POWERMODE_ACTIVE;
  setWakeup(WUoff,0);
}
